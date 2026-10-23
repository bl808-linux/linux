// SPDX-License-Identifier: GPL-2.0

//! UART drivers using the C serial core.
//!
//! This module implements only the operations needed by the Bouffalo Lab UART.
//! The TTY, line discipline and userspace interface remain owned by serial core.

use crate::{
    bindings,
    device::{Bound, Device},
    error::to_result,
    irq,
    prelude::*,
    sync::{Arc, Mutex},
    types::Opaque,
    ThisModule,
};
use core::{marker::PhantomData, ptr::NonNull};

/// A registered serial driver. Ports retain a reference until removal completes.
#[pin_data(PinnedDrop)]
pub struct Driver {
    name: &'static CStr,
    #[pin]
    inner: Opaque<bindings::uart_driver>,
}

// SAFETY: Serial core serializes registration and port management internally.
unsafe impl Send for Driver {}
// SAFETY: The driver exposes no mutable access to the C object.
unsafe impl Sync for Driver {}

impl Driver {
    /// Registers a driver with dynamically assigned device numbers.
    pub fn new(
        name: &'static CStr,
        device_name: &'static CStr,
        ports: u32,
        module: &'static ThisModule,
    ) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            name,
            inner <- Opaque::try_ffi_init(move |ptr: *mut bindings::uart_driver| {
                if ports == 0 || ports > i32::MAX as u32 {
                    return Err(EINVAL);
                }
                // SAFETY: The initializer exclusively owns the pinned slot. The strings and
                // module outlive registration; private fields are initially zero.
                unsafe {
                    ptr.write(bindings::uart_driver::default());
                    (*ptr).owner = module.as_ptr();
                    (*ptr).driver_name = name.as_char_ptr();
                    (*ptr).dev_name = device_name.as_char_ptr();
                    (*ptr).nr = ports as i32;
                    to_result(bindings::uart_register_driver(ptr))
                }
            }),
        })
    }
}

#[pinned_drop]
impl PinnedDrop for Driver {
    fn drop(self: Pin<&mut Self>) {
        // SAFETY: Registration succeeded and every port holds an Arc to this driver.
        unsafe { bindings::uart_unregister_driver(self.inner.get()) };
    }
}

/// Hardware callbacks. Methods receiving [`LockedPort`] run with the UART lock held.
///
/// Callbacks must not sleep, except `quiesce`, `startup` and `shutdown`, which are
/// called from the serial-core open/close path without the UART spinlock held.
pub trait Operations: Send + Sync + Sized {
    /// Whether both the transmit FIFO and shift register are empty.
    fn tx_empty(&self) -> bool;
    /// Sets modem outputs; two-wire drivers may document an empty implementation.
    fn set_mctrl(&self, port: &LockedPort<'_>, value: u32);
    /// Reads modem inputs.
    fn get_mctrl(&self, port: &LockedPort<'_>) -> u32;
    /// Starts transmission.
    fn start_tx(&self, port: &LockedPort<'_>);
    /// Stops transmission.
    fn stop_tx(&self, port: &LockedPort<'_>);
    /// Stops reception.
    fn stop_rx(&self, port: &LockedPort<'_>);
    /// Masks device interrupts before installing the IRQ handler.
    fn quiesce(&self, port: &Port<'_, Self>);
    /// Initializes the hardware and enables interrupts after IRQ registration.
    fn startup(&self, port: &Port<'_, Self>);
    /// Masks device interrupts before synchronizing and freeing the IRQ.
    fn shutdown(&self, port: &Port<'_, Self>);
    /// Sets framing and baud rate; the implementation must acquire the UART lock.
    fn set_termios(&self, port: &Port<'_, Self>, new: &mut Termios, old: Option<&Termios>);
    /// Services a hard interrupt under the UART lock.
    fn handle_irq(&self, port: &LockedPort<'_>) -> irq::IrqReturn;
}

/// Configuration supplied before registering a memory-mapped UART.
pub struct PortConfig {
    /// Serial alias index.
    pub line: u32,
    /// Physical register base, for serial-core reporting.
    pub mapbase: bindings::resource_size_t,
    /// Baud generator frequency in Hz.
    pub clock: u32,
    /// Hardware FIFO capacity in bytes.
    pub fifo_size: u32,
}

/// A pinned UART port and its hardware state.
///
/// # Invariants
///
/// The C port is registered and its private_data points to this object. The object,
/// device and hardware state remain valid until uart_remove_one_port completes.
#[pin_data(PinnedDrop)]
pub struct Port<'bound, T: Operations> {
    #[pin]
    inner: Opaque<bindings::uart_port>,
    #[pin]
    data: T,
    driver: Arc<Driver>,
    device: &'bound Device<Bound>,
    irq_number: u32,
    #[pin]
    irq: Mutex<Option<ActiveIrq<'bound, T>>>,
}

// SAFETY: Access to the C port is serialized by serial core and its port lock.
unsafe impl<'a, T: Operations + 'a> Send for Port<'a, T> {}
// SAFETY: Hardware state is Sync; the IRQ slot is protected by its mutex.
unsafe impl<'a, T: Operations + 'a> Sync for Port<'a, T> {}

impl<'bound, T: Operations + 'bound> Port<'bound, T> {
    /// Registers a port after initializing its hardware state.
    ///
    /// # Safety
    ///
    /// The IRQ must belong to `device`. The caller must not forget the resulting
    /// port or otherwise prevent its destructor from running before device unbind.
    pub unsafe fn new(
        driver: Arc<Driver>,
        device: &'bound Device<Bound>,
        irq_number: u32,
        config: PortConfig,
        data: impl PinInit<T, Error> + 'bound,
    ) -> impl PinInit<Self, Error> + 'bound {
        try_pin_init!(&this in Self {
            data <- data,
            driver,
            device,
            irq_number,
            irq <- crate::new_mutex!(None),
            inner <- Opaque::ffi_init(|ptr: *mut bindings::uart_port| {
                // SAFETY: The slot is exclusively owned and not yet published.
                unsafe {
                    ptr.write(bindings::uart_port::default());
                    (*ptr).dev = device.as_raw();
                    (*ptr).irq = *irq_number;
                    (*ptr).line = config.line;
                    (*ptr).mapbase = config.mapbase;
                    (*ptr).uartclk = config.clock;
                    (*ptr).fifosize = config.fifo_size;
                    (*ptr).iotype = bindings::uart_iotype_UPIO_MEM;
                    (*ptr).type_ = bindings::PORT_GENERIC as u32;
                    bindings::uart_port_set_boot_autoconf(ptr);
                    (*ptr).ops = &Self::OPS;
                    (*ptr).private_data = this.as_ptr().cast();
                }
            }),
            _: {
                // SAFETY: All fields are initialized and callbacks may now access them. Pin
                // initialization keeps the slot alive until remove or initialization failure.
                let port = unsafe { this.as_ref() };
                // SAFETY: The line is within the driver's advertised array. C rejects duplicate
                // lines. On failure it does not retain the port.
                unsafe {
                    if config.line >= (*port.driver.inner.get()).nr as u32 {
                        return Err(EINVAL);
                    }
                    to_result(bindings::uart_add_one_port(
                        port.driver.inner.get(), port.inner.get(),
                    ))?;
                }
            },
        })
    }

    /// Runs a closure with the serial-core UART lock held and local IRQs saved.
    pub fn with_lock<R>(&self, f: impl FnOnce(&LockedPort<'_>) -> R) -> R {
        let mut flags = 0;
        // SAFETY: The port is initialized and valid for this borrow.
        unsafe { bindings::uart_port_lock_irqsave(self.inner.get(), &mut flags) };
        let guard = PortGuard {
            port: self.inner.get(),
            flags,
        };
        let locked = LockedPort {
            raw: self.inner.get(),
            _lifetime: PhantomData,
        };
        let result = f(&locked);
        drop(guard);
        result
    }

    /// # Safety
    /// ptr must refer to this registered port for the returned borrow.
    unsafe fn from_raw<'a>(ptr: *mut bindings::uart_port) -> &'a Self {
        // SAFETY: private_data is initialized to the pinned Self in new.
        unsafe { &*((*ptr).private_data.cast::<Self>()) }
    }

    const OPS: bindings::uart_ops = bindings::uart_ops {
        tx_empty: Some(Self::tx_empty),
        set_mctrl: Some(Self::set_mctrl),
        get_mctrl: Some(Self::get_mctrl),
        start_tx: Some(Self::start_tx),
        stop_tx: Some(Self::stop_tx),
        stop_rx: Some(Self::stop_rx),
        startup: Some(Self::startup),
        shutdown: Some(Self::shutdown),
        set_termios: Some(Self::set_termios),
        type_: Some(Self::port_type),
        config_port: Some(Self::config_port),
        verify_port: Some(Self::verify_port),
        ..pin_init::zeroed()
    };

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn tx_empty(ptr: *mut bindings::uart_port) -> u32 {
        // SAFETY: Serial core holds the registration alive during callbacks.
        let port = unsafe { Self::from_raw(ptr) };
        if port.data.tx_empty() {
            bindings::TIOCSER_TEMT
        } else {
            0
        }
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn set_mctrl(ptr: *mut bindings::uart_port, value: u32) {
        // SAFETY: Serial core holds the UART lock and a live registration.
        let port = unsafe { Self::from_raw(ptr) };
        port.data.set_mctrl(
            &LockedPort {
                raw: ptr,
                _lifetime: PhantomData,
            },
            value,
        );
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn get_mctrl(ptr: *mut bindings::uart_port) -> u32 {
        // SAFETY: Serial core holds the UART lock and a live registration.
        let port = unsafe { Self::from_raw(ptr) };
        port.data.get_mctrl(&LockedPort {
            raw: ptr,
            _lifetime: PhantomData,
        })
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn start_tx(ptr: *mut bindings::uart_port) {
        // SAFETY: Serial core holds the UART lock and a live registration.
        let port = unsafe { Self::from_raw(ptr) };
        port.data.start_tx(&LockedPort {
            raw: ptr,
            _lifetime: PhantomData,
        });
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn stop_tx(ptr: *mut bindings::uart_port) {
        // SAFETY: Serial core holds the UART lock and a live registration.
        let port = unsafe { Self::from_raw(ptr) };
        port.data.stop_tx(&LockedPort {
            raw: ptr,
            _lifetime: PhantomData,
        });
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn stop_rx(ptr: *mut bindings::uart_port) {
        // SAFETY: Serial core holds the UART lock and a live registration.
        let port = unsafe { Self::from_raw(ptr) };
        port.data.stop_rx(&LockedPort {
            raw: ptr,
            _lifetime: PhantomData,
        });
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn startup(ptr: *mut bindings::uart_port) -> i32 {
        // SAFETY: Serial core serializes startup/shutdown and keeps the port alive.
        let port = unsafe { Self::from_raw(ptr) };
        let result = (|| {
            port.data.quiesce(port);
            // SAFETY: Port::new requires an IRQ belonging to device. No registration exists
            // before startup, and serial core serializes startup and shutdown.
            let request = unsafe { irq::IrqRequest::new(port.device, port.irq_number) };
            // SAFETY: shutdown and pinned drop release the registration before the port dies.
            let registration = unsafe {
                irq::Registration::new(
                    request,
                    irq::Flags::TRIGGER_NONE,
                    port.driver.name,
                    Ok(PortIrq {
                        port: NonNull::from(port),
                    }),
                )
            };
            *port.irq.lock() = Some(KBox::pin_init(registration, GFP_KERNEL)?);
            port.data.startup(port);
            Ok(0)
        })();
        result.unwrap_or_else(|e: Error| e.to_errno())
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn shutdown(ptr: *mut bindings::uart_port) {
        // SAFETY: Serial core serializes shutdown against startup and removal.
        let port = unsafe { Self::from_raw(ptr) };
        port.data.shutdown(port);
        // Free/synchronize outside the UART spinlock: the handler may be waiting on it.
        drop(port.irq.lock().take());
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn set_termios(
        ptr: *mut bindings::uart_port,
        new: *mut bindings::ktermios,
        old: *const bindings::ktermios,
    ) {
        // SAFETY: Serial core supplies live, exclusively borrowed new termios and optionally old.
        let port = unsafe { Self::from_raw(ptr) };
        // SAFETY: Termios is transparent; serial core guarantees these pointers for this call.
        unsafe {
            port.data
                .set_termios(port, &mut *new.cast(), old.cast::<Termios>().as_ref())
        };
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn port_type(ptr: *mut bindings::uart_port) -> *const crate::ffi::c_char {
        // SAFETY: The driver registration outlives its ports.
        let port = unsafe { Self::from_raw(ptr) };
        // SAFETY: driver_name is immutable after registration and points to a static C string.
        unsafe { (*port.driver.inner.get()).driver_name }
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn config_port(ptr: *mut bindings::uart_port, flags: i32) {
        if flags & bindings::UART_CONFIG_TYPE as i32 != 0 {
            // SAFETY: Serial core serializes port configuration.
            unsafe { (*ptr).type_ = bindings::PORT_GENERIC as u32 };
        }
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn verify_port(
        _ptr: *mut bindings::uart_port,
        serial: *mut bindings::serial_struct,
    ) -> i32 {
        // SAFETY: Serial core supplies a valid serial_struct for the callback.
        let kind = unsafe { (*serial).type_ };
        if kind == bindings::PORT_UNKNOWN as i32 || kind == bindings::PORT_GENERIC {
            0
        } else {
            EINVAL.to_errno()
        }
    }
}

#[pinned_drop]
impl<'a, T: Operations + 'a> PinnedDrop for Port<'a, T> {
    fn drop(self: Pin<&mut Self>) {
        // SAFETY: This port was successfully added and is still pinned. Removal hangs up all
        // users and invokes shutdown before returning, while hardware and driver remain alive.
        unsafe { bindings::uart_remove_one_port(self.driver.inner.get(), self.inner.get()) };
    }
}

type ActiveIrq<'a, T> = Pin<KBox<irq::Registration<'a, PortIrq<'a, T>>>>;

struct PortIrq<'bound, T: Operations + 'bound> {
    port: NonNull<Port<'bound, T>>,
}

// SAFETY: The pinned port is Sync and remains valid until IRQ unregistration synchronizes.
unsafe impl<'a, T: Operations + 'a> Send for PortIrq<'a, T> {}
// SAFETY: The handler accesses mutable UART state only under the port lock.
unsafe impl<'a, T: Operations + 'a> Sync for PortIrq<'a, T> {}

impl<'a, T: Operations + 'a> irq::Handler for PortIrq<'a, T> {
    fn handle(&self) -> irq::IrqReturn {
        // SAFETY: shutdown synchronizes the IRQ before the port can be destroyed.
        let port = unsafe { self.port.as_ref() };
        port.with_lock(|locked| port.data.handle_irq(locked))
    }
}

struct PortGuard {
    port: *mut bindings::uart_port,
    flags: crate::ffi::c_ulong,
}

impl Drop for PortGuard {
    fn drop(&mut self) {
        // SAFETY: PortGuard is created only after acquiring this UART lock.
        unsafe { bindings::uart_port_unlock_irqrestore(self.port, self.flags) };
    }
}

/// A UART borrowed while its serial-core lock is held. Cannot escape the borrow.
pub struct LockedPort<'a> {
    raw: *mut bindings::uart_port,
    _lifetime: PhantomData<&'a mut ()>,
}

/// Termios borrowed for a serial-core set_termios callback.
#[repr(transparent)]
pub struct Termios(bindings::ktermios);

impl Termios {
    /// Control mode flags.
    pub fn cflag(&self) -> u32 {
        self.0.c_cflag
    }
    /// Input mode flags.
    pub fn iflag(&self) -> u32 {
        self.0.c_iflag
    }
    /// Replaces the control mode flags, including reflecting unsupported modes.
    pub fn set_cflag(&mut self, value: u32) {
        self.0.c_cflag = value;
    }
    /// Data bits per character.
    pub fn char_size(&self) -> u32 {
        // SAFETY: c_cflag is a valid termios control flag value.
        unsafe { bindings::tty_get_char_size(self.0.c_cflag).into() }
    }
}

impl LockedPort<'_> {
    /// Selects a supported baud rate using serial-core termios rules.
    pub fn baud_rate(&self, new: &mut Termios, old: Option<&Termios>, min: u32, max: u32) -> u32 {
        // SAFETY: The port is locked, and the termios references are valid for this callback.
        unsafe {
            bindings::uart_get_baud_rate(
                self.raw,
                &mut new.0,
                old.map_or(core::ptr::null(), |t| &t.0),
                min,
                max,
            )
        }
    }

    /// Updates serial-core status masks, timeout, and the actual termios baud rate.
    pub fn configure(&self, new: &mut Termios, baud: u32, read_mask: u32, ignore_mask: u32) {
        // SAFETY: All port fields are accessed under the UART lock. new is exclusively borrowed.
        unsafe {
            (*self.raw).read_status_mask = read_mask;
            (*self.raw).ignore_status_mask = ignore_mask;
            bindings::uart_update_timeout(self.raw, new.0.c_cflag, baud);
            if bindings::tty_termios_baud_rate(&new.0) != 0 {
                bindings::tty_termios_encode_baud_rate(&mut new.0, baud, baud);
            }
        }
    }
}
