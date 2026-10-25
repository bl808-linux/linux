// SPDX-License-Identifier: GPL-2.0

//! Serial-core support for memory-mapped, two-wire polling consoles.
//!
//! C header: [`include/linux/serial_core.h`](srctree/include/linux/serial_core.h).
//!
//! [`Driver`] owns the serial driver and its legacy console. Each device-managed
//! port owns its hardware data and retains the driver until removal completes.
//! Registration failures release unpublished data, or remove a published port if
//! devres registration fails. Removal drains serial-core users and console SRCU
//! readers before releasing hardware data and the driver reference.
//! Device-managed resources can be revoked before port removal; callbacks must
//! use checked resource access rather than assume the parent is still bound.
//!
//! The adapters acquire the UART lock for [`SerialOps::set_termios`] and normal
//! console writes. Oops output may proceed without the lock after a failed
//! trylock; it never receives a [`LockedPort`]. Callbacks must not sleep.
//! Serial core owns TTY buffering; adapters provide IRQ-driven two-wire I/O.
//! TTY instances retain shared driver state and their UART module until cleanup.
//! DMA and modem control are outside this interface.

use crate::{
    bindings,
    device::{
        Bound,
        Device, //
    },
    devres,
    error::to_result,
    prelude::*,
    sync::{
        aref::ARef,
        Arc, //
    },
    types::Opaque,
    ThisModule, //
};
use core::{
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

mod console;
mod registration;
mod termios;

use self::console::ConsoleAdapter;
pub use self::termios::Termios;

/// A typed handle retaining serial state independently of its registration.
pub struct Driver<T: SerialOps> {
    state: Arc<DriverState>,
    _type: PhantomData<T>,
}

impl<T: SerialOps> core::ops::Deref for Driver<T> {
    type Target = DriverState;
    fn deref(&self) -> &DriverState {
        &self.state
    }
}

/// Owns admission of new ports and TTY instances.
pub struct DriverRegistration<T: SerialOps> {
    driver: Arc<Driver<T>>,
}

impl<T: SerialOps> DriverRegistration<T> {
    /// Registers a built-in or loadable UART driver.
    ///
    /// # Safety
    /// The registration must be dropped before its module is unloaded; it must
    /// not be forgotten or leaked. Ports must be removed before module exit ends.
    pub unsafe fn new(config: DriverConfig, module: &'static ThisModule) -> Result<Self> {
        let state = Arc::pin_init(DriverState::new::<T>(config, module), GFP_KERNEL)?;
        // SAFETY: No port can be published until this constructor returns.
        unsafe { registration::rust_serial_install_hooks(Arc::as_ptr(&state).cast_mut()) };
        let driver = match Arc::new(
            Driver {
                state: state.clone(),
                _type: PhantomData,
            },
            GFP_KERNEL,
        ) {
            Ok(driver) => driver,
            Err(error) => {
                // SAFETY: Keep an admission reference until all installers have left.
                unsafe { registration::rust_serial_stop_admission(Arc::as_ptr(&state).cast_mut()) };
                return Err(error.into());
            }
        };
        Ok(Self { driver })
    }
    /// Returns a handle for device-managed port registrations.
    pub fn driver(&self) -> Arc<Driver<T>> {
        self.driver.clone()
    }
}

impl<T: SerialOps> Drop for DriverRegistration<T> {
    fn drop(&mut self) {
        // SAFETY: The registration holds a live state reference throughout the barrier.
        unsafe {
            registration::rust_serial_stop_admission(Arc::as_ptr(&self.driver.state).cast_mut())
        };
    }
}

/// Shared C registration state. Foreign callbacks retain an Arc until cleanup.
#[doc(hidden)]
#[pin_data(PinnedDrop)]
pub struct DriverState {
    name: crate::str::CString,
    device_name: crate::str::CString,
    ports: u32,
    default_baud: u32,
    owner: *mut bindings::module,
    accepting: AtomicBool,
    lines: KVec<AtomicBool>,
    #[pin]
    console: Opaque<bindings::console>,
    #[pin]
    inner: Opaque<bindings::uart_driver>,
}

// SAFETY: Foreign fields use serial-core locks; admission uses tty_mutex and atomics.
unsafe impl Send for DriverState {}
// SAFETY: The pinned state and immutable owned names outlive all retained TTYs/ports.
unsafe impl Sync for DriverState {}

impl DriverState {
    /// Initializes a two-wire serial driver with dynamic TTY device numbers.
    ///
    /// `config.default_baud` supplies the initial default console rate.
    fn new<T: SerialOps>(
        config: DriverConfig,
        module: &'static ThisModule,
    ) -> impl PinInit<Self, Error> {
        pin_init::pin_init_scope(move || {
            config.validate()?;
            let name = crate::str::CString::try_from(config.name)?;
            let device_name = crate::str::CString::try_from(config.device_name)?;
            let mut lines = KVec::with_capacity(config.ports as usize, GFP_KERNEL)?;
            for _ in 0..config.ports {
                lines.push(AtomicBool::new(false), GFP_KERNEL)?;
            }
            Ok(try_pin_init!(&this in Self {
                ports: config.ports,
                default_baud: config.default_baud,
                name,
                device_name,
                owner: module.as_ptr(),
                accepting: AtomicBool::new(true),
                lines,
                console <- Opaque::ffi_init(|ptr: *mut bindings::console| {
                    let mut console = bindings::console {
                        write: Some(ConsoleAdapter::<T>::console_write),
                        setup: Some(ConsoleAdapter::<T>::console_setup),
                        device: Some(bindings::uart_console_device),
                        flags: bindings::cons_flags_CON_PRINTBUFFER as i16,
                        index: -1,
                        ispeed: config.default_baud,
                        ospeed: config.default_baud,
                        ..Default::default()
                    };
                    // SAFETY: The validated name fits with its trailing NUL, supplied by
                    // the zeroed array. The source and destination do not overlap, and the
                    // initializer exclusively owns the pinned destination slot.
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            (*this.as_ptr()).device_name.as_char_ptr(),
                            console.name.as_mut_ptr(),
                            config.device_name.to_bytes().len(),
                        );
                        ptr.write(console);
                    }
                }),
                inner <- Opaque::try_ffi_init(move |ptr: *mut bindings::uart_driver| {
                    // SAFETY: The preceding field initializer initialized the pinned console.
                    let console = unsafe { (*this.as_ptr()).console.get() };
                    let driver = bindings::uart_driver {
                        owner: module.as_ptr(),
                        // SAFETY: The preceding initializers own these stable C strings.
                        driver_name: unsafe { (*this.as_ptr()).name.as_char_ptr() },
                        // SAFETY: The device name was initialized before this C object.
                        dev_name: unsafe { (*this.as_ptr()).device_name.as_char_ptr() },
                        nr: config.ports as i32,
                        cons: console,
                        ..Default::default()
                    };
                    // SAFETY: Both objects are private and pinned. No callback can use these
                    // reciprocal pointers before registration below publishes the driver.
                    unsafe {
                        ptr.write(driver);
                        (*console).data = ptr.cast();
                    }
                    // SAFETY: All C fields and callbacks are initialized. Names and module
                    // outlive registration; ports retain the driver until removal completes.
                    to_result(unsafe { bindings::uart_register_driver(ptr) })
                }),
            }))
        })
    }
}

/// Configuration for a serial driver and its polling console.
#[derive(Clone, Copy)]
pub struct DriverConfig {
    /// Driver name used for serial-core reporting.
    pub name: &'static CStr,
    /// TTY and console name prefix.
    pub device_name: &'static CStr,
    /// Number of serial-core lines owned by this driver.
    pub ports: u32,
    /// Initial baud rate when console options do not supply one.
    pub default_baud: u32,
}

impl DriverConfig {
    fn validate(&self) -> Result {
        // struct console uses name[16] and a signed short index. Its setup
        // callback passes the baud rate to uart_set_options() as a signed int.
        const CONSOLE_NAME_LEN: usize = 16;
        if self.ports == 0
            || self.ports > i16::MAX as u32
            || self.default_baud == 0
            || self.default_baud > i32::MAX as u32
            || self.device_name.to_bytes().len() >= CONSOLE_NAME_LEN
        {
            return Err(EINVAL);
        }
        Ok(())
    }
}

#[pinned_drop]
impl PinnedDrop for DriverState {
    fn drop(self: Pin<&mut Self>) {
        // SAFETY: All port and TTY references have ended before this final Arc drop.
        unsafe { registration::rust_serial_unregister_state(self.inner.get()) };
    }
}

/// Serial hardware callbacks for a two-wire IRQ-driven UART and polling console.
///
/// Modem control and DMA are not supported. Startup, shutdown and interrupt
/// callbacks are serialized with transmit callbacks by the UART lock.
///
/// Callbacks must not sleep or allocate. The adapters lock the UART and save local
/// IRQ state for `set_termios` and normal `console_write` calls. During an oops,
/// console output may run without the lock. `tx_empty` acquires no lock and may
/// be called with the UART lock already held.
///
/// The parent device allocation remains live, but its managed resources may
/// already have been revoked. Callbacks must check access to those resources.
pub trait SerialOps: Send + Sync + Sized + 'static {
    /// Enables reception and UART interrupts after IRQ registration.
    fn startup(&self, _port: &LockedPort<'_>) -> Result {
        Ok(())
    }
    /// Disables UART interrupts and reception. Must be idempotent.
    fn shutdown(&self, _port: &LockedPort<'_>) {}
    /// Fills available hardware TX FIFO slots without waiting.
    fn start_tx(&self, _port: &LockedPort<'_>) {}
    /// Masks transmit interrupts.
    fn stop_tx(&self, _port: &LockedPort<'_>) {}
    /// Stops reception.
    fn stop_rx(&self, _port: &LockedPort<'_>) {}
    /// Handles pending UART interrupts under the UART lock.
    fn interrupt(&self, _port: &LockedPort<'_>) -> bool {
        false
    }

    /// Whether both the transmit FIFO and shift register are empty.
    fn tx_empty(&self, port: &Port<'_>) -> bool;
    /// Applies console framing and baud rate, reflecting unsupported modes in `new`.
    ///
    /// The UART lock is held and local IRQs are disabled throughout this call.
    fn set_termios(&self, port: &LockedPort<'_>, new: &mut Termios, old: Option<&Termios>);
    /// Writes a console record using bounded polling, including during an oops.
    fn console_write(&self, port: &Port<'_>, bytes: &[u8]);
}

/// Configuration supplied before registering a memory-mapped UART.
pub struct PortConfig {
    /// Serial-core line index.
    pub line: u32,
    /// Physical register base, for serial-core reporting.
    pub mapbase: bindings::resource_size_t,
    /// Baud generator frequency in Hz.
    pub clock: u32,
    /// Hardware FIFO capacity in bytes.
    pub fifo_size: u32,
    /// Linux IRQ obtained from the parent device; zero for inactive test ports.
    pub irq: u32,
}

/// Registers an owned port until the device is unbound.
///
/// All state is owned and `'static`; no callback can retain a borrow into probe's
/// stack. The registration is transferred to devres rather than returned to the
/// caller. Forgetting a caller-owned object cannot leave borrowed data registered.
/// On devres allocation failure, the registration is removed before returning.
pub fn register_port<T: SerialOps>(
    driver: Arc<Driver<T>>,
    device: &Device<Bound>,
    config: PortConfig,
    data: impl PinInit<T, Error>,
) -> Result {
    if config.line >= driver.ports {
        return Err(EINVAL);
    }
    if !driver.accepting.load(Ordering::Acquire) {
        return Err(ENODEV);
    }
    let line = config.line;
    let irq = config.irq;
    let port = KBox::pin_init(
        PortData::new(driver, device.into(), config, data),
        GFP_KERNEL,
    )?;
    let mut registration = Registration {
        port,
        added: false,
        irq_registered: false,
    };
    // SAFETY: The pinned allocation remains owned by registration until free_irq
    // has drained its callback, including all registration rollback paths.
    unsafe {
        let raw = registration.port.inner.get();
        if irq != 0 {
            to_result(bindings::request_irq(
                irq,
                Some(SerialAdapter::<T>::interrupt),
                0,
                driver_name(&registration.port),
                raw.cast(),
            ))?;
            registration.irq_registered = true;
        }
        to_result(bindings::uart_add_one_port(
            registration.port.driver.inner.get(),
            raw,
        ))?;
        registration.added = true;
        registration.port.published.store(true, Ordering::Release);
    }
    let state = registration.port.driver.state.clone();
    devres::register(device, registration, GFP_KERNEL)?;
    state.lines[line as usize].store(true, Ordering::Release);
    Ok(())
}

fn driver_name<T: SerialOps>(port: &PortData<T>) -> *const crate::ffi::c_char {
    port.driver.name.as_char_ptr()
}

#[pin_data]
struct PortData<T: SerialOps> {
    #[pin]
    inner: Opaque<bindings::uart_port>,
    #[pin]
    data: T,
    driver: Arc<Driver<T>>,
    device: ARef<Device>,
    published: AtomicBool,
}

// SAFETY: uart_port's mutable C fields live in Opaque and use serial-core locking.
// Data is Send + Sync; device/driver references keep their allocations alive.
unsafe impl<T: SerialOps> Send for PortData<T> {}
// SAFETY: Rust-owned fields are immutable after publication. C state changes use
// port_mutex, tty_port.mutex, the UART lock and console-list/SRCU synchronization.
unsafe impl<T: SerialOps> Sync for PortData<T> {}

impl<T: SerialOps> PortData<T> {
    fn new(
        driver: Arc<Driver<T>>,
        device: ARef<Device>,
        config: PortConfig,
        data: impl PinInit<T, Error>,
    ) -> impl PinInit<Self, Error> {
        try_pin_init!(&this in Self {
            data <- data,
            driver,
            device,
            published: AtomicBool::new(false),
            inner <- Opaque::ffi_init(|ptr: *mut bindings::uart_port| {
                let port = bindings::uart_port {
                    dev: device.as_raw(),
                    line: config.line,
                    irq: config.irq,
                    mapbase: config.mapbase,
                    uartclk: config.clock,
                    fifosize: config.fifo_size,
                    iotype: bindings::uart_iotype_UPIO_MEM,
                    type_: bindings::PORT_GENERIC as u32,
                    ops: &SerialAdapter::<T>::OPS,
                    private_data: this.as_ptr().cast(),
                    ..Default::default()
                };
                // SAFETY: The slot is private and exclusively owned until registration.
                unsafe {
                    ptr.write(port);
                    bindings::uart_port_set_boot_autoconf(ptr);
                }
            }),
        })
    }

    /// # Safety
    /// `ptr` must be a PortData<T> port in a serial-core callback. Registration
    /// owns its allocation until uart_remove_one_port returns; removal waits for
    /// console_srcu readers and serial-core users before this port is freed.
    /// Other device-managed resources may already have been revoked.
    unsafe fn with_port<R>(
        ptr: *mut bindings::uart_port,
        f: impl for<'a> FnOnce(&'a Self, &Port<'a>) -> R,
    ) -> R {
        // SAFETY: The private_data type is fixed by Driver<T>/register_port<T>.
        let state = unsafe { &*(*ptr).private_data.cast::<Self>() };
        f(
            state,
            // INVARIANT: The live callback allocation owns both this UART and device.
            &Port {
                raw: ptr,
                device: &state.device,
            },
        )
    }
}

/// Converts serial-core callbacks to the hardware operations and lock contexts.
struct SerialAdapter<T: SerialOps>(PhantomData<T>);

impl<T: SerialOps> SerialAdapter<T> {
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
        // SAFETY: Serial core retains the registered port throughout this callback.
        unsafe {
            PortData::<T>::with_port(ptr, |state, port| {
                if state.data.tx_empty(port) {
                    bindings::TIOCSER_TEMT
                } else {
                    0
                }
            })
        }
    }

    // Serial core requires these modem callbacks. This two-wire interface has no
    // modem outputs and reports carrier/DSR/CTS asserted without hardware access.
    extern "C" fn set_mctrl(_ptr: *mut bindings::uart_port, _value: u32) {}

    extern "C" fn get_mctrl(_ptr: *mut bindings::uart_port) -> u32 {
        bindings::TIOCM_CAR | bindings::TIOCM_DSR | bindings::TIOCM_CTS
    }

    // start/stop callbacks enter with the UART lock already held.
    /// # Safety
    /// Serial core supplies a live registered UART with its lock held.
    unsafe extern "C" fn start_tx(ptr: *mut bindings::uart_port) {
        // SAFETY: The serial-core callback contract provides the lock and lifetime.
        unsafe { Self::locked(ptr, |data, port| data.start_tx(port)) };
    }
    /// # Safety
    /// Serial core supplies a live registered UART with its lock held.
    unsafe extern "C" fn stop_tx(ptr: *mut bindings::uart_port) {
        // SAFETY: The serial-core callback contract provides the lock and lifetime.
        unsafe { Self::locked(ptr, |data, port| data.stop_tx(port)) };
    }
    /// # Safety
    /// Serial core supplies a live registered UART with its lock held.
    unsafe extern "C" fn stop_rx(ptr: *mut bindings::uart_port) {
        // SAFETY: The serial-core callback contract provides the lock and lifetime.
        unsafe { Self::locked(ptr, |data, port| data.stop_rx(port)) };
    }

    /// # Safety
    /// The caller retains the registered port and holds its UART lock.
    unsafe fn locked<R>(
        ptr: *mut bindings::uart_port,
        f: impl FnOnce(&T, &LockedPort<'_>) -> R,
    ) -> R {
        // SAFETY: C holds the UART lock and the registered allocation is live.
        unsafe {
            PortData::<T>::with_port(ptr, |state, port| {
                f(
                    &state.data,
                    &LockedPort {
                        raw: ptr,
                        device: port.device(),
                        preserve_b0: false,
                    },
                )
            })
        }
    }

    /// # Safety
    /// Serial core retains the port and holds tty_port.mutex, not the UART lock.
    unsafe extern "C" fn startup(ptr: *mut bindings::uart_port) -> i32 {
        // SAFETY: Serial core holds tty_port.mutex, but not the UART lock.
        unsafe {
            PortData::<T>::with_port(ptr, |state, port| {
                let result = if state.published.load(Ordering::Acquire) {
                    port.with_lock(|locked| state.data.startup(locked))
                } else {
                    Err(ENODEV)
                };
                if let Err(error) = result {
                    // The C allocation precedes startup. Detach under its UART lock,
                    // then free outside the lock, matching serial core's lock order.
                    let buffer = port.with_lock(|locked| locked.detach_xmit_buffer());
                    bindings::kfree(buffer.cast());
                    error.to_errno()
                } else {
                    0
                }
            })
        }
    }

    /// # Safety
    /// Serial core retains the port and holds tty_port.mutex, not the UART lock.
    unsafe extern "C" fn shutdown(ptr: *mut bindings::uart_port) {
        // SAFETY: Serial core holds tty_port.mutex, not the UART lock.
        unsafe {
            PortData::<T>::with_port(ptr, |state, port| {
                port.with_lock(|locked| state.data.shutdown(locked));
            })
        };
    }

    /// # Safety
    /// Invoked only by the IRQ registration retaining this pinned UART cookie.
    unsafe extern "C" fn interrupt(_irq: i32, ptr: *mut crate::ffi::c_void) -> u32 {
        let raw = ptr.cast::<bindings::uart_port>();
        // SAFETY: request_irq/free_irq retain this pinned allocation. Publication
        // guards the uninitialized UART lock during uart_add_one_port.
        let state = unsafe { &*(*raw).private_data.cast::<PortData<T>>() };
        if !state.published.load(Ordering::Acquire) {
            return bindings::irqreturn_IRQ_NONE;
        }
        // SAFETY: Publication completed; free_irq retains the allocation and lock.
        let handled = unsafe {
            PortData::<T>::with_port(raw, |state, port| {
                port.with_lock(|locked| state.data.interrupt(locked))
            })
        };
        if handled {
            bindings::irqreturn_IRQ_HANDLED
        } else {
            bindings::irqreturn_IRQ_NONE
        }
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn set_termios(
        ptr: *mut bindings::uart_port,
        new: *mut bindings::ktermios,
        old: *const bindings::ktermios,
    ) {
        // SAFETY: Serial core supplies live termios pointers and holds the port
        // registration through this callback. No view or termios reference escapes.
        unsafe {
            PortData::<T>::with_port(ptr, |state, port| {
                port.with_lock(|locked| {
                    let scoped = LockedPort {
                        raw: locked.raw,
                        device: locked.device,
                        preserve_b0: bindings::tty_termios_baud_rate(new) == 0,
                    };
                    state.data.set_termios(
                        &scoped,
                        Termios::from_raw_mut(new),
                        old.as_ref().map(|termios| Termios::from_raw(termios)),
                    );
                });
            });
        }
    }

    /// # Safety
    /// Must be called by serial core with a live port under the uart_ops contract.
    unsafe extern "C" fn port_type(ptr: *mut bindings::uart_port) -> *const crate::ffi::c_char {
        // SAFETY: The registered port owns its hardware data and driver throughout
        // the callback. The returned name is an immutable static C string.
        unsafe { PortData::<T>::with_port(ptr, |state, _| state.driver.name.as_char_ptr()) }
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

struct Registration<T: SerialOps> {
    port: Pin<KBox<PortData<T>>>,
    added: bool,
    irq_registered: bool,
}

impl<T: SerialOps> Drop for Registration<T> {
    fn drop(&mut self) {
        // SAFETY: Only successful registration constructs this guard. Serial core
        // hangs up TTY users, unregisters the console, waits for console_srcu readers
        // and drains uart_state references before returning. The allocation, hardware
        // data and Driver Arc are released only after this synchronization.
        unsafe {
            self.port.driver.lines[(*self.port.inner.get()).line as usize]
                .store(false, Ordering::Release);
            if self.added {
                bindings::uart_remove_one_port(self.port.driver.inner.get(), self.port.inner.get());
            }
            self.port.published.store(false, Ordering::Release);
            if self.irq_registered {
                bindings::free_irq((*self.port.inner.get()).irq, self.port.inner.get().cast());
            }
        };
    }
}

/// A callback-scoped view of a registered port.
///
/// Instances are only created by the FFI adapters. The registration and its
/// device allocation remain live throughout the callback. Managed hardware
/// resources need their own revocation-checked access guard.
/// This view exposes the callback context independently of the hardware data type.
///
/// # Invariants
///
/// `raw` points to a registered port belonging to `device`. The FFI adapter keeps
/// the port and its device allocation live for the entire borrow.
pub struct Port<'a> {
    raw: *mut bindings::uart_port,
    device: &'a Device,
}

impl Port<'_> {
    /// The parent device. Managed resources require revocation-checked access.
    #[inline]
    pub fn device(&self) -> &Device {
        self.device
    }

    /// Runs a closure with the serial-core UART lock held and local IRQs saved.
    fn with_lock<R>(&self, f: impl FnOnce(&LockedPort<'_>) -> R) -> R {
        let guard = PortGuard::lock_irqsave(self);
        let locked = LockedPort {
            raw: self.raw,
            device: self.device(),
            preserve_b0: false,
        };
        let result = f(&locked);
        drop(guard);
        result
    }
}

struct PortGuard<'a> {
    port: &'a Port<'a>,
    flags: crate::ffi::c_ulong,
    unlock: bool,
}

impl<'a> PortGuard<'a> {
    fn lock_irqsave(port: &'a Port<'_>) -> Self {
        let mut flags = 0;
        // SAFETY: The port remains live for this borrow. The caller does not
        // already hold its UART lock.
        unsafe { bindings::uart_port_lock_irqsave(port.raw, &mut flags) };
        Self {
            port,
            flags,
            unlock: true,
        }
    }

    fn console_lock(port: &'a Port<'_>) -> Self {
        let mut flags = 0;
        // SAFETY: The port remains live for this borrow. The C helper permits
        // emergency output without the lock if its trylock fails.
        let unlock = unsafe { bindings::uart_port_console_lock(port.raw, &mut flags) };
        Self {
            port,
            flags,
            unlock,
        }
    }
}

impl Drop for PortGuard<'_> {
    fn drop(&mut self) {
        if self.unlock {
            // SAFETY: The port is live and this guard acquired its UART lock.
            unsafe { bindings::uart_port_unlock_irqrestore(self.port.raw, self.flags) };
        }
    }
}

/// A UART borrowed while its serial-core lock is held. Cannot escape the borrow.
pub struct LockedPort<'a> {
    raw: *mut bindings::uart_port,
    device: &'a Device,
    preserve_b0: bool,
}

impl LockedPort<'_> {
    /// The parent device. The UART lock does not prevent resource revocation.
    #[inline]
    pub fn device(&self) -> &Device {
        self.device
    }
}

impl LockedPort<'_> {
    /// Returns and consumes the next software TX byte, including XON/XOFF.
    pub fn next_tx(&self) -> Option<u8> {
        // SAFETY: The UART lock serializes the C xmit FIFO, x_char and counters.
        unsafe {
            let port = &mut *self.raw;
            if port.x_char != 0 {
                let ch = port.x_char;
                port.x_char = 0;
                port.icount.tx += 1;
                return Some(ch);
            }
            if port.state.is_null() {
                return None;
            }
            let tty_port = &mut (*port.state).port;
            if tty_port.xmit_buf.is_null() {
                return None;
            }
            let tty = tty_port.tty;
            if port.hw_stopped
                || (!tty.is_null() && ((*tty).flow.stopped || (*tty).flow.tco_stopped))
            {
                return None;
            }
            let fifo = &mut tty_port.xmit_fifo.__bindgen_anon_1.kfifo;
            let mut ch = 0u8;
            if bindings::__kfifo_out(fifo, core::ptr::from_mut(&mut ch).cast(), 1) == 0 {
                return None;
            }
            port.icount.tx += 1;
            if fifo.in_.wrapping_sub(fifo.out) < bindings::WAKEUP_CHARS {
                bindings::uart_write_wakeup(self.raw);
            }
            Some(ch)
        }
    }

    /// Pushes a received byte and its status into serial core's TTY buffer.
    pub fn receive(&self, ch: u8, parity: bool, frame: bool, overrun: bool) {
        // SAFETY: The registered UART and tty_port remain live, and the lock
        // serializes counters and termios masks with serial core.
        unsafe {
            (*self.raw).icount.rx += 1;
            if parity {
                (*self.raw).icount.parity += 1;
            }
            if frame {
                (*self.raw).icount.frame += 1;
            }
            if overrun {
                (*self.raw).icount.overrun += 1;
            }
            let status = u32::from(parity) | (u32::from(frame) << 1) | (u32::from(overrun) << 2);
            let checked = status & (*self.raw).read_status_mask;
            let flag = if checked & 1 != 0 {
                bindings::TTY_PARITY
            } else if checked & 2 != 0 {
                bindings::TTY_FRAME
            } else {
                bindings::TTY_NORMAL
            };
            bindings::uart_insert_char(self.raw, status | 0x100, 4, ch, flag as u8);
        }
    }

    /// Makes the received batch available to the line discipline.
    pub fn push_rx(&self) {
        // SAFETY: tty flip-buffer publication is IRQ-safe; the UART state is live.
        unsafe { bindings::tty_flip_buffer_push(&mut (*(*self.raw).state).port) };
    }

    fn detach_xmit_buffer(&self) -> *mut u8 {
        // SAFETY: Startup failed under tty_port.mutex. The UART lock also excludes
        // writes by a privileged FD left open after an earlier failed startup.
        unsafe {
            let port = &mut (*(*self.raw).state).port;
            let buffer = core::mem::replace(&mut port.xmit_buf, core::ptr::null_mut());
            port.xmit_fifo.__bindgen_anon_1.kfifo = bindings::__kfifo {
                esize: 1,
                ..Default::default()
            };
            buffer
        }
    }
}
