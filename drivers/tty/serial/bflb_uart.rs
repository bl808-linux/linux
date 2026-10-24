// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Kwanghoon Son <asdfkwang@gmail.com>
// Based on bflb_uart.c, Copyright (C) 2022 Jisheng Zhang <jszhang@kernel.org>.

//! IRQ-driven Bouffalo Lab UART, initially targeting BL808 UART3.

use kernel::{
    bindings,
    clk::Clk,
    device::{Bound, Core, Device},
    driver,
    io::{mem::ExclusiveIoMem, Io, IoBackend, IoBase, MmioBackend},
    irq, of, platform,
    prelude::*,
    serial::{self, LockedPort, Port, Termios},
    sync::Arc,
};

#[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
use core::sync::atomic::{AtomicBool, AtomicUsize};
use core::sync::atomic::{AtomicU32, Ordering};
#[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
use kernel::dma::engine::CyclicRx;

const UTX_CONFIG: usize = 0x00;
const URX_CONFIG: usize = 0x04;
const BIT_PRD: usize = 0x08;
const DATA_CONFIG: usize = 0x0c;
const RTO_TIMER: usize = 0x18;
const SW_MODE: usize = 0x1c;
const INT_STS: usize = 0x20;
const INT_MASK: usize = 0x24;
const INT_CLEAR: usize = 0x28;
const INT_EN: usize = 0x2c;
const STATUS: usize = 0x30;
const FIFO_CONFIG0: usize = 0x80;
const FIFO_CONFIG1: usize = 0x84;
const FIFO_WDATA: usize = 0x88;
const FIFO_RDATA: usize = 0x8c;

const TX_FIFO: u32 = 1 << 2;
const RX_FIFO: u32 = 1 << 3;
const RX_TIMEOUT: u32 = 1 << 4;
const RX_PARITY: u32 = 1 << 5;
const RX_ERROR: u32 = 1 << 7;
const RX_INTS: u32 = RX_FIFO | RX_TIMEOUT | RX_PARITY | RX_ERROR;
// FIFO-ready/error sources are levels; only the documented latched events are W1C.
const W1C: u32 = (1 << 0) | (1 << 1) | RX_TIMEOUT | RX_PARITY | (1 << 8);
const RX_OVERFLOW: u32 = 1 << 6;
const RX_UNDERFLOW: u32 = 1 << 7;
const RX_CLEAR: u32 = 1 << 3;
const RX_DMA: u32 = 1 << 1;

kernel::sync::global_lock! {
    // SAFETY: Initialized once in module init, before registering the platform driver.
    unsafe(uninit) static SERIAL: Mutex<Option<Arc<serial::Driver>>> = None;
}

struct SerialOwner;

impl SerialOwner {
    fn new(module: &'static ThisModule) -> Result<Self> {
        // SAFETY: Module init runs once per load, before any access to SERIAL.
        unsafe { SERIAL.init() };
        *SERIAL.lock() = Some(Arc::pin_init(
            serial::Driver::new(
                c"bflb_uart",
                c"ttyS",
                4,
                cfg!(CONFIG_SERIAL_BFLB_RUST_CONSOLE)
                    .then_some(serial::ConsoleConfig { baud: 2_000_000 }),
                module,
            ),
            GFP_KERNEL,
        )?);
        Ok(Self)
    }
}

impl Drop for SerialOwner {
    fn drop(&mut self) {
        SERIAL.lock().take();
    }
}

#[pin_data]
struct UartModule {
    // Drop the bus registration (and all ports) before the serial driver owner.
    #[pin]
    _platform: driver::Registration<platform::Adapter<Uart>>,
    _serial: SerialOwner,
}

impl kernel::InPlaceModule for UartModule {
    fn init(module: &'static ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            _serial: SerialOwner::new(module)?,
            _platform <- driver::Registration::new(c"bflb_uart", module),
        })
    }
}

module! {
    type: UartModule,
    name: "bflb_uart",
    authors: ["Kwanghoon Son"],
    description: "Bouffalo Lab UART driver",
    license: "GPL",
}

struct Uart;

kernel::of_device_table!(OF_TABLE, (), [(of::DeviceId::new(c"bflb,uart"), ())]);

impl platform::Driver for Uart {
    type IdInfo = ();
    type Data<'bound> = Port<'bound, Hardware<'bound>>;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound ()>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        pin_init::pin_init_scope(move || {
            let dev: &Device<Bound> = pdev.as_ref();
            let line = dev.of_alias_id(c"serial")?;
            if line >= 4 {
                return Err(EINVAL);
            }
            let irq = pdev.irq_by_index(0)?.irq();
            let mapbase = pdev.resource_by_index(0).ok_or(ENODEV)?.start();
            let io = pdev
                .io_request_by_index(0)
                .ok_or(ENODEV)?
                .iomap_exclusive_sized::<0x90>()?;
            let apb = EnabledClock::get(dev, c"apb")?;
            let baud = EnabledClock::get(dev, c"baud")?;
            let clock = u32::try_from(baud.0.rate().as_hz())?;
            if clock == 0 {
                return Err(EINVAL);
            }
            let reset = kernel::reset::OptionalReset::get_exclusive(dev)?;
            // Do not pulse reset: firmware may be using the UART for early output.
            reset.deassert()?;
            #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
            // SAFETY: The DMA2 provider has an MMIO-reset termination fallback,
            // and the managed consumer link keeps it bound until port teardown.
            let rx_dma = unsafe { CyclicRx::request_optional(dev, c"rx", 16384, 512) }?;
            #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
            if rx_dma.is_some() {
                pr_info!("UART3 RX DMA: 16384-byte cyclic buffer, 512-byte periods\n");
            }
            let serial = SERIAL.lock().as_ref().ok_or(ENODEV)?.clone();
            // SAFETY: irq was obtained from pdev. Platform core owns the resulting pinned
            // port and drops it before releasing the device's bound lifetime and resources.
            Ok(unsafe {
                Port::new(
                    serial,
                    dev,
                    irq,
                    serial::PortConfig {
                        line,
                        mapbase,
                        clock,
                        fifo_size: 32,
                    },
                    Ok(Hardware {
                        io,
                        clock,
                        rx_pending: AtomicU32::new(0),
                        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
                        rx_dma,
                        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
                        dma_source: mapbase + FIFO_RDATA as u64,
                        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
                        dma_read: AtomicUsize::new(0),
                        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
                        dma_failed: AtomicBool::new(false),
                        _baud: baud,
                        _apb: apb,
                        _reset: reset,
                    }),
                )
            })
        })
    }
}

struct EnabledClock(Clk);

impl EnabledClock {
    fn get(dev: &Device, name: &CStr) -> Result<Self> {
        let clock = Clk::get(dev, Some(name))?;
        clock.prepare_enable()?;
        Ok(Self(clock))
    }
}

impl Drop for EnabledClock {
    fn drop(&mut self) {
        self.0.disable_unprepare();
    }
}

struct Hardware<'bound> {
    io: ExclusiveIoMem<'bound, 0x90>,
    clock: u32,
    rx_pending: AtomicU32,
    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    rx_dma: Option<CyclicRx<'bound>>,
    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    dma_source: bindings::dma_addr_t,
    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    dma_read: AtomicUsize,
    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    dma_failed: AtomicBool,
    _baud: EnabledClock,
    _apb: EnabledClock,
    _reset: kernel::reset::OptionalReset,
}

impl Hardware<'_> {
    fn dma_enabled(&self) -> bool {
        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
        {
            self.rx_dma.is_some()
        }
        #[cfg(not(CONFIG_SERIAL_BFLB_RUST_DMA))]
        {
            false
        }
    }

    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    unsafe extern "C" fn dma_complete(context: *mut kernel::ffi::c_void) {
        // SAFETY: startup publishes a pinned Port; shutdown joins callbacks
        // outside its lock before serial core or platform core may release it.
        let port = unsafe { &*context.cast::<Port<'_, Hardware<'_>>>() };
        port.with_lock(|locked| port.data().receive_dma(locked));
    }

    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    fn fail_dma(&self) {
        self.dma_failed.store(true, Ordering::Relaxed);
        self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() & !1);
        self.write::<FIFO_CONFIG0>(self.read::<FIFO_CONFIG0>() & !RX_DMA);
        self.write::<INT_MASK>(self.read::<INT_MASK>() | RX_INTS);
    }

    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
    fn receive_dma(&self, port: &LockedPort<'_>) {
        let Some(dma) = self.rx_dma.as_ref() else {
            return;
        };
        if self.dma_failed.load(Ordering::Relaxed) {
            return;
        }
        let position = match dma.position() {
            Ok(position) => position,
            Err(_) => {
                self.fail_dma();
                port.buffer_overrun(1);
                return;
            }
        };
        let mut offset = self.dma_read.load(Ordering::Relaxed);
        let mut remaining = (position + dma.len() - offset) % dma.len();
        let mut bytes = [0u8; 256];
        let received = remaining;
        while remaining != 0 {
            let count = remaining.min(bytes.len()).min(dma.len() - offset);
            // SAFETY: DMA progress published this contiguous range. Copying in
            // bounded chunks under the port lock completes well before a ring
            // wrap; DMA2 also detects missed complete rings at its period IRQ.
            unsafe { dma.read_completed(offset, &mut bytes[..count]) };
            port.receive(&bytes[..count], bindings::TTY_NORMAL as u8);
            offset = (offset + count) % dma.len();
            remaining -= count;
        }
        self.dma_read.store(offset, Ordering::Relaxed);
        if received != 0 {
            port.push();
        }
    }

    #[cfg(CONFIG_SERIAL_BFLB_RUST_CONSOLE)]
    fn console_poll(&self, ready: impl Fn() -> bool) -> bool {
        // Bound every wait, including a failed/stopped peripheral during a panic.
        for _ in 0..100_000 {
            if ready() {
                return true;
            }
            // SAFETY: cpu_relax is valid in atomic and panic contexts.
            unsafe { bindings::cpu_relax() };
        }
        false
    }

    #[cfg(CONFIG_SERIAL_BFLB_RUST_CONSOLE)]
    fn console_putchar(&self, ch: u8) -> bool {
        if !self.console_poll(|| self.read::<FIFO_CONFIG1>() & 0x3f != 0) {
            return false;
        }
        self.io.as_view().relaxed().write8(ch, FIFO_WDATA);
        true
    }

    fn read<const REG: usize>(&self) -> u32 {
        self.io.as_view().relaxed().read32(REG)
    }
    fn write<const REG: usize>(&self, value: u32) {
        self.io.as_view().relaxed().write32(value, REG);
    }
    fn mask_tx(&self) {
        self.write::<INT_MASK>(self.read::<INT_MASK>() | TX_FIFO);
    }

    fn receive(&self, port: &LockedPort<'_>, status: u32) {
        let fifo = self.read::<FIFO_CONFIG0>();
        let parity = status & RX_PARITY != 0;
        if parity {
            port.parity_error();
        }
        if fifo & RX_OVERFLOW != 0 {
            port.overrun(port.ignore_mask() & RX_ERROR == 0);
        }
        // Drain first, then hand one batch to TTY. Calling into TTY for each
        // newly arrived byte can keep this hard IRQ running across an entire stream.
        let mut bytes = [0u8; 256];
        let mut used = 0;
        while used < bytes.len() {
            let count = ((self.read::<FIFO_CONFIG1>() >> 8) & 0x3f)
                .min(32)
                .min((bytes.len() - used) as u32) as usize;
            if count == 0 {
                break;
            }
            let address = MmioBackend::as_ptr(self.io.as_view())
                .cast::<u8>()
                .wrapping_add(FIFO_RDATA);
            // SAFETY: The FIFO register lies within the exclusive mapping. The
            // destination has count writable bytes and repeated I/O holds the
            // device address fixed. The SDK uses byte-wide FIFO data accesses.
            unsafe { bindings::io_read8_fifo(address.cast(), bytes[used..].as_mut_ptr(), count) };
            used += count;
        }
        if used != 0 {
            if parity {
                let flag = if port.read_mask() & RX_PARITY != 0 {
                    bindings::TTY_PARITY as u8
                } else {
                    bindings::TTY_NORMAL as u8
                };
                if port.ignore_mask() & RX_PARITY != 0 {
                    port.discard(1);
                } else {
                    port.receive(&bytes[..1], flag);
                }
                port.receive(&bytes[1..used], bindings::TTY_NORMAL as u8);
            } else {
                port.receive(&bytes[..used], bindings::TTY_NORMAL as u8);
            }
        }
        if fifo & (RX_OVERFLOW | RX_UNDERFLOW) != 0 {
            // Salvage queued bytes before clearing FIFO error latches. Never reset TX here.
            self.write::<FIFO_CONFIG0>((fifo & !(1 << 2)) | RX_CLEAR);
        }
        // Coalesce TTY work during continuous RX. Small packets are delivered
        // by RX timeout, so a short read cannot wait for the batch threshold.
        let pending = self.rx_pending.load(Ordering::Relaxed) + used as u32;
        if pending >= 256 || status & (RX_TIMEOUT | RX_PARITY | RX_ERROR) != 0 {
            self.rx_pending.store(0, Ordering::Relaxed);
            port.push();
        } else {
            self.rx_pending.store(pending, Ordering::Relaxed);
        }
    }
}

impl serial::Operations for Hardware<'_> {
    const HAS_CONSOLE: bool = cfg!(CONFIG_SERIAL_BFLB_RUST_CONSOLE);

    #[cfg(CONFIG_SERIAL_BFLB_RUST_CONSOLE)]
    fn console_write(&self, port: &Port<'_, Self>, bytes: &[u8]) {
        // Serialize the whole record against IRQ-driven TTY output. During an oops
        // the adapter uses trylock so emergency output cannot wait on a held lock.
        port.with_console_lock(|| {
            let mask = self.read::<INT_MASK>();
            self.write::<INT_MASK>(u32::MAX);
            self.write::<UTX_CONFIG>(self.read::<UTX_CONFIG>() | 1);
            let mut complete = true;
            for &ch in bytes {
                if ch == b'\n' && !self.console_putchar(b'\r') {
                    complete = false;
                    break;
                }
                if !self.console_putchar(ch) {
                    complete = false;
                    break;
                }
            }
            if complete {
                self.console_poll(|| {
                    self.read::<FIFO_CONFIG1>() & 0x3f == 32 && self.read::<STATUS>() & 1 == 0
                });
            }
            self.write::<INT_MASK>(mask);
        });
    }

    fn tx_empty(&self) -> bool {
        self.read::<FIFO_CONFIG1>() & 0x3f == 32 && self.read::<STATUS>() & 1 == 0
    }

    fn set_mctrl(&self, _port: &LockedPort<'_>, _value: u32) {
        // Required by serial core; initial two-wire support has no RTS/DTR outputs.
    }

    fn get_mctrl(&self, _port: &LockedPort<'_>) -> u32 {
        // No modem inputs in two-wire operation: serial core requires these asserted.
        bindings::TIOCM_CAR | bindings::TIOCM_DSR | bindings::TIOCM_CTS
    }

    fn start_tx(&self, _port: &LockedPort<'_>) {
        self.write::<UTX_CONFIG>(self.read::<UTX_CONFIG>() | 1);
        self.write::<INT_MASK>(self.read::<INT_MASK>() & !TX_FIFO);
    }

    fn stop_tx(&self, _port: &LockedPort<'_>) {
        self.mask_tx();
    }

    fn stop_rx(&self, port: &LockedPort<'_>) {
        self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() & !1);
        self.write::<INT_MASK>(self.read::<INT_MASK>() | RX_INTS);
        if self.rx_pending.swap(0, Ordering::Relaxed) != 0 {
            port.push();
        }
    }

    fn quiesce(&self, port: &Port<'_, Self>) {
        port.with_lock(|_| {
            self.write::<INT_MASK>(u32::MAX);
            self.write::<INT_EN>(0);
        });
    }

    fn startup(&self, port: &Port<'_, Self>) -> Result {
        port.with_lock(|_| {
            self.rx_pending.store(0, Ordering::Relaxed);
            self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() & !1);
            self.write::<FIFO_CONFIG0>(RX_CLEAR); // Preserve TX; disable firmware DMA.
            self.write::<INT_CLEAR>(W1C);
            self.write::<DATA_CONFIG>(0);
            self.write::<SW_MODE>(0);
            self.write::<RTO_TIMER>(0x4f);
            let fifo = self.read::<FIFO_CONFIG1>() & !((0x1f << 24) | (0x1f << 16));
            let threshold = if self.dma_enabled() { 0 } else { 7 };
            self.write::<FIFO_CONFIG1>(fifo | (threshold << 24) | (15 << 16));
        });
        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
        if let Some(dma) = self.rx_dma.as_ref() {
            self.dma_read.store(0, Ordering::Relaxed);
            self.dma_failed.store(false, Ordering::Relaxed);
            // The DMA source is the physical UART FIFO, not the CPU's I/O VA.
            // SAFETY: Port is pinned, callbacks stop in shutdown, and UART
            // requests are disabled until the final initialization block.
            unsafe {
                dma.start(
                    self.dma_source,
                    Self::dma_complete,
                    (port as *const Port<'_, Self>).cast_mut().cast(),
                )?;
            }
        }
        port.with_lock(|_| {
            let receive = if self.dma_enabled() {
                RX_INTS & !RX_FIFO
            } else {
                RX_INTS
            };
            if self.dma_enabled() {
                self.write::<FIFO_CONFIG0>(RX_DMA);
            }
            self.write::<INT_EN>(RX_INTS | TX_FIFO);
            self.write::<INT_MASK>(!receive);
            self.write::<UTX_CONFIG>(self.read::<UTX_CONFIG>() | 1);
            self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() | 1);
        });
        Ok(())
    }

    fn shutdown(&self, port: &Port<'_, Self>) {
        port.with_lock(|_| {
            self.write::<INT_MASK>(u32::MAX);
            self.write::<INT_EN>(0);
            self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() & !1);
            self.write::<FIFO_CONFIG0>(self.read::<FIFO_CONFIG0>() & !RX_DMA);
        });
        #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
        if let Some(dma) = self.rx_dma.as_ref() {
            // SAFETY: Serial core serializes startup/shutdown. The callback's
            // UART lock is released before synchronizing the provider tasklet.
            assert!(unsafe { dma.stop() }.is_ok(), "RX DMA termination failed");
        }
    }

    fn set_termios(&self, port: &Port<'_, Self>, new: &mut Termios, old: Option<&Termios>) {
        port.with_lock(|locked| {
            if self.rx_pending.swap(0, Ordering::Relaxed) != 0 {
                locked.push();
            }
            let cflag = new.cflag() & !(bindings::CMSPAR | bindings::CRTSCTS);
            new.set_cflag(cflag);
            let mut tx = (new.char_size() - 1) << 8;
            if cflag & bindings::PARENB != 0 {
                tx |= 1 << 4;
                if cflag & bindings::PARODD != 0 {
                    tx |= 1 << 5;
                }
            }
            let mut rx = tx;
            tx |= if cflag & bindings::CSTOPB != 0 {
                3 << 11
            } else {
                1 << 11
            };
            tx |= 1 | (1 << 2); // Enable TX free-running mode.
            if cflag & bindings::CREAD != 0 {
                rx |= 1;
            }
            let baud = locked.baud_rate(
                new,
                old,
                self.clock.div_ceil(65536),
                self.clock.min(4000000),
            );
            let divisor = ((u64::from(self.clock) + u64::from(baud) / 2) / u64::from(baud))
                .clamp(1, 65536) as u32;
            self.write::<UTX_CONFIG>(tx);
            self.write::<URX_CONFIG>(rx);
            self.write::<BIT_PRD>((divisor - 1) | ((divisor - 1) << 16));
            let mut read_mask = RX_ERROR;
            let mut ignore_mask = 0;
            if new.iflag() & bindings::INPCK != 0 {
                read_mask |= RX_PARITY;
            }
            if new.iflag() & bindings::IGNPAR != 0 {
                ignore_mask |= RX_PARITY;
            }
            if new.iflag() & (bindings::IGNBRK | bindings::IGNPAR)
                == bindings::IGNBRK | bindings::IGNPAR
            {
                ignore_mask |= RX_ERROR;
            }
            locked.configure(new, self.clock / divisor, read_mask, ignore_mask);
        });
    }

    fn handle_irq(&self, port: &LockedPort<'_>) -> irq::IrqReturn {
        let status = self.read::<INT_STS>() & self.read::<INT_EN>() & !self.read::<INT_MASK>();
        if status == 0 {
            return irq::IrqReturn::None;
        }
        self.write::<INT_CLEAR>(status & W1C);
        if status & RX_INTS != 0 {
            if self.dma_enabled() {
                if status & RX_PARITY != 0 {
                    port.parity_error();
                }
                if self.read::<FIFO_CONFIG0>() & RX_OVERFLOW != 0 {
                    port.overrun(port.ignore_mask() & RX_ERROR == 0);
                    // An overflow already lost bytes. Quiesce RX, including its
                    // level error source, until a fresh open resets the FIFO.
                    #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
                    self.fail_dma();
                }
                #[cfg(CONFIG_SERIAL_BFLB_RUST_DMA)]
                self.receive_dma(port);
            } else {
                self.receive(port, status);
            }
        }
        if status & TX_FIFO != 0 {
            let count = (self.read::<FIFO_CONFIG1>() & 0x3f).min(32) as usize;
            let mut bytes = [0u8; 32];
            let fifo = MmioBackend::as_ptr(self.io.as_view())
                .cast::<u8>()
                .wrapping_add(FIFO_WDATA);
            port.transmit(&mut bytes[..count], |bytes| {
                // SAFETY: The byte-wide FIFO register lies in the exclusive MMIO region.
                // The batch is at most the free FIFO capacity and bytes is readable.
                unsafe { bindings::io_write8_fifo(fifo.cast(), bytes.as_ptr(), bytes.len()) };
            });
        }
        irq::IrqReturn::Handled
    }
}
