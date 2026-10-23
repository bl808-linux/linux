// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Kwanghoon Son <asdfkwang@gmail.com>
// Based on bflb_uart.c, Copyright (C) 2022 Jisheng Zhang <jszhang@kernel.org>.

//! IRQ-driven Bouffalo Lab UART, initially validated on BL808 UART3.

use kernel::{
    bindings,
    clk::Clk,
    device::{Bound, Core, Device},
    driver,
    io::{mem::ExclusiveIoMem, Io, IoBase},
    irq, of, platform,
    prelude::*,
    serial::{self, LockedPort, Port, Termios},
    sync::Arc,
};

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
            serial::Driver::new(c"bflb_uart", c"ttyS", 4, module),
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
    _baud: EnabledClock,
    _apb: EnabledClock,
    _reset: kernel::reset::OptionalReset,
}

impl Hardware<'_> {
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
        let mut parity = status & RX_PARITY != 0;
        if parity {
            port.parity_error();
        }
        if fifo & RX_OVERFLOW != 0 {
            port.overrun(port.ignore_mask() & RX_ERROR == 0);
        }
        // Bound the hard-IRQ work; a still-nonempty FIFO keeps the level interrupt asserted.
        let mut budget = 256;
        let mut bytes = [0u8; 32];
        while budget != 0 {
            let count = ((self.read::<FIFO_CONFIG1>() >> 8) & 0x3f)
                .min(32)
                .min(budget) as usize;
            if count == 0 {
                break;
            }
            for byte in &mut bytes[..count] {
                *byte = self.read::<FIFO_RDATA>() as u8;
            }
            budget -= count as u32;
            if parity {
                // The hardware latches parity for the FIFO, without per-byte error tags.
                // Associate it with the first recovered byte rather than inventing tags.
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
                port.receive(&bytes[1..count], bindings::TTY_NORMAL as u8);
                parity = false;
            } else {
                port.receive(&bytes[..count], bindings::TTY_NORMAL as u8);
            }
        }
        if fifo & (RX_OVERFLOW | RX_UNDERFLOW) != 0 {
            // Salvage queued bytes before clearing FIFO error latches. Never reset TX here.
            self.write::<FIFO_CONFIG0>((fifo & !(1 << 2)) | RX_CLEAR);
        }
        port.push();
    }
}

impl serial::Operations for Hardware<'_> {
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

    fn stop_rx(&self, _port: &LockedPort<'_>) {
        self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() & !1);
        self.write::<INT_MASK>(self.read::<INT_MASK>() | RX_INTS);
    }

    fn quiesce(&self, port: &Port<'_, Self>) {
        port.with_lock(|_| {
            self.write::<INT_MASK>(u32::MAX);
            self.write::<INT_EN>(0);
        });
    }

    fn startup(&self, port: &Port<'_, Self>) {
        port.with_lock(|_| {
            self.write::<FIFO_CONFIG0>(RX_CLEAR); // Disable firmware DMA; preserve TX FIFO.
            self.write::<INT_CLEAR>(W1C);
            self.write::<DATA_CONFIG>(0);
            self.write::<SW_MODE>(0);
            self.write::<RTO_TIMER>(0x4f);
            let fifo = self.read::<FIFO_CONFIG1>() & !((0x1f << 24) | (0x1f << 16));
            self.write::<FIFO_CONFIG1>(fifo | (7 << 24) | (15 << 16));
            self.write::<INT_EN>(RX_INTS | TX_FIFO);
            self.write::<INT_MASK>(!RX_INTS);
            self.write::<UTX_CONFIG>(self.read::<UTX_CONFIG>() | 1);
            self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() | 1);
        });
    }

    fn shutdown(&self, port: &Port<'_, Self>) {
        port.with_lock(|_| {
            self.write::<INT_MASK>(u32::MAX);
            self.write::<INT_EN>(0);
            self.write::<URX_CONFIG>(self.read::<URX_CONFIG>() & !1);
        });
    }

    fn set_termios(&self, port: &Port<'_, Self>, new: &mut Termios, old: Option<&Termios>) {
        port.with_lock(|locked| {
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
            self.receive(port, status);
        }
        if status & TX_FIFO != 0 {
            port.transmit(self.read::<FIFO_CONFIG1>() & 0x3f, |ch| {
                self.io.as_view().relaxed().write8(ch, FIFO_WDATA);
            });
        }
        irq::IrqReturn::Handled
    }
}
