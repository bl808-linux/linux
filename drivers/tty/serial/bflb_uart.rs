// SPDX-License-Identifier: GPL-2.0-or-later

//! Bouffalo Lab UART3 polling console.

use core::num::NonZeroU32;

use kernel::{
    clk::Clk,
    device::{
        Bound,
        Core,
        Device, //
    },
    devres::DevresGuard,
    driver,
    io::{
        mem::{
            DevresExclusiveIoMem,
            ExclusiveIoMem, //
        },
        poll::read_poll_timeout_atomic,
        Io,
        IoBase, //
    },
    of,
    platform,
    prelude::*,
    serial::{
        self,
        DriverConfig,
        LockedPort,
        Port,
        Termios, //
    },
    sync::Arc,
    time::Delta, //
};

const REG_SIZE: usize = 0x90;
const FIFO_DEPTH: u32 = 32;
const PORTS: u32 = 1;
const DEFAULT_BAUD: u32 = 115200;
const POLL_RETRIES: usize = 100_000;
const MAX_BIT_PERIOD: u32 = 65536;
const MAX_BAUD: u32 = 4000000;

const UTX_CONFIG: usize = 0x00;
const URX_CONFIG: usize = 0x04;
const BIT_PRD: usize = 0x08;
const DATA_CONFIG: usize = 0x0c;
const SW_MODE: usize = 0x1c;
const INT_MASK: usize = 0x24;
const INT_EN: usize = 0x2c;
const STATUS: usize = 0x30;
const FIFO_CONFIG0: usize = 0x80;
const FIFO_CONFIG1: usize = 0x84;
const FIFO_WDATA: usize = 0x88;

// BL808 UART register fields, named after the reference manual.
const UTX_EN: u32 = 1 << 0;
const UTX_FRM_EN: u32 = 1 << 2;
const UTX_PRT_EN: u32 = 1 << 4;
const UTX_PRT_SEL: u32 = 1 << 5;
const UTX_BIT_CNT_D_SHIFT: u32 = 8;
const UTX_BIT_CNT_P_SHIFT: u32 = 11;
const UTX_STOP_ONE: u32 = 1 << UTX_BIT_CNT_P_SHIFT;
const UTX_STOP_TWO: u32 = 3 << UTX_BIT_CNT_P_SHIFT;
const UTX_BUS_BUSY: u32 = 1 << 0;
const TX_FIFO_CNT_MASK: u32 = 0x3f;
const URX_BIT_PRD_SHIFT: u32 = 16;

kernel::sync::global_lock! {
    // SAFETY: Initialized once in module init, before registering the platform driver.
    unsafe(uninit) static SERIAL: Mutex<Option<Arc<serial::Driver<Hardware>>>> = None;
}

struct SerialOwner;

impl SerialOwner {
    fn new(module: &'static ThisModule) -> Result<Self> {
        // SAFETY: Module init runs once per load, before any access to SERIAL.
        unsafe { SERIAL.init() };
        *SERIAL.lock() = Some(Arc::pin_init(
            serial::Driver::new(
                DriverConfig {
                    name: c"bflb_uart",
                    device_name: c"ttyBFLB",
                    ports: PORTS,
                    default_baud: DEFAULT_BAUD,
                },
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
    // Fields drop in declaration order: remove platform devices and drain their
    // serial callbacks before releasing the module's serial-driver reference.
    #[pin]
    _platform: driver::Registration<platform::Adapter<Uart>>,
    _serial: SerialOwner,
}

impl kernel::InPlaceModule for UartModule {
    fn init(module: &'static ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            // Publish the serial driver before platform registration can probe.
            _serial: SerialOwner::new(module)?,
            _platform <- driver::Registration::new(c"bflb_uart", module),
        })
    }
}

module! {
    type: UartModule,
    name: "bflb_uart",
    authors: ["Kwanghoon Son"],
    description: "Bouffalo Lab polling serial console",
    license: "GPL",
}

struct Uart;

kernel::of_device_table!(OF_TABLE, (), [(of::DeviceId::new(c"bflb,uart"), ())]);

impl Uart {
    fn register(pdev: &platform::Device<Core<'_>>) -> Result {
        let dev: &Device<Bound> = pdev.as_ref();
        let mapbase = pdev.resource_by_index(0).ok_or(ENODEV)?.start();
        let io = pdev
            .io_request_by_index(0)
            .ok_or(ENODEV)?
            .iomap_exclusive_sized::<REG_SIZE>()?
            .into_devres()?;
        let baud = EnabledClock::get(dev, c"baud")?;
        let clock = u32::try_from(baud.0.rate().as_hz())?;
        if clock == 0 {
            return Err(EINVAL);
        }
        let serial = SERIAL.lock().as_ref().ok_or(ENODEV)?.clone();
        serial::register_port(
            serial,
            dev,
            serial::PortConfig {
                line: 0,
                mapbase,
                clock,
                fifo_size: FIFO_DEPTH,
            },
            Ok(Hardware {
                io,
                clock,
                _baud: baud,
            }),
        )?;
        Ok(())
    }
}

impl platform::Driver for Uart {
    type IdInfo = ();
    type Data<'bound> = ();
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound ()>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        Self::register(pdev)
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

struct Hardware {
    io: DevresExclusiveIoMem<REG_SIZE>,
    clock: u32,
    _baud: EnabledClock,
}

impl Hardware {
    fn registers(&self, dev: &Device) -> Option<Registers<'_>> {
        if !core::ptr::eq(self.io.device(), dev) {
            return None;
        }
        Some(Registers {
            io: self.io.try_access()?,
        })
    }
}

/// Keeps the mapping alive for the entire non-sleeping register operation.
/// Unbind can revoke other resources before the serial port itself is removed.
struct Registers<'a> {
    io: DevresGuard<'a, ExclusiveIoMem<'static, REG_SIZE>>,
}

impl Registers<'_> {
    fn read<const REG: usize>(&self) -> u32 {
        (&*self.io).as_view().read32(REG)
    }

    fn write<const REG: usize>(&self, value: u32) {
        (&*self.io).as_view().write32(value, REG);
    }

    fn wait_for(&self, ready: impl Fn() -> bool) -> Result {
        read_poll_timeout_atomic(|| Ok(ready()), |ready| *ready, Delta::ZERO, POLL_RETRIES)?;
        Ok(())
    }

    fn write_byte(&self, ch: u8) -> Result {
        self.wait_for(|| self.read::<FIFO_CONFIG1>() & TX_FIFO_CNT_MASK != 0)?;
        (&*self.io).as_view().write8(ch, FIFO_WDATA);
        Ok(())
    }

    fn tx_empty(&self) -> bool {
        self.read::<FIFO_CONFIG1>() & TX_FIFO_CNT_MASK == FIFO_DEPTH
            && self.read::<STATUS>() & UTX_BUS_BUSY == 0
    }

    fn write_record(&self, bytes: &[u8]) -> Result {
        for &ch in bytes {
            if ch == b'\n' {
                self.write_byte(b'\r')?;
            }
            self.write_byte(ch)?;
        }
        self.wait_for(|| self.tx_empty())
    }

    fn mask_interrupts(&self) -> u32 {
        let saved = self.read::<INT_MASK>();
        self.write::<INT_MASK>(u32::MAX);
        saved
    }

    fn enable_tx(&self) {
        self.write::<UTX_CONFIG>(self.read::<UTX_CONFIG>() | UTX_EN);
    }

    fn console_write(&self, bytes: &[u8]) {
        let saved_mask = self.mask_interrupts();
        self.enable_tx();
        let _ = self.write_record(bytes);
        self.write::<INT_MASK>(saved_mask);
    }
}

impl serial::SerialOps for Hardware {
    fn tx_empty(&self, port: &Port<'_>) -> bool {
        let Some(registers) = self.registers(port.device()) else {
            return false;
        };
        registers.tx_empty()
    }

    fn set_termios(&self, port: &LockedPort<'_>, new: &mut Termios, old: Option<&Termios>) {
        let Some(registers) = self.registers(port.device()) else {
            return;
        };
        new.disable_mark_space_parity();
        new.disable_hardware_flow_control();
        let mut tx = (new.char_size() - 1) << UTX_BIT_CNT_D_SHIFT;
        if new.parity_enabled() {
            tx |= UTX_PRT_EN;
            if new.odd_parity() {
                tx |= UTX_PRT_SEL;
            }
        }
        tx |= if new.two_stop_bits() {
            UTX_STOP_TWO
        } else {
            UTX_STOP_ONE
        };
        tx |= UTX_EN | UTX_FRM_EN;
        let Some(baud) = port.baud_rate(
            new,
            old,
            self.clock.div_ceil(MAX_BIT_PERIOD),
            self.clock.min(MAX_BAUD),
        ) else {
            return;
        };
        let baud = baud.get();
        let divisor = ((u64::from(self.clock) + u64::from(baud) / 2) / u64::from(baud))
            .clamp(1, u64::from(MAX_BIT_PERIOD)) as u32;
        let Some(actual_baud) = NonZeroU32::new(self.clock / divisor) else {
            return;
        };
        let _ = registers.wait_for(|| registers.tx_empty());
        registers.write::<INT_MASK>(u32::MAX);
        registers.write::<INT_EN>(0);
        registers.write::<FIFO_CONFIG0>(0);
        registers.write::<URX_CONFIG>(0);
        registers.write::<DATA_CONFIG>(0);
        registers.write::<SW_MODE>(0);
        registers.write::<BIT_PRD>((divisor - 1) | ((divisor - 1) << URX_BIT_PRD_SHIFT));
        registers.write::<UTX_CONFIG>(tx);
        port.configure(new, actual_baud);
    }

    fn console_write(&self, port: &Port<'_>, bytes: &[u8]) {
        let Some(registers) = self.registers(port.device()) else {
            return;
        };
        registers.console_write(bytes);
    }
}
