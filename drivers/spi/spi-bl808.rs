// SPDX-License-Identifier: GPL-2.0-only
//! BL808 D0 SPI1: bounded synchronous PIO, mode 0, 8-bit TX.
use kernel::{
    clk::Clk,
    device::{Bound, Core, Device},
    io::{mem::DevresExclusiveIoMem, Io, IoBase},
    of, platform,
    prelude::*,
    processor,
    reset::Reset,
    spi,
    syscon::Syscon,
    time::{Instant, Monotonic},
};
const CONFIG: usize = 0;
const INTERRUPT: usize = 4;
const BUSY: usize = 8;
const PERIOD: usize = 0x10;
const INTERVAL: usize = 0x14;
const FIFO: usize = 0x80;
const COUNTS: usize = 0x84;
const WRITE: usize = 0x88;
// MSB first, mode 0 (the BL808 phase bit uses the inverse CPHA encoding).
const CONFIG_BASE: u32 = (1 << 5) | (1 << 9);
struct EnabledClock(Clk);
impl Drop for EnabledClock {
    fn drop(&mut self) {
        self.0.disable_unprepare();
    }
}
struct Hardware {
    io: DevresExclusiveIoMem<0x90>,
    clock: EnabledClock,
    reset: Reset,
    _syscon: Syscon,
}
// Keep the production state machine shared with the KUnit fake register backend.
trait PioPort {
    fn read32<const REGISTER: usize>(&self) -> u32;
    fn write32<const REGISTER: usize>(&self, value: u32);
    fn elapsed_ms(&self) -> u64;
    fn reset(&self) -> Result;
}
struct MmioPort<'a> {
    view: kernel::io::Mmio<'a, kernel::io::Region<0x90>>,
    start: Instant<Monotonic>,
    reset: &'a Reset,
}
impl PioPort for MmioPort<'_> {
    fn read32<const REGISTER: usize>(&self) -> u32 {
        self.view.read32(REGISTER)
    }
    fn write32<const REGISTER: usize>(&self, value: u32) {
        self.view.write32(value, REGISTER);
    }
    fn elapsed_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }
    fn reset(&self) -> Result {
        self.reset.reset()
    }
}
fn transfer_pio(port: &impl PioPort, bytes: &[u8], rate: u64, speed: u32) -> Result {
    if rate == 0 || speed == 0 {
        return Err(EINVAL);
    }
    // Each SCLK period has two phases, each (prd + 1) input clocks.
    let half = rate.div_ceil(u64::from(speed) * 2).clamp(1, 256) as u32;
    let prd = half - 1;
    port.write32::<CONFIG>(CONFIG_BASE);
    port.write32::<FIFO>((1 << 2) | (1 << 3));
    port.write32::<PERIOD>(prd * 0x01010101);
    port.write32::<INTERVAL>(prd);
    port.write32::<INTERRUPT>(0x3f00); // Mask all leaf interrupts; no DMA.
    port.write32::<CONFIG>(CONFIG_BASE | 1);
    let budget_ms = (bytes.len() as u64 * 8000).div_ceil(u64::from(speed)) * 4 + 1000;
    let mut offset = 0;
    let result = (|| {
        while offset < bytes.len() {
            let count = (port.read32::<COUNTS>() & 63) as usize;
            if count > 32 {
                return Err(EIO);
            }
            // RX is unwired. Keep it drained to avoid a receive FIFO overflow.
            port.write32::<FIFO>(1 << 3);
            let end = (offset + count).min(bytes.len());
            while offset < end {
                port.write32::<WRITE>(u32::from(bytes[offset]));
                offset += 1;
            }
            if port.elapsed_ms() > budget_ms {
                return Err(ETIMEDOUT);
            }
            processor::cpu_relax();
        }
        while port.read32::<COUNTS>() & 63 != 32 || port.read32::<BUSY>() & 1 != 0 {
            port.write32::<FIFO>(1 << 3);
            if port.elapsed_ms() > budget_ms {
                return Err(ETIMEDOUT);
            }
            processor::cpu_relax();
        }
        Ok(())
    })();
    port.write32::<CONFIG>(CONFIG_BASE);
    port.write32::<FIFO>((1 << 2) | (1 << 3));
    if result.is_err() {
        port.reset()?;
    }
    result
}
impl spi::Operations for Hardware {
    fn transfer(&self, dev: &Device<Bound>, bytes: &[u8], speed: u32) -> Result {
        let io = self.io.access(dev)?;
        let port = MmioPort {
            view: io.as_view(),
            start: Instant::<Monotonic>::now(),
            reset: &self.reset,
        };
        let result = transfer_pio(&port, bytes, self.clock.0.rate().as_hz() as u64, speed);
        if result.is_err() {
            dev_err!(dev, "SPI1 PIO aborted ({} bytes)\n", bytes.len());
        }
        result
    }
}
#[cfg(CONFIG_SPI_BFLB_RUST_KUNIT_TEST)]
#[path = "spi-bl808/tests.rs"]
mod tests;

struct Spi;
kernel::of_device_table!(OF_TABLE, (), [(of::DeviceId::new(c"bflb,bl808-spi"), ())]);
impl platform::Driver for Spi {
    type IdInfo = ();
    type Data<'bound> = ();
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);
    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _: Option<&'bound ()>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        pin_init::pin_init_scope(move || {
            let dev: &Device<Bound> = pdev.as_ref();
            let io = pdev
                .io_request_by_index(0)
                .ok_or(ENODEV)?
                .iomap_exclusive_sized::<0x90>()?
                .into_devres()?;
            let clock = Clk::get(dev, None)?;
            clock.prepare_enable()?;
            let clock = EnabledClock(clock);
            if clock.0.rate().as_hz() == 0 {
                return Err(EINVAL);
            }
            let reset = Reset::get(dev)?;
            reset.reset()?;
            let syscon = Syscon::get(dev, c"bflb,glb")?;
            syscon.update_bits(0x510, 1 << 27, 1 << 27)?; // SPI1 master mode only.
            spi::register(
                dev,
                spi::ControllerConfig {
                    min_speed_hz: 100_000,
                    max_speed_hz: 4_000_000,
                },
                Ok(Hardware {
                    io,
                    clock,
                    reset,
                    _syscon: syscon,
                }),
            )?;
            dev_info!(dev, "SPI1 Rust PIO controller registered\n");
            Ok(Ok::<(), Error>(()))
        })
    }
}
kernel::module_platform_driver! {
    type: Spi,
    name: "spi_bl808",
    authors: ["Kwanghoon Son"],
    description: "Bouffalo Lab BL808 Rust SPI1 PIO controller",
    license: "GPL",
}
