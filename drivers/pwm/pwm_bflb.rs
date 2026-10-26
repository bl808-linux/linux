// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Kwanghoon Son

//! Rust Bouffalo Lab BL808 PWM driver.
//!
//! Hardware model from the BL808 reference manual (PWM chapter):
//!
//! - One register block per SoC at 0x2000A400 holding two independent
//!   four-channel controllers, "PWM 0" and "PWM 1" in the overview, which the
//!   manual's register descriptions call `pwm_mc0` and `pwm_mc1`.
//! - Each controller has a 16-bit clock divider and a 16-bit counter period;
//!   both are shared by its four channels.
//! - Each channel has a counter window `[threL, threH)`: the positive output
//!   is active while the channel window contains the counter. The window
//!   start implements a duty phase without extra registers.
//! - Three clock sources (bclk, xclk, f32k) selected by `reg_clk_sel`. This
//!   driver keeps the reset value `xclk`, the 40 MHz crystal input of the
//!   tested boards.
//!
//! Limitations:
//!
//! - Only normal polarity is implemented; the complementary (negative)
//!   outputs, dead time, break signals, stop modes and interrupts are left at
//!   their reset values and unused.
//! - A channel window cannot wrap around the end of a period: a duty offset
//!   plus duty length beyond the period is truncated.
//! - Changing the shared divider or period is refused while another channel
//!   of the same controller is on; disable the running channels first.
//!
//! The 16-bit divider is written as a plain division factor: the counter
//! advances once per `div` selected clock cycles (`div = 1` bypasses the
//! divider, and the reset default of zero is treated like one). A board that
//! measures a different frequency than this model expects must have the
//! divider interpretation corrected here.

use kernel::{
    clk::Clk,
    device::{Bound, Core, Device},
    io::{mem::DevresIoMem, Io},
    of,
    platform,
    prelude::*,
    pwm,
    time, //
};

/// Seconds represented in nanoseconds as an unsigned 64-bit value.
const NSEC64: u64 = time::NSEC_PER_SEC as u64;

/// Two controllers (`pwm_mc0` and `pwm_mc1`) with four channels each.
const BFLB_PWM_COUNT: u32 = 8;

/// Channels per controller.
const BFLB_PWM_CHANNELS: u32 = 4;

/// Positive-enable bits (`pwm_chy_pen`) of a controller `pwm_mc?_config1`.
const BFLB_PEN_MASK: u32 = 0x1111;

/// Largest divider accepted by the 16-bit `pwm_clk_div` field.
const BFLB_DIV_MAX: u64 = 0xffff;

/// Register block span of the PWM block, from `pwm_int_config` at
/// 0x2000A400 through the `pwm_mc1` registers at 0x2000A480..0x2000A4B0.
const BFLB_PWM_REG_SIZE: usize = 0x100;

/// Base offset of a controller: `pwm_mc0` at +0x40, `pwm_mc1` at +0x80.
fn mc_base(hwpwm: u32) -> usize {
    (0x40 + 0x40 * (hwpwm / BFLB_PWM_CHANNELS)) as usize
}

/// `pwm_mc?_config0`: clock source select and divider.
fn mc_config0(hwpwm: u32) -> usize {
    mc_base(hwpwm)
}

/// `pwm_mc?_config1`: per-channel polarity and output enables.
fn mc_config1(hwpwm: u32) -> usize {
    mc_base(hwpwm) + 0x4
}

/// `pwm_mc?_period`: 16-bit counter period in the low half.
fn mc_period(hwpwm: u32) -> usize {
    mc_base(hwpwm) + 0x8
}

/// `pwm_mc?_ch?_thre`: the channel's `[threL, threH)` counter window.
fn mc_thre(hwpwm: u32) -> usize {
    mc_base(hwpwm) + 0x10 + (hwpwm % BFLB_PWM_CHANNELS) as usize * 0x4
}

/// Positive-enable bit of one channel in `pwm_mc?_config1`.
fn ch_pen_bit(hwpwm: u32) -> u32 {
    1 << (4 * (hwpwm % BFLB_PWM_CHANNELS))
}

/// Counter steps taken during `ns` at the divided clock, rounding half up.
///
/// Callers bound `ns * rate` by the slowest representable period before they
/// call this, keeping the product below `u64::MAX`.
fn ns_to_counts(ns: u64, rate_hz: u64, div: u64) -> u64 {
    const NSEC_PER_SEC_U64: u64 = time::NSEC_PER_SEC as u64;

    let num = ns * rate_hz;
    let den = div * NSEC_PER_SEC_U64;
    (num + den / 2) / den
}

/// Nanoseconds taken by `counts` counter steps at the divided clock.
///
/// `counts * div` stays below `0xffff * 0xffff`, so the product with
/// `NSEC_PER_SEC` fits in 64 bits.
fn counts_to_ns(counts: u64, rate_hz: u64, div: u64) -> u64 {
    const NSEC_PER_SEC_U64: u64 = time::NSEC_PER_SEC as u64;

    (counts * div * NSEC_PER_SEC_U64) / rate_hz
}

/// Driver's hardware representation of a requested waveform.
///
/// The divider and the period are properties of the whole controller and
/// stay shared by its four channels.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
struct BflbWfHw {
    /// Selected clock divider in input clocks per counter step.
    div: u16,
    /// 16-bit counter period; the counter wraps to 0 when it reaches this.
    period: u16,
    /// First counter value with the output active (`threL`).
    thre_low: u16,
    /// First counter value after the window (`threH`); output inactive.
    thre_high: u16,
    /// Whether the channel's driver drives the pin (`pwm_chy_pen`).
    enabled: bool,
}

struct BflbPwmPlatformDriver;

struct EnabledClock(Clk);
impl core::ops::Deref for EnabledClock {
    type Target = Clk;
    fn deref(&self) -> &Clk {
        &self.0
    }
}
impl Drop for EnabledClock {
    fn drop(&mut self) {
        self.0.disable_unprepare();
    }
}

#[pin_data]
struct BflbPwmDriverData {
    #[pin]
    iomem: DevresIoMem<BFLB_PWM_REG_SIZE>,
    clk: EnabledClock,
}

fn round_waveform(rate_hz: u64, wf: &pwm::Waveform) -> Result<pwm::RoundedWaveform<BflbWfHw>> {
    let mut status = 0;

    if wf.period_length_ns == 0 {
        return Ok(pwm::RoundedWaveform {
            status,
            hardware_waveform: BflbWfHw::default(),
        });
    }

    if rate_hz == 0 {
        return Err(EINVAL);
    }

    // Slowest representable period: 16-bit divider times 16-bit period
    // counted at the selected clock. This check bounds every product
    // below, for the period and for the contained duty and offset.
    let max_period_ns = (BFLB_DIV_MAX * BFLB_DIV_MAX * NSEC64) / rate_hz.max(1);
    if wf.period_length_ns > max_period_ns {
        return Err(ERANGE);
    }

    // Bring the period below the 16-bit counter limit with the smallest
    // divider that achieves it.
    let counts_at_div1 = (wf.period_length_ns * rate_hz) / NSEC64;
    let div = counts_at_div1.div_ceil(BFLB_DIV_MAX).max(1);
    let period_counts = ns_to_counts(wf.period_length_ns, rate_hz, div).clamp(2, BFLB_DIV_MAX);
    if counts_to_ns(period_counts, rate_hz, div) > wf.period_length_ns {
        // The hardware can only run slower than requested.
        status = 1;
    }

    let mut thre_low = 0;
    let mut thre_high = 0;
    if wf.duty_length_ns != 0 {
        // The window must stay inside the period; offsets and lengths
        // beyond it are truncated.
        let start_ns = wf.duty_offset_ns.min(wf.period_length_ns);
        let end_ns = wf
            .duty_offset_ns
            .saturating_add(wf.duty_length_ns)
            .min(wf.period_length_ns);

        thre_low = ns_to_counts(start_ns, rate_hz, div).min(period_counts);
        if thre_low >= period_counts {
            // The truncated window never becomes non-empty; the channel
            // idles.
            return Ok(pwm::RoundedWaveform {
                status,
                hardware_waveform: BflbWfHw::default(),
            });
        }
        thre_high = ns_to_counts(end_ns, rate_hz, div)
            .max(thre_low + 1)
            .min(period_counts);
    }

    let wfhw = BflbWfHw {
        div: div as u16,
        period: period_counts as u16,
        thre_low: thre_low as u16,
        thre_high: thre_high as u16,
        enabled: wf.duty_length_ns != 0,
    };

    Ok(pwm::RoundedWaveform {
        status,
        hardware_waveform: wfhw,
    })
}

impl pwm::PwmOps for BflbPwmDriverData {
    type WfHw = BflbWfHw;

    fn round_waveform_tohw(
        chip: &pwm::Chip<Self>,
        _pwm: &pwm::Device,
        wf: &pwm::Waveform,
    ) -> Result<pwm::RoundedWaveform<Self::WfHw>> {
        round_waveform(chip.drvdata().clk.rate().as_hz() as u64, wf)
    }

    fn round_waveform_fromhw(
        chip: &pwm::Chip<Self>,
        _pwm: &pwm::Device,
        wfhw: &Self::WfHw,
        wf: &mut pwm::Waveform,
    ) -> Result {
        let data = chip.drvdata();
        let rate_hz = data.clk.rate().as_hz() as u64;
        let div = u64::from(wfhw.div).max(1);

        wf.period_length_ns = counts_to_ns(u64::from(wfhw.period), rate_hz, div);
        wf.duty_offset_ns = counts_to_ns(u64::from(wfhw.thre_low), rate_hz, div);
        wf.duty_length_ns = if wfhw.enabled && wfhw.thre_high > wfhw.thre_low {
            counts_to_ns(u64::from(wfhw.thre_high - wfhw.thre_low), rate_hz, div)
        } else {
            0
        };

        Ok(())
    }

    fn read_waveform(
        chip: &pwm::Chip<Self>,
        pwm: &pwm::Device,
        parent_dev: &Device<Bound>,
    ) -> Result<Self::WfHw> {
        let data = chip.drvdata();
        let hwpwm = pwm.hwpwm();
        let iomap = data.iomem.access(parent_dev)?;

        let config0 = iomap.try_read32(mc_config0(hwpwm))?;
        let config1 = iomap.try_read32(mc_config1(hwpwm))?;
        let period = iomap.try_read32(mc_period(hwpwm))?;
        let thre = iomap.try_read32(mc_thre(hwpwm))?;

        let wfhw = BflbWfHw {
            div: config0 as u16,
            period: period as u16,
            thre_low: thre as u16,
            thre_high: (thre >> 16) as u16,
            enabled: config1 & ch_pen_bit(hwpwm) != 0,
        };

        dev_dbg!(
            chip.device(),
            "PWM-{}: read_waveform: div {} period {} thre {}/{} enabled {}\n",
            hwpwm,
            wfhw.div,
            wfhw.period,
            wfhw.thre_low,
            wfhw.thre_high,
            wfhw.enabled
        );

        Ok(wfhw)
    }

    fn write_waveform(
        chip: &pwm::Chip<Self>,
        pwm: &pwm::Device,
        wfhw: &Self::WfHw,
        parent_dev: &Device<Bound>,
    ) -> Result {
        let data = chip.drvdata();
        let hwpwm = pwm.hwpwm();
        let pen = ch_pen_bit(hwpwm);
        let iomap = data.iomem.access(parent_dev)?;
        let config1 = iomap.try_read32(mc_config1(hwpwm))?;

        if !wfhw.enabled {
            if config1 & pen != 0 {
                dev_dbg!(chip.device(), "PWM-{}: disabling channel.\n", hwpwm);
                iomap.try_write32(config1 & !pen, mc_config1(hwpwm))?;
            }
            return Ok(());
        }

        // The divider and the period belong to the controller. Refuse to
        // change them while any other channel keeps running.
        let config0 = iomap.try_read32(mc_config0(hwpwm))?;
        let period = iomap.try_read32(mc_period(hwpwm))?;
        let shared_changed = config0 as u16 != wfhw.div || period as u16 != wfhw.period;
        if shared_changed {
            let others_pen = BFLB_PEN_MASK & !pen;
            if config1 & others_pen != 0 {
                dev_err!(
                    chip.device(),
                    "PWM-{}: the counter divider and period are shared by the running channels of its controller; disable them before changing these settings.\n",
                    hwpwm
                );
                return Err(EINVAL);
            }

            iomap.try_write32(u32::from(wfhw.div), mc_config0(hwpwm))?;
            iomap.try_write32(u32::from(wfhw.period), mc_period(hwpwm))?;
        }

        iomap.try_write32(
            (u32::from(wfhw.thre_high) << 16) | u32::from(wfhw.thre_low),
            mc_thre(hwpwm),
        )?;
        iomap.try_write32(config1 | pen, mc_config1(hwpwm))?;

        dev_dbg!(
            chip.device(),
            "PWM-{}: wrote div {} period {} thre {}/{}\n",
            hwpwm,
            wfhw.div,
            wfhw.period,
            wfhw.thre_low,
            wfhw.thre_high
        );

        Ok(())
    }
}

kernel::of_device_table!(
    OF_TABLE,
    <BflbPwmPlatformDriver as platform::Driver>::IdInfo,
    [(of::DeviceId::new(c"bflb,bl808-pwm"), ())]
);

impl platform::Driver for BflbPwmPlatformDriver {
    type IdInfo = ();
    type Data<'bound> = Self;
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _id_info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self, Error> + 'bound {
        let dev = pdev.as_ref();
        let request = pdev.io_request_by_index(0).ok_or(ENODEV)?;

        let clk = Clk::get(dev, None)?;

        clk.prepare_enable()?;
        let clk = EnabledClock(clk);

        let rate_hz = clk.rate().as_hz();
        if rate_hz == 0 {
            dev_err!(dev, "Clock rate is zero\n");
            return Err(EINVAL);
        }

        let chip = pwm::Chip::new(
            dev,
            BFLB_PWM_COUNT,
            try_pin_init!(BflbPwmDriverData {
                iomem <- request.iomap_sized::<BFLB_PWM_REG_SIZE>()?.into_devres(),
                clk <- clk,
            }),
        )?;

        chip.register()?;

        Ok(BflbPwmPlatformDriver)
    }
}

kernel::module_pwm_platform_driver! {
    type: BflbPwmPlatformDriver,
    name: "pwm-bflb",
    authors: ["Kwanghoon Son <asdfkwang@gmail.com>"],
    description: "Bouffalo Lab BL808 PWM driver",
    license: "GPL v2",
}

#[cfg(CONFIG_PWM_BFLB_KUNIT_TEST)]
#[path = "pwm_bflb/tests.rs"]
mod tests;
