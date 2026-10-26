// SPDX-License-Identifier: GPL-2.0

#[kernel::macros::kunit_tests(rust_bl808_pwm)]
mod tests {
    use super::super::*;

    #[test]
    fn register_offsets_separate_both_four_channel_controllers() {
        for channel in 0..8 {
            let base = if channel < 4 { 0x40 } else { 0x80 };
            assert_eq!(mc_config0(channel), base);
            assert_eq!(mc_config1(channel), base + 4);
            assert_eq!(mc_period(channel), base + 8);
            assert_eq!(mc_thre(channel), base + 0x10 + (channel % 4) as usize * 4);
            assert_eq!(ch_pen_bit(channel), 1 << (4 * (channel % 4)));
            assert_eq!(BFLB_PEN_MASK & ch_pen_bit(channel), ch_pen_bit(channel));
            assert!(mc_thre(channel) + 4 <= BFLB_PWM_REG_SIZE);
        }
    }

    #[test]
    fn counts_round_half_up_and_convert_back_without_overflow() {
        for (ns, counts) in [
            (12, 0),
            (13, 1),
            (25, 1),
            (37, 1),
            (38, 2),
            (1_000_000, 40_000),
        ] {
            assert_eq!(ns_to_counts(ns, 40_000_000, 1), counts);
        }
        for div in [1, 2, 65535] {
            for counts in [0, 1, 2, 65535] {
                let ns = counts_to_ns(counts, 40_000_000, div);
                assert_eq!(ns_to_counts(ns, 40_000_000, div), counts);
            }
        }
    }

    #[test]
    fn disabled_and_minimum_period_are_reported_correctly() {
        let disabled = round_waveform(40_000_000, &pwm::Waveform::default()).unwrap();
        assert_eq!(disabled.hardware_waveform, BflbWfHw::default());
        assert_eq!(disabled.status, 0);
        let wf = pwm::Waveform {
            period_length_ns: 1,
            duty_length_ns: 1,
            duty_offset_ns: 0,
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert_eq!(rounded.hardware_waveform.period, 2);
        assert_eq!(rounded.status, 1);
    }

    #[test]
    fn exact_waveform_preserves_duty_and_phase_window() {
        let wf = pwm::Waveform {
            period_length_ns: 1_000_000,
            duty_length_ns: 250_000,
            duty_offset_ns: 100_000,
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert_eq!(rounded.status, 0);
        assert_eq!(
            rounded.hardware_waveform,
            BflbWfHw {
                div: 1,
                period: 40000,
                thre_low: 4000,
                thre_high: 14000,
                enabled: true
            }
        );
    }

    #[test]
    fn counter_boundary_increases_divider_and_maximum_fits() {
        let wf = pwm::Waveform {
            period_length_ns: 65536 * 25,
            duty_length_ns: 32768 * 25,
            duty_offset_ns: 0,
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert_eq!(rounded.hardware_waveform.div, 2);
        assert_eq!(rounded.hardware_waveform.period, 32768);
        assert_eq!(rounded.hardware_waveform.thre_high, 16384);
        assert_eq!(rounded.status, 0);
        let max = 65535 * 65535 * 25u64;
        let wf = pwm::Waveform {
            period_length_ns: max,
            duty_length_ns: max,
            duty_offset_ns: 0,
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert_eq!(rounded.hardware_waveform.div, 65535);
        assert_eq!(rounded.hardware_waveform.period, 65535);
        assert_eq!(rounded.hardware_waveform.thre_high, 65535);
    }

    #[test]
    fn oversized_period_and_zero_clock_are_rejected() {
        for period in [65535 * 65535 * 25u64 + 1, u64::MAX] {
            let wf = pwm::Waveform {
                period_length_ns: period,
                duty_length_ns: 0,
                duty_offset_ns: 0,
            };
            assert_eq!(round_waveform(40_000_000, &wf), Err(ERANGE));
        }
        let wf = pwm::Waveform {
            period_length_ns: 1_000_000,
            duty_length_ns: 0,
            duty_offset_ns: 0,
        };
        assert_eq!(round_waveform(0, &wf), Err(EINVAL));
    }

    #[test]
    fn overflowing_duty_is_clipped_without_wrapping() {
        let wf = pwm::Waveform {
            period_length_ns: 1_000_000,
            duty_length_ns: u64::MAX,
            duty_offset_ns: 900_000,
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert_eq!(rounded.hardware_waveform.thre_low, 36000);
        assert_eq!(rounded.hardware_waveform.thre_high, 40000);
        assert!(rounded.hardware_waveform.enabled);
        let wf = pwm::Waveform {
            duty_offset_ns: 1_000_000,
            ..wf
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert!(!rounded.hardware_waveform.enabled);
    }

    #[test]
    fn zero_duty_disables_the_channel() {
        let wf = pwm::Waveform {
            period_length_ns: 1_000_000,
            duty_length_ns: 0,
            duty_offset_ns: 500_000,
        };
        let rounded = round_waveform(40_000_000, &wf).unwrap();
        assert!(!rounded.hardware_waveform.enabled);
        assert_eq!(rounded.hardware_waveform.thre_low, 0);
        assert_eq!(rounded.hardware_waveform.thre_high, 0);
    }
}
