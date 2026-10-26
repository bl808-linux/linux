// SPDX-License-Identifier: GPL-2.0-only

#[kernel::macros::kunit_tests(rust_bl808_spi)]
mod tests {
    use super::super::*;
    use core::cell::Cell;

    // Faults: 0 normal, 1 TX stalled, 2 shifter busy, 3 invalid FIFO count,
    // 4 intermittent FIFO backpressure, 5 a shifter that becomes idle later.
    struct Fake {
        fault: Cell<u8>,
        config: Cell<u32>,
        period: Cell<u32>,
        interval: Cell<u32>,
        sent: Cell<usize>,
        hash: Cell<u64>,
        polls: Cell<usize>,
        busy_reads: Cell<usize>,
        elapsed: Cell<u64>,
        resets: Cell<usize>,
        reset_error: Cell<bool>,
        bad_write: Cell<bool>,
        clears: Cell<usize>,
    }

    impl Fake {
        fn new(fault: u8) -> Self {
            Self {
                fault: Cell::new(fault),
                config: Cell::new(0),
                period: Cell::new(0),
                interval: Cell::new(0),
                sent: Cell::new(0),
                hash: Cell::new(0),
                polls: Cell::new(0),
                busy_reads: Cell::new(0),
                elapsed: Cell::new(0),
                resets: Cell::new(0),
                reset_error: Cell::new(false),
                bad_write: Cell::new(false),
                clears: Cell::new(0),
            }
        }
    }

    impl PioPort for Fake {
        fn read32<const REGISTER: usize>(&self) -> u32 {
            match REGISTER {
                COUNTS => {
                    let n = self.polls.get();
                    self.polls.set(n + 1);
                    match self.fault.get() {
                        1 => 0,
                        3 => 33,
                        4 => [0, 1, 7, 0, 32][n % 5],
                        _ => 32,
                    }
                }
                BUSY => {
                    let n = self.busy_reads.get();
                    self.busy_reads.set(n + 1);
                    u32::from(self.fault.get() == 2 || (self.fault.get() == 5 && n < 4))
                }
                _ => 0,
            }
        }
        fn write32<const REGISTER: usize>(&self, value: u32) {
            match REGISTER {
                CONFIG => self.config.set(value),
                PERIOD => self.period.set(value),
                INTERVAL => self.interval.set(value),
                WRITE => {
                    if self.config.get() & 1 == 0 || value > 255 {
                        self.bad_write.set(true);
                    }
                    self.sent.set(self.sent.get() + 1);
                    self.hash.set(
                        self.hash
                            .get()
                            .wrapping_mul(257)
                            .wrapping_add(u64::from(value)),
                    );
                }
                FIFO => {
                    if value & 3 != 0 {
                        self.bad_write.set(true);
                    } // No DMA.
                    if value == (1 << 2) | (1 << 3) {
                        self.clears.set(self.clears.get() + 1);
                    }
                }
                INTERRUPT => {
                    if value != 0x3f00 {
                        self.bad_write.set(true);
                    }
                }
                _ => self.bad_write.set(true),
            }
        }
        fn elapsed_ms(&self) -> u64 {
            if self.fault.get() == 1 || self.fault.get() == 2 {
                self.elapsed.set(self.elapsed.get() + 1000);
            }
            self.elapsed.get()
        }
        fn reset(&self) -> Result {
            self.resets.set(self.resets.get() + 1);
            if self.reset_error.get() {
                Err(EIO)
            } else {
                Ok(())
            }
        }
    }

    fn run_bytes(fake: &Fake, len: usize) -> Result {
        let mut bytes = KVec::new();
        for n in 0..len {
            bytes.push(n as u8, GFP_KERNEL)?;
        }
        transfer_pio(fake, &bytes, 40_000_000, 4_000_000)?;
        let hash = bytes.iter().fold(0u64, |hash, b| {
            hash.wrapping_mul(257).wrapping_add(u64::from(*b))
        });
        if fake.sent.get() != len
            || fake.hash.get() != hash
            || fake.config.get() != CONFIG_BASE
            || fake.clears.get() != 2
            || fake.bad_write.get()
        {
            return Err(EIO);
        }
        Ok(())
    }

    #[test]
    fn fifo_boundaries_and_full_frame_preserve_byte_order() {
        for len in [0, 1, 31, 32, 33, 65_536, 240 * 280 * 2] {
            let fake = Fake::new(0);
            assert_eq!(run_bytes(&fake, len), Ok(()));
            assert_eq!(fake.resets.get(), 0);
        }
    }

    #[test]
    fn divider_rounds_down_bus_speed() {
        for (speed, period) in [(4_000_000, 4), (3_000_000, 6), (100_000, 199)] {
            let fake = Fake::new(0);
            assert_eq!(transfer_pio(&fake, &[0x91], 40_000_000, speed), Ok(()));
            assert_eq!(fake.period.get(), period * 0x01010101);
            assert_eq!(fake.interval.get(), period);
            assert!(40_000_000 / (2 * (period + 1)) <= speed);
        }
    }

    #[test]
    fn backpressure_and_busy_shifter_are_waited_out() {
        let fake = Fake::new(4);
        assert_eq!(run_bytes(&fake, 65), Ok(()));
        assert!(fake.polls.get() > 3);
        let fake = Fake::new(5);
        assert_eq!(run_bytes(&fake, 33), Ok(()));
        assert_eq!(fake.busy_reads.get(), 5);
        assert_eq!(fake.resets.get(), 0);
    }

    #[test]
    fn stalled_fifo_and_busy_shifter_timeout_and_reset() {
        for fault in [1, 2] {
            let fake = Fake::new(fault);
            assert_eq!(
                transfer_pio(&fake, &[1, 2, 3], 40_000_000, 4_000_000),
                Err(ETIMEDOUT)
            );
            assert_eq!(fake.resets.get(), 1);
            assert_eq!(fake.config.get(), CONFIG_BASE);
            assert_eq!(fake.clears.get(), 2);
            assert!(!fake.bad_write.get());
            assert!(fake.polls.get() < 8);
        }
    }

    #[test]
    fn invalid_fifo_count_quiesces_before_any_write() {
        let fake = Fake::new(3);
        assert_eq!(
            transfer_pio(&fake, &[0xff], 40_000_000, 4_000_000),
            Err(EIO)
        );
        assert_eq!(fake.sent.get(), 0);
        assert_eq!(fake.config.get(), CONFIG_BASE);
        assert_eq!(fake.resets.get(), 1);
        assert_eq!(fake.clears.get(), 2);
    }

    #[test]
    fn invalid_clocks_do_not_touch_registers() {
        let fake = Fake::new(0);
        assert_eq!(transfer_pio(&fake, &[1], 0, 4_000_000), Err(EINVAL));
        assert_eq!(transfer_pio(&fake, &[1], 40_000_000, 0), Err(EINVAL));
        assert_eq!(fake.clears.get(), 0);
        assert_eq!(fake.resets.get(), 0);
        assert_eq!(fake.config.get(), 0);
    }

    #[test]
    fn failed_reset_still_leaves_engine_disabled() {
        let fake = Fake::new(1);
        fake.reset_error.set(true);
        assert_eq!(transfer_pio(&fake, &[1], 40_000_000, 4_000_000), Err(EIO));
        assert_eq!(fake.resets.get(), 1);
        assert_eq!(fake.config.get(), CONFIG_BASE);
        assert_eq!(fake.clears.get(), 2);
    }

    #[test]
    fn next_transfer_succeeds_after_fault_recovery() {
        let fake = Fake::new(3);
        assert_eq!(transfer_pio(&fake, &[1], 40_000_000, 4_000_000), Err(EIO));
        fake.fault.set(0);
        fake.clears.set(0);
        assert_eq!(run_bytes(&fake, 33), Ok(()));
        assert_eq!(fake.resets.get(), 1);
    }
}
