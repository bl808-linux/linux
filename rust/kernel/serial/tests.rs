// SPDX-License-Identifier: GPL-2.0

#[crate::macros::kunit_tests(rust_serial)]
mod tests {
    use crate::serial::PortData;
    use crate::ModuleMetadata;
    use crate::{
        faux,
        prelude::*,
        serial::*,
        sync::{
            atomic::{Atomic, Relaxed},
            Arc,
        },
    };

    struct Counts {
        dropped: Atomic<u32>,
        writes: Atomic<u32>,
    }

    struct Hardware(Arc<Counts>);

    impl SerialOps for Hardware {
        fn tx_empty(&self, _port: &Port<'_>) -> bool {
            true
        }
        fn set_termios(&self, port: &LockedPort<'_>, _new: &mut Termios, _old: Option<&Termios>) {
            // SAFETY: The adapter retains the port throughout this callback.
            let locked = unsafe { crate::bindings::uart_port_test_locked(port.raw) };
            assert!(locked);
        }
        fn console_write(&self, port: &Port<'_>, _bytes: &[u8]) {
            // These tests exercise normal output, where the adapter must lock.
            // SAFETY: The callback-scoped port remains registered.
            let locked = unsafe { crate::bindings::uart_port_test_locked(port.raw) };
            assert!(locked);
            self.0.writes.fetch_add(1, Relaxed);
        }
    }

    impl Drop for Hardware {
        fn drop(&mut self) {
            self.0.dropped.fetch_add(1, Relaxed);
        }
    }

    fn counts() -> Arc<Counts> {
        Arc::new(
            Counts {
                dropped: Atomic::new(0),
                writes: Atomic::new(0),
            },
            GFP_KERNEL,
        )
        .unwrap()
    }

    fn driver() -> Arc<Driver<Hardware>> {
        Arc::pin_init(
            Driver::new(
                c"rust_serial_test",
                c"ttyRustTest",
                2,
                115200,
                &crate::LocalModule::THIS_MODULE,
            ),
            GFP_KERNEL,
        )
        .unwrap()
    }

    fn config(line: u32) -> PortConfig {
        // Zero mapbase leaves the test console inactive. The serial-core state,
        // callback adapters, device creation and removal paths are still real.
        PortConfig {
            line,
            mapbase: 0,
            clock: 40000000,
            fifo_size: 32,
        }
    }

    #[test]
    fn registration_lives_until_parent_unbind() {
        let counts = counts();
        let driver = driver();
        for _ in 0..4 {
            let parent = faux::Registration::new(c"rust-serial-parent", None).unwrap();
            register_port(
                driver.clone(),
                parent.as_ref().as_ref(),
                config(0),
                Ok(Hardware(counts.clone())),
            )
            .unwrap();
            // SAFETY: The parent is still bound, and this is the only port on line 0.
            // The real C-to-Rust adapter exercises the typed private_data cast.
            let status = unsafe {
                let raw = (*(*driver.inner.get()).state).uart_port;
                PortData::<Hardware>::with_port(raw, |state, port| {
                    port.with_console_access(|| state.data.console_write(port, b"test"))
                });
                SerialAdapter::<Hardware>::tx_empty(raw)
            };
            assert_eq!(status, crate::bindings::TIOCSER_TEMT);
            drop(parent);
        }
        assert_eq!(counts.dropped.load(Relaxed), 4);
        assert_eq!(counts.writes.load(Relaxed), 4);
    }

    #[test]
    fn adapters_supply_callback_lock_contexts() {
        use crate::bindings;

        let counts = counts();
        let driver = driver();
        let parent = faux::Registration::new(c"rust-serial-locks", None).unwrap();
        register_port(
            driver.clone(),
            parent.as_ref().as_ref(),
            config(0),
            Ok(Hardware(counts.clone())),
        )
        .unwrap();
        // SAFETY: This bound parent owns line 0. No other reader or writer uses
        // the inactive console; the test supplies valid callback arguments.
        let status = unsafe {
            let raw = (*(*driver.inner.get()).state).uart_port;
            let mut termios = bindings::ktermios::default();
            SerialAdapter::<Hardware>::set_termios(raw, &mut termios, core::ptr::null());
            (*driver.console.get()).index = 0;
            ConsoleAdapter::<Hardware>::console_write(
                driver.console.get(),
                c"test".as_char_ptr(),
                4,
            );
            (*driver.console.get()).index = -1;
            // tx_empty must remain callable while the UART lock is already held.
            PortData::<Hardware>::with_port(raw, |_, port| {
                port.with_lock(|_| SerialAdapter::<Hardware>::tx_empty(raw))
            })
        };
        assert_eq!(status, bindings::TIOCSER_TEMT);
        assert_eq!(counts.writes.load(Relaxed), 1);
        drop(parent);
        assert_eq!(counts.dropped.load(Relaxed), 1);
    }

    #[test]
    fn termios_controls_preserve_other_flags() {
        use crate::bindings;

        for size in [bindings::CS5, bindings::CS6, bindings::CS7, bindings::CS8] {
            for parity in [
                0,
                bindings::PARENB,
                bindings::PARENB | bindings::PARODD,
                bindings::PARENB | bindings::CMSPAR,
                bindings::PARENB | bindings::CMSPAR | bindings::PARODD,
                bindings::CMSPAR | bindings::PARODD,
            ] {
                for stop in [0, bindings::CSTOPB] {
                    for flow in [0, bindings::CRTSCTS] {
                        let flags = size
                            | parity
                            | stop
                            | flow
                            | bindings::CREAD
                            | bindings::CLOCAL
                            | bindings::HUPCL
                            | bindings::B115200;
                        let mut raw = bindings::ktermios {
                            c_cflag: flags,
                            c_iflag: 1,
                            c_oflag: 2,
                            c_lflag: 4,
                            ..Default::default()
                        };
                        // SAFETY: raw is initialized and exclusively borrowed until the
                        // last use of termios below.
                        let termios = unsafe { Termios::from_raw_mut(&mut raw) };
                        assert_eq!(termios.parity_enabled(), parity & bindings::PARENB != 0);
                        assert_eq!(termios.odd_parity(), parity & bindings::PARODD != 0);
                        assert_eq!(termios.two_stop_bits(), stop != 0);
                        let bits = termios.char_size();
                        termios.disable_mark_space_parity();
                        termios.disable_hardware_flow_control();
                        assert_eq!(termios.char_size(), bits);
                        assert_eq!(raw.c_cflag, flags & !(bindings::CMSPAR | bindings::CRTSCTS));
                        assert_eq!((raw.c_iflag, raw.c_oflag, raw.c_lflag), (1, 2, 4));
                    }
                }
            }
        }
    }

    fn raw_termios(baud: u32) -> crate::bindings::ktermios {
        let mut raw = crate::bindings::ktermios::default();
        // SAFETY: raw is initialized and exclusively accessible.
        unsafe { crate::bindings::tty_termios_encode_baud_rate(&mut raw, baud, baud) };
        raw
    }

    fn with_locked_test_port<R>(f: impl FnOnce(&LockedPort<'_>) -> R) -> R {
        let driver = driver();
        let parent = faux::Registration::new(c"rust-serial-termios", None).unwrap();
        register_port(
            driver.clone(),
            parent.as_ref().as_ref(),
            config(0),
            Ok(Hardware(counts())),
        )
        .unwrap();
        // SAFETY: The bound parent owns this inactive port for the callback.
        unsafe {
            let raw = (*(*driver.inner.get()).state).uart_port;
            PortData::<Hardware>::with_port(raw, |_, port| port.with_lock(f))
        }
    }

    #[test]
    fn baud_selection_matches_serial_core() {
        use crate::bindings;

        for (baud, previous) in [
            (115200, None),
            (0, None),
            (1, Some(115200)),
            (4000001, None),
            (4000001, Some(115200)),
            (1, Some(0)),
        ] {
            let (actual, expected_baud, raw, expected) = with_locked_test_port(|port| {
                let mut raw = raw_termios(baud);
                let mut expected = raw;
                let old = previous.map(raw_termios);
                let old_ptr = old.as_ref().map_or(core::ptr::null(), core::ptr::from_ref);
                // SAFETY: The port is locked. Both termios values are initialized
                // and expected is exclusively borrowed for the C call.
                let expected_baud = unsafe {
                    bindings::uart_get_baud_rate(port.raw, &mut expected, old_ptr, 611, 4000000)
                };
                // SAFETY: raw is exclusively borrowed and old is immutable until
                // these callback-scoped references are no longer used.
                let actual = unsafe {
                    port.baud_rate(
                        Termios::from_raw_mut(&mut raw),
                        old.as_ref().map(|t| Termios::from_raw(t)),
                        611,
                        4000000,
                    )
                };
                (actual, expected_baud, raw, expected)
            });
            assert_eq!(actual, core::num::NonZeroU32::new(expected_baud));
            assert_eq!(raw.c_cflag, expected.c_cflag);
            assert_eq!(raw.c_ispeed, expected.c_ispeed);
            assert_eq!(raw.c_ospeed, expected.c_ospeed);
        }
    }

    #[test]
    fn baud_failure_keeps_serial_core_termios_changes() {
        let (actual, raw) = with_locked_test_port(|port| {
            let mut raw = raw_termios(1);
            let old = raw_termios(2);
            // Both requested rates are outside the range. The real C helper emits
            // its expected WARN_ON and returns zero after encoding min + 1.
            // SAFETY: The initialized termios values remain exclusively/shared
            // accessible respectively until the returned borrows are no longer used.
            let actual = unsafe {
                port.baud_rate(
                    Termios::from_raw_mut(&mut raw),
                    Some(Termios::from_raw(&old)),
                    611,
                    4000000,
                )
            };
            (actual, raw)
        });
        assert!(actual.is_none());
        assert_eq!(raw.c_ispeed, 612);
        assert_eq!(raw.c_ospeed, 612);
        assert_eq!(raw.c_cflag & crate::bindings::CBAUD, crate::bindings::B600);
    }

    #[test]
    fn duplicate_port_failure_preserves_original() {
        let counts = counts();
        let driver = driver();
        let parent = faux::Registration::new(c"rust-serial-duplicate", None).unwrap();
        let other = faux::Registration::new(c"rust-serial-duplicate-other", None).unwrap();
        register_port(
            driver.clone(),
            parent.as_ref().as_ref(),
            config(0),
            Ok(Hardware(counts.clone())),
        )
        .unwrap();
        assert!(register_port(
            driver.clone(),
            other.as_ref().as_ref(),
            config(0),
            Ok(Hardware(counts.clone()))
        )
        .is_err());
        assert_eq!(counts.dropped.load(Relaxed), 1);
        // SAFETY: The first registration and parent remain live despite the failed duplicate.
        let status = unsafe {
            let raw = (*(*driver.inner.get()).state).uart_port;
            SerialAdapter::<Hardware>::tx_empty(raw)
        };
        assert_eq!(status, crate::bindings::TIOCSER_TEMT);
        drop(parent);
        assert_eq!(counts.dropped.load(Relaxed), 2);
    }

    #[test]
    fn data_initialization_failure_does_not_publish_port() {
        let driver = driver();
        let parent = faux::Registration::new(c"rust-serial-init-error", None).unwrap();
        assert!(register_port(
            driver.clone(),
            parent.as_ref().as_ref(),
            config(0),
            Err::<Hardware, Error>(ENOMEM)
        )
        .is_err());
        // SAFETY: Registration failed before C publication; driver owns the state array.
        assert!(unsafe { (*(*driver.inner.get()).state).uart_port.is_null() });
        let counts = counts();
        register_port(
            driver.clone(),
            parent.as_ref().as_ref(),
            config(0),
            Ok(Hardware(counts.clone())),
        )
        .unwrap();
        drop(parent);
        assert_eq!(counts.dropped.load(Relaxed), 1);
    }

    #[test]
    fn invalid_line_releases_unpublished_data() {
        let counts = counts();
        let driver = driver();
        let parent = faux::Registration::new(c"rust-serial-invalid", None).unwrap();
        assert!(register_port(
            driver,
            parent.as_ref().as_ref(),
            config(2),
            Ok(Hardware(counts.clone()))
        )
        .is_err());
        assert_eq!(counts.dropped.load(Relaxed), 1);
    }
    #[test]
    fn console_removal_waits_for_srcu_readers() {
        use crate::{bindings, sync::Completion, workqueue};

        #[pin_data]
        struct Stages {
            #[pin]
            entered: Completion,
            #[pin]
            release: Completion,
            #[pin]
            removing: Completion,
            #[pin]
            removed: Completion,
        }

        let stages = Arc::pin_init(
            pin_init!(Stages {
                entered <- Completion::new(),
                release <- Completion::new(),
                removing <- Completion::new(),
                removed <- Completion::new(),
            }),
            GFP_KERNEL,
        )
        .unwrap();
        let counts = counts();
        let driver = driver();
        let parent = faux::Registration::new(c"rust-serial-srcu", None).unwrap();
        // SAFETY: The console core copies the name. No options pointer is retained.
        assert_eq!(
            unsafe {
                bindings::add_preferred_console(
                    c"ttyRustTest".as_char_ptr(),
                    0,
                    core::ptr::null_mut(),
                )
            },
            0
        );
        let mut cfg = config(0);
        // A reportable port causes serial core to register the test console.
        // Fake operations never dereference this MMIO address.
        cfg.mapbase = 1;
        register_port(
            driver.clone(),
            parent.as_ref().as_ref(),
            cfg,
            Ok(Hardware(counts.clone())),
        )
        .unwrap();
        // SAFETY: The bound parent owns the console; console_list_lock prevents
        // concurrent console-list updates while inspecting its membership.
        let registered = unsafe {
            bindings::console_list_lock();
            let registered = !(*driver.console.get()).node.pprev.is_null();
            bindings::console_list_unlock();
            registered
        };
        assert!(registered);

        // SAFETY: The default system workqueue is initialized before KUnit runs
        // and has static lifetime. The current Rust system_unbound() accessor
        // still selects the deprecated C queue, so use the current C default here.
        let queue = unsafe { workqueue::Queue::from_raw(bindings::system_dfl_wq) };
        let reader_stages = stages.clone();
        let reader_driver = driver.clone();
        queue
            .try_spawn(GFP_KERNEL, move || {
                // SAFETY: This is the same SRCU read section used by legacy printk.
                // The test parent remains bound until after entered is signaled.
                let cookie = unsafe { bindings::console_srcu_read_lock() };
                unsafe {
                    ConsoleAdapter::<Hardware>::console_write(
                        reader_driver.console.get(),
                        c"reader".as_char_ptr(),
                        6,
                    )
                };
                reader_stages.entered.complete_all();
                // SRCU read sections may sleep. The hardware callback above has returned;
                // keep its console reader active to test unregister's grace period.
                reader_stages.release.wait_for_completion();
                // SAFETY: Matches this worker's read lock; no console access follows it.
                unsafe { bindings::console_srcu_read_unlock(cookie) };
            })
            .unwrap();
        stages.entered.wait_for_completion();

        let remove_stages = stages.clone();
        queue
            .try_spawn(GFP_KERNEL, move || {
                remove_stages.removing.complete_all();
                drop(parent);
                remove_stages.removed.complete_all();
            })
            .unwrap();
        stages.removing.wait_for_completion();
        // SAFETY: KUnit runs in sleepable task context. Allow the other worker to
        // enter unregister_console while the SRCU reader is deliberately held.
        unsafe { bindings::msleep(20) };
        assert_eq!(counts.dropped.load(Relaxed), 0);
        stages.release.complete_all();
        stages.removed.wait_for_completion();
        assert_eq!(counts.dropped.load(Relaxed), 1);
        assert!(counts.writes.load(Relaxed) > 0);
    }
}
