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

    impl Operations for Hardware {
        fn tx_empty(&self, _port: &Port<'_, Self>) -> bool {
            true
        }
        fn set_termios(&self, _port: &Port<'_, Self>, _new: &mut Termios, _old: Option<&Termios>) {}
        fn console_write(&self, _port: &Port<'_, Self>, _bytes: &[u8]) {
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
                PortData::<Hardware>::with_port(raw, |data, port| {
                    data.console_write(port, b"test")
                });
                PortData::<Hardware>::tx_empty(raw)
            };
            assert_eq!(status, crate::bindings::TIOCSER_TEMT);
            drop(parent);
        }
        assert_eq!(counts.dropped.load(Relaxed), 4);
        assert_eq!(counts.writes.load(Relaxed), 4);
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
            PortData::<Hardware>::tx_empty(raw)
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
                    Driver::<Hardware>::console_write(
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
