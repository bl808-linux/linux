// SPDX-License-Identifier: GPL-2.0

#[crate::macros::kunit_tests(rust_spi)]
mod tests {
    use super::super::*;
    use crate::{
        faux,
        sync::{
            atomic::{Atomic, Relaxed},
            Arc, Completion,
        },
        workqueue,
    };
    use core::ptr::NonNull;

    #[pin_data]
    struct Stages {
        #[pin]
        release: Completion,
        #[pin]
        remove_start: Completion,
        entered: Atomic<bool>,
        removing: Atomic<bool>,
        removed: Atomic<bool>,
        submitted: Atomic<bool>,
        status: Atomic<i32>,
    }
    struct Counts {
        dropped: Atomic<u32>,
        calls: Atomic<u32>,
        bytes: Atomic<u32>,
        sum: Atomic<u32>,
        speed: Atomic<u32>,
        active: Atomic<u32>,
        bad_drop: Atomic<bool>,
        fail: Atomic<bool>,
    }
    struct Hardware {
        parent: ARef<Device>,
        counts: Arc<Counts>,
        stages: Option<Arc<Stages>>,
    }
    impl Operations for Hardware {
        fn transfer(&self, device: &Device<Bound>, bytes: &[u8], speed: u32) -> Result {
            if device.as_raw() != self.parent.as_raw() {
                return Err(ENODEV);
            }
            self.counts.active.fetch_add(1, Relaxed);
            if let Some(stages) = &self.stages {
                stages.entered.store(true, Relaxed);
                stages.release.wait_for_completion();
            }
            self.counts.calls.fetch_add(1, Relaxed);
            self.counts.bytes.fetch_add(bytes.len() as u32, Relaxed);
            self.counts
                .sum
                .fetch_add(bytes.iter().map(|b| u32::from(*b)).sum::<u32>(), Relaxed);
            self.counts.speed.store(speed, Relaxed);
            self.counts.active.fetch_sub(1, Relaxed);
            if self.counts.fail.load(Relaxed) {
                Err(EIO)
            } else {
                Ok(())
            }
        }
    }
    impl Drop for Hardware {
        fn drop(&mut self) {
            // Callback state must survive callbacks and controller device removal.
            if self.counts.active.load(Relaxed) != 0 || controller(&self.parent).is_some() {
                self.counts.bad_drop.store(true, Relaxed);
            }
            self.counts.dropped.fetch_add(1, Relaxed);
        }
    }

    fn counts() -> Arc<Counts> {
        Arc::new(
            Counts {
                dropped: Atomic::new(0),
                calls: Atomic::new(0),
                bytes: Atomic::new(0),
                sum: Atomic::new(0),
                speed: Atomic::new(0),
                active: Atomic::new(0),
                bad_drop: Atomic::new(false),
                fail: Atomic::new(false),
            },
            GFP_KERNEL,
        )
        .unwrap()
    }
    fn config() -> ControllerConfig {
        ControllerConfig {
            min_speed_hz: 100_000,
            max_speed_hz: 4_000_000,
        }
    }
    fn hardware(
        parent: &faux::Registration,
        counts: &Arc<Counts>,
        stages: Option<Arc<Stages>>,
    ) -> Hardware {
        Hardware {
            parent: parent.as_ref().as_ref().into(),
            counts: counts.clone(),
            stages,
        }
    }
    struct Controller(ARef<Device>);
    impl Controller {
        fn raw(&self) -> *mut bindings::spi_controller {
            // SAFETY: The sole faux child is the registered SPI controller's device.
            unsafe { crate::container_of!(self.0.as_raw(), bindings::spi_controller, dev) }
        }
    }
    fn controller(parent: &Device) -> Option<Controller> {
        unsafe extern "C" fn matches(
            _: *mut bindings::device,
            _: *const crate::ffi::c_void,
        ) -> i32 {
            1
        }
        // SAFETY: The faux test parent has no unrelated children. The finder returns
        // an owned device reference, consumed by ARef below even during removal.
        let raw = unsafe {
            bindings::device_find_child(parent.as_raw(), core::ptr::null(), Some(matches))
        };
        NonNull::new(raw).map(|raw| {
            Controller(unsafe { ARef::from_raw(NonNull::from(Device::from_raw(raw.as_ptr()))) })
        })
    }
    struct Peripheral(ARef<Device>);
    impl Peripheral {
        fn new(controller: &Controller) -> Self {
            let mut info = bindings::spi_board_info::default();
            for (slot, byte) in info.modalias.iter_mut().zip(b"rust-spi-kunit") {
                *slot = *byte as _;
            }
            info.max_speed_hz = 4_000_000;
            // SAFETY: This adds a software-only device on the fake controller.
            // SPI core owns registration; keep an extra reference until this handle drops.
            let spi = unsafe { bindings::spi_new_device(controller.raw(), &mut info) };
            if spi.is_null() {
                panic!("SPI test peripheral creation failed");
            }
            {
                let dev: &Device = unsafe { Device::from_raw(&raw mut (*spi).dev) };
                Self(dev.into())
            }
        }
        fn raw(&self) -> *mut bindings::spi_device {
            // SAFETY: The reference is the spi_device allocated by spi_new_device.
            unsafe { crate::container_of!(self.0.as_raw(), bindings::spi_device, dev) }
        }
        fn write(&self, bytes: &[u8], speed: u32) -> i32 {
            let mut xfer = bindings::spi_transfer::default();
            xfer.tx_buf = bytes.as_ptr().cast();
            xfer.len = bytes.len() as u32;
            xfer.speed_hz = speed;
            xfer.bits_per_word = 8;
            // SAFETY: SPI core completes synchronously before the borrowed buffer
            // and transfer go out of scope. This handle retains the device allocation.
            unsafe { bindings::spi_sync_transfer(self.raw(), &mut xfer, 1) }
        }
    }

    #[test]
    fn real_core_transfer_and_parent_removal_keep_state_alive() {
        let counts = counts();
        for _ in 0..4 {
            let parent = faux::Registration::new(c"rust-spi-parent", None).unwrap();
            register(
                parent.as_ref().as_ref(),
                config(),
                Ok(hardware(&parent, &counts, None)),
            )
            .unwrap();
            let controller = controller(parent.as_ref().as_ref()).unwrap();
            let spi = Peripheral::new(&controller);
            assert_eq!(spi.write(&[0x12, 0x34, 0x56], 2_000_000), 0);
            drop(spi);
            // Keep only our controller reference after parent removal. This
            // catches a leaked spi_alloc_host allocation reference as well.
            drop(parent);
            assert_eq!(
                unsafe { bindings::spi_controller_refcount(controller.raw()) },
                1
            );
        }
        assert_eq!(counts.calls.load(Relaxed), 4);
        assert_eq!(counts.bytes.load(Relaxed), 12);
        assert_eq!(counts.sum.load(Relaxed), 4 * (0x12 + 0x34 + 0x56));
        assert_eq!(counts.speed.load(Relaxed), 2_000_000);
        assert_eq!(counts.dropped.load(Relaxed), 4);
        assert!(!counts.bad_drop.load(Relaxed));
    }

    #[test]
    fn callback_error_reaches_spi_core_and_next_transfer_recovers() {
        let parent = faux::Registration::new(c"rust-spi-error", None).unwrap();
        let counts = counts();
        register(
            parent.as_ref().as_ref(),
            config(),
            Ok(hardware(&parent, &counts, None)),
        )
        .unwrap();
        let controller = controller(parent.as_ref().as_ref()).unwrap();
        let spi = Peripheral::new(&controller);
        counts.fail.store(true, Relaxed);
        assert_eq!(spi.write(&[1, 2], 1_000_000), EIO.to_errno());
        counts.fail.store(false, Relaxed);
        assert_eq!(spi.write(&[3], 1_000_000), 0);
        drop(parent);
        assert_eq!(counts.calls.load(Relaxed), 2);
        assert_eq!(counts.dropped.load(Relaxed), 1);
        assert!(!counts.bad_drop.load(Relaxed));
    }

    #[test]
    fn invalid_transfer_adapter_does_not_call_hardware() {
        let parent = faux::Registration::new(c"rust-spi-adapter", None).unwrap();
        let counts = counts();
        register(
            parent.as_ref().as_ref(),
            config(),
            Ok(hardware(&parent, &counts, None)),
        )
        .unwrap();
        let controller = controller(parent.as_ref().as_ref()).unwrap();
        let bytes = [7u8];
        let mut xfer = bindings::spi_transfer::default();
        xfer.tx_buf = bytes.as_ptr().cast();
        xfer.len = 1;
        xfer.speed_hz = 1_000_000;
        for bits in [0, 16, 32] {
            xfer.bits_per_word = bits;
            // SAFETY: A bound controller and a fully initialized transfer are retained.
            assert_eq!(
                unsafe { transfer::<Hardware>(controller.raw(), core::ptr::null_mut(), &mut xfer) },
                EOPNOTSUPP.to_errno()
            );
        }
        xfer.bits_per_word = 8;
        xfer.speed_hz = 0;
        assert_eq!(
            unsafe { transfer::<Hardware>(controller.raw(), core::ptr::null_mut(), &mut xfer) },
            EOPNOTSUPP.to_errno()
        );
        xfer.speed_hz = 1_000_000;
        xfer.rx_buf = core::ptr::dangling_mut::<u8>().cast();
        assert_eq!(
            unsafe { transfer::<Hardware>(controller.raw(), core::ptr::null_mut(), &mut xfer) },
            EOPNOTSUPP.to_errno()
        );
        xfer.rx_buf = core::ptr::null_mut();
        xfer.tx_buf = core::ptr::null();
        assert_eq!(
            unsafe { transfer::<Hardware>(controller.raw(), core::ptr::null_mut(), &mut xfer) },
            EINVAL.to_errno()
        );
        xfer.len = 0;
        assert_eq!(
            unsafe { transfer::<Hardware>(controller.raw(), core::ptr::null_mut(), &mut xfer) },
            0
        );
        assert_eq!(counts.calls.load(Relaxed), 0);
        drop(parent);
        assert!(!counts.bad_drop.load(Relaxed));
    }

    #[test]
    fn invalid_limits_release_unpublished_state() {
        let parent = faux::Registration::new(c"rust-spi-limits", None).unwrap();
        let counts = counts();
        for (min, max) in [(0, 1_000_000), (2_000_000, 1_000_000)] {
            assert_eq!(
                register(
                    parent.as_ref().as_ref(),
                    ControllerConfig {
                        min_speed_hz: min,
                        max_speed_hz: max
                    },
                    Ok(hardware(&parent, &counts, None))
                ),
                Err(EINVAL)
            );
        }
        assert_eq!(counts.dropped.load(Relaxed), 2);
        assert!(!counts.bad_drop.load(Relaxed));
        assert!(controller(parent.as_ref().as_ref()).is_none());
    }

    #[test]
    fn initialization_failure_does_not_publish_and_can_retry() {
        let parent = faux::Registration::new(c"rust-spi-init", None).unwrap();
        assert_eq!(
            register(
                parent.as_ref().as_ref(),
                config(),
                Err::<Hardware, Error>(ENOMEM)
            ),
            Err(ENOMEM)
        );
        assert!(controller(parent.as_ref().as_ref()).is_none());
        let counts = counts();
        register(
            parent.as_ref().as_ref(),
            config(),
            Ok(hardware(&parent, &counts, None)),
        )
        .unwrap();
        drop(parent);
        assert_eq!(counts.dropped.load(Relaxed), 1);
        assert!(!counts.bad_drop.load(Relaxed));
    }

    #[test]
    fn core_registration_failure_releases_unpublished_guard() {
        let parent = faux::Registration::new(c"rust-spi-register-fail", None).unwrap();
        let device: &Device<Bound> = parent.as_ref().as_ref();
        let counts = counts();
        let data = hardware(&parent, &counts, None);
        let state = KBox::pin_init(
            try_pin_init!(State { device: device.into(), data <- Ok::<_, Error>(data) }),
            GFP_KERNEL,
        )
        .unwrap();
        // SAFETY: The private guard owns a newly allocated controller and pinned state.
        let controller = unsafe { bindings::spi_alloc_host(device.as_raw()) };
        assert!(!controller.is_null());
        let guard = Registration {
            controller,
            registered: false,
            _state: state,
        };
        let ptr: *const State<Hardware> = &*guard._state;
        unsafe {
            bindings::spi_controller_init(
                controller,
                ptr.cast_mut().cast(),
                Some(transfer::<Hardware>),
                100_000,
                4_000_000,
            );
            // Zero chip selects is rejected before publication by actual SPI core.
            (*controller).num_chipselect = 0;
        }
        assert_eq!(
            unsafe { bindings::spi_register_controller(controller) },
            EINVAL.to_errno()
        );
        drop(guard);
        assert_eq!(counts.dropped.load(Relaxed), 1);
        assert!(!counts.bad_drop.load(Relaxed));
        assert!(self::controller(device).is_none());
    }

    fn wait(flag: &Atomic<bool>) -> bool {
        for _ in 0..200 {
            if flag.load(Relaxed) {
                return true;
            }
            // SAFETY: KUnit test and workqueue workers run in sleepable context.
            unsafe { bindings::msleep(10) };
        }
        false
    }

    #[test]
    fn parent_removal_drains_an_active_spi_core_transfer() {
        let stages = Arc::pin_init(
            pin_init!(Stages {
                release <- Completion::new(), remove_start <- Completion::new(),
                entered: Atomic::new(false), removing: Atomic::new(false),
                removed: Atomic::new(false), submitted: Atomic::new(false), status: Atomic::new(1),
            }),
            GFP_KERNEL,
        )
        .unwrap();
        let counts = counts();
        let parent = faux::Registration::new(c"rust-spi-drain", None).unwrap();
        register(
            parent.as_ref().as_ref(),
            config(),
            Ok(hardware(&parent, &counts, Some(stages.clone()))),
        )
        .unwrap();
        let controller = controller(parent.as_ref().as_ref()).unwrap();
        let spi = Peripheral::new(&controller);
        // SAFETY: The default queue is initialized and has system lifetime.
        let queue = unsafe { workqueue::Queue::from_raw(bindings::system_dfl_wq) };
        let removing = stages.clone();
        // Allocate removal work before any callback can block on release.
        queue
            .try_spawn(GFP_KERNEL, move || {
                removing.remove_start.wait_for_completion();
                removing.removing.store(true, Relaxed);
                drop(parent);
                removing.removed.store(true, Relaxed);
            })
            .unwrap();
        let submitting = stages.clone();
        let spawned = queue.try_spawn(GFP_KERNEL, move || {
            submitting
                .status
                .store(spi.write(&[0x5a, 0xc7], 1_000_000), Relaxed);
            submitting.submitted.store(true, Relaxed);
        });
        let entered = spawned.is_ok() && wait(&stages.entered);
        stages.remove_start.complete_all();
        let removing = wait(&stages.removing);
        unsafe { bindings::msleep(20) };
        let dropped_early = counts.dropped.load(Relaxed);
        let removed_early = stages.removed.load(Relaxed);
        // Always release blocked workers before any potentially aborting assertion.
        stages.release.complete_all();
        let removed = wait(&stages.removed);
        let submitted = spawned.is_ok() && wait(&stages.submitted);
        assert!(entered && removing && removed && submitted);
        assert_eq!(dropped_early, 0);
        assert!(!removed_early);
        assert_eq!(stages.status.load(Relaxed), 0);
        assert_eq!(counts.dropped.load(Relaxed), 1);
        assert!(!counts.bad_drop.load(Relaxed));
    }
}
