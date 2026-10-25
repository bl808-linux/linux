// SPDX-License-Identifier: GPL-2.0
//! Built-in TTY lifetime bridges; these callbacks must never live in the UART module.
use super::DriverState;
use crate::{
    bindings,
    sync::{Arc, ArcBorrow},
    types::Opaque,
};
use core::{cell::UnsafeCell, mem::MaybeUninit, sync::atomic::Ordering};

struct Tables {
    operations: UnsafeCell<MaybeUninit<bindings::tty_operations>>,
    original: UnsafeCell<*const bindings::tty_operations>,
}
// SAFETY: Initialization is serialized by tty_mutex before ports are published.
// The tables are immutable after initialization and have kernel lifetime.
unsafe impl Sync for Tables {}
static TABLES: Tables = Tables {
    operations: UnsafeCell::new(MaybeUninit::uninit()),
    original: UnsafeCell::new(core::ptr::null()),
};

/// # Safety
/// The live TTY driver must embed this adapter's pinned uart_driver.
unsafe fn state(driver: *mut bindings::tty_driver) -> *const DriverState {
    // SAFETY: All participating tty_drivers use this embedded pinned uart_driver.
    unsafe {
        crate::container_of!(
            (*driver)
                .driver_state
                .cast::<Opaque<bindings::uart_driver>>(),
            DriverState,
            inner
        )
    }
}

/// # Safety
/// The caller retains this state and has not published any UART port.
#[no_mangle]
#[inline(never)]
pub(super) unsafe extern "C" fn rust_serial_install_hooks(shared: *mut DriverState) {
    // SAFETY: The public TTY mutex serializes table changes with tty_init_dev.
    unsafe {
        bindings::mutex_lock(core::ptr::addr_of_mut!(bindings::tty_mutex));
        let driver = (*(*shared).inner.get()).tty_driver;
        if (*TABLES.original.get()).is_null() {
            *TABLES.original.get() = (*driver).ops;
            let mut ops = core::ptr::read((*driver).ops);
            ops.install = Some(rust_serial_tty_install);
            ops.cleanup = Some(rust_serial_tty_cleanup);
            #[cfg(CONFIG_CONSOLE_POLL)]
            {
                ops.poll_init = None;
                ops.poll_get_char = None;
                ops.poll_put_char = None;
            }
            (*TABLES.operations.get()).write(ops);
        }
        (*driver).ops = (*TABLES.operations.get()).as_ptr();
        bindings::mutex_unlock(core::ptr::addr_of_mut!(bindings::tty_mutex));
    }
}

/// # Safety
/// The registration retains shared throughout this admission barrier.
#[no_mangle]
#[inline(never)]
pub(super) unsafe extern "C" fn rust_serial_stop_admission(shared: *mut DriverState) {
    // SAFETY: Excludes install's Arc acquisition before the owner releases its reference.
    unsafe {
        bindings::mutex_lock(core::ptr::addr_of_mut!(bindings::tty_mutex));
        (*shared).accepting.store(false, Ordering::Release);
        bindings::mutex_unlock(core::ptr::addr_of_mut!(bindings::tty_mutex));
    }
}

/// # Safety
/// All UART ports and retained TTY instances have released this registration.
#[no_mangle]
#[inline(never)]
pub(super) unsafe extern "C" fn rust_serial_unregister_state(driver: *mut bindings::uart_driver) {
    // SAFETY: Proc removal drains reads before uart_unregister_driver frees state.
    unsafe {
        #[cfg(CONFIG_PROC_FS)]
        bindings::proc_tty_unregister_driver((*driver).tty_driver);
        bindings::uart_unregister_driver(driver);
    }
}

/// # Safety
/// TTY core calls under tty_mutex with a fresh, live tty_struct.
#[no_mangle]
#[inline(never)]
unsafe extern "C" fn rust_serial_tty_install(
    driver: *mut bindings::tty_driver,
    tty: *mut bindings::tty_struct,
) -> i32 {
    // SAFETY: Owner teardown cannot pass tty_mutex while this callback runs.
    unsafe {
        let raw = state(driver);
        if !(*raw).accepting.load(Ordering::Acquire)
            || (*tty).index < 0
            || (*tty).index as u32 >= (*raw).ports
            || !(&(*raw).lines)[(*tty).index as usize].load(Ordering::Acquire)
        {
            return -(bindings::ENODEV as i32);
        }
        let retained: Arc<DriverState> = ArcBorrow::from_raw(raw).into();
        let owner = retained.owner;
        if !bindings::try_module_get(owner) {
            return -(bindings::ENODEV as i32);
        }
        let result = ((*(*TABLES.original.get())).install.unwrap())(driver, tty);
        if result != 0 {
            drop(retained);
            bindings::module_put(owner);
            return result;
        }
        // One reference per installed tty, including failures after install.
        let _tty_reference = Arc::into_raw(retained);
        0
    }
}

/// # Safety
/// TTY core calls once after release_tty has stopped using tty_port.
#[no_mangle]
#[inline(never)]
unsafe extern "C" fn rust_serial_tty_cleanup(tty: *mut bindings::tty_struct) {
    // SAFETY: Successful install retained this reference until this final cleanup.
    unsafe {
        let retained = Arc::from_raw(state((*tty).driver));
        let owner = retained.owner;
        if let Some(original) = (*(*TABLES.original.get())).cleanup {
            original(tty);
        }
        drop(retained);
        // Callback code is built-in: no instruction in the UART module follows this put.
        bindings::module_put(owner);
    }
}
