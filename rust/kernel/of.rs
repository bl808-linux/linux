// SPDX-License-Identifier: GPL-2.0

//! Device Tree / Open Firmware abstractions.

#[cfg(CONFIG_OF)]
use crate::device::Device;

use crate::{
    bindings,
    device_id::{RawDeviceId, RawDeviceIdIndex},
    prelude::*,
};

/// Returns the device's alias index for `stem`, or an error if no node or alias exists.
#[cfg(CONFIG_OF)]
pub fn alias_id(dev: &Device, stem: &CStr) -> Result<u32> {
    // SAFETY: dev holds a live device; its OF node remains valid during this call.
    let node = unsafe { (*dev.as_raw()).of_node };
    if node.is_null() {
        return Err(ENODEV);
    }
    // SAFETY: node and the null-terminated alias stem are valid for this call.
    let index = unsafe { bindings::of_alias_get_id(node, stem.as_char_ptr()) };
    if index < 0 {
        Err(Error::from_errno(index))
    } else {
        Ok(index as u32)
    }
}

/// IdTable type for OF drivers.
pub type IdTable<T> = &'static dyn kernel::device_id::IdTable<DeviceId, T>;

/// An open firmware device id.
#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct DeviceId(bindings::of_device_id);

// SAFETY: `DeviceId` is a `#[repr(transparent)]` wrapper of `struct of_device_id` and
// does not add additional invariants, so it's safe to transmute to `RawType`.
unsafe impl RawDeviceId for DeviceId {
    type RawType = bindings::of_device_id;
}

// SAFETY: `DRIVER_DATA_OFFSET` is the offset to the `data` field.
unsafe impl RawDeviceIdIndex for DeviceId {
    const DRIVER_DATA_OFFSET: usize = core::mem::offset_of!(bindings::of_device_id, data);
}

impl DeviceId {
    /// Create a new device id from an OF 'compatible' string.
    pub const fn new(compatible: &'static CStr) -> Self {
        let src = compatible.to_bytes_with_nul();
        // Replace with `bindings::of_device_id::default()` once stabilized for `const`.
        // SAFETY: FFI type is valid to be zero-initialized.
        let mut of: bindings::of_device_id = unsafe { core::mem::zeroed() };

        // TODO: Use `copy_from_slice` once stabilized for `const`.
        let mut i = 0;
        while i < src.len() {
            of.compatible[i] = src[i];
            i += 1;
        }

        Self(of)
    }
}

/// Create an OF `IdTable` with an "alias" for modpost.
#[macro_export]
macro_rules! of_device_table {
    ($($tt:tt)*) => {
        $crate::module_device_table!("of", $crate::of::DeviceId, $($tt)*);
    };
}
