// SPDX-License-Identifier: GPL-2.0
//! Reset framework consumer references.
use crate::{
    bindings,
    device::Device,
    error::{from_err_ptr, to_result},
    prelude::*,
};

/// An exclusive reset consumer reference, independent of the provider language.
pub struct Reset(*mut bindings::reset_control);
// SAFETY: Reset core serializes provider operations and owns the referenced provider.
unsafe impl Send for Reset {}
// SAFETY: Shared access is synchronized by reset core/provider.
unsafe impl Sync for Reset {}
impl Reset {
    /// Gets the device's sole reset line.
    pub fn get(device: &Device) -> Result<Self> {
        // SAFETY: The device is live. The returned reference is released by Drop.
        Ok(Self(from_err_ptr(unsafe {
            bindings::reset_control_get_exclusive(device.as_raw())
        })?))
    }
    /// Pulses the reset using its provider's implementation.
    pub fn reset(&self) -> Result {
        // SAFETY: This object owns an exclusive reset reference.
        to_result(unsafe { bindings::reset_control_reset(self.0) })
    }
}
impl Drop for Reset {
    fn drop(&mut self) {
        // SAFETY: This reference was acquired exactly once and is no longer used.
        unsafe { bindings::reset_control_put(self.0) };
    }
}
