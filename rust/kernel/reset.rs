// SPDX-License-Identifier: GPL-2.0

//! Optional exclusive peripheral resets.

use crate::{
    bindings,
    device::Device,
    error::{from_err_ptr, to_result},
    prelude::*,
};

/// An optional exclusive reset handle. Absence of a reset is represented by NULL.
///
/// The handle owns the reference returned by reset_control_get_optional_exclusive.
/// Dropping the handle releases ownership without changing the hardware state.
pub struct OptionalReset(*mut bindings::reset_control);

// SAFETY: Reset handles may be released from another thread.
unsafe impl Send for OptionalReset {}
// SAFETY: The reset core serializes access; exclusive ownership excludes other consumers.
unsafe impl Sync for OptionalReset {}

impl OptionalReset {
    /// Gets the device's unnamed, optional exclusive reset.
    pub fn get_exclusive(dev: &Device) -> Result<Self> {
        // SAFETY: dev is valid; NULL is the reset API's representation of an absent reset.
        Ok(Self(from_err_ptr(unsafe {
            bindings::reset_control_get_optional_exclusive(dev.as_raw(), core::ptr::null())
        })?))
    }

    /// Deasserts the reset without first asserting it.
    pub fn deassert(&self) -> Result {
        // SAFETY: The owned reset handle (including NULL) is accepted by the reset API.
        to_result(unsafe { bindings::reset_control_deassert(self.0) })
    }
}

impl Drop for OptionalReset {
    fn drop(&mut self) {
        // SAFETY: Releases exactly the reference acquired at construction. NULL is accepted.
        unsafe { bindings::reset_control_put(self.0) };
    }
}
