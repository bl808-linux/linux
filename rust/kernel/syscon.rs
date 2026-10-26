// SPDX-License-Identifier: GPL-2.0
//! Syscon register map consumer access.
use crate::{
    bindings,
    device::Device,
    error::{from_err_ptr, to_result},
    prelude::*,
};

/// A syscon regmap. Syscon keeps its mappings alive for the system lifetime.
pub struct Syscon(*mut bindings::regmap);
// SAFETY: Regmap synchronizes register access; syscon mappings are never freed.
unsafe impl Send for Syscon {}
// SAFETY: Same synchronization applies to concurrent accesses.
unsafe impl Sync for Syscon {}
impl Syscon {
    /// Resolves a syscon phandle property on this device.
    pub fn get(device: &Device, property: &CStr) -> Result<Self> {
        // SAFETY: The property string and device are live throughout lookup.
        Ok(Self(from_err_ptr(unsafe {
            bindings::syscon_lookup(device.as_raw(), property.as_char_ptr())
        })?))
    }
    /// Atomically updates selected bits in a register bounded by this regmap.
    pub fn update_bits(&self, register: u32, mask: u32, value: u32) -> Result {
        // SAFETY: Syscon owns a live map; regmap checks offsets and serializes RMW.
        to_result(unsafe { bindings::regmap_update_bits(self.0, register, mask, value) })
    }
}
