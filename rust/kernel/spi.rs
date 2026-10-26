// SPDX-License-Identifier: GPL-2.0
//! Synchronous write-only SPI controller registration and resource consumers.
use crate::{
    bindings,
    device::{Bound, Device},
    devres,
    error::{from_err_ptr, to_result},
    prelude::*,
    sync::aref::ARef,
};

/// Callbacks for an 8-bit mode-0 write-only controller. SPI core serializes transfers.
pub trait Operations: Send + Sync + 'static {
    /// Send all bytes and wait for the last bit, or quiesce the controller on failure.
    fn transfer(&self, device: &Device<Bound>, bytes: &[u8], speed_hz: u32) -> Result;
}
#[pin_data]
struct State<T: Operations> {
    device: ARef<Device>,
    #[pin]
    data: T,
}
struct Registration<T: Operations> {
    controller: *mut bindings::spi_controller,
    registered: bool,
    _state: Pin<KBox<State<T>>>,
}
// SAFETY: C SPI core synchronizes its controller. State is pinned and Send + Sync.
unsafe impl<T: Operations> Send for Registration<T> {}
impl<T: Operations> Drop for Registration<T> {
    fn drop(&mut self) {
        // SAFETY: This guard owns the allocation/reference. Unregister first removes
        // peripherals and drains queued callbacks, while state and devres are live.
        unsafe {
            if self.registered {
                bindings::spi_unregister_controller(self.controller);
            } else {
                bindings::spi_controller_put(self.controller);
            }
        }
    }
}
/// Registers owned callback state, transferring removal to the parent's devres.
///
/// No caller-owned guard can be forgotten to retain borrowed hardware resources.
/// SPI core handles GPIO chip select, message ordering and transfer delays.
pub fn register<T: Operations>(
    device: &Device<Bound>,
    max_hz: u32,
    data: impl PinInit<T, Error>,
) -> Result {
    if max_hz < 100_000 {
        return Err(EINVAL);
    }
    let state = KBox::pin_init(
        try_pin_init!(State { device: device.into(), data <- data }),
        GFP_KERNEL,
    )?;
    // SAFETY: Allocation is owned by the private guard until devres takes it.
    let controller = unsafe { bindings::spi_alloc_host(device.as_raw()) };
    if controller.is_null() {
        return Err(ENOMEM);
    }
    let mut registration = Registration {
        controller,
        registered: false,
        _state: state,
    };
    let ptr: *const State<T> = &*registration._state;
    // SAFETY: Callback type and private state match, and the pinned state is fully
    // initialized before publication. Registration owns it until unregistration.
    unsafe {
        bindings::spi_controller_init(
            controller,
            ptr.cast_mut().cast(),
            Some(transfer::<T>),
            max_hz,
        );
        to_result(bindings::spi_register_controller(controller))?;
    }
    registration.registered = true;
    devres::register(device, registration, GFP_KERNEL)
}
unsafe extern "C" fn transfer<T: Operations>(
    controller: *mut bindings::spi_controller,
    _spi: *mut bindings::spi_device,
    xfer: *mut bindings::spi_transfer,
) -> i32 {
    // SAFETY: SPI core provides a live transfer and retains the registered state
    // for this synchronous callback. References cannot escape this call.
    unsafe {
        let xfer = &*xfer;
        if xfer.bits_per_word != 8 || !xfer.rx_buf.is_null() || xfer.speed_hz == 0 {
            return EOPNOTSUPP.to_errno();
        }
        if xfer.len == 0 {
            return 0;
        }
        if xfer.tx_buf.is_null() {
            return EINVAL.to_errno();
        }
        let state = &*bindings::spi_controller_get_devdata(controller).cast::<State<T>>();
        let bytes = core::slice::from_raw_parts(xfer.tx_buf.cast::<u8>(), xfer.len as usize);
        let result = state
            .data
            .transfer(state.device.as_bound(), bytes, xfer.speed_hz);
        result.map_or_else(|error| error.to_errno(), |_| 0)
    }
}
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
