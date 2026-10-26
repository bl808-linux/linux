// SPDX-License-Identifier: GPL-2.0
//! SPI core abstractions for synchronous write-only host controllers.
//!
//! Hardware drivers implement [`Operations`] and call [`register`] with owned
//! callback state and their [`ControllerConfig`]. The adapter connects that
//! state to the C SPI core's `transfer_one` callback. SPI core continues to own
//! peripheral enumeration, GPIO chip select, message queues and delays.
//! Register access and FIFO handling belong to the hardware driver.
use crate::{
    bindings,
    device::{Bound, Device},
    devres,
    error::to_result,
    prelude::*,
    sync::aref::ARef,
};

/// Callbacks for an 8-bit mode-0 write-only controller. SPI core serializes transfers.
pub trait Operations: Send + Sync + 'static {
    /// Send all bytes and wait for the last bit, or quiesce the controller on failure.
    fn transfer(&self, device: &Device<Bound>, bytes: &[u8], speed_hz: u32) -> Result;
}
/// Clock limits advertised to SPI core by a host controller.
pub struct ControllerConfig {
    /// Lowest supported bus clock, in Hz.
    pub min_speed_hz: u32,
    /// Highest supported bus clock, in Hz.
    pub max_speed_hz: u32,
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
        // SAFETY: This guard owns the allocation reference. Unregister removes
        // peripherals and drains the queue. Synchronous immediate transfers use
        // io_mutex without setting the queue's busy flag, so also acquire that
        // mutex before freeing callback state or hardware resources. Peripheral
        // removal has already stopped consumers from submitting new work.
        unsafe {
            if self.registered {
                bindings::spi_unregister_controller(self.controller);
                bindings::mutex_lock(&raw mut (*self.controller).io_mutex);
                bindings::mutex_unlock(&raw mut (*self.controller).io_mutex);
            }
            // spi_unregister_controller() removes the device; it does not put
            // the allocation reference obtained from spi_alloc_host().
            bindings::spi_controller_put(self.controller);
        }
    }
}
/// Registers owned callback state, transferring removal to the parent's devres.
///
/// No caller-owned guard can be forgotten to retain borrowed hardware resources.
/// SPI core handles GPIO chip select, message ordering and transfer delays.
pub fn register<T: Operations>(
    device: &Device<Bound>,
    config: ControllerConfig,
    data: impl PinInit<T, Error>,
) -> Result {
    if config.min_speed_hz == 0 || config.max_speed_hz < config.min_speed_hz {
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
            config.min_speed_hz,
            config.max_speed_hz,
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
