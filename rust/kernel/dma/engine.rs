// SPDX-License-Identifier: GPL-2.0

//! Minimal DMAengine support for a cyclic peripheral receive buffer.

use crate::{
    bindings,
    device::{Bound, Device},
    dma::Coherent,
    error::{from_err_ptr, to_result},
    prelude::*,
};
use core::{
    mem::ManuallyDrop,
    ptr::NonNull,
    sync::atomic::{AtomicI32, Ordering},
};

/// An owned channel and coherent cyclic receive allocation.
///
/// # Invariants
///
/// The supplier stays bound while the channel and consumer link exist. Running
/// callbacks are synchronized before the allocation or channel is released.
/// State-changing methods are serialized by the caller's open/close protocol.
pub struct CyclicRx<'bound> {
    channel: NonNull<bindings::dma_chan>,
    buffer: ManuallyDrop<Coherent<'bound, [u8]>>,
    cookie: AtomicI32,
    period: usize,
    _consumer: &'bound Device<Bound>,
}

// SAFETY: DMAengine serializes channel status. The unsafe start/stop contract
// serializes state changes, and completed-buffer reads have their own contract.
unsafe impl Send for CyclicRx<'_> {}
// SAFETY: No safe method exposes mutable buffer memory or descriptor state.
unsafe impl Sync for CyclicRx<'_> {}

impl<'bound> CyclicRx<'bound> {
    /// Requests an optional cyclic receive channel and allocates its buffer.
    ///
    /// # Safety
    ///
    /// The provider must remain bound until this object is dropped, and its
    /// synchronous termination must stop DMA and callbacks (including any
    /// device-specific reset fallback). The caller must not leak this object
    /// beyond the consumer's bound scope.
    pub unsafe fn request_optional(
        consumer: &'bound Device<Bound>,
        name: &CStr,
        length: usize,
        period: usize,
    ) -> Result<Option<Self>> {
        if length == 0 || period == 0 || length % period != 0 {
            return Err(EINVAL);
        }
        // SAFETY: The consumer is bound and name remains valid for this call.
        let raw = match unsafe {
            from_err_ptr(bindings::dma_request_chan(
                consumer.as_raw(),
                name.as_char_ptr(),
            ))
        } {
            Ok(channel) => channel,
            Err(ENODEV) => return Ok(None),
            Err(error) => return Err(error),
        };
        let channel = NonNull::new(raw).ok_or(ENODEV)?;
        let result = (|| {
            // SAFETY: The owned channel keeps its provider device alive; the
            // caller guarantees its bound lifetime. The managed link makes
            // supplier unbind remove this consumer before its DMA resources.
            let provider = unsafe { bindings::dmaengine_get_dma_device(raw) };
            let link = unsafe { bindings::device_link_add_consumer(consumer.as_raw(), provider) };
            if link.is_null() {
                return Err(ENOMEM);
            }
            // SAFETY: The constructor's provider-bound contract and channel
            // ownership justify this borrow until buffer destruction below.
            let provider: &'bound Device<Bound> = unsafe { Device::from_raw(provider) };
            let buffer = Coherent::zeroed_slice(provider, length, GFP_KERNEL)?;
            Ok(Some(Self {
                channel,
                buffer: ManuallyDrop::new(buffer),
                cookie: AtomicI32::new(0),
                period,
                _consumer: consumer,
            }))
        })();
        if result.is_err() {
            // SAFETY: No transfer has been prepared and this path owns the channel.
            unsafe { bindings::dma_release_channel(raw) };
        }
        result
    }

    /// Starts peripheral-to-memory transfers with a stable callback context.
    ///
    /// # Safety
    ///
    /// Calls to start and stop must be serialized. The callback context must
    /// remain valid until stop returns; the peripheral must use byte transfers
    /// and must not request DMA before initialization is complete.
    pub unsafe fn start(
        &self,
        source: bindings::dma_addr_t,
        callback: unsafe extern "C" fn(*mut crate::ffi::c_void),
        context: *mut crate::ffi::c_void,
    ) -> Result {
        if self.cookie.load(Ordering::Relaxed) != 0 {
            return Err(EBUSY);
        }
        let mut config = bindings::dma_slave_config::default();
        config.src_addr = source;
        config.src_addr_width = bindings::dma_slave_buswidth_DMA_SLAVE_BUSWIDTH_1_BYTE;
        config.dst_addr_width = bindings::dma_slave_buswidth_DMA_SLAVE_BUSWIDTH_1_BYTE;
        config.src_maxburst = 1;
        config.dst_maxburst = 1;
        // SAFETY: The owned channel is idle. DMAengine copies the configuration.
        to_result(unsafe { bindings::dmaengine_slave_config(self.channel.as_ptr(), &mut config) })?;
        // SAFETY: The coherent allocation belongs to this channel's provider
        // and outlives synchronous termination. The length is a period multiple.
        let desc = unsafe {
            bindings::dmaengine_prep_dma_cyclic(
                self.channel.as_ptr(),
                self.buffer.dma_address(),
                self.buffer.size(),
                self.period,
                bindings::dma_transfer_direction_DMA_DEV_TO_MEM,
                (bindings::dma_ctrl_flags_DMA_PREP_INTERRUPT
                    | bindings::dma_ctrl_flags_DMA_CTRL_ACK) as _,
            )
        };
        let desc = NonNull::new(desc).ok_or(ENOMEM)?;
        // SAFETY: The unsubmitted descriptor is exclusively owned here. The
        // callback's context validity is part of the caller's contract.
        let cookie = unsafe {
            (*desc.as_ptr()).callback = Some(callback);
            (*desc.as_ptr()).callback_param = context;
            bindings::dmaengine_submit(desc.as_ptr())
        };
        if cookie < 0 {
            // SAFETY: A failed submission did not publish an active transfer.
            unsafe { bindings::dmaengine_desc_free(desc.as_ptr()) };
            return to_result(cookie);
        }
        self.cookie.store(cookie, Ordering::Relaxed);
        // SAFETY: Initialization and cookie publication precede any callback.
        unsafe { bindings::dma_async_issue_pending(self.channel.as_ptr()) };
        Ok(())
    }

    /// Stops writes and synchronizes every callback. May sleep.
    ///
    /// # Safety
    /// Must be serialized with start, outside locks acquired by the callback.
    pub unsafe fn stop(&self) -> Result {
        if self.cookie.load(Ordering::Relaxed) == 0 {
            return Ok(());
        }
        // SAFETY: The channel is owned; the caller avoids callback lock inversion.
        to_result(unsafe { bindings::dmaengine_terminate_sync(self.channel.as_ptr()) })?;
        self.cookie.store(0, Ordering::Relaxed);
        Ok(())
    }

    /// Gets the producer's next-byte offset in the coherent receive ring.
    pub fn position(&self) -> Result<usize> {
        let cookie = self.cookie.load(Ordering::Relaxed);
        if cookie <= 0 {
            return Err(EINVAL);
        }
        let mut state = bindings::dma_tx_state::default();
        // SAFETY: The channel stays owned throughout the status query.
        let status =
            unsafe { bindings::dmaengine_tx_status(self.channel.as_ptr(), cookie, &mut state) };
        if status != bindings::dma_status_DMA_IN_PROGRESS
            || state.residue as usize > self.buffer.size()
        {
            return Err(EIO);
        }
        Ok((self.buffer.size() - state.residue as usize) % self.buffer.size())
    }

    /// Returns the receive ring size.
    pub fn len(&self) -> usize {
        self.buffer.size()
    }

    /// Copies a published, non-wrapping part of the ring into CPU-owned storage.
    ///
    /// # Safety
    /// DMA progress must have published the entire range. The caller must copy
    /// it before the device can wrap around and overwrite that range.
    pub unsafe fn read_completed(&self, offset: usize, bytes: &mut [u8]) {
        assert!(offset <= self.len() && bytes.len() <= self.len() - offset);
        // SAFETY: The publication and ownership contract orders the device's
        // completed writes before this read of the coherent allocation.
        unsafe {
            bindings::dma_rmb();
            core::ptr::copy_nonoverlapping(
                self.buffer.as_ptr().cast::<u8>().add(offset),
                bytes.as_mut_ptr(),
                bytes.len(),
            );
        }
    }
}

impl Drop for CyclicRx<'_> {
    fn drop(&mut self) {
        // SAFETY: Destruction is exclusive; callers arrange callback-safe lock
        // ordering. Failure violates the constructor's termination contract.
        assert!(
            unsafe { self.stop() }.is_ok(),
            "DMA did not stop before freeing its receive allocation"
        );
        // SAFETY: No device writes or callbacks can access the allocation now.
        // Free it while channel ownership still keeps its provider alive.
        unsafe {
            ManuallyDrop::drop(&mut self.buffer);
            bindings::dma_release_channel(self.channel.as_ptr());
        }
    }
}
