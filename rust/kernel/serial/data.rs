// SPDX-License-Identifier: GPL-2.0

//! Data-path operations which require the serial-core UART lock.

use super::LockedPort;
use crate::bindings;

impl LockedPort<'_> {
    /// Moves a bounded batch from serial core into a hardware FIFO.
    ///
    /// The caller supplies scratch space no larger than the hardware's free FIFO
    /// capacity. The sink must not sleep and must consume the supplied bytes.
    /// Priority x_char, counters, wakeups and stopping an empty queue are paired.
    pub fn transmit(&self, buffer: &mut [u8], send: impl FnOnce(&[u8])) {
        // SAFETY: The UART lock protects the transmit FIFO and counters. buffer is
        // writable for its full length; the helper returns at most that length.
        unsafe {
            let count = bindings::uart_tx_fill(self.raw, buffer.as_mut_ptr(), buffer.len());
            send(&buffer[..count]);
            bindings::uart_tx_finish(self.raw);
        }
    }

    /// Receives bytes with a common TTY flag, accounting for software-buffer loss.
    pub fn receive(&self, bytes: &[u8], flag: u8) {
        // SAFETY: The UART lock serializes the counters and protects the active port state.
        // The helper checks state and copies bytes synchronously into the TTY buffer.
        unsafe { bindings::uart_receive(self.raw, bytes.as_ptr(), bytes.len(), flag) };
    }

    /// Reports a FIFO-wide parity latch, once for the interrupt event.
    pub fn parity_error(&self) {
        // SAFETY: The UART lock protects icount.
        unsafe { (*self.raw).icount.parity = (*self.raw).icount.parity.wrapping_add(1) };
    }

    /// Reports hardware FIFO overflow separately from software buffer exhaustion.
    pub fn overrun(&self, report: bool) {
        // SAFETY: The UART lock protects icount. state is checked before inserting a marker.
        unsafe {
            (*self.raw).icount.overrun = (*self.raw).icount.overrun.wrapping_add(1);
            let state = (*self.raw).state;
            if report && !state.is_null() {
                bindings::tty_insert_flip_char(
                    &raw mut (*state).port,
                    0,
                    bindings::TTY_OVERRUN as u8,
                );
            }
        }
    }

    /// Status bits selected by termios input checking.
    pub fn read_mask(&self) -> u32 {
        // SAFETY: The UART lock protects this field.
        unsafe { (*self.raw).read_status_mask }
    }

    /// Counts received characters intentionally discarded by termios input handling.
    pub fn discard(&self, count: u32) {
        // SAFETY: The UART lock protects icount.
        unsafe { (*self.raw).icount.rx = (*self.raw).icount.rx.wrapping_add(count) };
    }

    /// Status bits suppressed by termios input handling.
    pub fn ignore_mask(&self) -> u32 {
        // SAFETY: The UART lock protects this field.
        unsafe { (*self.raw).ignore_status_mask }
    }

    /// Makes newly received data available to the line discipline.
    pub fn push(&self) {
        // SAFETY: The UART lock protects the active state; no buffer reference escapes.
        unsafe {
            let state = (*self.raw).state;
            if !state.is_null() {
                bindings::tty_flip_buffer_push(&raw mut (*state).port);
            }
        }
    }
}
