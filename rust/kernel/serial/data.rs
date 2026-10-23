// SPDX-License-Identifier: GPL-2.0

//! Data-path operations which require the serial-core UART lock.

use super::LockedPort;
use crate::bindings;

impl LockedPort<'_> {
    /// Services at most `limit` transmit bytes, including the priority x_char.
    ///
    /// The sink must not sleep. Serial-core wakeups and stop_tx are handled after
    /// the batch, following uart_port_tx_limited semantics.
    pub fn transmit(&self, limit: u32, mut put_char: impl FnMut(u8)) {
        // SAFETY: The port lock protects x_char, the transmit FIFO and counters. The helper
        // checks state before touching it; the sink cannot retain a borrowed FIFO reference.
        unsafe {
            let mut ch = 0;
            for _ in 0..limit {
                if !bindings::uart_tx_next(self.raw, &mut ch) {
                    break;
                }
                put_char(ch);
            }
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
