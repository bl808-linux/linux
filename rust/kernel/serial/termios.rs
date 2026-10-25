// SPDX-License-Identifier: GPL-2.0

//! Termios views and serial-core configuration under the UART lock.

use super::LockedPort;
use crate::bindings;
use core::num::NonZeroU32;

/// Termios borrowed for a serial-core set_termios callback.
#[repr(transparent)]
pub struct Termios(bindings::ktermios);

impl Termios {
    /// Borrows a termios supplied by serial core.
    ///
    /// # Safety
    ///
    /// `ptr` must point to an initialized `ktermios` that remains valid and is
    /// not mutated for the returned borrow's lifetime.
    #[inline]
    pub(super) unsafe fn from_raw<'a>(ptr: *const bindings::ktermios) -> &'a Self {
        // SAFETY: The caller guarantees a valid shared borrow. Self is transparent.
        unsafe { &*ptr.cast() }
    }

    /// Exclusively borrows a termios supplied by serial core.
    ///
    /// # Safety
    ///
    /// `ptr` must point to an initialized `ktermios` that remains valid and is
    /// exclusively accessible for the returned borrow's lifetime.
    #[inline]
    pub(super) unsafe fn from_raw_mut<'a>(ptr: *mut bindings::ktermios) -> &'a mut Self {
        // SAFETY: The caller guarantees an exclusive borrow. Self is transparent.
        unsafe { &mut *ptr.cast() }
    }

    /// Whether parity generation and checking are enabled.
    #[inline]
    pub fn parity_enabled(&self) -> bool {
        self.0.c_cflag & bindings::PARENB != 0
    }

    /// Whether odd parity, or mark parity in mark/space mode, is selected.
    #[inline]
    pub fn odd_parity(&self) -> bool {
        self.0.c_cflag & bindings::PARODD != 0
    }

    /// Whether two stop bits are requested.
    #[inline]
    pub fn two_stop_bits(&self) -> bool {
        self.0.c_cflag & bindings::CSTOPB != 0
    }

    /// Disables mark/space parity without changing parity enable or odd/even selection.
    #[inline]
    pub fn disable_mark_space_parity(&mut self) {
        self.0.c_cflag &= !bindings::CMSPAR;
    }

    /// Disables RTS/CTS hardware flow control.
    #[inline]
    pub fn disable_hardware_flow_control(&mut self) {
        self.0.c_cflag &= !bindings::CRTSCTS;
    }

    /// Data bits per character.
    #[inline]
    pub fn char_size(&self) -> u32 {
        // SAFETY: c_cflag is a valid termios control flag value.
        unsafe { bindings::tty_get_char_size(self.0.c_cflag).into() }
    }
}

impl LockedPort<'_> {
    /// Selects a supported baud rate using serial-core termios rules.
    ///
    /// Returns `None` if serial core fails to select a rate. Termios changes
    /// made by serial core are preserved, including on failure.
    #[inline]
    pub fn baud_rate(
        &self,
        new: &mut Termios,
        old: Option<&Termios>,
        min: u32,
        max: u32,
    ) -> Option<NonZeroU32> {
        // SAFETY: The port is locked, and the termios references are valid for this callback.
        let baud = unsafe {
            bindings::uart_get_baud_rate(
                self.raw,
                &mut new.0,
                old.map_or(core::ptr::null(), |t| &t.0),
                min,
                max,
            )
        };
        NonZeroU32::new(baud)
    }

    /// Updates serial-core timeout and the actual console baud rate.
    #[inline]
    pub fn configure(&self, new: &mut Termios, baud: NonZeroU32) {
        let baud = baud.get();
        // SAFETY: All port fields are accessed under the UART lock. new is exclusively borrowed.
        unsafe {
            (*self.raw).read_status_mask = 4 | if new.0.c_iflag & bindings::INPCK != 0 {
                3
            } else {
                0
            };
            (*self.raw).ignore_status_mask = (if new.0.c_iflag & bindings::IGNPAR != 0 {
                3
            } else {
                0
            }) | if new.receiver_enabled() { 0 } else { 0x100 };
            bindings::uart_update_timeout(self.raw, new.0.c_cflag, baud);
            if !self.preserve_b0 {
                bindings::tty_termios_encode_baud_rate(&mut new.0, baud, baud);
            }
        }
    }
}

impl Termios {
    /// Whether reception is enabled.
    #[inline]
    pub fn receiver_enabled(&self) -> bool {
        self.0.c_cflag & bindings::CREAD != 0
    }
}
