// SPDX-License-Identifier: GPL-2.0

#include <linux/serial_core.h>

__rust_helper
void rust_helper_uart_port_lock_irqsave(struct uart_port *port, unsigned long *flags)
{
	uart_port_lock_irqsave(port, flags);
}

__rust_helper
void rust_helper_uart_port_unlock_irqrestore(struct uart_port *port, unsigned long flags)
{
	uart_port_unlock_irqrestore(port, flags);
}

__rust_helper
unsigned int rust_helper_tty_get_char_size(tcflag_t cflag)
{
	return tty_get_char_size(cflag);
}

__rust_helper
speed_t rust_helper_tty_termios_baud_rate(const struct ktermios *termios)
{
	return tty_termios_baud_rate(termios);
}

__rust_helper
void rust_helper_uart_port_set_boot_autoconf(struct uart_port *port)
{
	port->flags = UPF_BOOT_AUTOCONF;
}
