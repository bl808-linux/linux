// SPDX-License-Identifier: GPL-2.0

#include <linux/serial_core.h>
#include <linux/console.h>

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
void rust_helper_uart_port_set_boot_autoconf(struct uart_port *port)
{
	port->flags = UPF_BOOT_AUTOCONF | UPF_FIXED_PORT | UPF_FIXED_TYPE;
}

#ifdef CONFIG_SERIAL_CORE_CONSOLE
__rust_helper
bool rust_helper_uart_port_console_lock(struct uart_port *port, unsigned long *flags)
{
	if (oops_in_progress)
		return uart_port_trylock_irqsave(port, flags);
	uart_port_lock_irqsave(port, flags);
	return true;
}
#endif
