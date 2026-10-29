// SPDX-License-Identifier: GPL-2.0

#include <linux/serial_core.h>
#include <linux/console.h>

#ifdef CONFIG_RUST_SERIAL_KUNIT_TEST
struct uart_port *rust_serial_test_trylock_port;
#endif

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
	bool trylock = oops_in_progress;

#ifdef CONFIG_RUST_SERIAL_KUNIT_TEST
	trylock |= READ_ONCE(rust_serial_test_trylock_port) == port;
#endif
	if (trylock)
		return uart_port_trylock_irqsave(port, flags);
	uart_port_lock_irqsave(port, flags);
	return true;
}
#endif

#ifdef CONFIG_RUST_SERIAL_KUNIT_TEST
/* Select the emergency trylock path for one inactive test port only. */
__rust_helper
void rust_helper_uart_port_test_set_console_trylock(struct uart_port *port)
{
	WRITE_ONCE(rust_serial_test_trylock_port, port);
}

__rust_helper
bool rust_helper_uart_port_test_irqs_disabled(void)
{
	return irqs_disabled();
}

/* Check the context supplied by the Rust adapters, including non-lockdep builds. */
__rust_helper
bool rust_helper_uart_port_test_locked(struct uart_port *port)
{
	lockdep_assert_held(&port->lock);
	return irqs_disabled() &&
		(!IS_ENABLED(CONFIG_SMP) || spin_is_locked(&port->lock));
}
#endif
