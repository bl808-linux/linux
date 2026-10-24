// SPDX-License-Identifier: GPL-2.0

#include <linux/serial_core.h>
#include <linux/tty_flip.h>

__rust_helper
bool rust_helper_uart_tx_next(struct uart_port *port, unsigned char *ch)
{
	if (!port->state)
		return false;
	if (port->x_char) {
		*ch = port->x_char;
		port->x_char = 0;
		port->icount.tx++;
		return true;
	}
	return !uart_tx_stopped(port) && uart_fifo_get(port, ch);
}

__rust_helper
void rust_helper_uart_tx_finish(struct uart_port *port)
{
	unsigned int pending;

	if (!port->state)
		return;
	pending = kfifo_len(&port->state->port.xmit_fifo);
	if (pending < WAKEUP_CHARS)
		uart_write_wakeup(port);
	if (!pending || uart_tx_stopped(port))
		port->ops->stop_tx(port);
}

__rust_helper
void rust_helper_uart_receive(struct uart_port *port, const unsigned char *data,
			      size_t count, unsigned char flag)
{
	size_t copied;

	if (!port->state)
		return;
	port->icount.rx += count;
	copied = tty_insert_flip_string_fixed_flag(&port->state->port, data,
						 flag, count);
	port->icount.buf_overrun += count - copied;
}

__rust_helper
int rust_helper_tty_insert_flip_char(struct tty_port *port, unsigned char ch,
				     unsigned char flag)
{
	return tty_insert_flip_char(port, ch, flag);
}

__rust_helper
int rust_helper_uart_prepare_receive(struct uart_port *port)
{
	return tty_buffer_prealloc(&port->state->port, 8192);
}
