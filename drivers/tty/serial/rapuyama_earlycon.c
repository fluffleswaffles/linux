// SPDX-License-Identifier: GPL-2.0
/*
 * Early console for the Nokia RapuYama UART (N8/E6/E7/C7)
 *
 * No public documentation exists, reversed from bc02_uart1.PDD of the E6
 * RM-609 firmware (the uart to the CSR BlueCore02 bluetooth chip, phys
 * 0x20025a00, test pads J6354 BTUARTIn = our tx and J6355 BTUARTOut = our
 * rx on the N8 board, 1.8 V levels).
 *
 *   0x04 active (1 while in use)   0x84 3 = tx + rx on    0x9c busy
 *   0x14 data bits (7 = 8 bit)     0x18 parity on         0x1c parity/stop
 *   0x2c rx fifo level             0x64 rx data (8..11 = errors)
 *   0xa0 tx fifo has room          0xa4 tx data
 *   baud from the 38.4 MHz ref: 0x08 0x0c 0x10 0x5c 0x60 0x88 0x8c 0xb8,
 *   computed exactly like the firmware does it (rapuyama_uart_set_baud)
 *
 * Power, clock and pin mux come from symbian (switch bluetooth on before
 * booting linux), this only sets the line up and pushes bytes out.
 *
 * earlycon=rapuyama,0x20025a00[,baud] keep_bootcon
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/serial_core.h>

#define RY_UART_ACTIVE		0x04
#define RY_UART_DIV_A		0x08
#define RY_UART_DIV_B		0x0c
#define RY_UART_DIV_C		0x10
#define RY_UART_BITS		0x14
#define RY_UART_PARITY		0x18
#define RY_UART_PARITY_TYPE	0x1c
#define RY_UART_SPECIAL		0x3c
#define RY_UART_DIV_D		0x5c
#define RY_UART_FRAC_A		0x60
#define RY_UART_ENABLE		0x84
#define RY_UART_DIV_Q1		0x88
#define RY_UART_DIV_Q		0x8c
#define RY_UART_BUSY		0x9c
#define RY_UART_TX_ROOM		0xa0
#define RY_UART_TX		0xa4
#define RY_UART_FRAC_B		0xb8

#define RY_UART_REF		38400000u
#define RY_UART_BITS_8		7
#define RY_UART_TX_RX_ON	3
#define RY_UART_SPIN		100000

static void __init rapuyama_uart_set_baud(void __iomem *b, u32 baud)
{
	u32 q = RY_UART_REF / baud;
	u32 r = RY_UART_REF % baud;
	u32 r2 = RY_UART_REF % (baud << 1);
	u32 a = (q - 3) >> 1;

	if (baud == 921600)
		writel(0, b + RY_UART_SPECIAL);
	else if (baud == 120000)
		writel(0x7fffff, b + RY_UART_SPECIAL);

	writel(a, b + RY_UART_DIV_A);
	writel(a / 5, b + RY_UART_DIV_D);
	writel(a + 1, b + RY_UART_DIV_B);
	writel(a, b + RY_UART_DIV_C);
	writel(q - 1, b + RY_UART_DIV_Q1);
	writel(q, b + RY_UART_DIV_Q);
	writel(r2 * (0x80000000u / baud) - 0x80000000u, b + RY_UART_FRAC_A);
	writel(r * (0x80000000u / (baud >> 1)), b + RY_UART_FRAC_B);
}

static void rapuyama_uart_putc(struct uart_port *port, unsigned char c)
{
	int n = RY_UART_SPIN;

	while (!readl(port->membase + RY_UART_TX_ROOM) && --n)
		cpu_relax();
	writel(c, port->membase + RY_UART_TX);
}

static void rapuyama_early_write(struct console *con, const char *s,
				 unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, rapuyama_uart_putc);
}

static int __init rapuyama_early_setup(struct earlycon_device *dev,
				       const char *opt)
{
	void __iomem *b = dev->port.membase;
	int n = RY_UART_SPIN;

	if (!b)
		return -ENODEV;

	while (readl(b + RY_UART_BUSY) == 1 && --n)
		cpu_relax();

	writel(1, b + RY_UART_ACTIVE);
	writel(RY_UART_TX_RX_ON, b + RY_UART_ENABLE);
	writel(RY_UART_BITS_8, b + RY_UART_BITS);
	writel(0, b + RY_UART_PARITY);
	writel(0, b + RY_UART_PARITY_TYPE);
	rapuyama_uart_set_baud(b, dev->baud ? dev->baud : 115200);

	dev->con->write = rapuyama_early_write;
	return 0;
}
EARLYCON_DECLARE(rapuyama, rapuyama_early_setup);
OF_EARLYCON_DECLARE(rapuyama, "nokia,rapuyama-uart", rapuyama_early_setup);