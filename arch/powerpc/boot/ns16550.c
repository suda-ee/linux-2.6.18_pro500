/*
 * 16550 serial console support.
 *
 * Original copied from <file:arch/ppc/boot/common/ns16550.c>
 * (which had no copyright)
 * Modifications: 2006 (c) MontaVista Software, Inc.
 *
 * Modified by: Mark A. Greer <mgreer@mvista.com>
 */
#include <stdarg.h>
#include <stddef.h>
#include "types.h"
#include "string.h"
#include "stdio.h"
#include "io.h"
#include "ops.h"

#define UART_DLL	0	/* Out: Divisor Latch Low */
#define UART_DLM	1	/* Out: Divisor Latch High */
#define UART_FCR	2	/* Out: FIFO Control Register */
#define UART_LCR	3	/* Out: Line Control Register */
#define UART_MCR	4	/* Out: Modem Control Register */
#define UART_LSR	5	/* In:  Line Status Register */
#define UART_LSR_THRE	0x20	/* Transmit-hold-register empty */
#define UART_LSR_DR	0x01	/* Receiver data ready */
#define UART_MSR	6	/* In:  Modem Status Register */
#define UART_SCR	7	/* I/O: Scratch Register */

static unsigned char *reg_base;
static u32 reg_shift;
static unsigned int clk,spd;

static int ns16550_open(void)
{
	static unsigned char lcr, dlm;
	/* save the LCR */
	lcr = in_8(reg_base + (UART_LCR << reg_shift));
	/* Access baud rate */
	out_8(reg_base + (UART_LCR << reg_shift), 0x80);
	dlm = in_8(reg_base + (UART_DLM << reg_shift));
	if ((dlm <= 4) && (lcr & 2))
		/* port is configured, put the old LCR back */
		out_8(reg_base + (UART_LCR << reg_shift), lcr);
	else {
		/* Input clock. */
		out_8(reg_base + (UART_DLL << reg_shift),
		(clk / (16 * spd)) & 0xFF);
		out_8(reg_base + (UART_DLM << reg_shift),
		(clk / (16 * spd)) >> 8);
		/* 8 data, 1 stop, no parity */
		out_8(reg_base + (UART_LCR << reg_shift), 0x03);
		/* RTS/DTR */
		out_8(reg_base + (UART_MCR << reg_shift), 0x03);
	}
	out_8(reg_base + (UART_FCR << reg_shift), 0x07);
	return 0;
}

static void ns16550_putc(unsigned char c)
{
	while ((in_8(reg_base + (UART_LSR << reg_shift)) & UART_LSR_THRE) == 0);
	out_8(reg_base, c);
}

static unsigned char ns16550_getc(void)
{
	while ((in_8(reg_base + (UART_LSR << reg_shift)) & UART_LSR_DR) == 0);
	return in_8(reg_base);
}

static u8 ns16550_tstc(void)
{
	return ((in_8(reg_base + (UART_LSR << reg_shift)) & UART_LSR_DR) != 0);
}

int ns16550_console_init(void *devp, struct serial_console_data *scdp)
{
	u32 reg[2];
	int n;
	unsigned long reg_phys;

	n = getprop(devp, "virtual-reg", &reg, sizeof(reg)) / 4;
	if (n < 1) {
		if (!dt_xlate_reg(devp, 0, &reg_phys, NULL))
			return -1;

		reg_base = (void *)reg_phys;
	} else
		reg_base = (unsigned char *)((n == 1) ? reg[0] : reg[0] | reg[1]);

	n = getprop(devp, "current-speed", &spd, sizeof(spd));
	if (n != sizeof(spd) || !spd)
		return -1;

	n = getprop(devp, "clock-frequency", &clk, sizeof(clk));
	if (n != sizeof(clk))
		return -1;

	n = getprop(devp, "reg-shift", &reg_shift, sizeof(reg_shift));
	if (n != sizeof(reg_shift))
		reg_shift = 0;

	scdp->open = ns16550_open;
	scdp->putc = ns16550_putc;
	scdp->getc = ns16550_getc;
	scdp->tstc = ns16550_tstc;
	scdp->close = NULL;

	return 0;
}
