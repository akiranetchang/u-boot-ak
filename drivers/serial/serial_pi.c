// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Anup Patel <anup@brainfault.org>
 */

#include <common.h>
#include <clk.h>
#include <debug_uart.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <watchdog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/compiler.h>
#include <serial.h>
#include <linux/err.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_TXFIFO_FULL	0x80000000
#define UART_RXFIFO_EMPTY	0x80000000
#define UART_RXFIFO_DATA	0x000000ff
#define UART_TXCTRL_TXEN	0x1
#define UART_RXCTRL_RXEN	0x1

/* IP register */
#define UART_IP_RXWM            0x2

struct uart_pi {
	u32 txfifo;
	u32 rxfifo;
	u32 txctrl;
	u32 rxctrl;
	u32 ie;
	u32 ip;
	u32 div;
};

struct pi_uart_plat {
	unsigned long clock;
	struct uart_pi *regs;
};

/**
 * Find minimum divisor divides in_freq to max_target_hz;
 * Based on uart driver n SiFive FSBL.
 *
 * f_baud = f_in / (div + 1) => div = (f_in / f_baud) - 1
 * The nearest integer solution requires rounding up as to not exceed
 * max_target_hz.
 * div  = ceil(f_in / f_baud) - 1
 *	= floor((f_in - 1 + f_baud) / f_baud) - 1
 * This should not overflow as long as (f_in - 1 + f_baud) does not exceed
 * 2^32 - 1, which is unlikely since we represent frequencies in kHz.
 */
static inline unsigned int uart_min_clk_divisor(unsigned long in_freq,
						unsigned long max_target_hz)
{
	unsigned long quotient =
			(in_freq + max_target_hz - 1) / (max_target_hz);
	/* Avoid underflow */
	if (quotient == 0)
		return 0;
	else
		return quotient - 1;
}

/* Set up the baud rate in gd struct */
static void _pi_serial_setbrg(struct uart_pi *regs,
				  unsigned long clock, unsigned long baud)
{
	writel((uart_min_clk_divisor(clock, baud)), &regs->div);
}

static void _pi_serial_setbrg_rom(struct uart_pi *regs,
				  unsigned long clock, unsigned long baud)
{
#if 1 //def MIKEY
	writel((uart_min_clk_divisor(clock, baud)), &regs->div);
#else
	//writel(0x48, &regs->div);
	writel(0xE1, &regs->div);
#endif
}
static void _pi_serial_init(struct uart_pi *regs)
{
	writel(UART_TXCTRL_TXEN, &regs->txctrl);
	writel(UART_RXCTRL_RXEN, &regs->rxctrl);
	writel(0, &regs->ie);
}

static void _pi_serial_init_rom(struct uart_pi *regs)
{
	writel(UART_TXCTRL_TXEN, &regs->txctrl);
	writel(UART_RXCTRL_RXEN, &regs->rxctrl);
	writel(0, &regs->ie);
}
static int _pi_serial_putc(struct uart_pi *regs, const char c)
{
	if (readl(&regs->txfifo) & UART_TXFIFO_FULL)
		return -EAGAIN;

	writel(c, &regs->txfifo);

	return 0;
}

static int _pi_serial_getc(struct uart_pi *regs)
{
	int ch = readl(&regs->rxfifo);

	if (ch & UART_RXFIFO_EMPTY)
		return -EAGAIN;
	ch &= UART_RXFIFO_DATA;

	return ch;
}

static int pi_serial_setbrg(struct udevice *dev, int baudrate)
{
	int ret;
	struct clk clk;
	struct pi_uart_plat *plat = dev_get_plat(dev);
	u32 clock = 0;

	ret = clk_get_by_index(dev, 0, &clk);
	if (IS_ERR_VALUE(ret)) {
		debug("SiFive UART failed to get clock\n");
		ret = dev_read_u32(dev, "clock-frequency", &clock);
		if (IS_ERR_VALUE(ret)) {
			debug("SiFive UART clock not defined\n");
			return 0;
		}
	} else {
		clock = clk_get_rate(&clk);
		if (IS_ERR_VALUE(clock)) {
			debug("SiFive UART clock get rate failed\n");
			return 0;
		}
	}
	plat->clock = clock;
	_pi_serial_setbrg(plat->regs, plat->clock, baudrate);

	return 0;
}

static int pi_serial_probe(struct udevice *dev)
{
	struct pi_uart_plat *plat = dev_get_plat(dev);

	/* No need to reinitialize the UART after relocation */
	if (gd->flags & GD_FLG_RELOC)
		return 0;

	_pi_serial_init(plat->regs);

	return 0;
}

static int pi_serial_getc(struct udevice *dev)
{
	int c;
	struct pi_uart_plat *plat = dev_get_plat(dev);
	struct uart_pi *regs = plat->regs;

	while ((c = _pi_serial_getc(regs)) == -EAGAIN) ;

	return c;
}

static int pi_serial_putc(struct udevice *dev, const char ch)
{
	int rc;
	struct pi_uart_plat *plat = dev_get_plat(dev);

	while ((rc = _pi_serial_putc(plat->regs, ch)) == -EAGAIN) ;

	return rc;
}

static int pi_serial_pending(struct udevice *dev, bool input)
{
	struct pi_uart_plat *plat = dev_get_plat(dev);
	struct uart_pi *regs = plat->regs;

	if (input)
		return (readl(&regs->ip) & UART_IP_RXWM);
	else
		return !!(readl(&regs->txfifo) & UART_TXFIFO_FULL);
}

static int pi_serial_of_to_plat(struct udevice *dev)
{
	struct pi_uart_plat *plat = dev_get_plat(dev);

	plat->regs = (struct uart_pi *)(uintptr_t)dev_read_addr(dev);
	if (IS_ERR(plat->regs))
		return PTR_ERR(plat->regs);

	return 0;
}

static const struct dm_serial_ops pi_serial_ops = {
	.putc = pi_serial_putc,
	.getc = pi_serial_getc,
	.pending = pi_serial_pending,
	.setbrg = pi_serial_setbrg,
};

static const struct udevice_id pi_serial_ids[] = {
	{ .compatible = "pi,uart0" },
	{ .compatible = "sifive,uart0" },
	{ }
};

U_BOOT_DRIVER(serial_pi) = {
	.name	= "serial_pi",
	.id	= UCLASS_SERIAL,
	.of_match = pi_serial_ids,
	.of_to_plat = pi_serial_of_to_plat,
	.plat_auto	= sizeof(struct pi_uart_plat),
	.probe = pi_serial_probe,
	.ops	= &pi_serial_ops,
};

#ifdef CONFIG_DEBUG_UART_PI
static inline void _debug_uart_init(void)
{
	struct uart_pi *regs =
			(struct uart_pi *)CONFIG_VAL(DEBUG_UART_BASE);

#ifdef MIKEY
#else
	_pi_serial_setbrg_rom(regs, CONFIG_DEBUG_UART_CLOCK,
			      CONFIG_BAUDRATE);
	_pi_serial_init_rom(regs);
#endif
}

static inline void _debug_uart_putc(int ch)
{
	struct uart_pi *regs =
			(struct uart_pi *)CONFIG_VAL(DEBUG_UART_BASE);

	while (_pi_serial_putc(regs, ch) == -EAGAIN)
		WATCHDOG_RESET();
}

DEBUG_UART_FUNCS

#endif
