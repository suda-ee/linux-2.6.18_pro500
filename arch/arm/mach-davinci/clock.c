/*
 * TI DaVinci clock config file
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/io.h>

#include <asm/arch/psc.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mux.h>
#include "clock.h"

/* PLL/Reset register offsets */
#define PLLM		0x110

/* PSC register offsets */
#define EPCPR		0x070
#define PTCMD		0x120
#define PTSTAT		0x128
#define PDSTAT		0x200
#define PDCTL1		0x304
#define MDSTAT		0x800
#define MDCTL		0xA00

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clockfw_lock);

static unsigned int commonrate;
static unsigned int div_by_four;
static unsigned int div_by_six;
static unsigned int div_by_eight;
static unsigned int armrate;
static unsigned int fixedrate = 27000000;	/* 27 MHZ */

void (*davinci_pinmux_setup)(unsigned int id);

/* Enable or disable a PSC domain */
void davinci_psc_config(unsigned int domain, unsigned int id, char enable)
{
	u32 epcpr, ptcmd, ptstat, pdstat, pdctl1, mdstat, mdctl, mdstat_mask;

	if (id < 0)
		return;

	mdctl = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE + MDCTL + 4 * id);
	if (enable)
		mdctl |= 0x00000003;	/* Enable Module */
	else
		mdctl &= 0xFFFFFFF2;	/* Disable Module */
	davinci_writel(mdctl, DAVINCI_PWR_SLEEP_CNTRL_BASE + MDCTL + 4 * id);

	pdstat = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE + PDSTAT);
	if ((pdstat & 0x00000001) == 0) {
		pdctl1 = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE + PDCTL1);
		pdctl1 |= 0x1;
		davinci_writel(pdctl1, DAVINCI_PWR_SLEEP_CNTRL_BASE + PDCTL1);

		ptcmd = 1 << domain;
		davinci_writel(ptcmd, DAVINCI_PWR_SLEEP_CNTRL_BASE + PTCMD);

		do {
			epcpr = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE +
					      EPCPR);
		} while ((((epcpr >> domain) & 1) == 0));

		pdctl1 = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE + PDCTL1);
		pdctl1 |= 0x100;
		davinci_writel(pdctl1, DAVINCI_PWR_SLEEP_CNTRL_BASE + PDCTL1);

		do {
			ptstat = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE +
					       PTSTAT);
		} while (!(((ptstat >> domain) & 1) == 0));
	} else {
		ptcmd = 1 << domain;
		davinci_writel(ptcmd, DAVINCI_PWR_SLEEP_CNTRL_BASE + PTCMD);

		do {
			ptstat = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE +
					       PTSTAT);
		} while (!(((ptstat >> domain) & 1) == 0));
	}

	if (enable)
		mdstat_mask = 0x3;
	else
		mdstat_mask = 0x2;

	do {
		mdstat = davinci_readl(DAVINCI_PWR_SLEEP_CNTRL_BASE +
				       MDSTAT + 4 * id);
	} while (!((mdstat & 0x0000001F) == mdstat_mask));

	if (enable && davinci_pinmux_setup)
		davinci_pinmux_setup(id);
}

/*
 * Returns a clock. Note that we first try to use device id on the bus
 * and clock name. If this fails, we try to use clock name only.
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);
	int idno;

	if (dev == NULL || dev->bus != &platform_bus_type)
		idno = -1;
	else
		idno = to_platform_device(dev)->id;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(p, &clocks, node) {
		if (p->id == idno &&
		    strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			goto found;
		}
	}

	list_for_each_entry(p, &clocks, node) {
		if (strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			break;
		}
	}

found:
	mutex_unlock(&clocks_mutex);

	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
	if (clk && !IS_ERR(clk))
		module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);

static int __clk_enable(struct clk *clk)
{
	if (clk->flags & ALWAYS_ENABLED)
		return 0;

	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, clk->lpsc, 1);
	return 0;
}

static void __clk_disable(struct clk *clk)
{
	if (clk->usecount)
		return;

	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, clk->lpsc, 0);
}

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;

	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if (clk->usecount++ == 0) {
		spin_lock_irqsave(&clockfw_lock, flags);
		ret = __clk_enable(clk);
		spin_unlock_irqrestore(&clockfw_lock, flags);
	}

	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return;

	if (clk->usecount > 0 && !(--clk->usecount)) {
		spin_lock_irqsave(&clockfw_lock, flags);
		__clk_disable(clk);
		spin_unlock_irqrestore(&clockfw_lock, flags);
	}
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	return *(clk->rate);
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	return *(clk->rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	/* changing the clk rate is not supported */
	return -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_register(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	list_add(&clk->node, &clocks);
	mutex_unlock(&clocks_mutex);

	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);

static struct clk davinci_dm644x_clks[] = {
	{
		.name = "ARMCLK",
		.rate = &armrate,
		.lpsc = -1,
		.flags = ALWAYS_ENABLED,
	},
	{
		.name = "UART0",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_UART0,
	},
	{
		.name = "EMACCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_EMAC_WRAPPER,
	},
	{
		.name = "I2CCLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_I2C,
	},
	{
		.name = "IDECLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_ATA,
	},
	{
		.name = "McBSPCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_McBSP0,
	},
	{
		.name = "MMCSDCLK0",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_MMC_SD0,
	},
	{
		.name = "SPICLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_SPI,
	},
	{
		.name = "gpio",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_GPIO,
	},
	{
		.name = "AEMIFCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_AEMIF,
		.usecount = 1,
	},
	{
		.name = "PWM0_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM0,
	},
	{
		.name = "PWM1_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM1,
	},
	{
		.name = "PWM2_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM2,
	},
	{
		.name = "USBCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_USB,
	},
	{
		.name = "VLYNQ_CLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_VLYNQ,
	},
};
static struct clk davinci_dm6467_clks[] = {
	{
		.name = "ARMCLK",
		.rate = &armrate,
		.lpsc = -1,
		.flags = ALWAYS_ENABLED,
	},
	{
		.name = "UART0",
		.rate = &fixedrate,
		.lpsc = DAVINCI_DM646X_LPSC_UART0,
	},
	{
		.name = "UART1",
		.rate = &fixedrate,
		.lpsc = DAVINCI_DM646X_LPSC_UART1,
	},
	{
		.name = "UART2",
		.rate = &fixedrate,
		.lpsc = DAVINCI_DM646X_LPSC_UART2,
	},
	{
		.name = "EMACCLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_EMAC,
	},
	{
		.name = "I2CCLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_I2C,
	},
	{
		.name = "IDECLK",
		.rate = &div_by_six,
		.lpsc = DAVINCI_LPSC_ATA,
	},
	{
		.name = "McASPCLK0",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_McASP0,
	},
	{
		.name = "McASPCLK1",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_McASP1,
	},
	{
		.name = "SPICLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_SPI,
	},
	{
		.name = "gpio",
		.rate = &commonrate,
		.lpsc = DAVINCI_DM646X_LPSC_GPIO,
	},
	{
		.name = "AEMIFCLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_AEMIF,
		.usecount = 1,
	},
	{
		.name = "PWM0_CLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_PWM0,
	},
	{
		.name = "PWM1_CLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_DM646X_LPSC_PWM1,
	},
	{
		.name = "USBCLK",
		.rate = &div_by_four,
		.lpsc = DAVINCI_LPSC_USB,
	},
	{
		.name = "VLYNQ_CLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_VLYNQ,
	},
	{
		.name = "TSIF0_CLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_DM646X_LPSC_TSIF0,
	},
	{
		.name = "TSIF1_CLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_DM646X_LPSC_TSIF1,
	},
};
static struct clk davinci_dm355_clks[] = {
	{
		.name = "ARMCLK",
		.rate = &armrate,
		.lpsc = -1,
		.flags = ALWAYS_ENABLED,
	},
	{
		.name = "UART0",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_UART0,
	},
	{
		.name = "UART1",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_UART1,
	},
	{
		.name = "UART2",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_UART2,
	},
	{
		.name = "EMACCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_EMAC_WRAPPER,
	},
	{
		.name = "I2CCLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_I2C,
	},
	{
		.name = "IDECLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_ATA,
	},
	{
		.name = "McBSPCLK0",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_McBSP0,
	},
	{
		.name = "McBSPCLK1",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_McBSP1,
	},
	{
		.name = "MMCSDCLK0",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_MMC_SD0,
	},
	{
		.name = "MMCSDCLK1",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_MMC_SD1,
	},
	{
		.name = "SPICLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_SPI,
	},
	{
		.name = "gpio",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_GPIO,
	},
	{
		.name = "AEMIFCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_AEMIF,
		.usecount = 1,
	},
	{
		.name = "PWM0_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM0,
	},
	{
		.name = "PWM1_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM1,
	},
	{
		.name = "PWM2_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM2,
	},
	{
		.name = "PWM3_CLK",
		.rate = &fixedrate,
		.lpsc = DAVINCI_LPSC_PWM3,
	},
	{
		.name = "USBCLK",
		.rate = &commonrate,
		.lpsc = DAVINCI_LPSC_USB,
	},
};

int __init davinci_clk_init(void)
{
	struct clk *clkp;
	static struct clk *board_clks;
	int count = 0, num_clks;
	u32 pll_mult;

	pll_mult = davinci_readl(DAVINCI_PLL_CNTRL0_BASE + PLLM);
	commonrate = ((pll_mult + 1) * 27000000) / 6;
	armrate = ((pll_mult + 1) * 27000000) / 2;

	if (cpu_is_davinci_dm355()) {
		/*
		 * FIXME
		 * We're assuming a 24MHz reference, but the DM355 also
		 * supports a 36MHz reference.
		 */
		unsigned long postdiv;

		/*
		 * Read the PLL1 POSTDIV register to determine if the post
		 * divider is /1 or /2
		 */
		postdiv = (davinci_readl(DAVINCI_PLL_CNTRL0_BASE + 0x128)
			& 0x1f) + 1;

		fixedrate = 24000000;
		armrate = (pll_mult + 1) * (fixedrate / (16 * postdiv));
		commonrate = armrate / 2;

		board_clks = davinci_dm355_clks;
		num_clks = ARRAY_SIZE(davinci_dm355_clks);
	} else if (cpu_is_davinci_dm6467()) {
		fixedrate = 24000000;
		div_by_four = ((pll_mult + 1) * 27000000) / 4;
		div_by_six = ((pll_mult + 1) * 27000000) / 6;
		div_by_eight = ((pll_mult + 1) * 27000000) / 8;
		armrate = ((pll_mult + 1) * 27000000) / 2;

		board_clks = davinci_dm6467_clks;
		num_clks = ARRAY_SIZE(davinci_dm6467_clks);
	} else {

		fixedrate = 27000000;
		armrate = (pll_mult + 1) * (fixedrate / 2);
		commonrate = armrate / 3;

		board_clks = davinci_dm644x_clks;
		num_clks = ARRAY_SIZE(davinci_dm644x_clks);
	}

	for (clkp = board_clks; count < num_clks; count++, clkp++) {
		clk_register(clkp);

		/* Turn on clocks that have been enabled in the
		 * table above */
		if (clkp->usecount)
			clk_enable(clkp);
	}

	return 0;
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *davinci_ck_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *davinci_ck_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void davinci_ck_stop(struct seq_file *m, void *v)
{
}

static int davinci_ck_show(struct seq_file *m, void *v)
{
	struct clk *cp;

	list_for_each_entry(cp, &clocks, node)
		seq_printf(m,"%s %d %d\n", cp->name, *(cp->rate), cp->usecount);

	return 0;
}

static struct seq_operations davinci_ck_op = {
	.start	= davinci_ck_start,
	.next	= davinci_ck_next,
	.stop	= davinci_ck_stop,
	.show	= davinci_ck_show
};

static int davinci_ck_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &davinci_ck_op);
}

static struct file_operations proc_davinci_ck_operations = {
	.open		= davinci_ck_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init davinci_ck_proc_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("davinci_clocks", 0, NULL);
	if (entry)
		entry->proc_fops = &proc_davinci_ck_operations;
	return 0;

}
__initcall(davinci_ck_proc_init);
#endif	/* CONFIG_DEBUG_PROC_FS */
