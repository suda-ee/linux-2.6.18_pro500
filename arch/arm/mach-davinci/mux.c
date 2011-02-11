/*
 * Utility to set the DAVINCI MUX register from a table in mux.h
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on linux/arch/arm/plat-omap/mux.c:
 * Copyright (C) 2003 - 2005 Nokia Corporation
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/arch/mux.h>

#ifdef CONFIG_DAVINCI_MUX

static struct pin_config *pin_table;
static unsigned long pin_table_sz;

int __init davinci_mux_register(struct pin_config *pins, unsigned long size)
{
	pin_table = pins;
	pin_table_sz = size;

	return 0;
}

/*
 * Sets the DAVINCI MUX register based on the table
 */
int __init_or_module davinci_cfg_reg(const unsigned long index)
{
	static DEFINE_SPINLOCK(mux_spin_lock);

	unsigned long flags;
	struct pin_config *cfg;
	unsigned int reg_orig = 0, reg = 0;
	unsigned int mask, warn = 0;

	if (!pin_table)
		BUG();

	if (index >= pin_table_sz) {
		printk(KERN_ERR "Invalid pin mux index: %lu (%lu)\n",
		       index, pin_table_sz);
		dump_stack();
		return -ENODEV;
	}

	cfg = (struct pin_config *)&pin_table[index];

	/* Check the mux register in question */
	if (cfg->mux_reg) {
		unsigned	tmp1, tmp2;

		spin_lock_irqsave(&mux_spin_lock, flags);
		reg_orig = davinci_readl(cfg->mux_reg);

		mask = (cfg->mask << cfg->mask_offset);
		tmp1 = reg_orig & mask;
		reg = reg_orig & ~mask;

		tmp2 = (cfg->mode << cfg->mask_offset);
		reg |= tmp2;

		if (tmp1 != tmp2)
			warn = 1;

		davinci_writel(reg, cfg->mux_reg);
		spin_unlock_irqrestore(&mux_spin_lock, flags);
	}

	if (warn) {
#ifdef CONFIG_DAVINCI_MUX_WARNINGS
		printk(KERN_WARNING "MUX: initialized %s\n", cfg->name);
#endif
	}

#ifdef CONFIG_DAVINCI_MUX_DEBUG
	if (cfg->debug || warn) {
		printk(KERN_WARNING "MUX: Setting register %s\n", cfg->name);
		printk(KERN_WARNING "      %s (0x%08x) = 0x%08x -> 0x%08x\n",
		       cfg->mux_reg_name, cfg->mux_reg, reg_orig, reg);
	}
#endif

	return 0;
}
EXPORT_SYMBOL(davinci_cfg_reg);
#else
#define davinci_mux_init()	do {} while (0)
#define davinci_cfg_reg(x)	do {} while (0)
#endif	/* CONFIG_DAVINCI_MUX */
