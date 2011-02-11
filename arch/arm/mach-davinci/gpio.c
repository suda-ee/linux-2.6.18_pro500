/*
 * TI DaVinci GPIO Support
 *
 * Copyright (c) 2006 David Brownell
 * Copyright (c) 2007, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/bitops.h>

#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/cpu.h>
#include <asm/arch/gpio.h>

#include <asm/mach/irq.h>

/* If a new chip is added with number of GPIO greater than 104, please
   update DAVINCI_MAX_N_GPIO in include/asm-arm/arch-davinci/irqs.h */
#define DM646x_N_GPIO	48
#define DM644x_N_GPIO	71
#define DM355_N_GPIO	104

static DEFINE_SPINLOCK(gpio_lock);
static struct gpio_bank *gpio_bank;

#ifdef CONFIG_ARCH_DAVINCI644x
static DECLARE_BITMAP(dm644x_gpio_in_use, DM644x_N_GPIO);
struct gpio_bank gpio_bank_dm6446 = {
	.base		= DAVINCI_GPIO_BASE,
	.num_gpio	= DM644x_N_GPIO,
	.irq_num	= IRQ_DM644X_GPIOBNK0,
	.irq_mask	= 0x1f,
	.in_use		= dm644x_gpio_in_use,
};
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM355
static DECLARE_BITMAP(dm355_gpio_in_use, DM355_N_GPIO);
struct gpio_bank gpio_bank_dm355 = {
	.base		= DAVINCI_GPIO_BASE,
	.num_gpio	= DM355_N_GPIO,
	.irq_num	= IRQ_DM355_GPIOBNK0,
	.irq_mask	= 0x3f,
	.in_use		= dm355_gpio_in_use,
};
#endif

#ifdef CONFIG_ARCH_DAVINCI_DM646x
static DECLARE_BITMAP(dm646x_gpio_in_use, DM646x_N_GPIO);
struct gpio_bank gpio_bank_dm646x = {
	.base		= DAVINCI_GPIO_BASE,
	.num_gpio	= DM646x_N_GPIO,
	.irq_num	= IRQ_DM646X_GPIOBNK0,
	.irq_mask	= 0x1f,
	.in_use		= dm646x_gpio_in_use,
};
#endif
int gpio_request(unsigned gpio, const char *tag)
{
	if (gpio >= gpio_bank->num_gpio)
		return -EINVAL;

	if (test_and_set_bit(gpio, gpio_bank->in_use))
		return -EBUSY;

	return 0;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	if (gpio >= gpio_bank->num_gpio)
		return;

	clear_bit(gpio, gpio_bank->in_use);
}
EXPORT_SYMBOL(gpio_free);

/* The __gpio_to_controller() and __gpio_mask() functions inline to constants
 * with constant parameters; or in outlined code they execute at runtime.
 *
 * You'd access the controller directly when reading or writing more than
 * one gpio value at a time, and to support wired logic where the value
 * being driven by the cpu need not match the value read back.
 *
 * These are NOT part of the cross-platform GPIO interface
 */
struct gpio_controller *__iomem __gpio_to_controller(unsigned gpio)
{
	void *__iomem ptr;

	if (gpio >= gpio_bank->num_gpio)
		return NULL;

	if (gpio < 32)
		ptr = (void *__iomem)IO_ADDRESS(gpio_bank->base + 0x10);
	else if (gpio < 64)
		ptr = (void *__iomem)IO_ADDRESS(gpio_bank->base + 0x38);
	else if (gpio < 96)
		ptr = (void *__iomem)IO_ADDRESS(gpio_bank->base + 0x60);
	else
		ptr = (void *__iomem)IO_ADDRESS(gpio_bank->base + 0x88);
	return ptr;
}

void gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(value)) {
		struct gpio_controller	*__iomem g;
		u32			mask;

		if (gpio >= gpio_bank->num_gpio)
			__error_inval_gpio();

		g = __gpio_to_controller(gpio);
		mask = __gpio_mask(gpio);
		if (value)
			__raw_writel(mask, &g->set_data);
		else
			__raw_writel(mask, &g->clr_data);
		return;
	}

	__gpio_set(gpio, value);
}
EXPORT_SYMBOL(gpio_set_value);

/* Returns zero or nonzero; works for gpios configured as inputs OR
 * as outputs.
 *
 * NOTE: changes in reported values are synchronized to the GPIO clock.
 * This is most easily seen after calling gpio_set_value() and then immediatly
 * gpio_get_value(), where the gpio_get_value() would return the old value
 * until the GPIO clock ticks and the new value gets latched.
 */

int gpio_get_value(unsigned gpio)
{
	struct gpio_controller *__iomem g;

	if (!__builtin_constant_p(gpio))
		return __gpio_get(gpio);

	if (gpio >= gpio_bank->num_gpio)
		return __error_inval_gpio();

	g = __gpio_to_controller(gpio);
	return !!(__gpio_mask(gpio) & __raw_readl(&g->in_data));
}
EXPORT_SYMBOL(gpio_get_value);

/* create a non-inlined version */
static struct gpio_controller *__iomem gpio2controller(unsigned gpio)
{
	return __gpio_to_controller(gpio);
}

/*
 * Assuming the pin is muxed as a gpio output, set its output value.
 */
void __gpio_set(unsigned gpio, int value)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);

	__raw_writel(__gpio_mask(gpio), value ? &g->set_data : &g->clr_data);
}
EXPORT_SYMBOL(__gpio_set);


/*
 * Read the pin's value (works even if it's set up as output);
 * returns zero/nonzero.
 *
 * Note that changes are synched to the GPIO clock, so reading values back
 * right after you've set them may give old values.
 */
int __gpio_get(unsigned gpio)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);

	return !!(__gpio_mask(gpio) & __raw_readl(&g->in_data));
}
EXPORT_SYMBOL(__gpio_get);


/*--------------------------------------------------------------------------*/

/*
 * board setup code *MUST* set PINMUX0 and PINMUX1 as
 * needed, and enable the GPIO clock.
 */

int gpio_direction_input(unsigned gpio)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);
	u32 temp;
	u32 mask;

	if (!g)
		return -EINVAL;

	spin_lock(&gpio_lock);
	mask = __gpio_mask(gpio);
	temp = __raw_readl(&g->dir);
	temp |= mask;
	__raw_writel(temp, &g->dir);
	spin_unlock(&gpio_lock);
	return 0;
}
EXPORT_SYMBOL(gpio_direction_input);

int gpio_direction_output(unsigned gpio, int value)
{
	struct gpio_controller *__iomem g = gpio2controller(gpio);
	u32 temp;
	u32 mask;

	if (!g)
		return -EINVAL;

	spin_lock(&gpio_lock);
	mask = __gpio_mask(gpio);
	temp = __raw_readl(&g->dir);
	temp &= ~mask;
	__raw_writel(mask, value ? &g->set_data : &g->clr_data);
	__raw_writel(temp, &g->dir);
	spin_unlock(&gpio_lock);
	return 0;
}
EXPORT_SYMBOL(gpio_direction_output);

/*
 * We expect irqs will normally be set up as input pins, but they can also be
 * used as output pins ... which is convenient for testing.
 *
 * NOTE:  GPIO0..GPIO7 also have direct INTC hookups, which work in addition
 * to their GPIOBNK0 irq (but with a bit less overhead).  But we don't have
 * a good way to hook those up ...
 *
 * All those INTC hookups (GPIO0..GPIO7 plus five IRQ banks) can also
 * serve as EDMA event triggers.
 */

static void gpio_irq_disable(unsigned irq)
{
	struct gpio_controller *__iomem g = get_irq_chip_data(irq);
	u32 mask = __gpio_mask(irq_to_gpio(irq));

	__raw_writel(mask, &g->clr_falling);
	__raw_writel(mask, &g->clr_rising);
}

static void gpio_irq_enable(unsigned irq)
{
	struct gpio_controller *__iomem g = get_irq_chip_data(irq);
	u32 tmp;
	u32 mask = __gpio_mask(irq_to_gpio(irq));

	if (irq_desc[irq].status & IRQ_TYPE_EDGE_FALLING) {
		tmp = mask | __raw_readl(&g->set_falling);
		__raw_writel(tmp, &g->set_falling);
	}
	if (irq_desc[irq].status & IRQ_TYPE_EDGE_RISING) {
		tmp = mask | __raw_readl(&g->set_rising);
		__raw_writel(tmp, &g->set_rising);
	}
}

static int gpio_irq_type(unsigned irq, unsigned trigger)
{
	struct gpio_controller *__iomem g = get_irq_chip_data(irq);
	u32 tmp;
	u32 mask = __gpio_mask(irq_to_gpio(irq));

	if (trigger & ~(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	irq_desc[irq].status &= ~IRQ_TYPE_SENSE_MASK;
	irq_desc[irq].status |= trigger;

	if (trigger & IRQ_TYPE_EDGE_FALLING) {
		tmp = mask | __raw_readl(&g->set_falling);
		__raw_writel(tmp, &g->set_falling);
	} else {
		__raw_writel(mask, &g->clr_falling);
	}

	if (trigger & IRQ_TYPE_EDGE_RISING) {
		tmp = mask | __raw_readl(&g->set_rising);
		__raw_writel(tmp, &g->set_rising);
	} else {
		__raw_writel(mask, &g->clr_rising);
	}

	return 0;
}

static struct irq_chip gpio_irqchip = {
	.name		= "GPIO",
	.enable		= gpio_irq_enable,
	.disable	= gpio_irq_disable,
	.set_type	= gpio_irq_type,
};

static void
gpio_irq_handler(unsigned irq, struct irq_desc *desc, struct pt_regs *regs)
{
	struct gpio_controller *__iomem g = get_irq_chip_data(irq);
	u32 mask = 0xffff;

	/* we only care about one bank */
	if (irq & 1)
		mask <<= 16;

	/* temporarily mask (level sensitive) parent IRQ */
	desc->chip->ack(irq);
	while (1) {
		u32		status;
		struct irq_desc	*gpio;
		int		n;
		int		res;

		/* ack any irqs */
		status = __raw_readl(&g->intstat) & mask;
		if (!status)
			break;
		__raw_writel(status, &g->intstat);
		if (irq & 1)
			status >>= 16;

		/* now demux them to the right lowlevel handler */
		n = (int)get_irq_data(irq);
		gpio = &irq_desc[n];
		while (status) {
			res = ffs(status);
			n += res;
			gpio += res;
			desc_handle_irq(n - 1, gpio - 1, regs);
			status >>= res;
		}
	}
	desc->chip->unmask(irq);
	/* now it may re-trigger */
}

/*
 * NOTE:  for suspend/resume, probably best to make a sysdev (and class)
 * with its suspend/resume calls hooking into the results of the set_wake()
 * calls ... so if no gpios are wakeup events the clock can be disabled,
 * with outputs left at previously set levels, and so that VDD3P3V.IOPWDN0
 * can be set appropriately for GPIOV33 pins.
 */

static int __init davinci_gpio_irq_setup(void)
{
	unsigned	gpio, irq, bank;
	struct clk	*clk;

	clk = clk_get(NULL, "gpio");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "Error %ld getting gpio clock?\n",
		       PTR_ERR(clk));
		return 0;
	}

	clk_enable(clk);

	for (gpio = 0, irq = gpio_to_irq(0), bank = gpio_bank->irq_num;
	     gpio < gpio_bank->num_gpio; bank++) {
		struct gpio_controller	*__iomem g = gpio2controller(gpio);
		unsigned		i;

		__raw_writel(~0, &g->clr_falling);
		__raw_writel(~0, &g->clr_rising);

		/* set up all irqs in this bank */
		set_irq_chained_handler(bank, gpio_irq_handler);
		set_irq_chip_data(bank, g);
		set_irq_data(bank, (void *)irq);

		for (i = 0; i < 16 && gpio < gpio_bank->num_gpio;
		     i++, irq++, gpio++) {
			set_irq_chip(irq, &gpio_irqchip);
			set_irq_chip_data(irq, g);
			set_irq_handler(irq, handle_simple_irq);
			set_irq_flags(irq, IRQF_VALID);
		}
	}

	/* BINTEN -- per-bank interrupt enable. genirq would also let these
	 * bits be set/cleared dynamically.
	 */
	__raw_writel(gpio_bank->irq_mask, (void *__iomem)
		     IO_ADDRESS(gpio_bank->base + 0x08));

	printk(KERN_INFO "DaVinci: %d gpio irqs\n", irq - gpio_to_irq(0));

	return 0;
}

void davinci_gpio_init(void)
{

#ifdef CONFIG_ARCH_DAVINCI644x
	if (cpu_is_davinci_dm644x())
		gpio_bank = &gpio_bank_dm6446;
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM355
	if (cpu_is_davinci_dm355())
		gpio_bank = &gpio_bank_dm355;
#endif
#ifdef CONFIG_ARCH_DAVINCI_DM646x
	if (cpu_is_davinci_dm6467())
		gpio_bank = &gpio_bank_dm646x;
#endif
	davinci_gpio_irq_setup();
}
