/*
 * linux/arch/arm/mach-davinci/devices.c
 *
 * DaVinci platform device setup/initialization
 *
 * Copyright (C) 2006 Komal Shah <komal_shah802003@yahoo.com>
 * Copyright (c) 2007, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/musb.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <asm/arch/i2c.h>
#include <asm/arch/cpu.h>

#define DAVINCI_WDOG_BASE	(IO_PHYS + 0x21C00)

static struct resource i2c_resources[] = {
	{
		.start		= DAVINCI_I2C_BASE,
		.end		= DAVINCI_I2C_BASE + 0x40,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IRQ_I2C,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct davinci_i2c_platform_data dm644x_i2c_data = {
	.bus_freq	= 20,
	.bus_delay	= 100,
};

static struct davinci_i2c_platform_data dm355_i2c_data = {
	.bus_freq	= 20,
	.bus_delay	= 100,
};

static struct davinci_i2c_platform_data dm646x_i2c_data = {
	.bus_freq	= 100,
	.bus_delay	= 0,
};

static struct platform_device i2c_device = {
	.name           = "i2c_davinci",
	.id             = 1,
	.dev		= {
		.platform_data = &dm355_i2c_data,
	},
	.num_resources	= ARRAY_SIZE(i2c_resources),
	.resource	= i2c_resources,
};

struct resource watchdog_resources[] = {
	{
		.start = DAVINCI_WDOG_BASE,
		.end = DAVINCI_WDOG_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device watchdog_device = {
	.name = "watchdog",
	.id = -1,
	.num_resources = ARRAY_SIZE(watchdog_resources),
	.resource = watchdog_resources,
};


static struct musb_hdrc_platform_data usb_data = {
#if     defined(CONFIG_USB_MUSB_OTG)
	/* OTG requires a Mini-AB connector */
	.mode		= MUSB_OTG,
#elif   defined(CONFIG_USB_MUSB_PERIPHERAL)
	.mode		= MUSB_PERIPHERAL,
#elif   defined(CONFIG_USB_MUSB_HOST)
	.mode		= MUSB_HOST,
#endif
	/* irlml6401 switches 5V */
	.power		= 250,		/* sustains 3.0+ Amps (!) */
	.potpgt		= 4,		/* ~8 msec */

	/* REVISIT multipoint is a _chip_ capability; not board specific */
	.multipoint	= 1,
};

static struct resource usb_resources [] = {
	{
		.start	= DAVINCI_USB_OTG_BASE,
		.end	= DAVINCI_USB_OTG_BASE + 0x5ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_USBINT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start  = 0, /*by default no dedicated dma irq*/
		.flags  = IORESOURCE_IRQ,
	}
};

static u64 usb_dmamask = DMA_32BIT_MASK;

static struct platform_device usb_device = {
        .name		= "musb_hdrc",
        .id		= -1,
        .dev = {
                .platform_data		= &usb_data,
                .dma_mask		= &usb_dmamask,
                .coherent_dma_mask	= DMA_32BIT_MASK,
        },
        .resource	= usb_resources,
        .num_resources	= ARRAY_SIZE(usb_resources),
};

static struct platform_device *devices[] __initdata = {
	&i2c_device,
	&watchdog_device,
	&usb_device,
};

static void __init davinci_init_cpu_i2c(void)
{
	if (cpu_is_davinci_dm644x())
		i2c_device.dev.platform_data = &dm644x_i2c_data;
	else if (cpu_is_davinci_dm6467())
		i2c_device.dev.platform_data = &dm646x_i2c_data;

	/* all others default to use dm355 because dm355 uses the max speed */
}

static void __init davinci_init_cpu_usb(void)
{
	if (cpu_is_davinci_dm6467()) {
		/*
		 * overwrite default settings
		 * as DM6467 uses different irq lines
		 */
		usb_device.resource[1].start = IRQ_DM646X_USBINT;
		usb_device.resource[2].start = IRQ_DM646X_USBDMAINT;
	}
}

static int __init davinci_init_devices(void)
{
	davinci_init_cpu_i2c();
	davinci_init_cpu_usb();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	return 0;
}
arch_initcall(davinci_init_devices);
