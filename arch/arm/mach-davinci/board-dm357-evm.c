/*
 * TI DaVinci EVM board support
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * (C) 2007-2008 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/physmap.h>
#endif
#include <linux/serial.h>
#include <linux/serial_8250.h>

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/common.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/clock.h>
#include <asm/arch/nand.h>
#include <asm/arch/mmc.h>
#include <asm/arch/edma.h>
#include <asm/arch/i2c-client.h>

/* other misc. init functions */
void __init davinci_serial_init(struct platform_device *pdev);
void __init davinci_irq_init(void);
void __init davinci_map_common_io(void);
void __init davinci_init_common_hw(void);

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (char *)IO_ADDRESS(DAVINCI_UART0_BASE),
		.mapbase	= (unsigned long)DAVINCI_UART0_BASE,
		.irq		= IRQ_UARTINT0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 27000000,
	},
	{
		.flags		= 0
	},
};

static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
/* NOR Flash base address set to CS0 by default */
#define NOR_FLASH_PHYS 0x02000000

static struct mtd_partition davinci_evm_partitions[] = {
	/* bootloader (U-Boot, etc) in first 4 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sectors */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

static struct physmap_flash_data davinci_evm_flash_data = {
	.width		= 2,
	.parts		= davinci_evm_partitions,
	.nr_parts	= ARRAY_SIZE(davinci_evm_partitions),
};

/* NOTE: CFI probe will correctly detect flash part as 32M, but EMIF
 * limits addresses to 16M, so using addresses past 16M will wrap */
static struct resource davinci_evm_flash_resource = {
	.start		= NOR_FLASH_PHYS,
	.end		= NOR_FLASH_PHYS + SZ_16M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device davinci_evm_flash_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &davinci_evm_flash_data,
	},
	.num_resources	= 1,
	.resource	= &davinci_evm_flash_resource,
};
#endif

#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#ifdef CONFIG_DM357_STORAGE_NAND
static struct mtd_partition davinci_nand_partitions[] = {
	{
		.name		= "filesystem1",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_512M,
		.mask_flags	= 0
	},

	{
		.name		= "filesystem2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

/* flash bbt decriptors */
static uint8_t nand_davinci_bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t nand_davinci_mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr nand_davinci_bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 2,
	.len = 4,
	.veroffs = 16,
	.maxblocks = 4,
	.pattern = nand_davinci_bbt_pattern
};

static struct nand_bbt_descr nand_davinci_bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 2,
	.len = 4,
	.veroffs = 16,
	.maxblocks = 4,
	.pattern = nand_davinci_mirror_pattern
};

static struct nand_davinci_platform_data davinci_nand_data = {
	.options	= 0,
	.ecc_mode	= NAND_ECC_SOFT,
	.cle_mask	= 0x10,
	.ale_mask	= 0x08,
	.bbt_td         = &nand_davinci_bbt_main_descr,
	.bbt_md         = &nand_davinci_bbt_mirror_descr,
	.parts		= davinci_nand_partitions,
	.nr_parts	= ARRAY_SIZE(davinci_nand_partitions),
};

static struct resource davinci_nand_resources[] = {
	[0] = {		/* First memory resource is AEMIF control registers */
		.start	= DM644X_ASYNC_EMIF_CNTRL_BASE,
		.end	= DM644X_ASYNC_EMIF_CNTRL_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {         /* Second memory resource is NAND I/O window */
		.start  = DAVINCI_ASYNC_EMIF_DATA_CE2_BASE,
		.end    = DAVINCI_ASYNC_EMIF_DATA_CE2_BASE + SZ_16K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {		/*
			* Third (optional) memory resource is NAND I/O window
			* for second NAND chip select
			*/
		.start  = DAVINCI_ASYNC_EMIF_DATA_CE2_BASE + SZ_16K,
		.end    = DAVINCI_ASYNC_EMIF_DATA_CE2_BASE + SZ_16K +
				SZ_16K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static void setup_nand(void)
{
	void __iomem *pinmux0 =
		(void __iomem *) IO_ADDRESS(0x01c40000);

	/* Configure the pin multiplexing to enable EMIF CS2(0x06000000) */
	__raw_writel(0x83000c1f, pinmux0);
	
}

#else 

static struct mtd_partition davinci_nand_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_256K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct nand_davinci_platform_data davinci_nand_data = {
	.options	= 0,
	.ecc_mode	= NAND_ECC_HW,
	.cle_mask	= 0x10,
	.ale_mask	= 0x08,
	.bbt_td		= NULL,
	.bbt_md		= NULL,
	.parts		= davinci_nand_partitions,
	.nr_parts	= ARRAY_SIZE(davinci_nand_partitions),
};

static struct resource davinci_nand_resources[] = {
	[0] = {		/* First memory resource is AEMIF control registers */
		.start		= DM644X_ASYNC_EMIF_CNTRL_BASE,
		.end		= DM644X_ASYNC_EMIF_CNTRL_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {		/* Second memory resource is NAND I/O window */
		.start		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_16K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static void setup_nand(void)
{
}
#endif

static struct platform_device davinci_nand_device = {
	.name			= "nand_davinci",
	.id			= 0,
	.dev			= {
		.platform_data	= &davinci_nand_data,
	},

	.num_resources		= ARRAY_SIZE(davinci_nand_resources),
	.resource		= davinci_nand_resources,
};

#endif

static struct platform_device rtc_dev = {
	.name		= "rtc_davinci_evm",
	.id		= -1,
};

static u64 davinci_fb_dma_mask = DMA_32BIT_MASK;

static struct platform_device davinci_fb_device = {
	.name		= "davincifb",
	.id		= -1,
	.dev = {
		.dma_mask		= &davinci_fb_dma_mask,
		.coherent_dma_mask      = DMA_32BIT_MASK,
	},
	.num_resources = 0,
};

#if defined(CONFIG_MMC_DAVINCI) || defined(CONFIG_MMC_DAVINCI_MODULE)
static struct resource mmc0_resources[] = {
	[0] = {			/* registers */
		.start		= DAVINCI_MMC_SD_BASE,
		.end		= DAVINCI_MMC_SD_BASE + SZ_1K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {			/* interrupt */
		.start		= IRQ_MMCINT,
		.end		= IRQ_MMCINT,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {			/* dma rx */
		.start		= DM644X_DMACH_MMCRXEVT,
		.end		= DM644X_DMACH_MMCRXEVT,
		.flags		= IORESOURCE_DMA,
	},
	[3] = {			/* dma tx */
		.start		= DM644X_DMACH_MMCTXEVT,
		.end		= DM644X_DMACH_MMCTXEVT,
		.flags		= IORESOURCE_DMA,
	},
};

static struct davinci_mmc_platform_data mmc0_platform_data = {
	.mmc_clk		= "MMCSDCLK0",
	.rw_threshold		= 32,
	.use_4bit_mode		= 1,
};

static struct platform_device mmc0_device = {
	.name			= "davinci-mmc",
	.id			= 0,
	.dev			= {
		.platform_data	= &mmc0_platform_data,
	},

	.num_resources		= ARRAY_SIZE(mmc0_resources),
	.resource		= mmc0_resources,
};

static void setup_mmc(void)
{
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_MMC_SD0, 1);
}
#else
#define setup_mmc()
#endif

#if defined(CONFIG_BLK_DEV_PALMCHIP_BK3710) || \
    defined(CONFIG_BLK_DEV_PALMCHIP_BK3710_MODULE)
static struct resource ide_resources[] = {
	{
		.start		= DAVINCI_CFC_ATA_BASE,
		.end		= DAVINCI_CFC_ATA_BASE + 0x7ff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IRQ_IDE,
		.end		= IRQ_IDE,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device davinci_ide_device = {
	.name		= "palm_bk3710",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ide_resources),
	.resource	= ide_resources,
};
#endif

static struct platform_device *davinci_evm_devices[] __initdata = {
	&serial_device,
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	&davinci_evm_flash_device,
#endif
#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
	&davinci_nand_device,
#endif
	&rtc_dev,
	&davinci_fb_device,
#if defined(CONFIG_BLK_DEV_PALMCHIP_BK3710) || \
    defined(CONFIG_BLK_DEV_PALMCHIP_BK3710_MODULE)
	&davinci_ide_device,
#endif
#if defined(CONFIG_MMC_DAVINCI) || defined(CONFIG_MMC_DAVINCI_MODULE)
	&mmc0_device,
#endif
};

static void dm644x_setup_pinmux(unsigned int id)
{
	switch (id) {
	case DAVINCI_LPSC_ATA:
		davinci_cfg_reg(DM644X_HDIREN);
		davinci_cfg_reg(DM644X_ATAEN);
		break;
	case DAVINCI_LPSC_MMC_SD:
		/* VDD power manupulations are done in U-Boot for CPMAC
		 * so applies to MMC as well
		 */
		/*Set up the pull regiter for MMC */
		davinci_writel(0, DAVINCI_VDD3P3V_PWDN);
		davinci_cfg_reg(DM644X_MSTK);
		break;
	case DAVINCI_LPSC_I2C:
		davinci_cfg_reg(DM644X_I2C);
		break;
	case DAVINCI_LPSC_McBSP:
		davinci_cfg_reg(DM644X_MCBSP);
		break;
	case DAVINCI_LPSC_PWM0:
		davinci_cfg_reg(DM644X_PWM0);
		break;
	case DAVINCI_LPSC_PWM1:
		davinci_cfg_reg(DM644X_PWM1);
		break;
	case DAVINCI_LPSC_PWM2:
		davinci_cfg_reg(DM644X_PWM2);
		break;
	case DAVINCI_LPSC_VLYNQ:
		davinci_cfg_reg(DM644X_VLINQEN);
		davinci_cfg_reg(DM644X_VLINQWD);
		break;
	default:
		break;
	}
}

static void __init
davinci_evm_map_io(void)
{
	davinci_pinmux_setup = dm644x_setup_pinmux;
	davinci_map_common_io();
}

static void __init davinci_psc_init(void)
{
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VPSSMSTR, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_VPSSSLV, 1);
 	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPCC, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPTC0, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TPTC1, 1);
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_GPIO, 1);
#if defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_USB, 1);
#endif

	/* Turn on WatchDog timer LPSC.	 Needed for RESET to work */
	davinci_psc_config(DAVINCI_GPSC_ARMDOMAIN, DAVINCI_LPSC_TIMER2, 1);
}

static __init void davinci_evm_i2c_expander_setup(void)
{
#if defined(CONFIG_BLK_DEV_PALMCHIP_BK3710) || \
    defined(CONFIG_BLK_DEV_PALMCHIP_BK3710_MODULE)
	/*
	 * ATA_SEL is 1 -> disable, 0 -> enable.
	 * CF_SEL  is 1 -> disable, 0 -> enable.
	 *
	 * Ensure both are not enabled at once.
	 */
#ifdef CONFIG_DAVINCI_EVM_CF_SUPPORT
	davinci_i2c_expander_op(0x3A, ATA_SEL,	1);
	davinci_i2c_expander_op(0x3A, CF_RESET, 1);
	davinci_i2c_expander_op(0x3A, CF_SEL,	0);
#else
	davinci_i2c_expander_op(0x3A, CF_SEL,	1);
	davinci_i2c_expander_op(0x3A, ATA_SEL,	0);
#endif
#endif
}

static __init void davinci_evm_init(void)
{
	davinci_gpio_init();
	davinci_psc_init();

	board_i2c_expander_setup = davinci_evm_i2c_expander_setup;

#if defined(CONFIG_BLK_DEV_PALMCHIP_BK3710) || \
    defined(CONFIG_BLK_DEV_PALMCHIP_BK3710_MODULE)
	printk(KERN_WARNING "WARNING: both IDE and NOR flash are enabled, "
	       "but share pins.\n\tDisable IDE for NOR support.\n");
#endif
#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
	printk(KERN_WARNING "WARNING: both NAND and NOR flash are enabled, "
	       "but share pins.\n\tDisable NAND for NOR support.\n");
#endif

	davinci_serial_init(&serial_device);
	setup_mmc();
	setup_nand();
	platform_add_devices(davinci_evm_devices,
			     ARRAY_SIZE(davinci_evm_devices));
}

static __init void davinci_evm_irq_init(void)
{
	davinci_init_common_hw();
	davinci_irq_init();
}

MACHINE_START(DAVINCI_EVM, "DM357 EVM")
	/* Maintainer: MontaVista Software <source@mvista.com> */
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (io_p2v(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (DAVINCI_DDR_BASE + 0x100),
	.map_io	      = davinci_evm_map_io,
	.init_irq     = davinci_evm_irq_init,
	.timer	      = &davinci_timer,
	.init_machine = davinci_evm_init,
MACHINE_END


