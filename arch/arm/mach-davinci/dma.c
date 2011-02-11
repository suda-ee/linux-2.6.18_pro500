/*
 * TI DaVinci DMA Support
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (c) 2007-2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <linux/io.h>

#include <asm/arch/hardware.h>
#include <asm/arch/memory.h>
#include <asm/arch/irqs.h>
#include <asm/arch/edma.h>
#include <asm/arch/cpu.h>

struct edma_map {
	int param1;
	int param2;
};

static unsigned int *edma_channels_arm;
static unsigned char *qdma_channels_arm;
static unsigned int *param_entry_arm;
static unsigned int *tcc_arm;
static unsigned int *param_entry_reserved;
unsigned int davinci_cpu_index;

const unsigned int davinci_qdma_ch_map[] = {
	EDMA_DM644X_NUM_PARAMENTRY,
	EDMA_DM646X_NUM_PARAMENTRY,
	EDMA_DM355_NUM_PARAMENTRY,
};

/* SoC specific EDMA3 hardware information, should be provided for a new SoC */

/* DaVinci DM644x specific EDMA3 information */

/*
 * Each bit field of the elements below indicate the corresponding DMA channel
 * availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm644x_edma_channels_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu,  0xFFFFFFFFu
};

/*
 * Each bit field of the elements below indicate the corresponding QDMA channel
 * availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned char dm644x_qdma_channels_arm[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0x00000010u
};

/*
 *  Each bit field of the elements below indicate corresponding PaRAM entry
 *  availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm644x_param_entry_arm[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
 *  Each bit field of the elements below indicate corresponding TCC
 *  availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm644x_tcc_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
 *  Each bit field of the elements below indicate whether the corresponding
 *  PaRAM entry is available for ANY DMA channel or not.
 *   1- reserved, 0 - not
 *   (First 64 PaRAM Sets are reserved for 64 DMA Channels)
 */
static unsigned int dm644x_param_entry_reserved[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0x0u, 0x0u
};

static struct edma_map dm644x_queue_priority_mapping[EDMA_DM644X_NUM_EVQUE] = {
	/* {Event Queue No, Priority} */
	{0, 0},
	{1, 1}
};

static struct edma_map dm644x_queue_watermark_level[EDMA_DM644X_NUM_EVQUE] = {
	/* {Event Queue No, Watermark Level} */
	{0, 16},
	{1, 16}
};

static struct edma_map dm644x_queue_tc_mapping[EDMA_DM644X_NUM_EVQUE] = {
	/* {Event Queue No, TC no} */
	{0, 0},
	{1, 1}
};

/* DaVinci DM646x specific EDMA3 information */

/*
 * Each bit field of the elements below indicate the corresponding DMA channel
 * availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm646x_edma_channels_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0x30FF1FF0u,  0x00C007FFu
};

/*
 * Each bit field of the elements below indicate the corresponding QDMA channel
 * availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned char dm646x_qdma_channels_arm[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0x00000080
};

/*
 *  Each bit field of the elements below indicate corresponding PaRAM entry
 *  availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm646x_param_entry_arm[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u
};

/*
 *  Each bit field of the elements below indicate corresponding TCC
 *  availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm646x_tcc_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0x30FF1FF0u, 0x00C007FFu
};

/*
 *  Each bit field of the elements below indicate whether the corresponding
 *  PaRAM entry is available for ANY DMA channel or not.
 *   1- reserved, 0 - not
 *   (First 64 PaRAM Sets are reserved for 64 DMA Channels)
 */
static unsigned int dm646x_param_entry_reserved[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u
};

static struct edma_map dm646x_queue_priority_mapping[EDMA_DM646X_NUM_EVQUE] = {
	/* {Event Queue No, Priority} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3}
};

static struct edma_map dm646x_queue_watermark_level[EDMA_DM646X_NUM_EVQUE] = {
	/* {Event Queue No, Watermark Level} */
	{0, 16},
	{1, 16},
	{2, 16},
	{3, 16}
};

static struct edma_map dm646x_queue_tc_mapping[EDMA_DM646X_NUM_EVQUE] = {
	/* {Event Queue No, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
};

/* DaVinci DM355 specific EDMA3 information */

/*
 * Each bit field of the elements below indicate the corresponding DMA channel
 * availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm355_edma_channels_arm[] = {
	0xFFFFFFFFu, 0x00000000u
};

/*
 * Each bit field of the elements below indicate the corresponding QDMA channel
 * availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned char dm355_qdma_channels_arm[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0x000000FFu
};

/*
 *  Each bit field of the elements below indicate corresponding PaRAM entry
 *  availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm355_param_entry_arm[] = {
	0xFFFFFFFFu, 0x00000000u, 0x00000000u, 0xFFFFFFC0u,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
};

/*
 *  Each bit field of the elements below indicate corresponding TCC
 *  availability on EDMA_MASTER_SHADOW_REGION side events
 */
static unsigned int dm355_tcc_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
 *  Each bit field of the elements below indicate whether the corresponding
 *  PaRAM entry is available for ANY DMA channel or not.
 *   1- reserved, 0 - not
 *   (First 64 PaRAM Sets are reserved for 64 DMA Channels)
 */
static unsigned int dm355_param_entry_reserved[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0x0u, 0x0u
};

static struct edma_map dm355_queue_priority_mapping[] = {
	/* {Event Queue No, Priority} */
	{0, 0},
	{1, 7}
};

static struct edma_map dm355_queue_watermark_level[] = {
	/* {Event Queue No, Watermark Level} */
	{0, 16},
	{1, 16}
};

static struct edma_map dm355_queue_tc_mapping[] = {
	/* {Event Queue No, TC no} */
	{0, 0},
	{1, 1}
};

static spinlock_t dma_chan_lock;

/*
 * Edma Driver Internal Data Structures
 */

/*
 * Array to maintain the Callback details registered
 * against a particular TCC. Used to call the callback
 * functions linked to the particular channel.
 */
static struct davinci_dma_lch_intr {
	void (*callback) (int lch, u16 ch_status, void *data);
	void *data;
} intr_data[EDMA_NUM_TCC];

#define dma_handle_cb(lch, status)	do { \
	if (intr_data[lch].callback) \
		intr_data[lch].callback(lch, status, intr_data[lch].data); \
} while (0)

/*
 * Resources bound to a Logical Channel (DMA/QDMA/LINK)
 *
 * When a request for a channel is made, the resources PaRAM Set and TCC
 * get bound to that channel. This information is needed internally by the
 * driver when a request is made to free the channel (Since it is the
 * responsibility of the driver to free up the channel-associated resources
 * from the Resource Manager layer).
 */
struct edma3_ch_bound_res {
	/* PaRAM Set number associated with the particular channel */
	unsigned int param_id;
	/* TCC associated with the particular channel */
	unsigned int tcc;
};

static struct edma3_ch_bound_res *dma_ch_bound_res;
static int edma_max_logical_ch;
static unsigned int davinci_edma_num_evtq;
static unsigned int davinci_edma_chmap_exist;
static unsigned int davinci_edma_num_tc;
static unsigned int davinci_edma_num_param;
static unsigned int *davinci_edmatc_base_addrs;
static unsigned int *edma2event_map;

/*
 * Mapping of DMA channels to Hardware Events from
 * various peripherals, which use EDMA for data transfer.
 * All channels need not be mapped, some can be free also.
 */
static unsigned int dm644x_dma_ch_hw_event_map[EDMA_NUM_DMA_CHAN_DWRDS] = {
	DM644X_DMACH2EVENT_MAP0,
	DM644X_DMACH2EVENT_MAP1
};

static unsigned int dm355_dma_ch_hw_event_map[EDMA_NUM_DMA_CHAN_DWRDS] = {
	DM355_DMACH2EVENT_MAP0,
	DM355_DMACH2EVENT_MAP1
};

static unsigned int dm646x_dma_ch_hw_event_map[EDMA_NUM_DMA_CHAN_DWRDS] = {
	DM646X_DMACH2EVENT_MAP0,
	DM646X_DMACH2EVENT_MAP1
};

/*
 *  Each bit field of the elements below indicate whether a DMA Channel
 *  is free or in use
 *  1 - free
 *  0 - in use
 */
static unsigned int dma_ch_use_status[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu,
	0xFFFFFFFFu
};

/*
 *  Each bit field of the elements below indicate whether a interrupt
 *  is free or in use
 *  1 - free
 *  0 - in use
 */
static unsigned char qdma_ch_use_status[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0xFFu
};

/*
 *  Each bit field of the elements below indicate whether a PaRAM entry
 *  is free or in use
 *  1 - free
 *  0 - in use
 */
static unsigned int param_entry_use_status[EDMA_MAX_PARAM_SET/32u] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
 *  Each bit field of the elements below indicate whether a intrerrupt
 *  is free or in use
 *  1 - free
 *  0 - in use
 */
static unsigned long tcc_use_status[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu,
	0xFFFFFFFFu
};



/*
 *  Global Array to store the mapping between DMA channels and Interrupt
 *  channels i.e. TCCs.
 *  DMA channel X can use any TCC Y. Transfer completion
 *  interrupt will occur on the TCC Y (IPR/IPRH Register, bit Y), but error
 *  interrupt will occur on DMA channel X (EMR/EMRH register, bit X). In that
 *  scenario, this DMA channel <-> TCC mapping will be used to point to
 *  the correct callback function.
 */
static unsigned int edma_dma_ch_tcc_mapping [EDMA_NUM_DMACH];


/*
 *  Global Array to store the mapping between QDMA channels and Interrupt
 *  channels i.e. TCCs.
 *  QDMA channel X can use any TCC Y. Transfer completion
 *  interrupt will occur on the TCC Y (IPR/IPRH Register, bit Y), but error
 *  interrupt will occur on QDMA channel X (QEMR register, bit X). In that
 *  scenario, this QDMA channel <-> TCC mapping will be used to point to
 *  the correct callback function.
 */
static unsigned int edma_qdma_ch_tcc_mapping [EDMA_NUM_QDMACH];


/*
 * The list of Interrupt Channels which get allocated while requesting the
 * TCC. It will be used while checking the IPR/IPRH bits in the RM ISR.
 */
static unsigned int allocated_tccs[2u] = {0u, 0u};


/* Array containing physical addresses of all the TCs present */
u32 dm644x_edmatc_base_addrs[EDMA_MAX_TC] = {
	(u32)DAVINCI_DMA_3PTC0_BASE,
	(u32)DAVINCI_DMA_3PTC1_BASE,
};
u32 dm646x_edmatc_base_addrs[EDMA_MAX_TC] = {
	(u32)DAVINCI_DMA_3PTC0_BASE,
	(u32)DAVINCI_DMA_3PTC1_BASE,
	(u32)DAVINCI_DM646X_DMA_3PTC2_BASE,
	(u32)DAVINCI_DM646X_DMA_3PTC3_BASE,
};
u32 dm355_edmatc_base_addrs[EDMA_MAX_TC] = {
	(u32)DAVINCI_DMA_3PTC0_BASE,
	(u32)DAVINCI_DMA_3PTC1_BASE,
};

/*
 * Variable which will be used internally for referring transfer controllers'
 * error interrupts.
 */
unsigned int dm644x_tc_error_int[EDMA_MAX_TC] = {
	IRQ_TCERRINT0, IRQ_TCERRINT,
	0, 0, 0, 0, 0, 0,
};
unsigned int dm646x_tc_error_int[EDMA_MAX_TC] = {
	IRQ_TCERRINT0, IRQ_TCERRINT,
	IRQ_DM646X_TCERRINT2, IRQ_DM646X_TCERRINT3,
	0, 0, 0, 0,
};
unsigned int dm355_tc_error_int[EDMA_MAX_TC] = {
	IRQ_TCERRINT0, IRQ_TCERRINT,
	0, 0, 0, 0, 0, 0,
};

static char tc_error_int_name[EDMA_MAX_TC][20];

/*
 * EDMA Driver Internal Functions
 */

/* EDMA3 TC0 Error Interrupt Handler ISR Routine */

static irqreturn_t dma_tc0_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/* EDMA3 TC1 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc1_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/* EDMA3 TC2 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc2_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/* EDMA3 TC3 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc3_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/* EDMA3 TC4 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc4_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/* EDMA3 TC5 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc5_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/* EDMA3 TC6 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc6_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/*  EDMA3 TC7 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc7_err_handler(int irq, void *dev_id,
					struct pt_regs *data);


/*
 * EDMA3 TC ISRs which need to be registered with the underlying OS by the user
 * (Not all TC error ISRs need to be registered, register only for the
 * available Transfer Controllers).
 */
irqreturn_t (*ptr_edmatc_isrs[EDMA_MAX_TC])(int irq, void *dev_id,
		struct pt_regs *data) = {
	&dma_tc0_err_handler,
	&dma_tc1_err_handler,
	&dma_tc2_err_handler,
	&dma_tc3_err_handler,
	&dma_tc4_err_handler,
	&dma_tc5_err_handler,
	&dma_tc6_err_handler,
	&dma_tc7_err_handler,
};

/* Function registering different ISRs with the OS */
static int register_dma_interrupts(void);

static void map_dma_ch_evt_queue(unsigned int dma_ch, unsigned int evt_queue)
{
	CLEAR_REG_VAL(DMAQNUM_CLR_MASK(dma_ch), EDMA_DMAQNUM(dma_ch >> 3));
	SET_REG_VAL(DMAQNUM_SET_MASK(dma_ch, evt_queue),
		    EDMA_DMAQNUM(dma_ch >> 3));
}

static void map_qdma_ch_evt_queue(unsigned int qdma_ch, unsigned int evt_queue)
{
	/* Map QDMA channel to event queue */
	CLEAR_REG_VAL(QDMAQNUM_CLR_MASK(qdma_ch), EDMA_QDMAQNUM);
	SET_REG_VAL(QDMAQNUM_SET_MASK(qdma_ch, evt_queue), EDMA_QDMAQNUM);
}

static void map_dma_ch_param_set(unsigned int lch, unsigned int param_set)
{

	if (davinci_edma_chmap_exist == 1)  {
		/* Map PaRAM set number for specified lch */
		CLEAR_REG_VAL(DMACH_PARAM_CLR_MASK, EDMA_DCHMAP(lch));
		SET_REG_VAL(DMACH_PARAM_SET_MASK(param_set), EDMA_DCHMAP(lch));
	}
}

static void map_qdma_ch_param_set(unsigned int qdma_ch, unsigned int param_set)
{
	/* Map PaRAM Set Number for specified qdma_ch */
	CLEAR_REG_VAL(QDMACH_PARAM_CLR_MASK, EDMA_QCHMAP(qdma_ch));
	SET_REG_VAL(QDMACH_PARAM_SET_MASK(param_set), EDMA_QCHMAP(qdma_ch));

	/* Set CCNT as default Trigger Word */
	CLEAR_REG_VAL(QDMACH_TRWORD_CLR_MASK, EDMA_QCHMAP(qdma_ch));
	SET_REG_VAL(QDMACH_TRWORD_SET_MASK(param_set), EDMA_QCHMAP(qdma_ch));
}

static void register_callback(unsigned int tcc,
			void (*callback) (int lch, unsigned short ch_status,
					void *data),
			void *data)
{
	/* If callback function is not NULL */
	if (callback == NULL)
		return;

	if (tcc < 32) {
		SET_REG_VAL(1 << tcc, EDMA_SH_IESR(EDMA_MASTER_SHADOW_REGION));

		pr_debug("ier = %x \r\n",
			    EDMA_SH_IER(EDMA_MASTER_SHADOW_REGION));

	} else if (tcc < EDMA_NUM_TCC) {
		SET_REG_VAL(1 << (tcc - 32),
			    EDMA_SH_IESRH(EDMA_MASTER_SHADOW_REGION));

		pr_debug("ierh = %x \r\n",
			    EDMA_SH_IERH(EDMA_MASTER_SHADOW_REGION));
	} else {
		printk(KERN_WARNING "WARNING: dma register callback failed - "
			"invalid tcc %d\n", tcc);
		return;
	}

	/* Save the callback function also */
	intr_data[tcc].callback = callback;
	intr_data[tcc].data = data;
}

static void unregister_callback(unsigned int lch, enum resource_type ch_type)
{
	unsigned int tcc;

	pr_debug("[%s]: start\n", __func__);
	pr_debug("lch = %d\n", lch);

	switch (ch_type) {
	case RES_DMA_CHANNEL:
		tcc = edma_dma_ch_tcc_mapping[lch];
		pr_debug("mapped tcc for DMA channel = %d\n", tcc);
		/* reset */
		edma_dma_ch_tcc_mapping[lch] = EDMA_NUM_TCC;
		break;

	case RES_QDMA_CHANNEL:
		tcc = edma_qdma_ch_tcc_mapping[lch - EDMA_QDMA_CHANNEL_0];
		pr_debug("mapped tcc for QDMA channel = %d\n", tcc);
		/* reset */
		edma_qdma_ch_tcc_mapping[lch - EDMA_QDMA_CHANNEL_0] =
					EDMA_NUM_TCC;
		break;

	default:
		return;
	}

	/* Remove the callback function and disable the interrupts */
	if (tcc < 32) {
		SET_REG_VAL(1 << tcc, EDMA_SH_IECR(EDMA_MASTER_SHADOW_REGION));
	} else if (tcc < EDMA_NUM_TCC) {
		SET_REG_VAL(1 << (tcc - 32),
			    EDMA_SH_IECRH(EDMA_MASTER_SHADOW_REGION));
	} else {
		printk(KERN_WARNING "WARNING: dma unregister callback failed - "
			"invalid tcc %d on lch %d\n", tcc, lch);
		return;
	}

	if (tcc < EDMA_NUM_TCC) {
		intr_data[tcc].callback = 0;
		intr_data[tcc].data = 0;
	}

	pr_debug("[%s]: end\n", __func__);
}

static int reserve_one_edma_channel(unsigned int res_id,
				    unsigned int res_id_set)
{
	int result = -1;
	u32 idx, reg;

	idx = res_id / 32;

	spin_lock(&dma_chan_lock);
	if (((edma_channels_arm[idx] & res_id_set) != 0) &&
	    ((dma_ch_use_status[idx] & res_id_set) != 0)) {
		/* Mark it as non-available now */
		dma_ch_use_status[idx] &= ~res_id_set;
		if (res_id < 32u)  {
			/* Enable the DMA channel in the DRAE register */
			reg = EDMA_DRAE(EDMA_MASTER_SHADOW_REGION);
			SET_REG_VAL(res_id_set, reg);
			pr_debug("drae = %x\n", dma_read(reg));
			reg = EDMA_SH_EECR(EDMA_MASTER_SHADOW_REGION);
			SET_REG_VAL(res_id_set, reg);
		} else {
			reg = EDMA_DRAEH(EDMA_MASTER_SHADOW_REGION);
			SET_REG_VAL(res_id_set, reg);
			pr_debug("draeh = %x\n", dma_read(reg));
			reg = EDMA_SH_EECRH(EDMA_MASTER_SHADOW_REGION);
			SET_REG_VAL(res_id_set, reg);
		}
		result = res_id;
	}
	spin_unlock(&dma_chan_lock);
	return result;
}

static int reserve_any_edma_channel(void)
{
	int avl_id;
	int result = -1;
	u32 idx, mask;

	for (avl_id = 0; avl_id < EDMA_NUM_DMACH; ++avl_id) {
		idx = avl_id / 32;
		mask = 1 << (avl_id % 32);
		if ((~edma2event_map[idx] & mask) != 0) {
			result = reserve_one_edma_channel(avl_id, mask);
			if (result != -1)
				break;
		}
	}
	return result;
}

static int reserve_one_qdma_channel(unsigned int res_id,
				     unsigned int res_id_mask)
{
	int result = -1;
	int idx = res_id / 32;
	u32 reg;

	if (res_id >= EDMA_NUM_QDMACH)
		return result;

	spin_lock(&dma_chan_lock);
	if (((qdma_channels_arm[idx] & res_id_mask) != 0) &&
	    ((qdma_ch_use_status[idx] & res_id_mask) != 0))  {
		/* QDMA Channel Available, mark it as unavailable */
		qdma_ch_use_status[idx] &= ~res_id_mask;

		/* Enable the QDMA channel in the QRAE regs */
		reg = EDMA_QRAE(EDMA_MASTER_SHADOW_REGION);
		SET_REG_VAL(res_id_mask, reg);
		pr_debug("qdma = %x qrae = %x\n", res_id, dma_read(reg));

		result = res_id;
	}
	spin_unlock(&dma_chan_lock);
	return result;
}

static int reserve_any_qdma_channel(void)
{
	int result = -1;
	int avl_id;
	u32 mask;

	for (avl_id = 0; avl_id < EDMA_NUM_QDMACH; ++avl_id) {
		mask = 1 << (avl_id % 32);
		result = reserve_one_qdma_channel(avl_id, mask);
		if (result != -1)
			break;
	}
	return result;
}

static int reserve_one_tcc(unsigned int res_id, unsigned int res_id_mask)
{
	int result = -1;
	int idx;
	u32 reg;

	idx = res_id / 32;

	spin_lock(&dma_chan_lock);
	if (((tcc_arm[idx] & res_id_mask) != 0) &&
	    ((tcc_use_status[idx] & res_id_mask) != 0)) {
		pr_debug("tcc = %x\n", res_id);

		/* Mark it as non-available now */
		tcc_use_status[idx] &= ~res_id_mask;

		/* Enable the TCC in the DRAE/DRAEH registers */
		if (res_id < 32u) {
			reg = EDMA_DRAE(EDMA_MASTER_SHADOW_REGION);
			SET_REG_VAL(res_id_mask, reg);
			pr_debug("drae = %x\n", dma_read(reg));

			/* Add it to the Allocated TCCs list */
			allocated_tccs[0u] |= res_id_mask;
		} else {
			reg = EDMA_DRAEH(EDMA_MASTER_SHADOW_REGION);
			SET_REG_VAL(res_id_mask, reg);
			pr_debug("draeh = %x\n", dma_read(reg));

			/* Add it to the Allocated TCCs list */
			allocated_tccs[1u] |= res_id_mask;
		}
		result = res_id;
	}
	spin_unlock(&dma_chan_lock);
	return result;
}

static int reserve_any_tcc(void)
{
	int result = -1;
	int avl_id;
	u32 mask;

	for (avl_id = 0; avl_id < EDMA_NUM_TCC; ++avl_id) {
		mask = 1 << (avl_id % 32);
		if ((~(edma2event_map[avl_id / 32]) & mask) != 0) {
			result = reserve_one_tcc(avl_id, mask);
			if (result != -1)
				break;
		}
	}
	return result;
}

static int reserve_one_edma_param(unsigned int res_id, unsigned int res_id_mask)
{
	int result = -1;
	int idx;
	u32 reg;

	idx = res_id / 32;

	spin_lock(&dma_chan_lock);
	if (((param_entry_arm[idx] & res_id_mask) != 0) &&
	    ((param_entry_use_status[idx] & res_id_mask) != 0)) {
		pr_debug("edma param = %x\n", res_id);
		/* Mark it as non-available now */
		param_entry_use_status[idx] &= ~res_id_mask;
		result = res_id;

		/* Also, make the actual PARAM Set NULL */
		reg = EDMA_PARAM_OPT(res_id);
		memset((void *)IO_ADDRESS(reg), 0x00, EDMA_PARAM_ENTRY_SIZE);
	}
	spin_unlock(&dma_chan_lock);
	return result;
}

static int reserve_any_edma_param(void)
{
	int result = -1;
	int avl_id;
	u32 mask;

	for (avl_id = 0; avl_id < davinci_edma_num_param; ++avl_id) {
		mask = 1 << (avl_id % 32);
		if ((~(param_entry_reserved[avl_id / 32]) & mask) != 0) {
			result = reserve_one_edma_param(avl_id, mask);
			if (result != -1)
				break;
		}
	}
	return result;
}

static int alloc_resource(unsigned int res_id, enum resource_type res_type)
{
	int result = -1;
	unsigned int res_id_set = 1u << (res_id % 32u);

	switch (res_type) {
	case RES_DMA_CHANNEL :
		if (res_id == EDMA_DMA_CHANNEL_ANY)
			result = reserve_any_edma_channel();
		else if (res_id < EDMA_NUM_DMACH)
			result = reserve_one_edma_channel(res_id, res_id_set);
		break;
	case RES_QDMA_CHANNEL:
		if (res_id == EDMA_QDMA_CHANNEL_ANY)
			result = reserve_any_qdma_channel();
		else if (res_id < EDMA_NUM_QDMACH)
			result = reserve_one_qdma_channel(res_id, res_id_set);
		break;
	case RES_TCC:
		if (res_id == EDMA_TCC_ANY)
			result = reserve_any_tcc();
		else if (res_id < EDMA_NUM_TCC)
			result = reserve_one_tcc(res_id, res_id_set);
		break;
	case RES_PARAM_SET:
		if (res_id == DAVINCI_EDMA_PARAM_ANY)
			result = reserve_any_edma_param();
		else if (res_id < davinci_edma_num_param)
			result = reserve_one_edma_param(res_id, res_id_set);
		break;
	}
	return result;
}

static void free_resource(unsigned int res_id,
			enum resource_type res_type)
{
	unsigned int res_id_set = 0x0;

	res_id_set = (1u << (res_id % 32u));

	spin_lock(&dma_chan_lock);

	switch (res_type) {
	case RES_DMA_CHANNEL :
		if (res_id >= EDMA_NUM_DMACH)
			break;

		if (((edma_channels_arm[res_id/32]) & (res_id_set)) == 0)
			break;

		if ((~(dma_ch_use_status[res_id/32u]) & (res_id_set)) == 0)
			break;

		/* Make it as available */
		dma_ch_use_status[res_id/32u] |= res_id_set;

		/* Reset the DRAE/DRAEH bit also */
		if (res_id < 32u) {
			CLEAR_REG_VAL(res_id_set,
				      EDMA_DRAE(EDMA_MASTER_SHADOW_REGION));
		} else {
			CLEAR_REG_VAL(res_id_set,
				      EDMA_DRAEH(EDMA_MASTER_SHADOW_REGION));
		}
		break;
	case RES_QDMA_CHANNEL:
		if (res_id >= EDMA_NUM_QDMACH)
			break;

		if (((qdma_channels_arm[0]) & (res_id_set)) == 0)
			break;

		if ((~(qdma_ch_use_status[0]) & (res_id_set)) == 0)
			break;

		/* Make it as available */
		qdma_ch_use_status[0] |= res_id_set;

		/* Reset the DRAE/DRAEH bit also */
		CLEAR_REG_VAL(res_id_set, EDMA_QRAE(EDMA_MASTER_SHADOW_REGION));
		break;
	case RES_TCC:
		if (res_id >= EDMA_NUM_TCC)
			break;

		if (((tcc_arm[res_id/32]) & (res_id_set)) == 0)
			break;

		if ((~(tcc_use_status[res_id/32u]) & (res_id_set)) == 0)
			break;

		/* Make it as available */
		tcc_use_status[res_id/32u] |= res_id_set;

		/* Reset the DRAE/DRAEH bit also */
		if (res_id < 32u) {
			CLEAR_REG_VAL(res_id_set,
				      EDMA_DRAE(EDMA_MASTER_SHADOW_REGION));

			/* Remove it from the Allocated TCCs list */
			allocated_tccs[0u] &= (~res_id_set);
		} else {
			CLEAR_REG_VAL(res_id_set,
				      EDMA_DRAEH(EDMA_MASTER_SHADOW_REGION));

			/* Remove it from the Allocated TCCs list */
			allocated_tccs[1u] &= (~res_id_set);
		}
		break;
	case RES_PARAM_SET:
		if (res_id >= davinci_edma_num_param)
			break;

		if (((param_entry_arm[res_id/32]) & (res_id_set)) == 0)
			break;

		if ((~(param_entry_use_status[res_id/32u]) & (res_id_set)) == 0)
			break;

		/* Make it as available */
		param_entry_use_status[res_id/32u] |= res_id_set;
		break;
	}

	spin_unlock(&dma_chan_lock);
}

/*
 * EDMA3 CC Transfer Completion Interrupt Handler
 */
static irqreturn_t dma_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int cnt = 0;

	if (!(dma_read(EDMA_SH_IPR(0)) || dma_read(EDMA_SH_IPRH(0))))
		return IRQ_NONE;

	/* Loop while cnt < 10, breaks when no pending interrupt is found */
	while (cnt < 10u) {
		u32 status_l = dma_read(EDMA_SH_IPR(0));
		u32 status_h = dma_read(EDMA_SH_IPRH(0));
		int lch;
		int i;

		status_h &= allocated_tccs[1];
		if (!(status_l || status_h))
			break;

		lch = 0;
		while (status_l) {
			i = ffs(status_l);
			lch += i;

			/*
			 * If the user has not given any callback function
			 * while requesting the TCC, its TCC specific bit
			 * in the IPR register will NOT be cleared.
			 */
			if (intr_data[lch - 1].callback) {
				/* Clear the corresponding IPR bits */
				SET_REG_VAL(1 << (lch - 1), EDMA_SH_ICR(0));

				/* Call the callback function now */
				dma_handle_cb(lch - 1, DMA_COMPLETE);
			}
			status_l >>= i;
		}

		lch = 32;
		while (status_h) {
			i = ffs(status_h);
			lch += i;

			/*
			 * If the user has not given any callback function
			 * while requesting the TCC, its TCC specific bit
			 * in the IPRH register will NOT be cleared.
			 */
			if (intr_data[lch - 1].callback) {
				/* Clear the corresponding IPR bits */
				SET_REG_VAL(1 << (lch - 33), EDMA_SH_ICRH(0));

				/* Call the callback function now */
				dma_handle_cb(lch - 1, DMA_COMPLETE);
			}
			status_h >>= i;
		}

		cnt++;
	}

	dma_write(0x1, EDMA_SH_IEVAL(0));

	return IRQ_HANDLED;
}

/*
 * EDMA3 CC Error Interrupt Handler
 */
static irqreturn_t dma_ccerr_handler(int irq, void *dev_id,
				     struct pt_regs *regs)
{
	unsigned int mapped_tcc = 0;

	if (!(dma_read(EDMA_EMR) || dma_read(EDMA_EMRH) ||
	      dma_read(EDMA_QEMR) || dma_read(EDMA_CCERR)))
		return IRQ_NONE;

	while (1) {
		u32 status_emr = dma_read(EDMA_EMR);
		u32 status_emrh = dma_read(EDMA_EMRH);
		u32 status_qemr = dma_read(EDMA_QEMR);
		u32 status_ccerr = dma_read(EDMA_CCERR);
		int lch;
		int i;

		if (!(status_emr || status_emrh || status_qemr || status_ccerr))
			break;

		lch = 0;
		while (status_emr) {
			i = ffs(status_emr);
			lch += i;
			/* Clear the corresponding EMR bits */
			SET_REG_VAL(1 << (lch - 1), EDMA_EMCR);
			/* Clear any SER */
			SET_REG_VAL(1 << (lch - 1), EDMA_SH_SECR(0));

			mapped_tcc = edma_dma_ch_tcc_mapping[lch - 1];
			dma_handle_cb(mapped_tcc, DMA_CC_ERROR);
			status_emr >>= i;
		}

		lch = 32;
		while (status_emrh) {
			i = ffs(status_emrh);
			lch += i;
			/* Clear the corresponding IPR bits */
			SET_REG_VAL(1 << (lch - 33), EDMA_EMCRH);
			/* Clear any SER */
			SET_REG_VAL(1 << (lch - 33), EDMA_SH_SECRH(0));

			mapped_tcc = edma_dma_ch_tcc_mapping[lch - 1];
			dma_handle_cb(mapped_tcc, DMA_CC_ERROR);
			status_emrh >>= i;
		}

		lch = 0;
		while (status_qemr) {
			i = ffs(status_qemr);
			lch += i;
			/* Clear the corresponding IPR bits */
			SET_REG_VAL(1 << (lch - 1), EDMA_QEMCR);
			SET_REG_VAL(1 << (lch - 1), EDMA_SH_QSECR(0));

			mapped_tcc = edma_qdma_ch_tcc_mapping[lch - 1];
			dma_handle_cb(mapped_tcc, QDMA_EVT_MISS_ERROR);
			status_qemr >>= i;
		}


		lch = 0;
		while (status_ccerr) {
			i = ffs(status_ccerr);
			lch += i;
			/* Clear the corresponding IPR bits */
			SET_REG_VAL(1 << (lch - 1), EDMA_CCERRCLR);
			status_ccerr >>= i;
		}
	}
	dma_write(0x1, EDMA_EEVAL);

	return IRQ_HANDLED;
}

/*
 * EDMA3 Transfer Controller Error Interrupt Handler
 */
static int dma_tc_err_handler(unsigned int tc_num)
{
	u32 tcregs;
	u32 err_stat;

	if (tc_num >= davinci_edma_num_tc)
		return -EINVAL;

	tcregs = davinci_edmatc_base_addrs[tc_num];
	if (tcregs == (u32)NULL)
		return 0;

	err_stat = dma_read(EDMATC_ERRSTAT(tcregs));
	if (err_stat) {
		if (err_stat & (1 << EDMA_TC_ERRSTAT_BUSERR_SHIFT))
			dma_write(1 << EDMA_TC_ERRSTAT_BUSERR_SHIFT,
				  EDMATC_ERRCLR(tcregs));

		if (err_stat & (1 << EDMA_TC_ERRSTAT_TRERR_SHIFT))
			dma_write(1 << EDMA_TC_ERRSTAT_TRERR_SHIFT,
				  EDMATC_ERRCLR(tcregs));

		if (err_stat & (1 << EDMA_TC_ERRSTAT_MMRAERR_SHIFT))
			dma_write(1 << EDMA_TC_ERRSTAT_MMRAERR_SHIFT,
				  EDMATC_ERRCLR(tcregs));
	}
	return 0;
}

/*
 * EDMA3 TC0 Error Interrupt Handler
 */
static irqreturn_t dma_tc0_err_handler(int irq, void *dev_id,
				       struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC0 */
	dma_tc_err_handler(0);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC1 Error Interrupt Handler
 */
static irqreturn_t dma_tc1_err_handler(int irq, void *dev_id,
				      struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC1*/
	dma_tc_err_handler(1);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC2 Error Interrupt Handler
 */
static irqreturn_t dma_tc2_err_handler(int irq, void *dev_id,
				      struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC2*/
	dma_tc_err_handler(2);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC3 Error Interrupt Handler
 */
static irqreturn_t dma_tc3_err_handler(int irq, void *dev_id,
				       struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC3*/
	dma_tc_err_handler(3);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC4 Error Interrupt Handler
 */
static irqreturn_t dma_tc4_err_handler(int irq, void *dev_id,
				       struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC4*/
	dma_tc_err_handler(4);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC5 Error Interrupt Handler
 */
static irqreturn_t dma_tc5_err_handler(int irq, void *dev_id,
				       struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC5*/
	dma_tc_err_handler(5);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC6 Error Interrupt Handler
 */
static irqreturn_t dma_tc6_err_handler(int irq, void *dev_id,
				       struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC6*/
	dma_tc_err_handler(6);

	return IRQ_HANDLED;
}

/*
 * EDMA3 TC7 Error Interrupt Handler
 */
static irqreturn_t dma_tc7_err_handler(int irq, void *dev_id,
				       struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC7*/
	dma_tc_err_handler(7);

	return IRQ_HANDLED;
}

/*
 * davinci_get_qdma_channel - convert QDMA channel to logical channel
 * Arguments:
 *      ch     - input QDMA channel.
 *
 * Return: logical channel associated with QDMA channel or logical channel
 *     associated with QDMA channel 0 for out of range channel input.
 */
int davinci_get_qdma_channel(int ch)
{
	if ((ch >= 0) || (ch <= EDMA_MAX_CHANNEL))
		return (davinci_qdma_ch_map[davinci_cpu_index] + ch);
	else    /* return channel 0 for out of range values */
		return davinci_qdma_ch_map[davinci_cpu_index];
}
EXPORT_SYMBOL(davinci_get_qdma_channel);

/*
 * davinci_request_dma - requests for the DMA device passed if it is free
 *
 * Arguments:
 *      dev_id     - request for the PaRAM entry device ID
 *      dev_name   - device name
 *      callback   - pointer to the channel callback.
 *      Arguments:
 *          lch  - channel number which is the IPR bit position,
 *         indicating from which channel the interrupt arised.
 *          data - channel private data, which is received as one of the
 *         arguments in davinci_request_dma.
 *      data - private data for the channel to be requested which is used to
 *                   pass as a parameter in the callback function
 *           in IRQ handler.
 *      lch - contains the device id allocated
 *  tcc        - Transfer Completion Code, used to set the IPR register bit
 *                   after transfer completion on that channel.
 *  eventq_no  - Event Queue no to which the channel will be associated with
 *               (valid only if you are requesting for a DMA MasterChannel)
 *               Values : 0 to 7
 * INPUT:   dev_id
 * OUTPUT:  *dma_ch_out
 *
 * Return: zero on success, or corresponding error number on failure
 */
int davinci_request_dma(int dev_id, const char *dev_name,
			void (*callback) (int lch, u16 ch_status, void *data),
			void *data, int *lch, int *tcc,
			enum dma_event_q eventq_no)
{
	int ret_val = 0;
	int param_id = 0;
	int tcc_val = 0;
	u32 reg;

	pr_debug("[%s]: start\n", __func__);
	if (dev_name != NULL)
		pr_debug("dev id %d dev_name %s\n", dev_id, dev_name);

	/* Validating the arguments passed first */
	if ((!lch) || (!tcc) || (eventq_no >= davinci_edma_num_evtq)) {
		ret_val = -EINVAL;
		goto request_dma_exit;
	}

	if (dma_is_edmach(dev_id)) {
		if (alloc_resource(dev_id, RES_DMA_CHANNEL) != dev_id)  {
			/* Dma channel allocation failed */
			pr_debug("DMA channel allocation  failed \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		*lch = dev_id;
		pr_debug("DMA channel %d allocated\r\n", *lch);

		/*
		 * Allocate PaRAM Set.
		 * 64 DMA Channels are mapped to the first 64 PaRAM entries.
		 */
		if (alloc_resource(dev_id, RES_PARAM_SET) != dev_id) {
			/* PaRAM Set allocation failed */
			/*free previously allocated resources*/
			 free_resource(dev_id, RES_DMA_CHANNEL);

			pr_debug("PaRAM Set allocation  failed \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}

		/* Allocate TCC (1-to-1 mapped with the DMA channel) */
		if (alloc_resource(dev_id, RES_TCC) != dev_id)  {
			/* TCC allocation failed */
			/* free previously allocated resources */
			free_resource(dev_id, RES_PARAM_SET);
			free_resource(dev_id, RES_DMA_CHANNEL);

			pr_debug("TCC allocation failed \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}

		param_id = dev_id;
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[dev_id].param_id = param_id;
		spin_unlock(&dma_chan_lock);
		pr_debug("PaRAM Set %d allocated\r\n", param_id);

		*tcc = dev_id;
		pr_debug("TCC %d allocated\r\n", *tcc);

		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[dev_id].tcc = *tcc;
		spin_unlock(&dma_chan_lock);

		/* all resources allocated */
		/* Store the mapping b/w DMA channel and TCC first. */
		edma_dma_ch_tcc_mapping[*lch] = *tcc;

		/* Register callback function */
		register_callback((*tcc), callback, data);

		/* Map DMA channel to event queue */
		map_dma_ch_evt_queue(*lch, eventq_no);

		/* Map DMA channel to PaRAM Set */
		map_dma_ch_param_set(*lch, param_id);

	} else if (dma_is_qdmach(dev_id)) {
		/*
		 * Allocate QDMA channel first.
		 * Modify the *lch to point it to the correct QDMA
		 * channel and then check whether the same channel
		 * has been allocated or not.
		 */
		*lch = dev_id - EDMA_QDMA_CHANNEL_0;
		if (alloc_resource((*lch), RES_QDMA_CHANNEL) != (*lch)) {
			/* QDMA Channel allocation failed */
			pr_debug("QDMA channel allocation  failed \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}

		/* Requested Channel allocated successfully */
		*lch = dev_id;
		pr_debug("QDMA channel %d allocated\r\n", (*lch));

		/* Allocate param set */
		param_id = alloc_resource(DAVINCI_EDMA_PARAM_ANY,
					  RES_PARAM_SET);

		if (param_id == -1) {
			/* PaRAM Set allocation failed. */
			/*free previously allocated resources*/
			free_resource((dev_id - EDMA_QDMA_CHANNEL_0),
				      RES_QDMA_CHANNEL);

			pr_debug("PaRAM channel allocation  failed \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("PaRAM Set %d allocated\r\n", param_id);

		/* Allocate TCC */
		tcc_val = alloc_resource(*tcc, RES_TCC);
		if (tcc_val == -1) {
			/* TCC allocation failed */
			/* free previously allocated resources */
			free_resource(param_id, RES_PARAM_SET);

			free_resource((dev_id - EDMA_QDMA_CHANNEL_0),
				      RES_QDMA_CHANNEL);

			pr_debug("TCC channel allocation  failed \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}

		pr_debug("TCC %d allocated\n", tcc_val);
		*tcc = tcc_val;

		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[*lch].param_id = param_id;
		dma_ch_bound_res[*lch].tcc = *tcc;
		spin_unlock(&dma_chan_lock);

		/* all resources allocated */
		/* Store the mapping b/w QDMA channel and TCC first. */
		edma_qdma_ch_tcc_mapping[(*lch) - EDMA_QDMA_CHANNEL_0] = *tcc;

		/* Register callback function */
		register_callback((*tcc), callback, data);

		/* Map QDMA channel to event queue */
		map_qdma_ch_evt_queue((*lch) - EDMA_QDMA_CHANNEL_0, eventq_no);

		/* Map QDMA channel to PaRAM Set */
		map_qdma_ch_param_set((*lch) - EDMA_QDMA_CHANNEL_0, param_id);

	} else if (dev_id == EDMA_DMA_CHANNEL_ANY) {
		*lch = alloc_resource(EDMA_DMA_CHANNEL_ANY, RES_DMA_CHANNEL);
		if ((*lch) == -1) {
			pr_debug("EINVAL \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("EDMA_DMA_CHANNEL_ANY::channel %d allocated\n",
			    (*lch));

		/* Allocate param set tied to the DMA channel
		   (1-to-1 mapping) */
		param_id = alloc_resource((*lch), RES_PARAM_SET);
		if (param_id == -1) {
			/*
			 * PaRAM Set allocation failed, free previously
			 * allocated resources.
			 */
			pr_debug("PaRAM Set allocation failed \r\n");
			free_resource((*lch), RES_DMA_CHANNEL);
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("EDMA_DMA_CHANNEL_ANY::param %d allocated\n",
			    param_id);

		/* Allocate TCC */
		*tcc = alloc_resource(*tcc, RES_TCC);

		if (*tcc == -1) {
			/* free previously allocated resources */
			free_resource(param_id, RES_PARAM_SET);
			free_resource((*lch), RES_DMA_CHANNEL);

			pr_debug("free resource \r\n");
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("EDMA_DMA_CHANNEL_ANY:: tcc %d allocated\n",
			    (*tcc));

		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[*lch].param_id = param_id;
		dma_ch_bound_res[*lch].tcc = *tcc;
		spin_unlock(&dma_chan_lock);

		/* all resources allocated */
		/* Store the mapping b/w DMA channel and TCC first. */
		edma_dma_ch_tcc_mapping[*lch] = *tcc;

		/* Register callback function */
		register_callback((*tcc), callback, data);

		/* Map DMA channel to event queue */
		map_dma_ch_evt_queue(*lch, eventq_no);

		/* Map DMA channel to PaRAM Set */
		map_dma_ch_param_set(*lch, param_id);

	} else if (dev_id == EDMA_QDMA_CHANNEL_ANY) {
		*lch = alloc_resource(dev_id, RES_QDMA_CHANNEL);

		if ((*lch) == -1)   {
			/* QDMA Channel allocation failed */
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		/* Channel allocated successfully */
		*lch = ((*lch) + EDMA_QDMA_CHANNEL_0);

		pr_debug("EDMA_QDMA_CHANNEL_ANY::channel %d allocated\n",
			    (*lch));

		/* Allocate param set */
		param_id = alloc_resource(DAVINCI_EDMA_PARAM_ANY,
					  RES_PARAM_SET);

		if (param_id == -1) {
			/*
			 * PaRAM Set allocation failed, free previously
			 * allocated resources.
			 */
			free_resource((dev_id - EDMA_QDMA_CHANNEL_0),
				      RES_QDMA_CHANNEL);
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("EDMA_QDMA_CHANNEL_ANY::param %d allocated\n",
			    param_id);

		/* Allocate TCC */
		tcc_val = alloc_resource(*tcc, RES_TCC);

		if (tcc_val == -1) {
			/* free previously allocated resources */
			free_resource(param_id, RES_PARAM_SET);
			free_resource((dev_id - EDMA_QDMA_CHANNEL_0),
					RES_QDMA_CHANNEL);

			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("EDMA_QDMA_CHANNEL_ANY:: tcc %d allocated\n",
			    tcc_val);
		*tcc = tcc_val;

		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[*lch].param_id = param_id;
		dma_ch_bound_res[*lch].tcc = *tcc;
		spin_unlock(&dma_chan_lock);

		/* all resources allocated */
		/* Store the mapping b/w QDMA channel and TCC first. */
		edma_qdma_ch_tcc_mapping[(*lch) - EDMA_QDMA_CHANNEL_0] = *tcc;

		/* Register callback function */
		register_callback((*tcc), callback, data);

		/* Map QDMA channel to event queue */
		map_qdma_ch_evt_queue((*lch) - EDMA_QDMA_CHANNEL_0, eventq_no);

		/* Map QDMA channel to PaRAM Set */
		map_qdma_ch_param_set((*lch) - EDMA_QDMA_CHANNEL_0, param_id);

	} else if (dev_id == DAVINCI_EDMA_PARAM_ANY) {
		/* Check for the valid TCC */
		if ((*tcc) >= EDMA_NUM_TCC)   {
			/* Invalid TCC passed. */
			ret_val = -EINVAL;
			goto request_dma_exit;
		}

		/* Allocate a PaRAM Set */
		*lch = alloc_resource(dev_id, RES_PARAM_SET);
		if ((*lch) == -1) {
			ret_val = -EINVAL;
			goto request_dma_exit;
		}
		pr_debug("DAVINCI_EDMA_PARAM_ANY:: link channel %d "
			    "allocated\n", (*lch));

		/* link channel allocated */
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[*lch].param_id = *lch;
		spin_unlock(&dma_chan_lock);

		/* assign the link field to NO link. i.e 0xFFFF */
		SET_REG_VAL(0xFFFFu, EDMA_PARAM_LINK_BCNTRLD(*lch));

		/*
		 *  Check whether user has passed a NULL TCC or not.
		 *  If it is not NULL, use that value to set the OPT.TCC field
		 *  of the link channel and enable the interrupts also.
		 *  Otherwise, disable the interrupts.
		 */
		reg = EDMA_PARAM_OPT(*lch);
		if (*tcc >= 0) {
			/* Set the OPT.TCC field */
			CLEAR_REG_VAL(TCC, reg);
			SET_REG_VAL(((0x3F & (*tcc)) << 12), reg);

			/* Set TCINTEN bit in PaRAM entry */
			SET_REG_VAL(TCINTEN, reg);

			/* Store the TCC also */
			spin_lock(&dma_chan_lock);
			dma_ch_bound_res[*lch].tcc = *tcc;
			spin_unlock(&dma_chan_lock);
		} else {
			CLEAR_REG_VAL(TCINTEN, reg);
		}
		goto request_dma_exit;

	} else {
		ret_val = -EINVAL;
		goto request_dma_exit;
	}

	reg = EDMA_PARAM_OPT(param_id);
	if (callback) {
		CLEAR_REG_VAL(TCC, reg);
		SET_REG_VAL(((0x3F & (*tcc)) << 12), reg);

		/* Set TCINTEN bit in PaRAM entry */
		SET_REG_VAL(TCINTEN, reg);
	} else {
		CLEAR_REG_VAL(TCINTEN, reg);
	}

	/* assign the link field to NO link. i.e 0xFFFF */
	SET_REG_VAL(0xFFFFu, EDMA_PARAM_LINK_BCNTRLD(param_id));

request_dma_exit:
	pr_debug("[%s]: end\n", __func__);

	return ret_val;
}
EXPORT_SYMBOL(davinci_request_dma);

/*
 * davinci_free_dma - free DMA channel
 * Arguments:
 *      dev_id     - request for the PaRAM entry device ID
 *
 * Return: zero on success, or corresponding error no on failure
 */
int davinci_free_dma(int lch)
{
	int param_id = 0;
	int tcc = 0;
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);
	pr_debug("lch = %d\n", lch);

	if (lch >=0 && lch < EDMA_NUM_DMACH)   {
		/* Disable any ongoing transfer first */
		davinci_stop_dma(lch);

		/* Un-register the callback function */
		unregister_callback(lch, RES_DMA_CHANNEL);

		/* Remove DMA channel to PaRAM Set mapping */
		if (davinci_edma_chmap_exist == 1)
			CLEAR_REG_VAL(DMACH_PARAM_CLR_MASK, EDMA_DCHMAP(lch));

		param_id = dma_ch_bound_res[lch].param_id;
		tcc = dma_ch_bound_res[lch].tcc;

		pr_debug("Free ParamSet %d\n", param_id);
		free_resource(param_id, RES_PARAM_SET);
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[lch].param_id = 0;
		spin_unlock(&dma_chan_lock);

		pr_debug("Free TCC %d\n", tcc);
		free_resource(tcc, RES_TCC);
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[lch].tcc = 0;
		spin_unlock(&dma_chan_lock);

		pr_debug("Free DMA channel %d\n", lch);
		free_resource(lch, RES_DMA_CHANNEL);
	} else  if (lch >= EDMA_NUM_DMACH && lch < davinci_edma_num_param) {
		param_id = dma_ch_bound_res[lch].param_id;

		pr_debug("Free LINK channel %d\n", param_id);
		free_resource(param_id, RES_PARAM_SET);
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[lch].param_id = 0;
		spin_unlock(&dma_chan_lock);
	} else if (dma_is_qdmach(lch)) {
		/* Disable any ongoing transfer first */
		davinci_stop_dma(lch);

		/* Un-register the callback function */
		unregister_callback(lch, RES_QDMA_CHANNEL);

		/* Remove QDMA channel to PaRAM Set mapping */
		CLEAR_REG_VAL(QDMACH_PARAM_CLR_MASK,
			      EDMA_QCHMAP(lch - EDMA_QDMA_CHANNEL_0));
		/* Reset trigger word */
		CLEAR_REG_VAL(QDMACH_TRWORD_CLR_MASK,
			      EDMA_QCHMAP(lch - EDMA_QDMA_CHANNEL_0));

		param_id = dma_ch_bound_res[lch].param_id;
		tcc = dma_ch_bound_res[lch].tcc;

		pr_debug("Free ParamSet %d\n", param_id);
		free_resource(param_id, RES_PARAM_SET);
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[lch].param_id = 0;
		spin_unlock(&dma_chan_lock);

		pr_debug("Free TCC %d\n", tcc);
		free_resource(tcc, RES_TCC);
		spin_lock(&dma_chan_lock);
		dma_ch_bound_res[lch].tcc = 0;
		spin_unlock(&dma_chan_lock);

		pr_debug("Free QDMA channel %d\n", lch);
		free_resource(lch - EDMA_QDMA_CHANNEL_0, RES_QDMA_CHANNEL);
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_free_dma);

/*
 * DMA source parameters setup
 * Arguments:
 *     lch - logical channel number
 *     src_port - Source port address
 *     mode - indicates wether addressing mode is FIFO
 */
int davinci_set_dma_src_params(int lch, u32 src_port,
			       enum address_mode mode, enum fifo_width width)
{
	int param_id = 0;
	u32 reg;

	pr_debug("[%s]: start\n", __func__);

	if (!(lch >= 0 && lch < edma_max_logical_ch)) {
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	/* Address in FIFO mode not 32 bytes aligned */
	if ((mode) && ((src_port & 0x1Fu) != 0)) {
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	param_id = dma_ch_bound_res[lch].param_id;

	/* Set the source port address in source register of PaRAM structure */
	dma_write(src_port, EDMA_PARAM_SRC(param_id));

	/* Set the FIFO addressing mode */
	if (mode) {
		reg = EDMA_PARAM_OPT(param_id);
		/* reset SAM and FWID */
		CLEAR_REG_VAL(SAM | EDMA_FWID, reg);
		/* set SAM and program FWID */
		SET_REG_VAL(SAM | ((width & 0x7) << 8), reg);
	}
	pr_debug("[%s]: end\n", __func__);
	return 0;
}
EXPORT_SYMBOL(davinci_set_dma_src_params);

/*
 * DMA destination parameters setup
 * Arguments:
 *     lch - logical channel number or PaRAM device
 *     dest_port - destination port address
 *     mode - indicates wether addressing mode is FIFO
 */

int davinci_set_dma_dest_params(int lch, u32 dest_port,
				enum address_mode mode, enum fifo_width width)
{
	int param_id = 0;
	u32 reg;

	pr_debug("[%s]: start\n", __func__);

	if (!(lch >= 0 && lch < edma_max_logical_ch)) {
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	if ((mode) && ((dest_port & 0x1Fu) != 0))   {
		/* Address in FIFO mode not 32 bytes aligned */
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	param_id = dma_ch_bound_res[lch].param_id;

	/* Set the dest port address in dest register of PaRAM structure */
	dma_write(dest_port, EDMA_PARAM_DST(param_id));

	/* Set the FIFO addressing mode */
	if (mode) {
		reg = EDMA_PARAM_OPT(param_id);
		/* reset DAM and FWID */
		CLEAR_REG_VAL((DAM | EDMA_FWID), reg);
		/* set DAM and program FWID */
		SET_REG_VAL((DAM | ((width & 0x7) << 8)), reg);
	}
	pr_debug("[%s]: end\n", __func__);
	return 0;
}
EXPORT_SYMBOL(davinci_set_dma_dest_params);

/*
 * DMA source index setup
 * Arguments:
 *     lch - logical channel number or param device
 *     srcbidx - source B-register index
 *     srccidx - source C-register index
 */

int davinci_set_dma_src_index(int lch, u16 src_bidx, u16 src_cidx)
{
	int param_id = 0;
	u32 reg;

	pr_debug("[%s]: start\n", __func__);
	if (!(lch >= 0 && lch < edma_max_logical_ch)) {
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	param_id = dma_ch_bound_res[lch].param_id;

	reg = EDMA_PARAM_SRC_DST_BIDX(param_id);
	CLEAR_REG_VAL(0xffff, reg);
	SET_REG_VAL(src_bidx, reg);

	reg = EDMA_PARAM_SRC_DST_CIDX(param_id);
	CLEAR_REG_VAL(0xffff, reg);
	SET_REG_VAL(src_cidx, reg);

	pr_debug("[%s]: end\n", __func__);
	return 0;
}
EXPORT_SYMBOL(davinci_set_dma_src_index);

/*
 * DMA destination index setup
 * Arguments:
 *     lch - logical channel number or param device
 *     dest_bidx - source B-register index
 *     dest_cidx - source C-register index
 */

int davinci_set_dma_dest_index(int lch, u16 dest_bidx, u16 dest_cidx)
{
	int param_id = 0;
	u32 reg;

	pr_debug("[%s]: start\n", __func__);
	if (!(lch >= 0 && lch < edma_max_logical_ch)) {
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	param_id = dma_ch_bound_res[lch].param_id;

	reg = EDMA_PARAM_SRC_DST_BIDX(param_id);
	CLEAR_REG_VAL(0xffff0000, reg);
	SET_REG_VAL(dest_bidx << 16, reg);

	reg = EDMA_PARAM_SRC_DST_CIDX(param_id);
	CLEAR_REG_VAL(0xffff0000, reg);
	SET_REG_VAL(dest_cidx << 16, reg);

	pr_debug("[%s]: end\n", __func__);
	return 0;
}
EXPORT_SYMBOL(davinci_set_dma_dest_index);

/*
 * DMA transfer parameters setup
 * ARGUMENTS:
 *      lch  - channel or param device for configuration of aCount, bCount and
 *         cCount regs.
 *      acnt - acnt register value to be configured
 *      bcnt - bcnt register value to be configured
 *      ccnt - ccnt register value to be configured
 */
int davinci_set_dma_transfer_params(int lch, u16 acnt, u16 bcnt, u16 ccnt,
				    u16 bcntrld, enum sync_dimension sync_mode)
{
	u32 reg;
	int param_id = 0;
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);

	if ((lch >= 0) && (lch < edma_max_logical_ch)) {

		param_id = dma_ch_bound_res[lch].param_id;

		reg = EDMA_PARAM_LINK_BCNTRLD(param_id);
		CLEAR_REG_VAL(0xffff0000, reg);
		SET_REG_VAL(((u32)bcntrld & 0xffff) << 16, reg);

		reg = EDMA_PARAM_OPT(param_id);
		if (sync_mode == ASYNC)
			CLEAR_REG_VAL(SYNCDIM, reg);
		else
			SET_REG_VAL(SYNCDIM, reg);

		/* Set the acount, bcount, ccount registers */
		dma_write((((u32)bcnt & 0xffff) << 16) | acnt,
			  EDMA_PARAM_A_B_CNT(param_id));
		dma_write(ccnt, EDMA_PARAM_CCNT(param_id));
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_set_dma_transfer_params);

/*
 * davinci_set_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 */
int davinci_set_dma_params(int lch, struct paramentry_descriptor *d)
{
	int param_id = 0;
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);
	if (d && (lch >= 0) && (lch < edma_max_logical_ch)) {
		param_id = dma_ch_bound_res[lch].param_id;

		dma_write(d->opt, EDMA_PARAM_OPT(param_id));
		dma_write(d->src, EDMA_PARAM_SRC(param_id));
		dma_write(d->a_b_cnt, EDMA_PARAM_A_B_CNT(param_id));
		dma_write(d->dst, EDMA_PARAM_DST(param_id));
		dma_write(d->src_dst_bidx, EDMA_PARAM_SRC_DST_BIDX(param_id));
		dma_write(d->link_bcntrld, EDMA_PARAM_LINK_BCNTRLD(param_id));
		dma_write(d->src_dst_cidx, EDMA_PARAM_SRC_DST_CIDX(param_id));
		dma_write(d->ccnt, EDMA_PARAM_CCNT(param_id));
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_set_dma_params);

/*
 * davinci_get_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 */
int davinci_get_dma_params(int lch, struct paramentry_descriptor *d)
{
	int param_id = 0;

	pr_debug("[%s]: start\n", __func__);
	if ((d == NULL) || (lch >= edma_max_logical_ch)) {
		pr_debug("[%s]: end\n", __func__);
		return -EINVAL;
	}

	param_id = dma_ch_bound_res[lch].param_id;

	d->opt = dma_read(EDMA_PARAM_OPT(param_id));
	d->src = dma_read(EDMA_PARAM_SRC(param_id));
	d->a_b_cnt = dma_read(EDMA_PARAM_A_B_CNT(param_id));
	d->dst = dma_read(EDMA_PARAM_DST(param_id));
	d->src_dst_bidx = dma_read(EDMA_PARAM_SRC_DST_BIDX(param_id));
	d->link_bcntrld = dma_read(EDMA_PARAM_LINK_BCNTRLD(param_id));
	d->src_dst_cidx = dma_read(EDMA_PARAM_SRC_DST_CIDX(param_id));
	d->ccnt = dma_read(EDMA_PARAM_CCNT(param_id));
	pr_debug("[%s]: end\n", __func__);
	return 0;
}
EXPORT_SYMBOL(davinci_get_dma_params);

/*
 * davinci_start_dma - starts the DMA on the channel passed
 * Arguments:
 *     lch - logical channel number
 */
int davinci_start_dma(int lch)
{
	int ret = 0;
	int mask = 0;

	pr_debug("[%s]: start\n", __func__);
	if (dma_is_edmach(lch)) {
		/* DMA Channel */
		if (edmach_has_event(lch)) {

			pr_debug("ER=%d\n", dma_read(EDMA_SH_ER(0)));

			if (lch < 32)   {
				mask = 1 << lch;
				/* Clear any pedning error */
				dma_write(mask, EDMA_EMCR);
				/* Clear any SER */
				dma_write(mask, EDMA_SH_SECR(0));
				dma_write(mask, EDMA_SH_EESR(0));
				dma_write(mask, EDMA_SH_ECR(0));
			} else {
				mask = 1 << (lch - 32);
				/* Clear any pedning error */
				dma_write(mask, EDMA_EMCRH);
				/* Clear any SER */
				dma_write(mask, EDMA_SH_SECRH(0));
				dma_write(mask, EDMA_SH_EESRH(0));
				dma_write(mask, EDMA_SH_ECRH(0));
			}
			pr_debug("EER=%d\n", dma_read(EDMA_SH_EER(0)));
		} else {
			pr_debug("ESR=%x\n", dma_read(EDMA_SH_ESR(0)));

			if (lch < 32)
				dma_write(1 << lch, EDMA_SH_ESR(0));
			else
				dma_write(1 << (lch - 32), EDMA_SH_ESRH(0));
		}
	} else if (dma_is_qdmach(lch)) {
		/* QDMA Channel */
		dma_write(1 << (lch - EDMA_QDMA_CHANNEL_0), EDMA_SH_QEESR(0));
	} else {
		ret = EINVAL;
	}
	pr_debug("[%s]: end\n", __func__);
	return ret;
}
EXPORT_SYMBOL(davinci_start_dma);

/*
 * davinci_stop_dma - stops the DMA on the channel passed
 * Arguments:
 *     lch - logical channel number
 */
int davinci_stop_dma(int lch)
{
	u32 reg;
	u32 mask;
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);

	if (lch >= 0 && lch < EDMA_NUM_DMACH) {
		/* DMA Channel */
		if (lch < 32) {
			mask = 1 << lch;
			if (edmach_has_event(lch)) {
				reg = EDMA_SH_EECR(0);
				dma_write(mask, reg);
				CLEAR_EVENT(mask, EDMA_SH_ER(0),
					    EDMA_SH_ECR(0));
			}
			CLEAR_EVENT(mask, EDMA_SH_SER(0), EDMA_SH_SECR(0));
			CLEAR_EVENT(mask, EDMA_EMR, EDMA_EMCR);
		} else {
			mask = 1 << (lch - 32);
			if (edmach_has_event(lch)) {
				reg = EDMA_SH_EECRH(0);
				dma_write(mask, reg);
				CLEAR_EVENT(mask, EDMA_SH_ERH(0),
					    EDMA_SH_ECRH(0));
			}
			CLEAR_EVENT(mask, EDMA_SH_SERH(0), EDMA_SH_SECRH(0));
			CLEAR_EVENT(mask, EDMA_EMRH, EDMA_EMCRH);
		}
		pr_debug("EER=%d\n", dma_read(EDMA_SH_EER(0)));
	} else if (dma_is_qdmach(lch)) {
		/* QDMA Channel */
		dma_write(1 << (lch - EDMA_QDMA_CHANNEL_0), EDMA_QEECR);
		pr_debug("QER=%d\n", dma_read(EDMA_QER));
		pr_debug("QEER=%d\n", dma_read(EDMA_QEER));
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_stop_dma);

/*
 * davinci_dma_link_lch - link two logical channels passed through by linking
 *			  the link field of head to the param pointed by the
 * 			  lch_queue.
 * Arguments:
 *     lch_head  - logical channel number in which the link field is linked
 *                 to the PaRAM pointed to by lch_queue
 *     lch_queue - logical channel number or the PaRAM entry number which is
 *                 to be linked to the lch_head
 */
int davinci_dma_link_lch(int lch_head, int lch_queue)
{
	u16 link;
	u32 reg;
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);

	if ((lch_head  >=0 && lch_head  < edma_max_logical_ch) ||
	    (lch_queue >=0 && lch_queue < edma_max_logical_ch)) {
		unsigned int param1_id = 0;
		unsigned int param2_id = 0;

		param1_id = dma_ch_bound_res[lch_head].param_id;
		param2_id = dma_ch_bound_res[lch_queue].param_id;

		/* program LINK */
		link = (u16) IO_ADDRESS(EDMA_PARAM_OPT(param2_id));

		reg = EDMA_PARAM_LINK_BCNTRLD(param1_id);
		CLEAR_REG_VAL(0xffff, reg);
		SET_REG_VAL(link, reg);
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_dma_link_lch);

/*
 * davinci_dma_unlink_lch - unlink the two logical channels passed through by
 *			    setting the link field of head to 0xffff.
 * Arguments:
 *     lch_head - logical channel number from which the link field is
 *                to be removed
 *     lch_queue - logical channel number or the PaRAM entry number,
 *                 which is to be unlinked from lch_head
 */
int davinci_dma_unlink_lch(int lch_head, int lch_queue)
{
	u32 reg;
	unsigned int param_id = 0;
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);

	if ((lch_head  >=0 && lch_head  < edma_max_logical_ch) ||
	    (lch_queue >=0 && lch_queue < edma_max_logical_ch)) {
		param_id = dma_ch_bound_res[lch_head].param_id;
		reg = EDMA_PARAM_LINK_BCNTRLD(param_id);
		SET_REG_VAL(0xffff, reg);
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_dma_unlink_lch);

/*
 * davinci_dma_chain_lch - chains two logical channels passed through
 * ARGUMENTS:
 * lch_head - logical channel number which will trigger the chained channel
 *              'lch_queue'
 * lch_queue - logical channel number which will be triggered by 'lch_head'
 */
int davinci_dma_chain_lch(int lch_head, int lch_queue)
{
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);

	if ((lch_head  >=0 && lch_head  < edma_max_logical_ch) ||
	    (lch_queue >=0 && lch_queue < edma_max_logical_ch)) {
		unsigned int param_id = 0;

		param_id = dma_ch_bound_res[lch_head].param_id;

		/* set TCCHEN */
		SET_REG_VAL(TCCHEN, EDMA_PARAM_OPT(param_id));

		/* Program TCC */
		CLEAR_REG_VAL(TCC, EDMA_PARAM_OPT(param_id));
		SET_REG_VAL((lch_queue & 0x3f) << 12, EDMA_PARAM_OPT(param_id));
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_dma_chain_lch);

/*
 * davinci_dma_unchain_lch - unchain the logical channels passed through
 * ARGUMENTS:
 * lch_head - logical channel number from which the link field is to be removed
 * lch_queue - logical channel number or the PaRAM entry number which is to be
 *             unlinked from lch_head
 */
int davinci_dma_unchain_lch(int lch_head, int lch_queue)
{
	int ret_code = 0;

	pr_debug("[%s]: start\n", __func__);

	if ((lch_head  >=0 && lch_head  < edma_max_logical_ch) ||
	    (lch_queue >=0 && lch_queue < edma_max_logical_ch)) {
		unsigned int param_id = 0;

		param_id = dma_ch_bound_res[lch_head].param_id;

		/* reset TCCHEN */
		SET_REG_VAL(~TCCHEN, EDMA_PARAM_OPT(param_id));
		/* reset ITCCHEN */
		SET_REG_VAL(~ITCCHEN, EDMA_PARAM_OPT(param_id));
	} else
		ret_code = -EINVAL;

	pr_debug("[%s]: end\n", __func__);
	return ret_code;
}
EXPORT_SYMBOL(davinci_dma_unchain_lch);

/*
 * davinci_clean_channel - clean PaRAM entry and bring back EDMA to initial
 *			   state if media has been removed before EDMA has
 *			   finished.  It is useful for removable media.
 * Arguments:
 *     lch - logical channel number
 */
void davinci_clean_channel(int lch)
{
	u32 mask, value = 0, count;

	pr_debug("[%s]: start\n", __func__);

	if (lch < 0 || lch >= EDMA_NUM_DMACH)
		return;
	if (lch < 32) {
		pr_debug("EMR =%d\n", dma_read(EDMA_EMR));
		mask = 1 << lch;
		dma_write(mask, EDMA_SH_ECR(0));
		/* Clear the corresponding EMR bits */
		dma_write(mask, EDMA_EMCR);
		/* Clear any SER */
		dma_write(mask, EDMA_SH_SECR(0));
		/* Clear any EER */
		dma_write(mask, EDMA_SH_EECR(0));

	} else {
		pr_debug("EMRH =%d\n", dma_read(EDMA_EMRH));
		mask = 1 << (lch - 32);
		dma_write(mask, EDMA_SH_ECRH(0));
		/* Clear the corresponding EMRH bits */
		dma_write(mask, EDMA_EMCRH);
		/* Clear any SER */
		dma_write(mask, EDMA_SH_SECRH(0));
		/* Clear any EERH */
		dma_write(mask, EDMA_SH_EECRH(0));
	}

	for (count = 0; count < davinci_edma_num_evtq; count++)
		value |= (1u << count);

	dma_write((1 << 16) | value, EDMA_CCERRCLR);

	pr_debug("[%s]: end\n", __func__);
}
EXPORT_SYMBOL(davinci_clean_channel);

/*
 * davinci_dma_getposition - returns the current transfer points for the DMA
 * source and destination
 * Arguments:
 *     lch - logical channel number
 *     src - source port position
 *     dst - destination port position
 */
void davinci_dma_getposition(int lch, dma_addr_t *src, dma_addr_t *dst)
{
	struct paramentry_descriptor temp;

	davinci_get_dma_params(lch, &temp);
	if (src != NULL)
		*src = temp.src;
	if (dst != NULL)
		*dst = temp.dst;
}
EXPORT_SYMBOL(davinci_dma_getposition);

/*
 * EDMA3 Initialisation on DaVinci
 */
int __init arch_dma_init(void)
{
	struct edma_map *q_pri, *q_wm, *q_tc;
	unsigned int i = 0u;
	u32 reg;

	if (cpu_is_davinci_dm6467()) {
		davinci_edma_num_evtq = EDMA_DM646X_NUM_EVQUE;
		davinci_edma_chmap_exist = EDMA_DM646X_CHMAP_EXIST;
		davinci_edma_num_tc = EDMA_DM646X_NUM_TC;
		davinci_edmatc_base_addrs = dm646x_edmatc_base_addrs;
		edma_max_logical_ch = EDMA_NUM_QDMACH +
				      EDMA_DM646X_NUM_PARAMENTRY;
		davinci_edma_num_param = EDMA_DM646X_NUM_PARAMENTRY;
		edma2event_map = dm646x_dma_ch_hw_event_map;

		edma_channels_arm = dm646x_edma_channels_arm;
		qdma_channels_arm = dm646x_qdma_channels_arm;
		param_entry_arm = dm646x_param_entry_arm;
		tcc_arm = dm646x_tcc_arm;
		param_entry_reserved = dm646x_param_entry_reserved;

		q_pri = dm646x_queue_priority_mapping;
		q_tc = dm646x_queue_tc_mapping;
		q_wm = dm646x_queue_watermark_level;

		davinci_cpu_index = 1;
	} else if (cpu_is_davinci_dm355()) {
		davinci_edma_num_evtq = EDMA_DM355_NUM_EVQUE;
		davinci_edma_chmap_exist = EDMA_DM355_CHMAP_EXIST;
		davinci_edma_num_tc = EDMA_DM355_NUM_TC;
		davinci_edmatc_base_addrs = dm355_edmatc_base_addrs;
		edma_max_logical_ch = EDMA_NUM_QDMACH +
				      EDMA_DM355_NUM_PARAMENTRY;
		davinci_edma_num_param = EDMA_DM355_NUM_PARAMENTRY;
		edma2event_map = dm355_dma_ch_hw_event_map;

		edma_channels_arm = dm355_edma_channels_arm;
		qdma_channels_arm = dm355_qdma_channels_arm;
		param_entry_arm = dm355_param_entry_arm;
		tcc_arm = dm355_tcc_arm;
		param_entry_reserved = dm355_param_entry_reserved;

		q_pri = dm355_queue_priority_mapping;
		q_tc = dm355_queue_tc_mapping;
		q_wm = dm355_queue_watermark_level;

		davinci_cpu_index = 2;
	} else {
		davinci_edma_num_evtq = EDMA_DM644X_NUM_EVQUE;
		davinci_edma_chmap_exist = EDMA_DM644X_CHMAP_EXIST;
		davinci_edma_num_tc = EDMA_DM644X_NUM_TC;
		davinci_edmatc_base_addrs = dm644x_edmatc_base_addrs;
		edma_max_logical_ch = EDMA_NUM_QDMACH +
				      EDMA_DM644X_NUM_PARAMENTRY;
		davinci_edma_num_param = EDMA_DM644X_NUM_PARAMENTRY;
		edma2event_map = dm644x_dma_ch_hw_event_map;

		edma_channels_arm = dm644x_edma_channels_arm;
		qdma_channels_arm = dm644x_qdma_channels_arm;
		param_entry_arm = dm644x_param_entry_arm;
		tcc_arm = dm644x_tcc_arm;
		param_entry_reserved = dm644x_param_entry_reserved;

		q_pri = dm644x_queue_priority_mapping;
		q_tc = dm644x_queue_tc_mapping;
		q_wm = dm644x_queue_watermark_level;

		davinci_cpu_index = 0;
	}
	dma_ch_bound_res =
		kmalloc(sizeof(struct edma3_ch_bound_res) * edma_max_logical_ch,
			 GFP_KERNEL);

	/* Reset global data */
	/* Reset the DCHMAP registers if they exist */
	if (davinci_edma_chmap_exist == 1)
		memset((void *)IO_ADDRESS(EDMA_DCHMAP(0)), 0x00,
		       EDMA_DCHMAP_SIZE);

	/* Reset book-keeping info */
	memset(dma_ch_bound_res, 0x00u,  (sizeof(struct edma3_ch_bound_res) *
		edma_max_logical_ch));
	memset(intr_data, 0x00u, sizeof(intr_data));
	memset(edma_dma_ch_tcc_mapping, 0x00u, sizeof(edma_dma_ch_tcc_mapping));
	memset(edma_qdma_ch_tcc_mapping, 0x00u,
	       sizeof(edma_qdma_ch_tcc_mapping));

	memset((void *)IO_ADDRESS(EDMA_PARAM_OPT(0)), 0x00, EDMA_PARAM_SIZE);

	/* Clear Error Registers */
	dma_write(0xFFFFFFFFu, EDMA_EMCR);
	dma_write(0xFFFFFFFFu, EDMA_EMCRH);
	dma_write(0xFFFFFFFFu, EDMA_QEMCR);
	dma_write(0xFFFFFFFFu, EDMA_CCERRCLR);

	for (i = 0; i < davinci_edma_num_evtq; i++) {
		/* Event Queue to TC mapping, if it exists */
		if (EDMA_EVENT_QUEUE_TC_MAPPING == 1u) {
			reg = EDMA_QUETCMAP;
			CLEAR_REG_VAL(QUETCMAP_CLR_MASK(q_tc[i].param1), reg);
			SET_REG_VAL(QUETCMAP_SET_MASK(q_tc[i].param1,
						      q_tc[i].param2), reg);
		}

		/* Event Queue Priority */
		reg = EDMA_QUEPRI;
		CLEAR_REG_VAL(QUEPRI_CLR_MASK(q_pri[i].param1), reg);
		SET_REG_VAL(QUEPRI_SET_MASK(q_pri[i].param1, q_pri[i].param2),
			    reg);

		/* Event Queue Watermark Level */
		reg = EDMA_QWMTHRA;
		CLEAR_REG_VAL(QUEWMTHR_CLR_MASK(q_wm[i].param1), reg);
		SET_REG_VAL(QUEWMTHR_SET_MASK(q_wm[i].param1, q_wm[i].param2),
			    reg);
	}

	/* Reset the Allocated TCCs Array first. */
	allocated_tccs[0u] = 0x0u;
	allocated_tccs[1u] = 0x0u;

	/* Clear region specific Shadow Registers */
	reg = EDMA_MASTER_SHADOW_REGION;
	dma_write(edma_channels_arm[0] | tcc_arm[0], EDMA_SH_ECR(reg));
	dma_write(edma_channels_arm[1] | tcc_arm[1], EDMA_SH_ECRH(reg));
	dma_write(edma_channels_arm[0] | tcc_arm[0], EDMA_SH_EECR(reg));
	dma_write(edma_channels_arm[1] | tcc_arm[1], EDMA_SH_EECRH(reg));
	dma_write(edma_channels_arm[0] | tcc_arm[0], EDMA_SH_SECR(reg));
	dma_write(edma_channels_arm[1] | tcc_arm[1], EDMA_SH_SECRH(reg));
	dma_write(edma_channels_arm[0] | tcc_arm[0], EDMA_SH_IECR(reg));
	dma_write(edma_channels_arm[1] | tcc_arm[1], EDMA_SH_IECRH(reg));
	dma_write(edma_channels_arm[0] | tcc_arm[0], EDMA_SH_ICR(reg));
	dma_write(edma_channels_arm[1] | tcc_arm[1], EDMA_SH_ICRH(reg));
	dma_write(qdma_channels_arm[0], EDMA_SH_QEECR(reg));
	dma_write(qdma_channels_arm[0], EDMA_SH_QSECR(reg));

	/* Reset Region Access Enable Registers for the Master Shadow Region */
	dma_write(0, EDMA_DRAE(EDMA_MASTER_SHADOW_REGION));
	dma_write(0, EDMA_DRAEH(EDMA_MASTER_SHADOW_REGION));
	dma_write(0, EDMA_QRAE(EDMA_MASTER_SHADOW_REGION));

	if (register_dma_interrupts())
		return -EINVAL;


	spin_lock_init(&dma_chan_lock);

	return 0;
}

/*
 * Register different ISRs with the underlying OS
 */
int register_dma_interrupts(void)
{
	int result = 0;
	int i;
	unsigned int *tc_error_int;

	if (cpu_is_davinci_dm6467())
		tc_error_int = dm646x_tc_error_int;
	else if (cpu_is_davinci_dm355())
		tc_error_int = dm355_tc_error_int;
	else
		tc_error_int = dm644x_tc_error_int;

	result = request_irq(EDMA_XFER_COMPLETION_INT, dma_irq_handler, 0,
			     "EDMA Completion", NULL);
	if (result < 0) {
		pr_debug("request_irq failed for dma_irq_handler, "
			    "error=%d\n", result);
		return result;
	}

	result = request_irq(EDMA_CC_ERROR_INT, dma_ccerr_handler, 0,
			     "EDMA CC Err", NULL);
	if (result < 0) {
		pr_debug("request_irq failed for dma_ccerr_handler, "
			    "error=%d\n", result);
		return result;
	}

	for (i = 0; i < davinci_edma_num_tc; i++) {
		snprintf(tc_error_int_name[i], 19, "EDMA TC%d Error", i);
		result = request_irq(tc_error_int[i], ptr_edmatc_isrs[i], 0,
				     tc_error_int_name[i], NULL);
		if (result < 0) {
			pr_debug("request_irq failed for dma_tc%d "
				    "err_handler\n", i);
			pr_debug("error = %d \n", result);
			return result;
		}
	}

	return result;
}
arch_initcall(arch_dma_init);

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

