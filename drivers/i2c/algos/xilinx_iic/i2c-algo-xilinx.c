/*
 * i2c-algo-xilinx.c
 *
 * Xilinx IIC Adapter component to interface IIC component to Linux
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

/*
 * I2C drivers are split into two pieces: the adapter and the algorithm.
 * The adapter is responsible for actually manipulating the hardware and
 * the algorithm is the layer above that that handles the higher level
 * tasks such as transmitting or receiving a buffer.  The best example
 * (in my opinion) of this is the bit banging algorithm has a number of
 * different adapters that can plug in under it to actually wiggle the
 * SDA and SCL.
 *
 * The interesting part is that the drivers Xilinx provides with their
 * IP are also split into two pieces where one part is the OS
 * independent code and the other part is the OS dependent code.  All of
 * the other sources in this directory are the OS independent files as
 * provided by Xilinx with no changes made to them.
 *
 * As it turns out, this maps quite well into the I2C driver philosophy.
 * This file is the I2C algorithm that communicates with the Xilinx OS
 * independent function that will serve as our I2C adapter.  The
 * unfortunate part is that the term "adapter" is overloaded in our
 * context.  Xilinx refers to the OS dependent part of a driver as an
 * adapter.  So from an I2C driver perspective, this file is not an
 * adapter; that role is filled by the Xilinx OS independent files.
 * From a Xilinx perspective, this file is an adapter; it adapts their
 * OS independent code to Linux.
 *
 * Another thing to consider is that the Xilinx OS dependent code knows
 * nothing about Linux I2C adapters, so even though this file is billed
 * as the I2C algorithm, it takes care of the i2c_adapter structure.
 *
 * Fortunately, naming conventions will give you a clue as to what comes
 * from where.  Functions beginning with XIic_ are provided by the
 * Xilinx OS independent files.  Functions beginning with i2c_ are
 * provided by the I2C Linux core.  All functions in this file that are
 * called by Linux have names that begin with xiic_.  The functions in
 * this file that have Handler in their name are registered as callbacks
 * with the underlying Xilinx OS independent layer.  Any other functions
 * are static helper functions.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/xilinx_devices.h>

#include <asm/delay.h>
#include <asm/io.h>
#include <asm/irq.h>

#include "xbasic_types.h"
#include "xiic.h"
#include "xiic_i.h"

MODULE_AUTHOR("MontaVista Software, Inc. <source@mvista.com>");
MODULE_DESCRIPTION("Xilinx IIC driver");
MODULE_LICENSE("GPL");
static int scan = 0;		/* have a look at what's hanging 'round */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
MODULE_PARM(scan, "i");
#else
module_param(scan, int, 0644);
#endif
MODULE_PARM_DESC(scan, "Scan for active chips on the bus");

/* SAATODO: actually use these? */
#define XIIC_TIMEOUT           100
#define XIIC_RETRY             3

#define XILINX_IIC             "xilinx_iic"

static int __init xilinx_iic_probe(struct of_device *of_dev,
				   const struct of_device_id *match);
static int __exit xilinx_iic_remove(struct of_device *dev);

/* Our private per device data. */
struct xiic_data {
	struct i2c_adapter adap;	/* The Linux I2C core data  */
	int index;		/* index taken from platform_device */
	struct completion complete;	/* for waiting for interrupts */
	u32 base;		/* base memory address */
	unsigned int irq;	/* device IRQ number    */
	volatile u32 transmit_intr_flag;	/* semaphore across task and interrupt - ECM */
	volatile u32 receive_intr_flag;	/* semaphore across task and interrupt - ECM */
	volatile u32 status_intr_flag;	/* semaphore across task and interrupt - ECM */
	/*
	 * The underlying OS independent code needs space as well.  A
	 * pointer to the following XIic structure will be passed to
	 * any XIic_ function that requires it.  However, we treat the
	 * data as an opaque object in this file (meaning that we never
	 * reference any of the fields inside of the structure).
	 */
	XIic Iic;

	/*
	 * The following bit fields are used to keep track of what
	 * all has been done to initialize the xiic_dev to make
	 * error handling out of probe() easier.
	 */
	unsigned int reqirq:1;	/* Has request_irq() been called? */
	unsigned int remapped:1;	/* Has ioremap() been called? */
	unsigned int started:1;	/* Has XIic_Start() been called? */
	unsigned int added:1;	/* Has i2c_add_adapter() been called? */
};

/*******************************************************************************
 * This configuration stuff should become unnecessary after EDK version 8.x is
 * released.
 ******************************************************************************/

static DECLARE_MUTEX(cfg_sem);
static int
xiic_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
	struct xiic_data *dev = (struct xiic_data *)i2c_adap;
	struct i2c_msg *pmsg;
	u32 options;
	int i, retries;
	XStatus Status;
	u32 writeop;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		if (!pmsg->len)	/* If length is zero */
			continue;	/* on to the next request. */

		/*
		 * This code checks up to 16 times for the
		 * bus busy condition.
		 */
		retries = 4;
		while ((XIic_IsIicBusy(&dev->Iic) == TRUE) && (retries-- != 0)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ / 250);
		}

		/* If bus is still busy, bail */
		if (XIic_IsIicBusy(&dev->Iic) == TRUE) {
			printk(KERN_WARNING
			       "%s #%d: Could not talk to device 0x%2x (%d), bus always busy, trying to reset\n",
			       dev->adap.name, dev->index, pmsg->addr,
			       dev->status_intr_flag);

			/* Try stopping, reseting and starting device to clear condition
			 */
			if (XIic_Stop(&dev->Iic) != XST_SUCCESS) {
				/* The bus was in use.. */
				printk(KERN_WARNING
				       "%s #%d: Could not stop device. Restart from higher layer.\n",
				       dev->adap.name, dev->index);
				return -ENXIO;
			} else {
				XIic_Reset(&dev->Iic);
				if (XIic_Start(&dev->Iic) != XST_SUCCESS) {
					printk(KERN_ERR
					       "%s #%d: Could not start device.\n",
					       dev->adap.name, dev->index);
					return -ENODEV;
				}

				return -ENXIO;
			}
		}

		options = 0;
		if (pmsg->flags & I2C_M_TEN)
			options |= XII_SEND_10_BIT_OPTION;
		XIic_SetOptions(&dev->Iic, options);

		if (XIic_SetAddress(&dev->Iic, XII_ADDR_TO_SEND_TYPE,
				    pmsg->addr) != XST_SUCCESS) {
			printk(KERN_WARNING
			       "%s #%d: Could not set address to 0x%2x.\n",
			       dev->adap.name, dev->index, pmsg->addr);
			return -EIO;
		}

		dev->transmit_intr_flag = 0xFFFFFFFF;
		dev->receive_intr_flag = 0xFFFFFFFF;
		dev->status_intr_flag = 0xFFFFFFFF;

		/* set the writeop flag to 0 so the adapter does not wait
		 * at bottom of loop
		 */
		writeop = 0;

		dev->Iic.Stats.TxErrors = 0;

		if (pmsg->flags & I2C_M_RD) {
			Status =
			    XIic_MasterRecv(&dev->Iic, pmsg->buf, pmsg->len);
		} else {
			Status =
			    XIic_MasterSend(&dev->Iic, pmsg->buf, pmsg->len);
		}

		if (Status != XST_SUCCESS) {
			printk(KERN_WARNING
			       "%s #%d: Unexpected error %d.\n",
			       dev->adap.name, dev->index, Status);
			return -EIO;
		}

		/*
		 * Wait till the data is transmitted or received. If there is an error
		 * retry for 160 times.
		 */
		retries = 160;

		if (pmsg->flags & I2C_M_RD) {
			while ((((volatile int)(dev->receive_intr_flag)) != 0)
			       && (retries != 0)) {
				if (dev->Iic.Stats.TxErrors != 0) {
					udelay(25);
					Status =
					    XIic_MasterRecv(&dev->Iic,
							    pmsg->buf,
							    pmsg->len);
					dev->Iic.Stats.TxErrors = 0;
					retries--;
				}

				udelay(25);
			}
		} else {
			while ((((volatile int)(dev->transmit_intr_flag)) != 0)
			       && (retries != 0)) {
				if (dev->Iic.Stats.TxErrors != 0) {
					udelay(25);
					Status =
					    XIic_MasterSend(&dev->Iic,
							    pmsg->buf,
							    pmsg->len);
					dev->Iic.Stats.TxErrors = 0;
					retries--;
				}

				udelay(25);
			}
		}

		if (retries == 0) {
			printk("Unable to talk to Device\n");
			printk("Wrong Slave address or Slave device Busy\n");
		}
	}
	return num;
}

static int xiic_algo_control(struct i2c_adapter *adapter,
			     unsigned int cmd, unsigned long arg)
{
	return 0;
}

static u32 xiic_bit_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR |
	    I2C_FUNC_PROTOCOL_MANGLING;
}

static struct i2c_algorithm xiic_algo = {
	.master_xfer = xiic_xfer,	/* master_xfer          */
	.smbus_xfer = NULL,	/* smbus_xfer           */
	.slave_send = NULL,	/* slave_send           */
	.slave_recv = NULL,	/* slave_recv           */
	.algo_control = xiic_algo_control,	/* algo_control         */
	.functionality = xiic_bit_func,	/* functionality        */
};

/*
 * This routine is registered with the OS as the function to call when
 * the IIC interrupts.  It in turn, calls the Xilinx OS independent
 * interrupt function.  The Xilinx OS independent interrupt function
 * will in turn call any callbacks that we have registered for various
 * conditions.
 */
static irqreturn_t xiic_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct xiic_data *dev = dev_id;

	XIic_InterruptHandler(&dev->Iic);
	return IRQ_HANDLED;
}

static void RecvHandler(void *CallbackRef, int ByteCount)
{
	struct xiic_data *dev = (struct xiic_data *)CallbackRef;

	if (ByteCount == 0) {
		(dev->receive_intr_flag) = XST_SUCCESS;
		complete(&dev->complete);
	}
}

static void SendHandler(void *CallbackRef, int ByteCount)
{
	struct xiic_data *dev = (struct xiic_data *)CallbackRef;

	if (ByteCount == 0) {
		(dev->transmit_intr_flag) = XST_SUCCESS;
		complete(&dev->complete);
	}
}

static void StatusHandler(void *CallbackRef, XStatus Status)
{
	struct xiic_data *dev = (struct xiic_data *)CallbackRef;

	(dev->status_intr_flag) = Status;
	complete(&dev->complete);
}

static char *xilinx_iic_do_scan(struct xiic_data *dev)
{
	int i;
	char *page = kmalloc(PAGE_SIZE, SLAB_KERNEL);
	char *cptr = page;
	u8 data;
	XStatus status;

	for (i = 0x08; i < 0x78 && cptr; i++) {

		snprintf(cptr, PAGE_SIZE - (cptr - page), "%02X: ", i);
		cptr += strlen(cptr);

		init_completion(&dev->complete);
		if (XIic_SetAddress(&dev->Iic, XII_ADDR_TO_SEND_TYPE,
				    i) != XST_SUCCESS) {

			snprintf(cptr, PAGE_SIZE - (cptr - page),
				 "can't set address\n");
			cptr += strlen(cptr);
			continue;
		}

		dev->receive_intr_flag = ~(XStatus) 0;
		status = XIic_MasterRecv(&dev->Iic, &data, sizeof(data));
		if (status != XST_SUCCESS) {
			snprintf(cptr, PAGE_SIZE - (cptr - page),
				 "unexpected error\n");
			cptr += strlen(cptr);
			continue;
		}

		wait_for_completion(&dev->complete);

		snprintf(cptr, PAGE_SIZE - (cptr - page),
			 dev->receive_intr_flag == XST_SUCCESS ?
			 "OK\n" : "not respoding\n");
		cptr += strlen(cptr);
	}

	return page;
}

static ssize_t scan_show(struct device *d, struct device_attribute *attr,
			 char *text)
{
	int len = 0;
	char *scan_text = xilinx_iic_do_scan(dev_get_drvdata(d));

	if (scan_text) {
		len = strlen(scan_text);
		memcpy(text, scan_text, len);
		kfree(scan_text);
	}
	return len;
}

static DEVICE_ATTR(scan, S_IRUGO, scan_show, NULL);

static int __exit xilinx_iic_remove(struct of_device *device)
{
	struct xiic_data *dev;

	dev = dev_get_drvdata(&device->dev);

	/*
	 * If we've told the core I2C code about this dev, tell
	 * the core I2C code to forget the dev.
	 */
	if (dev->added) {
		/*
		 * If an error is returned, there's not a whole lot we can
		 * do.  An error has already been printed out so we'll
		 * just keep trundling along.
		 */
		(void)i2c_del_adapter(&dev->adap);
	}

	/* Tell the Xilinx code to take this IIC interface down. */
	if (dev->started) {
		while (XIic_Stop(&dev->Iic) != XST_SUCCESS) {
			/* The bus was busy.  Retry. */
			printk(KERN_WARNING
			       "%s #%d: Could not stop device.  Will retry.\n",
			       dev->adap.name, dev->index);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ / 2);
		}
	}

	/*
	 * Now that the Xilinx code isn't using the IRQ or registers,
	 * unmap the registers and free the IRQ.
	 */
	if (dev->remapped) {
		iounmap((void *)dev->Iic.BaseAddress);
	}

	if (dev->reqirq) {
		disable_irq(dev->irq);
		free_irq(dev->irq, dev);
	}

	device_remove_file(&device->dev, &dev_attr_scan);
	kfree(dev);

	return 0;
}

static int __init xilinx_iic_probe(struct of_device *of_dev,
				   const struct of_device_id *match)
{
	XIic_Config xiic_cfg;
	struct xiic_data *dev;
	char *scan_results;
	struct resource mem;
	struct device_node *dev_node = of_dev->node;
	int error;
	const u32 *did;

	/* Allocate the dev and zero it out. */
	dev = kzalloc(sizeof(struct xiic_data), SLAB_KERNEL);
	if (!dev) {
		dev_err(&of_dev->dev, "Cannot allocate struct xiic_data\n");
		error = -ENOMEM;
		goto out2;
	}

	dev_set_drvdata(&of_dev->dev, dev);

	/* get resources from device tree */
	dev->irq = irq_of_parse_and_map(dev_node, 0);
	if (dev->irq == 0 /* NOIRQ */ ) {
		error = -ENODEV;
		goto out;
	}

	if (of_address_to_resource(dev_node, 0, &mem)) {
		error = -ENODEV;
		goto out;
	}

	/* initialize fields to satisfy i2c  */
	strcpy(dev->adap.name, of_dev->dev.bus_id);

	did = of_get_property(dev_node, "index", NULL);
	if (did == NULL) {
		printk
		    ("%s: Can not get device index from device tree, assume 0\n",
		     __FUNCTION__);
		dev->index = 0;
	} else
		dev->index = *did;

	init_completion(&dev->complete);

	memset(&xiic_cfg, 0, sizeof(XIic_Config));
	xiic_cfg.DeviceId = dev->index;

	/* Change the addresses to be virtual; save the old ones to restore. */
	dev->base = mem.start;
	xiic_cfg.BaseAddress =
	    (u32) ioremap(mem.start, mem.end - mem.start + 1);

	dev->remapped = 1;
	down(&cfg_sem);

	/* Tell the Xilinx code to bring this IIC interface up. */
	if (XIic_CfgInitialize(&dev->Iic, &xiic_cfg, xiic_cfg.BaseAddress) !=
	    XST_SUCCESS) {
		up(&cfg_sem);
		dev_err(&of_dev->dev, "could not initialize device.\n");
		error = -ENODEV;
		goto out;
	}
	up(&cfg_sem);
	XIic_SetRecvHandler(&dev->Iic, (void *)dev, RecvHandler);
	XIic_SetSendHandler(&dev->Iic, (void *)dev, SendHandler);
	XIic_SetStatusHandler(&dev->Iic, (void *)dev, StatusHandler);

	/* Grab the IRQ */
	error = request_irq(dev->irq, xiic_interrupt, 0, dev->adap.name, dev);
	if (error) {
		dev_err(&of_dev->dev, "could not allocate interrupt %d.\n",
			dev->irq);
		goto out;
	}
	dev->reqirq = 1;

	if (XIic_Start(&dev->Iic) != XST_SUCCESS) {
		dev_err(&of_dev->dev, "could not start device\n");
		error = -ENODEV;
		goto out;
	}
	dev->started = 1;

	/* Now tell the core I2C code about our new device. */
	/*
	 * SAATODO: Get a real ID (perhaps I2C_HW_XILINX) after
	 * initial release.  Will need to email lm78@stimpy.netroedge.com
	 * per http://www2.lm-sensors.nu/~lm78/support.html
	 */
	dev->adap.id = I2C_DRIVERID_I2CDEV;
	dev->adap.algo = &xiic_algo;
	dev->adap.algo_data = NULL;
	dev->adap.timeout = XIIC_TIMEOUT;
	dev->adap.retries = XIIC_RETRY;
	error = i2c_add_adapter(&dev->adap);

	if (error) {
		dev_err(&of_dev->dev, "could not add i2c adapter\n");
		goto out;
	}
	dev->added = 1;

	printk("%s #%d at 0x%08X mapped to 0x%08X, irq=%d\n",
	       dev->adap.name, dev->index,
	       dev->base, dev->Iic.BaseAddress, dev->irq);

	if (scan) {
		scan_results = xilinx_iic_do_scan(dev);
		if (scan_results) {
			printk(scan_results);
			kfree(scan_results);
		}
	}

	device_create_file(&of_dev->dev, &dev_attr_scan);
      out:
	if (error)
		xilinx_iic_remove(of_dev);
      out2:
	return error;
}

static struct of_device_id xilinx_iic_match[] = {
	{
	 .type = "iic",
	 .compatible = "xilinx,iic",
	 },
	{},
};

static struct of_platform_driver xilinx_iic_driver = {
	.name = XILINX_IIC,
	.match_table = xilinx_iic_match,
	.probe = xilinx_iic_probe,
	.remove = xilinx_iic_remove,
};
static int __init xiic_init(void)
{
	return of_register_platform_driver(&xilinx_iic_driver);
}

static void __exit xiic_cleanup(void)
{
	of_unregister_platform_driver(&xilinx_iic_driver);
}

module_init(xiic_init);
module_exit(xiic_cleanup);
