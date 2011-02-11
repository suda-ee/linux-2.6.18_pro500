/*
 *  linux/drivers/davinci/i2c-davinci-client.c
 *
 * Copyright (C) 2006 Texas Instruments Inc
 * Copyright (C) 2008 MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/clk.h>

#include <asm/semaphore.h>
#include <asm/arch/cpu.h>
#include <asm/arch/i2c-client.h>

static DEFINE_MUTEX(expander_lock);
static struct i2c_client *client_handle;

/* This function is used for internal initialization */
int davinci_i2c_read(u8 size, u8 * val, u16 client_addr)
{
	int err;
	struct i2c_client *client = client_handle;

	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client_addr;
	msg->flags = I2C_M_RD;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		return 0;
	}

	return err;
}

EXPORT_SYMBOL(davinci_i2c_read);

/* This function is used for internal initialization */
int davinci_i2c_write(u8 size, u8 * val, u16 client_addr)
{
	int err;
	struct i2c_client *client = client_handle;

	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client_addr;
	msg->flags = 0;
	msg->len = size;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	return err;
}

EXPORT_SYMBOL(davinci_i2c_write);

int davinci_i2c_expander_op(u16 client_addr, u35_expander_ops pin, u8 val)
{
	int err = 0;
	u8 data_to_u35 = 0;

	if (val > 1)
		return -1;

	if (client_addr != 0x3a) {
		printk(KERN_WARNING "Only IO Expander at address "
		       "0x3A is supported\n");
		return -EINVAL;
	}

	if (cpu_is_davinci_dm6467()) {
		switch (pin) {
		case ATA_SEL_DM646X:
		case ATA_PWD_DM646X:
		case VSCALE_ON_DM646X:
		case VLYNQ_RESET_DM646X:
		case I2C_INT_DM646X:
		case USB_FB_DM646X:
		case CIR_MOD_DM646X:
		case CIR_DEMOD_DM646X:
			break;
		default:
			return -EINVAL;
		}
	} else {
		switch (pin) {
		case USB_DRVVBUS:
		case VDDIMX_EN:
		case VLYNQ_ON:
		case CF_RESET:
		case WLAN_RESET:
		case ATA_SEL:
		case CF_SEL:
			break;
		default:
			return -EINVAL;
	    }
	}

	mutex_lock(&expander_lock);

	err = davinci_i2c_read(1, &data_to_u35, client_addr);
	if (err) {
		mutex_unlock(&expander_lock);
		return err;
	}
	if (!cpu_is_davinci_dm6467() && pin == CF_SEL) {
		static const char cmd[4] = { 4, 6, 0x00, 0x09 };

		err = davinci_i2c_write(4, cmd, 0x23);
		if (err) {
			mutex_unlock(&expander_lock);
			return err;
		}
	}

	data_to_u35 &= 0xff ^ (1 << pin);
	data_to_u35 |= val << pin;

	err = davinci_i2c_write(1, &data_to_u35, client_addr);

	mutex_unlock(&expander_lock);

	return err;
}

EXPORT_SYMBOL(davinci_i2c_expander_op);

static struct i2c_driver davinci_i2c_client_driver;

void (*board_i2c_expander_setup)(void);

static int davinci_i2c_attach_client(struct i2c_adapter *adap, int addr)
{
	struct i2c_client *client;
	int err;
	u8 data_to_u35 = 0xf6;

	if (!(client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	client_handle = client;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	client->flags = 0;
	client->driver = &davinci_i2c_client_driver;
	client->adapter = adap;
	strlcpy(client->name, client->driver->driver.name, I2C_NAME_SIZE);

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		goto exit_kfree;
	}

	err = davinci_i2c_write(1, &data_to_u35, 0x3A);

	if (board_i2c_expander_setup)
		board_i2c_expander_setup();

	return 0;

 exit_kfree:
	kfree(client);
 exit:
	return err;
}

static int davinci_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;
	return err;
}

static int davinci_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return davinci_i2c_attach_client(adap, 0x3A);
}

/* This is the driver that will be inserted */
static struct i2c_driver davinci_i2c_client_driver = {
	.driver = {
		/* there are 3 expanders, one is leds-only ... */
		.name	= "davinci_evm_expander1",
	},
	.attach_adapter	= davinci_i2c_probe_adapter,
	.detach_client	= davinci_i2c_detach_client,
};

static int __init davinci_i2c_client_init(void)
{
	return i2c_add_driver(&davinci_i2c_client_driver);
}

static void __exit davinci_i2c_client_exit(void)
{
	i2c_del_driver(&davinci_i2c_client_driver);
}

module_init(davinci_i2c_client_init);
module_exit(davinci_i2c_client_exit);
