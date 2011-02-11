/*
 * Copyright (C) 2006 Texas Instruments Inc
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci_vpfe.c */
/* !FIXME needs to add code to program the i2c switch on the EVM to select tvp5146 or mt9t001 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/dma-mapping.h>

#include <media/davinci/davinci_vpfe.h>
#ifdef CONFIG_MACH_DAVINCI_DM355_EVM
#include <media/davinci/ccdc_dm355.h>
#else
#include <media/davinci/ccdc_davinci.h>
#endif

#if 0
#define VPFE_DEBUG
#endif

#ifdef VPFE_DEBUG
#include <linux/proc_fs.h>
struct vpfe_stats_s {
	int qbuf_count;
	int dqbuf_count;
};

static struct vpfe_stats_s stats;

#define PROCNAME "vpfe_stats"
static struct proc_dir_entry *vpfe_stats_dir, *vpfe_stats;
#endif

MODULE_LICENSE("GPL");
/* Global variable for insmode command which will decide that either MT9T001
 * (device =0)  or TVP5146 (device = 1) is the device
 */
static int device_type = TVP5146;
//static int device_type = MT9T001;

struct device *vpfe_dev;

module_param(device_type, int, 0);

#ifndef MODULE
/*
 * Pass boot-time options by adding the following string to the boot params:
 * 	v4l2_video_capture:[option[:option]]
 * Valid options:
 * 	device=[MT9T031|MT9T001|TVP5146]
 */

char *dm355_devices[] = { "MT9T001", "TVP5146", "MT9T031" };
int __init dm355_v4l2_device_setup(char *options)
{
	char *this_opt;

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ":")) != NULL) {

		if (!*this_opt)
			continue;

		if (!strncmp(this_opt, "device=", 6)) {
			if (!strncmp(this_opt + 7, "MT9T031", 7)) {
				device_type = MT9T031;
			} else if (!strncmp(this_opt + 7, "MT9T001", 7)) {
				device_type = MT9T001;
			} else if (!strncmp(this_opt + 7, "TVP5146", 7)) {
				device_type = TVP5146;
			}
		}
	}
	return 0;
}

__setup("v4l2_video_capture=", dm355_v4l2_device_setup);
#endif

static struct v4l2_rect ntsc_bounds = VPFE_WIN_NTSC;
static struct v4l2_rect pal_bounds = VPFE_WIN_PAL;
static struct v4l2_fract ntsc_aspect = VPFE_PIXELASPECT_NTSC;
static struct v4l2_fract pal_aspect = VPFE_PIXELASPECT_PAL;
static struct v4l2_rect ntscsp_bounds = VPFE_WIN_NTSC_SP;
static struct v4l2_rect palsp_bounds = VPFE_WIN_PAL_SP;
static struct v4l2_fract sp_aspect = VPFE_PIXELASPECT_NTSC_SP;

static struct v4l2_rect VGA_bounds = VPFE_WIN_VGA;
static struct v4l2_rect SVGA_bounds = VPFE_WIN_SVGA;
static struct v4l2_rect XGA_bounds = VPFE_WIN_XGA;
static struct v4l2_rect P480_bounds = VPFE_WIN_480p;
static struct v4l2_rect P576_bounds = VPFE_WIN_576p;
static struct v4l2_rect P720_bounds = VPFE_WIN_720p;
static struct v4l2_rect P1080_bounds = VPFE_WIN_1080p;
static struct v4l2_fract default_aspect = VPFE_PIXELASPECT_DEFAULT;

static vpfe_obj vpfe_device_ycbcr = {	/* the default format is NTSC */
	.usrs = 0,
	.io_usrs = 0,
	.std = VPFE_STD_AUTO,
	.vwin = VPFE_WIN_PAL,
	.bounds = VPFE_WIN_PAL,
	.pixelaspect = VPFE_PIXELASPECT_PAL,
	.pixelfmt = V4L2_PIX_FMT_UYVY,
	.field = V4L2_FIELD_INTERLACED,
	.numbuffers = VPFE_DEFNUM_FBUFS,
	.capture_device = TVP5146,
	.tvp5146_params = {
			   .mode = TVP5146_MODE_AUTO,
			   .amuxmode = TVP5146_AMUX_COMPOSITE,
			   .enablebt656sync = TRUE,
			   .data_width = TVP5146_WIDTH_8BIT},

	.irqlock = SPIN_LOCK_UNLOCKED
};

static vpfe_obj vpfe_device_raw = {	/*Default is vga mode */
	.usrs = 0,
	.io_usrs = 0,
	.std = V4L2_STD_MT9T001_VGA_30FPS,
	.vwin = VPFE_WIN_VGA,
	.bounds = VPFE_WIN_VGA,
	.pixelaspect = VPFE_PIXELASPECT_DEFAULT,
	.pixelfmt = V4L2_PIX_FMT_SBGGR8,
	.field = V4L2_FIELD_NONE,
	.numbuffers = VPFE_DEFNUM_FBUFS,
	.capture_device = MT9T001,
	.irqlock = SPIN_LOCK_UNLOCKED
	/* .resizer_no = -1, */    /* Extra field for resizer number filled in open */
	/* .otf_on = 0 */          /* On the Fly Flag initialize to 0 i.e. Disabled */
};

static ccdc_frmfmt frm_format;
static ccdc_imgwin image_window;
static vpfe_obj vpfe_device = { 0 };

struct v4l2_capability vpfe_drvcap = {
	.driver = "vpfe driver",
#ifdef CONFIG_MACH_DAVINCI_DM355_EVM
	.card = "dm355 EVM",
#else
	.card = "DaVinci EVM",
#endif
	.bus_info = "Platform",
	.version = VPFE_VERSION_CODE,
	.capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING
};

/* inline function to free reserved pages  */
void inline free_reserved_pages(unsigned long bufaddr, unsigned long bufsize)
{
	unsigned long size, addr;
	if (!bufaddr)
		return;
	addr = bufaddr;
	size = PAGE_SIZE << (get_order(bufsize));
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(bufaddr, get_order(bufsize));
}

/*
 * ======== sense_std ========
 */
/*This function will get current input standard for TVP5146*/
static int sense_std(v4l2_std_id * std_id)
{
	v4l2_std_id id = 0;
	tvp5146_mode mode;
	int ret;
	dev_dbg(vpfe_dev, "\nStarting Davinci_vpfe sense_std...");
	ret = tvp5146_ctrl(TVP5146_GET_STD, &mode);
	if (ret < 0)
		return ret;
	switch (mode & 0x7) {
	case TVP5146_MODE_NTSC:
		id = V4L2_STD_NTSC;
		break;
	case TVP5146_MODE_PAL:
		id = V4L2_STD_PAL;
		break;
	case TVP5146_MODE_PAL_M:
		id = V4L2_STD_PAL_M;
		break;
	case TVP5146_MODE_PAL_CN:
		id = V4L2_STD_PAL_N;
		break;
	case TVP5146_MODE_SECAM:
		id = V4L2_STD_SECAM;
		break;
	case TVP5146_MODE_PAL_60:
		id = V4L2_STD_PAL_60;
		break;
	}
	if (mode & 0x8) {	/* square pixel mode */
		id <<= 32;
	}
	if (mode == TVP5146_MODE_AUTO) {
		id = VPFE_STD_AUTO;	/* auto-detection for all other modes */
	} else if (mode == TVP5146_MODE_AUTO_SQP) {
		id = VPFE_STD_AUTO_SQP;
	}
	if (id == 0)
		return -EINVAL;
	*std_id = id;
	dev_dbg(vpfe_dev, "\nEnd of Davinci_vpfe sense_std...");
	return 0;
}

/*
 * ======== vpfe_isr ========
 */
/*ISR for VINT0*/
static irqreturn_t vpfe_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	vpfe_obj *vpfe = &vpfe_device;
	int fid;
	unsigned long jiffies_time = get_jiffies_64();
	struct timeval timevalue;
	int val = 0;

	val = ccdc_sbl_reset();

	/*Convert time representations between jiffies and struct timeval */
	jiffies_to_timeval(jiffies_time, &timevalue);

	dev_dbg(vpfe_dev, "\nStarting Davinci_vpfe\vpfe_isr...");
	if (frm_format == CCDC_FRMFMT_INTERLACED) {
		/* check which field we are in hardware */
		fid = ccdc_getfid();
		/* switch the software maintained field id */
		vpfe->field_id ^= 1;
		dev_dbg(vpfe_dev, "field id = %x:%x.\n", fid, vpfe->field_id);
		if (fid == vpfe->field_id) {	/* we are in-sync here,continue */
			if (fid == 0) {
				if (vpfe->first_frame)
					vpfe->fld_one_recvd = 1;
				else {
					/*  One frame is just being captured. If the
				 	 * next frame is available, release the current
				 	 * frame and move on
				 	 */
					if (vpfe->curFrm != vpfe->nextFrm) {
						/* Copy frame capture time value in
					 	 * curFrm->ts
					 	 */
						vpfe->curFrm->ts = timevalue;
						vpfe->curFrm->field = vpfe->field;
						vpfe->curFrm->size = vpfe->sizeimage;
						vpfe->curFrm->state = STATE_DONE;
						wake_up_interruptible(&vpfe->
								      curFrm->done);
						vpfe->curFrm = vpfe->nextFrm;
					}
					/* based on whether the two fields are stored
				 	 * interleavely or separately in memory,
				 	 * reconfigure the CCDC memory address
				 	 */
					if (vpfe->field == V4L2_FIELD_SEQ_TB) {
						u32 addr =
					    		vpfe->curFrm->boff +
					    			vpfe->field_offset;
						ccdc_setfbaddr((unsigned long) addr);
					}
				}
			} else if (fid == 1) {
				if (vpfe->first_frame) {
					if (vpfe->fld_one_recvd) {
						vpfe->first_frame = 0;
						vpfe->fld_one_recvd = 0;
					} 
				}
				if (!vpfe->first_frame) {
					/* if one field is just being captured */
					/* configure the next frame */
					/* get the next frame from the empty queue */
					/* if no frame is available, */
					/* hold on to the current buffer */
					if (!list_empty(&vpfe->dma_queue) &&
						vpfe->curFrm == vpfe->nextFrm) {
						vpfe->nextFrm =
					    		list_entry(vpfe->dma_queue.
						       		next, struct
						       		videobuf_buffer, queue);
						list_del(&vpfe->nextFrm->queue);
						vpfe->nextFrm->state = STATE_ACTIVE;
						ccdc_setfbaddr((unsigned long)
						       vpfe->nextFrm->boff);
					}
					if (vpfe->mode_changed) {
						ccdc_setwin(&image_window,
						    frm_format, 2);
						/* update the field offset */
						vpfe->field_offset =
					    		(vpfe->vwin.height -
					     		2) * vpfe->vwin.width;
						vpfe->mode_changed = FALSE;
					}
				}
			}
		} else if (fid == 0) {
			/* recover from any hardware out-of-sync due to */
			/* possible switch of video source              */
			/* for fid == 0, sync up the two fids           */
			/* for fid == 1, no action, one bad frame will  */
			/* go out, but it is not a big deal             */
			vpfe->field_id = fid;
		}
	} else if (frm_format == CCDC_FRMFMT_PROGRESSIVE) {

		dev_dbg(vpfe_dev, "\nframe format is progressive...");
		if (vpfe->curFrm != vpfe->nextFrm) {
			/* Copy frame capture time value in curFrm->ts */
			vpfe->curFrm->ts = timevalue;
			vpfe->curFrm->state = STATE_DONE;
			wake_up_interruptible(&vpfe->curFrm->done);
			vpfe->curFrm = vpfe->nextFrm;
		}

	}
	dev_dbg(vpfe_dev, "interrupt returned.\n");
	return IRQ_RETVAL(1);
}

static irqreturn_t vdint1_isr(int irq, void *dev_id, struct pt_regs *regs)
{

	vpfe_obj *vpfe = &vpfe_device;

	dev_dbg(vpfe_dev, "\nInside vdint1_isr...");

	if (frm_format == CCDC_FRMFMT_PROGRESSIVE) {
		if (!list_empty(&vpfe->dma_queue)
		    && vpfe->curFrm == vpfe->nextFrm) {
			vpfe->nextFrm =
			    list_entry(vpfe->dma_queue.next,
				       struct videobuf_buffer, queue);
			list_del(&vpfe->nextFrm->queue);
			vpfe->nextFrm->state = STATE_ACTIVE;
			ccdc_setfbaddr((unsigned long)vpfe->nextFrm->boff);
		}
	}
	return IRQ_RETVAL(1);
}

/*
 * ======== buffer_prepare ========
 */
/* this is the callback function called from videobuf_qbuf() function */
/* the buffer is prepared and queued into the dma queue */
static int buffer_prepare(struct videobuf_queue *q,
			  struct videobuf_buffer *vb, enum v4l2_field field)
{
	vpfe_obj *vpfe = &vpfe_device;
	unsigned int buf_size;
	dev_dbg(vpfe_dev, "\nstarting buffer_prepare");
	if (device_type == TVP5146) {
		buf_size = VPFE_TVP5146_MAX_FBUF_SIZE;
	} else {
		buf_size = VPFE_MT9T001_MAX_FBUF_SIZE;
	}
	if (vb->state == STATE_NEEDS_INIT) {
		vb->width = vpfe->vwin.width;
		vb->height = vpfe->vwin.height;
		vb->size = buf_size;
		vb->field = field;
	}
	vb->state = STATE_PREPARED;
	dev_dbg(vpfe_dev, "\nEnd of buffer_prepare");
#ifdef VPFE_DEBUG
	stats.dqbuf_count++;
#endif
	return 0;

}

/*
 * ======== buffer_config ========
 */
 /* This function is responsible to queue up vpfe buffer is into video buffer
  * queue.
  */
static void buffer_config(struct videobuf_queue *q, unsigned int count)
{
	vpfe_obj *vpfe = &vpfe_device;
	int i;
	dev_dbg(vpfe_dev, "\nstarting buffer_config");
	for (i = 0; i < count; i++) {
		q->bufs[i]->boff = virt_to_phys(vpfe->fbuffers[i]);
		dev_dbg(vpfe_dev, "buffer address: %x\n", q->bufs[i]->boff);
	}
	dev_dbg(vpfe_dev, "\nEnd of buffer_config");
}

/*
 * ======== buffer_setup ========
 */
 /* This function allocate free pages and makes it sure that they will not
  * swapped
  */
static int
buffer_setup(struct videobuf_queue *q, unsigned int *count, unsigned int *size)
{
	vpfe_obj *vpfe = &vpfe_device;
	int i;
	unsigned int buf_size;
	dev_dbg(vpfe_dev, "\nstarting buffer_setup");
	if (device_type == TVP5146) {
		*size = buf_size = VPFE_TVP5146_MAX_FBUF_SIZE;
	} else {
		*size = buf_size = VPFE_MT9T001_MAX_FBUF_SIZE;
	}

	for (i = VPFE_DEFNUM_FBUFS; i < *count; i++) {
		u32 size = PAGE_SIZE << (get_order(buf_size));
		void *mem = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
						     get_order(buf_size));
		if (mem) {
			unsigned long adr = (unsigned long)mem;
			while (size > 0) {
				/* make sure the frame buffers are never
				   swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			vpfe->fbuffers[i] = mem;
		} else {
			break;
		}
	}
	*count = vpfe->numbuffers = i;
	dev_dbg(vpfe_dev, "\nEnd of buffer_setup");
	return 0;
}

/*
 * ======== buffer_queue ========
 */
 /* This function adds the buffer to DMA queue */
static void buffer_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	vpfe_obj *vpfe = &vpfe_device;
	/* add the buffer to the DMA queue */
	dev_dbg(vpfe_dev, "\nstarting buffer_queue");
	list_add_tail(&vb->queue, &vpfe->dma_queue);
	vb->state = STATE_QUEUED;
#ifdef VPFE_DEBUG
	stats.qbuf_count++;
#endif
	dev_dbg(vpfe_dev, "\nEnding buffer_queue");
}

/*
 * ======== buffer_release ========
 */
 /* This function will free the buffer if it is not one
  * of the 3 allocated at initialization time.
  */
static void buffer_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	unsigned int buf_size;
	dev_dbg(vpfe_dev, "\nStarting buffer_release");
	if (device_type == TVP5146) {
		buf_size = VPFE_TVP5146_MAX_FBUF_SIZE;
	} else {
		buf_size = VPFE_MT9T001_MAX_FBUF_SIZE;
	}

	/* free the buffer if it is not one of the 3 allocated at initializaiton
	 * time
	 */
	if (vb->i < vpfe_device.numbuffers
	    && vb->i >= VPFE_DEFNUM_FBUFS && vpfe_device.fbuffers[vb->i]) {
		free_reserved_pages((unsigned long)vpfe_device.
				    fbuffers[vb->i], get_order(buf_size));
		vpfe_device.fbuffers[vb->i] = NULL;
	}
	vb->state = STATE_NEEDS_INIT;
	dev_dbg(vpfe_dev, "\nEnd of buffer_release");
}

static struct videobuf_queue_ops video_qops = {
	.buf_setup = buffer_setup,
	.buf_prepare = buffer_prepare,
	.buf_queue = buffer_queue,
	.buf_release = buffer_release,
	.buf_config = buffer_config,
};

static int vpfe_try_format(struct v4l2_pix_format *pixfmt, int check)
{
	int bpp, min_height, min_width = 32;
	vpfe_obj *vpfe = &vpfe_device;

	if (device_type == MT9T001 || device_type == MT9T031) {
		if (pixfmt->pixelformat != V4L2_PIX_FMT_SBGGR8) {
			if (check)
				return -1;
			else
				pixfmt->pixelformat = V4L2_PIX_FMT_SBGGR8;
		}
		if (pixfmt->field != V4L2_FIELD_NONE &&
		    pixfmt->field != V4L2_FIELD_ANY) {
			if (check)
				return -1;
			else
				pixfmt->field = V4L2_FIELD_NONE;
		} else if (pixfmt->field == V4L2_FIELD_ANY)
			pixfmt->field = V4L2_FIELD_NONE;
		min_height = 1;
	} else {
		/* device is TVP5146 */
		if (pixfmt->field != V4L2_FIELD_INTERLACED &&
		    pixfmt->field != V4L2_FIELD_ANY &&
		    pixfmt->field != V4L2_FIELD_SEQ_TB) {
			if (check)
				return -1;
			else
				pixfmt->field = V4L2_FIELD_INTERLACED;
		} else if (pixfmt->field == V4L2_FIELD_ANY)
			pixfmt->field = V4L2_FIELD_INTERLACED;
		if (pixfmt->pixelformat != V4L2_PIX_FMT_UYVY && 
		    pixfmt->pixelformat != V4L2_PIX_FMT_YUYV) {
			if (check)
				return -1;
			else
				pixfmt->pixelformat = V4L2_PIX_FMT_UYVY;
		}
		min_height = 2;
	}

	if (pixfmt->pixelformat == V4L2_PIX_FMT_SBGGR8)
		bpp = 1;
	else
		bpp = 2;
	min_width /= bpp;
	if (pixfmt->width < min_width)
		pixfmt->width = min_width;
	else if (pixfmt->width > vpfe->bounds.width)
		pixfmt->width = vpfe->bounds.width;
	else
		pixfmt->width = (pixfmt->width + (min_width-1)) & ~(min_width-1);
	if (pixfmt->height <= min_height)
		pixfmt->height = min_height;
	else if (pixfmt->height > vpfe->bounds.height)
		pixfmt->height = vpfe->bounds.height;
	else {
		if (pixfmt->field == V4L2_FIELD_INTERLACED ||
		    pixfmt->field == V4L2_FIELD_SEQ_TB)
			pixfmt->height &= ~1;
	}
	pixfmt->bytesperline = pixfmt->width * bpp;
	pixfmt->sizeimage = (pixfmt->bytesperline * pixfmt->height);
	return 0;
}
static void vpfe_update_std_info(v4l2_std_id id)
{
	vpfe_obj *vpfe = &vpfe_device;
	if (id & V4L2_STD_625_50) {
		vpfe->std = id;
		vpfe->bounds = vpfe->vwin = pal_bounds;
		vpfe->pixelaspect = pal_aspect;
		ccdc_set_image_window(CCDC_YCBCR, pal_bounds);
	} else if (id & V4L2_STD_525_60) {
		vpfe->std = id;
		vpfe->bounds = vpfe->vwin = ntsc_bounds;
		vpfe->pixelaspect = ntsc_aspect;
		ccdc_set_image_window(CCDC_YCBCR, ntsc_bounds);
	} else if (id & VPFE_STD_625_50_SQP) {
		vpfe->std = id;
		vpfe->bounds = vpfe->vwin = palsp_bounds;
		vpfe->pixelaspect = sp_aspect;
		ccdc_set_image_window(CCDC_YCBCR, palsp_bounds);
	} else if (id & VPFE_STD_525_60_SQP) {
		vpfe->std = id;
		vpfe->std = id;
		vpfe->bounds = vpfe->vwin = ntscsp_bounds;
		vpfe->pixelaspect = sp_aspect;
		ccdc_set_image_window(CCDC_YCBCR, ntscsp_bounds);
	}
}
/*
 * ======== vpfe_doioctl ========
 */
 /* This function will provide different V4L2 commands.This function can be
  * used to configure driver or get status of driver as per command passed
  * by application.
  */
static int vpfe_doioctl(struct inode *inode, struct file *file,
			unsigned int cmd, void *arg)
{
	vpfe_obj *vpfe = &vpfe_device;
	vpfe_fh *fh = file->private_data;
	int ret = 0;
	unsigned long addr = 0, flags;

	switch (cmd) {
	case VIDIOC_S_CTRL:
	case VIDIOC_S_FMT:
	case VIDIOC_S_STD:
	case VIDIOC_S_CROP:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_S_CTRL ioctl");
		ret = v4l2_prio_check(&vpfe->prio, &fh->prio);
		if (0 != ret) {
			return ret;
		}
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_S_CTRL ioctl");
		break;
	}

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap =
			    (struct v4l2_capability *)arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_QUEYCAP ioctl");
			memset(cap, 0, sizeof(*cap));
			*cap = vpfe_drvcap;
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_QUEYCAP ioctl");
			break;
		}
	case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *fmt = (struct v4l2_fmtdesc *)arg;
			u32 index = fmt->index;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_ENUM_FMT ioctl");
			memset(fmt, 0, sizeof(*fmt));

			fmt->index = index;
			if (device_type == TVP5146) {
				if (index == 0) {
					/* only yuv4:2:2 format is supported
					 * at this point
					 */
					fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
					strcpy(fmt->description,
					       "YCbCr4:2:2 Interleaved UYUV");
					fmt->pixelformat = V4L2_PIX_FMT_UYVY;
				} else if (index == 1) {
					fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
					strcpy(fmt->description,
					       "YCbCr4:2:2 Interleaved YUYV");
					fmt->pixelformat = V4L2_PIX_FMT_YUYV;
				} else {
					ret = -EINVAL;
				}
			} else if (device_type == MT9T001
				   || device_type == MT9T031) {
				if (index == 0) {
					/* only Bayer Raw Mode format is
					 * supported at this point
					 */
					fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
					strcpy(fmt->description,
					       "Raw Mode -Bayer Pattern GrRBGb");
					fmt->pixelformat = V4L2_PIX_FMT_SBGGR8;
				} else {
					ret = -EINVAL;
				}
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_ENUM_FMT ioctl");

			break;
		}
	case VIDIOC_G_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_G_FMT ioctl");
			if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				ret = -EINVAL;
			} else {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				down_interruptible(&vpfe->lock);
				pixfmt->width = vpfe->vwin.width;
				pixfmt->height = vpfe->vwin.height;
				pixfmt->field = vpfe->field;
				pixfmt->pixelformat = vpfe->pixelfmt;
				if (device_type == TVP5146) {
					pixfmt->bytesperline =
					    pixfmt->width * 2;
				} else {
					pixfmt->bytesperline =
					    (((ccdc_raw_data_size() == _8BITS)
					      || ccdc_alaw_enable())? pixfmt->
					     width : (pixfmt->width * 2));

				}
				pixfmt->sizeimage =
				    pixfmt->bytesperline * pixfmt->height;
				pixfmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
				up(&vpfe->lock);
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_G_FMT ioctl");
			break;
		}
	case VIDIOC_S_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
			enum hw_frame frame_type;

			if (device_type == TVP5146)
				frame_type = CCDC_YCBCR;
			else
				frame_type = CCDC_RAW;

			dev_dbg(vpfe_dev, "\nStarting VIDIOC_S_FMT ioctl");
			if (vpfe->started) {
				/* make sure streaming is not started */
				ret = -EBUSY;
				break;
			}

			down_interruptible(&vpfe->lock);
			dev_dbg(vpfe_dev, "\nAfter down_interruptible");
			if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				ret = -EINVAL;
				up(&vpfe->lock);
				break;
			}

			ret = vpfe_try_format(pixfmt, 1);
			if (ret) {
				ret = -EINVAL;
				dev_dbg(vpfe_dev, "\n window size error!");
				up(&vpfe->lock);
				break;
			}

			/* crop window is directed modified */
			vpfe->vwin.height = pixfmt->height;
			vpfe->vwin.width = pixfmt->width;
			vpfe->sizeimage = pixfmt->sizeimage;
			vpfe->pixelfmt = pixfmt->pixelformat;
			vpfe->field = pixfmt->field;
			ccdc_set_image_window(frame_type, vpfe->vwin);

			/* setup the CCDC parameters accordingly */
			if (device_type == TVP5146) {
				if (pixfmt->pixelformat == V4L2_PIX_FMT_YUYV) {
					ccdc_set_pix_order
					    (CCDC_PIXORDER_YCBYCR);
				} else {
					ccdc_set_pix_order
					    (CCDC_PIXORDER_CBYCRY);
					vpfe->pixelfmt = pixfmt->pixelformat;
				}
			}

			/* Configure buffer type and frame format as per field
			 *  value passed
			 */
			if (pixfmt->field == V4L2_FIELD_INTERLACED) {
				ccdc_set_buf_type(frame_type,
						  CCDC_BUFTYPE_FLD_INTERLEAVED);
				ccdc_set_frame_format(frame_type,
						      CCDC_FRMFMT_INTERLACED);
			} else if (pixfmt->field == V4L2_FIELD_SEQ_TB) {
				ccdc_set_buf_type(frame_type,
						  CCDC_BUFTYPE_FLD_SEPARATED);
				ccdc_set_frame_format(frame_type,
						      CCDC_FRMFMT_INTERLACED);
			} else if (pixfmt->field == V4L2_FIELD_NONE) {
				ccdc_set_frame_format(frame_type,
						      CCDC_FRMFMT_PROGRESSIVE);
			}
			up(&vpfe->lock);
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_S_FMT ioctl");
			break;
		}
	case VIDIOC_TRY_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_TRY_FMT ioctl");
			if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				ret = -EINVAL;
			} else {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				ret = vpfe_try_format(pixfmt, 0);
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_TRY_FMT ioctl");
			break;
		}
	case VIDIOC_G_STD:
		{
			v4l2_std_id *id = (v4l2_std_id *) arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_G_STD ioctl");
			if (device_type == TVP5146) {
				ret = sense_std(id);
				if (!ret)
					vpfe_update_std_info(*id);
			} else {
				*id = vpfe->std;
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_G_STD ioctl");
			break;
		}
	case VIDIOC_S_STD:
		{
			v4l2_std_id id = *(v4l2_std_id *) arg;
			tvp5146_mode mode = TVP5146_MODE_INV;
			int sqp = 0;
			dev_dbg(vpfe_dev, "\nStarting of VIDIOC_S_STD ioctl");
			/* make sure streaming is not started */
			if (vpfe->started) {
				ret = -EBUSY;
				break;
			}
			down_interruptible(&vpfe->lock);

			if (device_type == TVP5146) {
				if (id & V4L2_STD_625_50) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = pal_bounds;
					vpfe->pixelaspect = pal_aspect;
					ccdc_set_image_window(CCDC_YCBCR,
							      pal_bounds);
				} else if (id & V4L2_STD_525_60) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = ntsc_bounds;
					vpfe->pixelaspect = ntsc_aspect;
					ccdc_set_image_window(CCDC_YCBCR,
							      ntsc_bounds);
				} else if (id & VPFE_STD_625_50_SQP) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin =
					    palsp_bounds;
					vpfe->pixelaspect = sp_aspect;
					sqp = 1;
					id >>= 32;
				} else if (id & VPFE_STD_525_60_SQP) {
					vpfe->std = id;
					sqp = 1;
					vpfe->std = id;
					id >>= 32;
					vpfe->bounds = vpfe->vwin =
					    ntscsp_bounds;
					vpfe->pixelaspect = sp_aspect;
					ccdc_set_image_window(CCDC_YCBCR,
							      ntscsp_bounds);
				} else if (id & VPFE_STD_AUTO) {
					mode = TVP5146_MODE_AUTO;
					vpfe->bounds = vpfe->vwin = pal_bounds;
					vpfe->pixelaspect = pal_aspect;
					ccdc_set_image_window(CCDC_YCBCR,
							      pal_bounds);
					vpfe->std = id;
				} else if (id & VPFE_STD_AUTO_SQP) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin =
					    palsp_bounds;
					vpfe->pixelaspect = sp_aspect;
					sqp = 1;
					mode = TVP5146_MODE_AUTO_SQP;
					vpfe->pixelaspect = sp_aspect;
				} else {
					ret = -EINVAL;
				}
				if (id == V4L2_STD_PAL_60) {
					mode = TVP5146_MODE_PAL_60;
				} else if (id == V4L2_STD_PAL_M) {
					mode = TVP5146_MODE_PAL_M;
				} else if (id == V4L2_STD_PAL_Nc
					   || id == V4L2_STD_PAL_N) {
					mode = TVP5146_MODE_PAL_CN;
				} else if (id & V4L2_STD_PAL) {
					mode = TVP5146_MODE_PAL;
				} else if (id & V4L2_STD_NTSC) {
					mode = TVP5146_MODE_NTSC;
				} else if (id & V4L2_STD_SECAM) {
					mode = TVP5146_MODE_SECAM;
				}
				vpfe->tvp5146_params.mode = mode | (sqp << 3);
				tvp5146_ctrl(TVP5146_CONFIG,
					     &vpfe->tvp5146_params);
			} else if (device_type == MT9T001
				   || device_type == MT9T031) {
				/* Store image window paramters and pixel
				 * aspect values as per standard
				 * passed by application
				 */
				if ((id == V4L2_STD_MT9T001_VGA_30FPS)
				    || (id == V4L2_STD_MT9T001_VGA_60FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = VGA_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      VGA_bounds);
				} else if ((id == V4L2_STD_MT9T001_SVGA_30FPS)
					   || (id ==
					       V4L2_STD_MT9T001_SVGA_60FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = SVGA_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      SVGA_bounds);
				} else if ((id == V4L2_STD_MT9T001_XGA_30FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = XGA_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      XGA_bounds);
				} else if ((id == V4L2_STD_MT9T001_480p_30FPS)
					   || (id ==
					       V4L2_STD_MT9T001_480p_60FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = P480_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      P480_bounds);
				} else if ((id == V4L2_STD_MT9T001_576p_25FPS)
					   || (id ==
					       V4L2_STD_MT9T001_576p_50FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = P576_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      P576_bounds);
				} else if ((id == V4L2_STD_MT9T001_720p_24FPS)
					   || (id ==
					       V4L2_STD_MT9T001_720p_30FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin = P720_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      P720_bounds);
				} else if ((id == V4L2_STD_MT9T001_1080p_18FPS)) {
					vpfe->std = id;
					vpfe->bounds = vpfe->vwin =
					    P1080_bounds;
					vpfe->pixelaspect = default_aspect;
					ccdc_set_image_window(CCDC_RAW,
							      P1080_bounds);
				} else {
					ret = -EINVAL;
				}
				if (ret != -EINVAL) {
					/* Call device control function to
					 * configure video standard
					 */
					dev_dbg(vpfe_dev,
						"\ncalling mt9t001 setup in SET_STD");
					ret =
					    vpfe->
					    config_dev_fxn(MT9T001_SET_STD,
							   &vpfe->std,
							   vpfe->device_params);
				}
			}
			dev_dbg(vpfe_dev, "\nAbove up(&vpfe->lock)");
			up(&vpfe->lock);
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_S_STD ioctl");
			break;
		}
	case VIDIOC_ENUMSTD:
		{
			struct v4l2_standard *std = (struct v4l2_standard *)arg;
			u32 index = std->index;
			dev_dbg(vpfe_dev, "\nStarting of VIDIOC_ENUMSTD ioctl");
			memset(std, 0, sizeof(*std));
			std->index = index;

			if (device_type == TVP5146) {
				if (index == 0) {
					std->id = V4L2_STD_525_60;
					strcpy(std->name, "SD-525line-30fps");
					std->framelines = 525;
					std->frameperiod.numerator = 1001;
					std->frameperiod.denominator = 30000;
				} else if (index == 1) {
					std->id = V4L2_STD_625_50;
					strcpy(std->name, "SD-625line-25fps");
					std->framelines = 625;
					std->frameperiod.numerator = 1;
					std->frameperiod.denominator = 25;
				} else if (index == 2) {
					std->id = VPFE_STD_625_50_SQP;
					strcpy(std->name,
					       "SD-625line-25fps square pixel");
					std->framelines = 625;
					std->frameperiod.numerator = 1;
					std->frameperiod.denominator = 25;
				} else if (index == 3) {
					std->id = VPFE_STD_525_60_SQP;
					strcpy(std->name,
					       "SD-525line-25fps square pixel");
					std->framelines = 525;
					std->frameperiod.numerator = 1001;
					std->frameperiod.denominator = 30000;
				} else if (index == 4) {
					std->id = VPFE_STD_AUTO;
					strcpy(std->name, "automatic detect");
					std->framelines = 625;
					std->frameperiod.numerator = 1;
					std->frameperiod.denominator = 1;
				} else if (index == 5) {
					std->id = VPFE_STD_AUTO_SQP;
					strcpy(std->name,
					       "automatic detect square pixel");
					std->framelines = 625;
					std->frameperiod.numerator = 1;
					std->frameperiod.denominator = 1;
				} else {
					ret = -EINVAL;
				}
			} else if (device_type == MT9T001
				   || device_type == MT9T031) {
				/* fill number of lines per frame and FPS value
				 *  as per standard passed by application
				 */
				if (index == 0) {
					std->id = V4L2_STD_MT9T001_VGA_30FPS;
					strcpy(std->name, "VGA-480line- 30fps");
					std->framelines = FMT_VGA_NUMLINES;
					std->frameperiod.numerator =
					    FPS_30_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_30_DENOMIRATOR;
				} else if (index == 1) {
					std->id = V4L2_STD_MT9T001_VGA_60FPS;
					strcpy(std->name, "VGA-480line- 60fps");
					std->framelines = FMT_VGA_NUMLINES;
					std->frameperiod.numerator =
					    FPS_60_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_60_DENOMIRATOR;
				} else if (index == 2) {
					std->id = V4L2_STD_MT9T001_SVGA_30FPS;
					strcpy(std->name,
					       "SVGA-600line- 30fps");
					std->framelines = FMT_SVGA_NUMLINES;
					std->frameperiod.numerator =
					    FPS_30_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_30_DENOMIRATOR;
				} else if (index == 3) {
					std->id = V4L2_STD_MT9T001_SVGA_60FPS;
					strcpy(std->name,
					       "SVGA-600line- 60fps");
					std->framelines = FMT_SVGA_NUMLINES;
					std->frameperiod.numerator =
					    FPS_60_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_30_DENOMIRATOR;
				} else if (index == 4) {
					std->id = V4L2_STD_MT9T001_XGA_30FPS;
					strcpy(std->name, "XGA-768line- 30fps");
					std->framelines = FMT_XGA_NUMLINES;
					std->frameperiod.numerator =
					    FPS_30_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_30_DENOMIRATOR;
				} else if (index == 5) {
					std->id = V4L2_STD_MT9T001_480p_30FPS;
					strcpy(std->name, "480p-480line-30fps");
					std->framelines = FMT_480p_NUMLINES;
					std->frameperiod.numerator =
					    FPS_30_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_30_DENOMIRATOR;
				} else if (index == 6) {
					std->id = V4L2_STD_MT9T001_480p_60FPS;
					strcpy(std->name, "480p-480line-60fps");
					std->framelines = FMT_480p_NUMLINES;
					std->frameperiod.numerator =
					    FPS_60_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_60_DENOMIRATOR;
				} else if (index == 7) {
					std->id = V4L2_STD_MT9T001_576p_25FPS;
					strcpy(std->name, "576p-576line-25fps");
					std->framelines = FMT_576p_NUMLINES;
					std->frameperiod.numerator =
					    FPS_25_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_25_DENOMIRATOR;
				} else if (index == 8) {
					std->id = V4L2_STD_MT9T001_576p_50FPS;
					strcpy(std->name, "576p-576line-50fps");
					std->framelines = FMT_576p_NUMLINES;
					std->frameperiod.numerator =
					    FPS_50_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_50_DENOMIRATOR;
				} else if (index == 9) {
					std->id = V4L2_STD_MT9T001_720p_24FPS;
					strcpy(std->name, "720p-720line-24fps");
					std->framelines = FMT_720p_NUMLINES;
					std->frameperiod.numerator =
					    FPS_24_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_24_DENOMIRATOR;
				} else if (index == 10) {
					std->id = V4L2_STD_MT9T001_720p_30FPS;
					strcpy(std->name, "720p-720line-30fps");
					std->framelines = FMT_720p_NUMLINES;
					std->frameperiod.numerator =
					    FPS_30_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_30_DENOMIRATOR;
				} else if (index == 11) {
					std->id = V4L2_STD_MT9T001_1080p_18FPS;
					strcpy(std->name,
					       "1080p-1080line-18fps");
					std->framelines = FMT_1080i_NUMLINES;
					std->frameperiod.numerator =
					    FPS_18_NUMERATOR;
					std->frameperiod.denominator =
					    FPS_18_DENOMIRATOR;
				} else {
					ret = -EINVAL;
				}
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_ENUMSTD ioctl");
			break;
		}
	case VIDIOC_ENUMINPUT:
		{
			u32 index = 0;
			struct v4l2_input *input = (struct v4l2_input *)arg;
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_ENUMINPUT ioctl");
			if (device_type != TVP5146) {
				return -1;
			}
			/* only two inputs are available */
			if (input->index > 1)
				ret = -EINVAL;
			index = input->index;
			memset(input, 0, sizeof(*input));
			input->index = index;
			input->type = V4L2_INPUT_TYPE_CAMERA;
			input->std = V4L2_STD_ALL;
			if (input->index == 0) {
				sprintf(input->name, "COMPOSITE");
			} else if (input->index == 1) {
				sprintf(input->name, "S-VIDEO");
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_ENUMINPUT ioctl");
			break;
		}
	case VIDIOC_G_INPUT:
		{
			int *index = (int *)arg;
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_G_INPUT ioctl");
			if (device_type != TVP5146) {
				return -1;
			}
			*index = vpfe->tvp5146_params.amuxmode;
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_G_INPUT ioctl");
			break;
		}
	case VIDIOC_S_INPUT:
		{
			int *index = (int *)arg;
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_S_INPUT ioctl");
			if (device_type != TVP5146) {
				return -1;
			}
			if (*index > 1 || *index < 0) {
				ret = -EINVAL;
			}
			vpfe->tvp5146_params.amuxmode = *index;
			tvp5146_ctrl(TVP5146_SET_AMUXMODE, index);
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_S_INPUT ioctl");
			break;
		}
	case VIDIOC_CROPCAP:
		{
			struct v4l2_cropcap *cropcap =
			    (struct v4l2_cropcap *)arg;
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_CROPCAP ioctl");
			cropcap->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			down_interruptible(&vpfe->lock);
			cropcap->bounds = cropcap->defrect = vpfe->vwin;
			cropcap->pixelaspect = vpfe->pixelaspect;
			up(&vpfe->lock);
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_CROPCAP ioctl");
			break;
		}
	case VIDIOC_G_PARM:
		{
			struct v4l2_streamparm *parm =
			    (struct v4l2_streamparm *)arg;
			dev_dbg(vpfe_dev, "\nStart of VIDIOC_G_PARM ioctl");
			if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				/* only capture is supported */
				ret = -EINVAL;
			} else {
				struct v4l2_captureparm *capparm =
				    &parm->parm.capture;
				memset(capparm, 0,
				       sizeof(struct v4l2_captureparm));
				down_interruptible(&vpfe->lock);

				if (device_type == TVP5146) {
					if (vpfe->std & V4L2_STD_625_50) {
						/* PAL 25fps */
						capparm->timeperframe.
						    numerator = 1;
						capparm->timeperframe.
						    denominator = 25;
					} else {
						/*NTSC 29.97fps */
						capparm->timeperframe.
						    numerator = 1001;
						capparm->timeperframe.
						    denominator = 30000;
					}
				} else if (device_type == MT9T001
					   || device_type == MT9T031) {
					/* fill FPS value as per standard */
					if (vpfe->
					    std & V4L2_STD_MT9T001_VGA_30FPS) {
						capparm->timeperframe.
						    numerator =
						    FPS_30_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_30_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_VGA_60FPS) {
						capparm->timeperframe.
						    numerator =
						    FPS_60_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_60_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_SVGA_30FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_30_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_30_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_SVGA_60FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_60_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_60_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_XGA_30FPS) {
						capparm->timeperframe.
						    numerator =
						    FPS_30_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_30_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_480p_30FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_30_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_30_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_480p_60FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_60_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_60_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_576p_25FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_25_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_25_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_576p_50FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_50_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_50_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_720p_24FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_24_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_24_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_720p_30FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_30_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_30_DENOMIRATOR;
					} else if (vpfe->
						   std &
						   V4L2_STD_MT9T001_1080p_18FPS)
					{
						capparm->timeperframe.
						    numerator =
						    FPS_18_NUMERATOR;
						capparm->timeperframe.
						    denominator =
						    FPS_18_DENOMIRATOR;
					}

				}

				/* Copy number of buffers allocated to arg
				 * passed
				 */
				capparm->readbuffers = vpfe->numbuffers;

				up(&vpfe->lock);
				dev_dbg(vpfe_dev,
					"\nEnd of VIDIOC_G_PARM ioctl");
			}
			break;
		}
	case VIDIOC_G_CTRL:
		dev_dbg(vpfe_dev, "\nStart of VIDIOC_G_CTRL ioctl");
		down_interruptible(&vpfe->lock);

		if (device_type == TVP5146) {
			tvp5146_ctrl(VIDIOC_G_CTRL, arg);
		} else if (device_type == MT9T001 || device_type == MT9T031) {
			/* Call device control function to get control value */
			ret = vpfe->config_dev_fxn(VIDIOC_G_CTRL, arg,
						   vpfe->device_params);
		}
		up(&vpfe->lock);
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_G_CTRL ioctl");
		break;
	case VIDIOC_S_CTRL:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_S_CTRL ioctl");
		down_interruptible(&vpfe->lock);

		if (device_type == TVP5146) {
			ret = tvp5146_ctrl(VIDIOC_S_CTRL, arg);
		} else if (device_type == MT9T001 || device_type == MT9T031) {
			/* Call device control function to configure control
			 * value
			 */
			ret = vpfe->config_dev_fxn(VIDIOC_S_CTRL, arg,
						   vpfe->device_params);
		}
		up(&vpfe->lock);
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_S_CTRL ioctl");
		break;
	case VIDIOC_QUERYCTRL:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_QUERYCTRL ioctl");
		down_interruptible(&vpfe->lock);

		if (device_type == TVP5146) {
			tvp5146_ctrl(VIDIOC_QUERYCTRL, arg);
		} else if (device_type == MT9T001 || device_type == MT9T031) {
			/* Call device control function to query about
			 * supported control commands
			 */
			ret = vpfe->config_dev_fxn(VIDIOC_QUERYCTRL, arg,
						   vpfe->device_params);
		}

		up(&vpfe->lock);
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_QUERYCTRL ioctl");

		break;
	case VIDIOC_G_CROP:
		{
			struct v4l2_crop *crop = arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_G_CROP ioctl");
			if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
				ret = -EINVAL;
			} else {
				crop->c = vpfe->vwin;
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_G_CROP ioctl");
			break;
		}
	case VIDIOC_S_CROP:
		{
			struct v4l2_crop *crop = arg;
			enum hw_frame frame_type =
			    (device_type == TVP5146) ? CCDC_YCBCR : CCDC_RAW;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_S_CROP ioctl");
			if (vpfe->started) {
				/* make sure streaming is not started */
				ret = -EBUSY;
				break;
			}
			/* adjust the width to 16 pixel boundry */
			crop->c.width = ((crop->c.width + 15) / 16) * 16;

			/* make sure parameters are valid */
			if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE
			    && (crop->c.left + crop->c.width
				<= vpfe->bounds.left + vpfe->bounds.width)
			    && (crop->c.top + crop->c.height
				<= vpfe->bounds.top + vpfe->bounds.height)) {

				down_interruptible(&vpfe->lock);
				vpfe->vwin = crop->c;
				ccdc_set_image_window(frame_type, vpfe->vwin);
				up(&vpfe->lock);
			} else {
				ret = -EINVAL;
			}
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_S_CROP ioctl");
			break;
		}
	case VIDIOC_QUERYSTD:
		{
			v4l2_std_id *id = (v4l2_std_id *) arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_QUERYSTD ioctl");

			if (device_type == TVP5146) {
				down_interruptible(&vpfe->lock);
				ret = sense_std(id);
				if (!ret)
					vpfe_update_std_info(*id);
				up(&vpfe->lock);
			} else {
				/* return -EINVAL indicating this ioctl
				 * is not supported
				 */
				ret = -EINVAL;
			}

			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_QUERYSTD ioctl");
			break;
		}
	case VIDIOC_G_PRIORITY:
		{
			enum v4l2_priority *p = arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_G_PRIORITY ioctl");
			*p = v4l2_prio_max(&vpfe->prio);
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_G_PRIORITY ioctl");
			break;
		}
	case VIDIOC_S_PRIORITY:
		{
			enum v4l2_priority *p = arg;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_S_PRIORITY ioctl");
			ret = v4l2_prio_change(&vpfe->prio, &fh->prio, *p);
			dev_dbg(vpfe_dev, "\nEnd of VIDIOC_S_PRIORITY ioctl");
			break;
		}

	case VIDIOC_REQBUFS:
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_REQBUFS ioctl");
		down_interruptible(&vpfe->lock);
		if (vpfe->io_usrs != 0) {
			ret = -EBUSY;
			up(&vpfe->lock);
			break;
		}
		videobuf_queue_init(&vpfe->bufqueue, &video_qops, NULL,
				    &vpfe->irqlock,
				    V4L2_BUF_TYPE_VIDEO_CAPTURE,
				    vpfe->field,
				    sizeof(struct videobuf_buffer), fh);

		videobuf_set_buftype(&vpfe->bufqueue, VIDEOBUF_BUF_LINEAR);

		fh->io_allowed = TRUE;
		vpfe->io_usrs = 1;
		INIT_LIST_HEAD(&vpfe->dma_queue);
		ret = videobuf_reqbufs(&vpfe->bufqueue, arg);
		up(&vpfe->lock);
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_REQBUFS ioctl");
		break;
	case VIDIOC_QUERYBUF:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_QUERYBUF ioctl");
		ret = videobuf_querybuf(&vpfe->bufqueue, arg);
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_QUERYBUF ioctl");
		break;
	case VIDIOC_QBUF:
		{
			struct v4l2_buffer tbuf = *(struct v4l2_buffer *)arg;
			struct videobuf_buffer *buf1;
			dev_dbg(vpfe_dev, "\nStarting VIDIOC_QBUF ioctl");
			if (!fh->io_allowed) {
				dev_dbg(vpfe_dev, "\nfh->io_allowed");
				ret = -EACCES;
				break;
			}
			if (!(list_empty(&vpfe->dma_queue)) ||
			    (vpfe->curFrm != vpfe->nextFrm) ||
			    !(vpfe->started) ||
			    (vpfe->started && (0 == vpfe->field_id))) {
				ret =
				    videobuf_qbuf(&vpfe->bufqueue,
						  (struct v4l2_buffer *)
						  arg);
				break;
			}

			/* bufferqueue is empty store buffer address
			 *  in VPFE registers */
			mutex_lock(&vpfe->bufqueue.lock);
			tbuf = *(struct v4l2_buffer *)arg;
			buf1 = vpfe->bufqueue.bufs[tbuf.index];
			if (buf1->memory != tbuf.memory) {
				dev_err(vpfe_dev, "invalid buffer" " type\n");
				mutex_unlock(&vpfe->bufqueue.lock);
				return -EINVAL;
			}
			if ((buf1->state == STATE_QUEUED) ||
			    (buf1->state == STATE_ACTIVE)) {
				dev_err(vpfe_dev, "invalid state\n");
				mutex_unlock(&vpfe->bufqueue.lock);
				return -EINVAL;
			}

			switch (buf1->memory) {
			case V4L2_MEMORY_MMAP:
				if (buf1->baddr == 0) {
					mutex_unlock(&vpfe->bufqueue.lock);
					return -EINVAL;
				}
				break;
			default:
				mutex_unlock(&vpfe->bufqueue.lock);
				return -EINVAL;
			}
			local_irq_save(flags);
			ret =
			    buffer_prepare(&vpfe->bufqueue,
					   buf1, vpfe->bufqueue.field);
			if (ret < 0) {
				local_irq_restore(flags);
				mutex_unlock(&vpfe->bufqueue.lock);
				return -EINVAL;
			}
			buf1->state = STATE_ACTIVE;
			addr = buf1->boff;
			ccdc_setfbaddr((unsigned long)addr);
			vpfe->nextFrm = buf1;
			local_irq_restore(flags);
			list_add_tail(&buf1->stream, &(vpfe->bufqueue.stream));
			mutex_unlock(&vpfe->bufqueue.lock);
			break;
		}
	case VIDIOC_DQBUF:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_DQBUF ioctl");
		if (!fh->io_allowed)
			ret = -EACCES;
		else {
			if (file->f_flags & O_NONBLOCK)
				ret = videobuf_dqbuf(&vpfe->bufqueue, arg, 1);
			else
				ret = videobuf_dqbuf(&vpfe->bufqueue, arg, 0);
		}
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_DQBUF ioctl");
		break;

	case VIDIOC_STREAMON:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_STREAMON ioctl");
		if (!fh->io_allowed) {
			ret = -EACCES;
			break;
		}
		if (vpfe->started) {
			ret = -EBUSY;
			break;
		}
		ret = videobuf_streamon(&vpfe->bufqueue);
		if (ret)
			break;

		down_interruptible(&vpfe->lock);
		/* get the current and next frame buffers */
		/* we expect at least one buffer is in driver at this point */
		/* if not, error is returned */
		if (list_empty(&vpfe->dma_queue)) {
			ret = -EIO;
			break;
		}
		dev_dbg(vpfe_dev, "cur frame %x.\n",
			(unsigned int)vpfe->dma_queue.next);
		vpfe->nextFrm = vpfe->curFrm =
		    list_entry(vpfe->dma_queue.next,
			       struct videobuf_buffer, queue);
		/* remove the buffer from the queue */
		list_del(&vpfe->curFrm->queue);
		vpfe->curFrm->state = STATE_ACTIVE;

		if (device_type == TVP5146) {
			/* sense the current video input standard */
			tvp5146_ctrl(TVP5146_CONFIG, &vpfe->tvp5146_params);
			frm_format = ccdc_get_frame_format(CCDC_YCBCR);
			ccdc_get_image_window(CCDC_YCBCR, &image_window);
			/* configure the ccdc and resizer as needed   */
			/* start capture by enabling CCDC and resizer */
			ccdc_config_ycbcr();
		} else {
			frm_format = ccdc_get_frame_format(CCDC_YCBCR);
			frm_format = ccdc_get_frame_format(CCDC_RAW);
			ccdc_get_image_window(CCDC_RAW, &image_window);
			/* configure the ccdc and resizer as needed   */
			/* start capture by enabling CCDC and resizer */
			ccdc_config_raw();
		}

		/* setup the memory address for the frame buffer */
		ccdc_setfbaddr(((unsigned long)(vpfe->curFrm->boff)));
		/* enable CCDC */
		vpfe->field_id = 0;
		vpfe->first_frame = 1;
		vpfe->fld_one_recvd = 0;
		vpfe->started = TRUE;
		vpfe->mode_changed = FALSE;
		vpfe->field_offset = (vpfe->vwin.height - 2) * vpfe->vwin.width;
		ccdc_enable(TRUE);
		up(&vpfe->lock);
		dev_dbg(vpfe_dev, "started video streaming.\n");
		break;
	case VIDIOC_STREAMOFF:
		dev_dbg(vpfe_dev, "\nStarting VIDIOC_STREAMOFF ioctl");
		if (!fh->io_allowed) {
			ret = -EACCES;
			break;
		}
		if (!vpfe->started) {
			ret = -EINVAL;
			break;
		}
		/* disable CCDC */
		down_interruptible(&vpfe->lock);
		ccdc_enable(FALSE);
		vpfe->started = FALSE;
		up(&vpfe->lock);
		ret = videobuf_streamoff(&vpfe->bufqueue);
		dev_dbg(vpfe_dev, "\nEnd of VIDIOC_STREAMOFF ioctl");
		break;

	case VPFE_CMD_CONFIG_CCDC_YCBCR:
		/* this can be used directly and bypass the V4L2 APIs */
		{

			if (vpfe->started) {
				/* only allowed if streaming is not started */
				ret = -EBUSY;
				break;
			}
			down_interruptible(&vpfe->lock);
			/* make sure the other v4l2 related fields
			   have consistant settings */
			ccdc_update_ycbcr_params(arg);
			ccdc_get_image_window(CCDC_YCBCR, &vpfe->vwin);

			if (ccdc_get_buf_type(CCDC_YCBCR) ==
			    CCDC_BUFTYPE_FLD_INTERLEAVED)
				vpfe->field = V4L2_FIELD_INTERLACED;
			else if (ccdc_get_buf_type(CCDC_YCBCR) ==
				 CCDC_BUFTYPE_FLD_SEPARATED)
				vpfe->field = V4L2_FIELD_SEQ_TB;

			if (ccdc_get_pix_order() == CCDC_PIXORDER_YCBYCR)
				vpfe->pixelfmt = V4L2_PIX_FMT_YUYV;
			else if (ccdc_get_pix_order() == CCDC_PIXORDER_CBYCRY)
				vpfe->pixelfmt = V4L2_PIX_FMT_UYVY;

			up(&vpfe->lock);
			break;
		}
	case VPFE_CMD_CONFIG_CCDC_RAW:
		/* This command is used to configure driver for CCDC Raw
		 * mode parameters
		 */
		{
			if (vpfe->started) {
				/* only allowed if streaming is not started */
				ret = -EBUSY;
				break;
			}

			if (validate_ccdc_param((ccdc_config_params_raw *) arg)
			    == -1) {
				dev_err(vpfe_dev,
					"\nValidation of ccdc parameters failed \n");
				return -EINVAL;
			} else {
				dev_err(vpfe_dev, "\n Validation pass\n");
				if (ccdc_update_raw_params(arg) == -1) {
					dev_err(vpfe_dev,
						"\n ccdc parameters update failed \n");
					return -EINVAL;
				}
			}
			break;
		}

	case VPFE_CMD_CONFIG_TVP5146:
		/* this can be used directly and bypass the V4L2 APIs */
		{
			/* the settings here must be consistant with that of
			   the CCDC's,driver does not check the consistancy */
			tvp5146_params *params = (tvp5146_params *) arg;
			v4l2_std_id std = 0;
			dev_dbg(vpfe_dev,
				"\nStarting VPFE_CMD_CONFIG_TVP5146 ioctl");
			if (vpfe->started) {
				/* only allowed if streaming is not started */
				ret = -EBUSY;
				break;
			}
			down_interruptible(&vpfe->lock);
			/* make sure the other v4l2 related fields have
			 * consistant settings
			 */
			switch (params->mode & 0x7) {
			case TVP5146_MODE_NTSC:
				std = V4L2_STD_NTSC;
				break;
			case TVP5146_MODE_PAL:
				std = V4L2_STD_PAL;
				break;
			case TVP5146_MODE_PAL_M:
				std = V4L2_STD_PAL_M;
				break;
			case TVP5146_MODE_PAL_CN:
				std = V4L2_STD_PAL_N;
				break;
			case TVP5146_MODE_SECAM:
				std = V4L2_STD_SECAM;
				break;
			case TVP5146_MODE_PAL_60:
				std = V4L2_STD_PAL_60;
				break;
			}
			dev_dbg(vpfe_dev,
				"\nVPFE_CMD_CONFIG_TVP5146:std = %d", (int)std);
			if (params->mode & 0x8) {	/* square pixel mode */
				std <<= 32;
			}
			/* auto-detection modes */
			if (params->mode == TVP5146_MODE_AUTO) {
				std = VPFE_STD_AUTO;
			} else if (params->mode == TVP5146_MODE_AUTO_SQP) {
				std = VPFE_STD_AUTO_SQP;
			}

			if (std & V4L2_STD_625_50) {
				vpfe->bounds = pal_bounds;
				vpfe->pixelaspect = pal_aspect;
			} else if (std & V4L2_STD_525_60) {
				vpfe->bounds = ntsc_bounds;
				vpfe->pixelaspect = ntsc_aspect;
			} else if (std & VPFE_STD_625_50_SQP) {
				vpfe->bounds = palsp_bounds;
				vpfe->pixelaspect = sp_aspect;
			} else if (std & VPFE_STD_525_60_SQP) {
				vpfe->bounds = ntscsp_bounds;
				vpfe->pixelaspect = sp_aspect;
			}
			vpfe->std = std;
			ret = tvp5146_ctrl(TVP5146_CONFIG, params);
			dev_dbg(vpfe_dev,
				"\nVPFE_CMD_CONFIG_TVP5146:ret = %d", (int)ret);
			vpfe->tvp5146_params = *params;
			up(&vpfe->lock);
			dev_dbg(vpfe_dev,
				"\nEnd VPFE_CMD_CONFIG_TVP5146 ioctl");
			break;
		}
	case VPFE_CMD_S_MT9T001_PARAMS:
		/* This command configures MT9T001 */
		{
			dev_dbg(vpfe_dev,
				"\nStarting VPFE_CMD_S_MT9T001_PARAMS ioctl");
			/* Call device control function to configure MT9T001 */
			ret = vpfe->config_dev_fxn(MT9T001_SET_PARAMS, arg,
						   vpfe->device_params);
			dev_dbg(vpfe_dev,
				"\nEnd VPFE_CMD_S_MT9T001_PARAMS ioctl");
			break;
		}
	case VPFE_CMD_G_MT9T001_PARAMS:
		/* This command returns MT9T001 configuration values to
		 * application.
		 */
		{
			dev_dbg(vpfe_dev,
				"\nStarting VPFE_CMD_G_MT9T001_PARAMS ioctl");
			/* Call device control function to get
			 * MT9T001 configuration values
			 */
			ret = vpfe->config_dev_fxn(MT9T001_GET_PARAMS, arg,
						   vpfe->device_params);
			dev_dbg(vpfe_dev,
				"\nEnd of VPFE_CMD_G_MT9T001_PARAMS ioctl");
			break;
		}
	default:
		dev_dbg(vpfe_dev, "\nDefault ioctl");
		ret = -ENOIOCTLCMD;
		break;
	}			/* end switch(cmd) */
	return ret;
}

/*
 * ======== vpfe_ioctl ========
 */
static int vpfe_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	char sbuf[128];
	void *mbuf = NULL;
	void *parg = NULL;

	dev_dbg(vpfe_dev, "Start of vpfe ioctl\n");
	if (ISEXTERNALCMD(cmd)) {
		ret = vpfe_doioctl(inode, file, cmd, (void *)arg);
		if (ret == -ENOIOCTLCMD)
			ret = -EINVAL;
		goto out;
	}

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_NONE:
		parg = NULL;
		break;
	case _IOC_READ:
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
		if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
			parg = sbuf;
		} else {
			/* too big to allocate from stack */
			mbuf = kmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
			if (NULL == mbuf)
				return -ENOMEM;
			parg = mbuf;
		}

		ret = -EFAULT;
		if (_IOC_DIR(cmd) & _IOC_WRITE)
			if (copy_from_user(parg, (void __user *)arg,
					   _IOC_SIZE(cmd)))
				goto out;
		break;
	}

	/* call driver */
	ret = vpfe_doioctl(inode, file, cmd, (void *)parg);
	if (ret == -ENOIOCTLCMD)
		ret = -EINVAL;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_READ:
	case (_IOC_WRITE | _IOC_READ):
		if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd)))
			ret = -EFAULT;
		break;
	}
out:
	kfree(mbuf);

	dev_dbg(vpfe_dev, "End of vpfe ioctl\n");
	return ret;
}

/*
 * ======== vpfe_mmap ========
 */
static int vpfe_mmap(struct file *file, struct vm_area_struct *vma)
{
	dev_dbg(vpfe_dev, "\nStarting of vpfe_mmap...");
	return videobuf_mmap_mapper(&vpfe_device.bufqueue, vma);
}

/*
 * ======== vpfe_open ========
 */
 /* Creates a file handle for video object. Initialize itwith default value */
static int vpfe_open(struct inode *inode, struct file *filep)
{
	int minor = iminor(inode);
	vpfe_obj *vpfe = NULL;
	vpfe_fh *fh = NULL;

	dev_dbg(vpfe_dev, "vpfe: open minor=%d\n", minor);

	/* check to make sure the minor numbers match */
	if (vpfe_device.video_dev && vpfe_device.video_dev->minor == minor) {
		vpfe = &vpfe_device;
	} else {		/* device not found here */
		return -ENODEV;
	}

	/* allocate per filehandle data */
	if ((fh = kmalloc(sizeof(*fh), GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}
	filep->private_data = fh;
	fh->dev = vpfe;
	fh->io_allowed = FALSE;
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&vpfe->prio, &fh->prio);
	vpfe->usrs++;
	dev_dbg(vpfe_dev, "\nvpfe_open done ...");
	return 0;
}

/*
 * ======== vpfe_release ========
 */
 /* This function disables the CCDC, Deletes the buffer queue and frees
  * the vpfe file handle.
  */
static int vpfe_release(struct inode *inode, struct file *filep)
{
	vpfe_fh *fh = filep->private_data;
	vpfe_obj *vpfe = fh->dev;
	dev_dbg(vpfe_dev, "\nStarting of vpfe_release...");
	down_interruptible(&vpfe->lock);
	if (fh->io_allowed) {
		vpfe->io_usrs = 0;
		ccdc_enable(FALSE);
		vpfe->started = FALSE;
		videobuf_queue_cancel(&vpfe->bufqueue);
		videobuf_mmap_free(&vpfe->bufqueue);
		vpfe->numbuffers = VPFE_DEFNUM_FBUFS;
	}
	vpfe->usrs--;
	v4l2_prio_close(&vpfe->prio, &fh->prio);
	filep->private_data = NULL;
	if (fh != NULL)
		kfree(fh);
	up(&vpfe->lock);
	dev_dbg(vpfe_dev, "\nEnd of vpfe_release...");
	return 0;
}

static struct file_operations vpfe_fops = {
	.owner = THIS_MODULE,
	.open = vpfe_open,
	.release = vpfe_release,
	.ioctl = vpfe_ioctl,
	.mmap = vpfe_mmap
};

static struct video_device vpfe_video_template = {
	.name = "vpfe",
	.type = VID_TYPE_CAPTURE | VID_TYPE_CLIPPING | VID_TYPE_SCALES,
	.hardware = 0,
	.fops = &vpfe_fops,
	.minor = -1,
};

static void vpfe_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero. */
}

/*
 * ======== vpfe_probe ========
 */
 /* This function will allocate video device initialize and
  * registers the device
  */
static int __init vpfe_probe(struct device *device)
{
	struct video_device *vfd;
	vpfe_obj *vpfe = &vpfe_device;
	vpfe_dev = device;
	dev_dbg(vpfe_dev, "\nStarting of vpfe_probe...");
	/* alloc video device */
	if ((vfd = video_device_alloc()) == NULL) {
		return -ENOMEM;
	}
	*vfd = vpfe_video_template;
	vfd->dev = device;
	vfd->release = video_device_release;

	snprintf(vfd->name, sizeof(vfd->name),
		 "DaVinci_VPFE_DRIVER_V%d.%d.%d",
		 (VPFE_VERSION_CODE >> 16) & 0xff,
		 (VPFE_VERSION_CODE >> 8) & 0xff, (VPFE_VERSION_CODE) & 0xff);

	vpfe->video_dev = vfd;
	vpfe->usrs = 0;
	vpfe->io_usrs = 0;
	vpfe->started = FALSE;
	vpfe->latest_only = TRUE;

	v4l2_prio_init(&vpfe->prio);
	init_MUTEX(&vpfe->lock);

	/* register video device */
	dev_dbg(vpfe_dev, "trying to register vpfe device.\n");
	dev_dbg(vpfe_dev, "vpfe=%x,vpfe->video_dev=%x\n", (int)vpfe,
		(int)&vpfe->video_dev);
	if (video_register_device(vpfe->video_dev, VFL_TYPE_GRABBER, -1) < 0) {
		video_device_release(vpfe->video_dev);
		vpfe->video_dev = NULL;
		return -1;
	}

	dev_dbg(vpfe_dev, "DaVinci vpfe: driver version V%d.%d.%d loaded\n",
		(VPFE_VERSION_CODE >> 16) & 0xff,
		(VPFE_VERSION_CODE >> 8) & 0xff, (VPFE_VERSION_CODE) & 0xff);

	dev_dbg(vpfe_dev, "vpfe: registered device video%d\n",
		vpfe->video_dev->minor & 0x1f);

	/* all done */
	return 0;
}

/*
 * ======== vpfe_remove ========
 */
static int vpfe_remove(struct device *device)
{
	/* un-register device */
	dev_dbg(vpfe_dev, "\nUnregistering device...");
	video_unregister_device(vpfe_device.video_dev);

	return 0;
}

static struct device_driver vpfe_driver = {
	.name = "vpfe",
	.bus = &platform_bus_type,
	.probe = vpfe_probe,
	.remove = vpfe_remove,
};

static struct platform_device _vpfe_device = {
	.name = "vpfe",
	.id = 1,
	.dev = {
		.release = vpfe_platform_release,
		}
};

#ifdef VPFE_DEBUG
static int vpfe_read_proc(char *page, char **start,
			  off_t off, int count, int *eof, void *data)
{
	int len;

	/* DON'T DO THAT - buffer overruns are bad */
	len = sprintf(page, "qbuf_count = %d, dqbuf_count = %d\n",
		      stats.qbuf_count, stats.dqbuf_count);

	return len;
}

static int vpfe_write_proc(struct file *file,
			   const char *buffer, unsigned long count, void *data)
{
	int len = 0;
	stats.qbuf_count = 0;
	stats.dqbuf_count = 0;
	printk(KERN_NOTICE "reset stats count\n");
	return len;
}
#endif

/*
 * ======== vpfe_init ========
 */
 /*This function allocates free pages and register the driver. Then reset the
  * CCDC and configure capture device with default parameters
  */
static int vpfe_init(void)
{
	int i = 0;
	int fbuf_size;
	ccdc_frmfmt frame_format;
	void *mem;
	int ret = 0;

#ifdef VPFE_DEBUG
	vpfe_stats_dir = proc_mkdir(PROCNAME, NULL);
	if (vpfe_stats_dir != NULL) {
		vpfe_stats_dir->owner = THIS_MODULE;
		vpfe_stats = create_proc_entry("stats", 0644, vpfe_stats_dir);
		if (vpfe_stats != NULL) {
			vpfe_stats->read_proc = vpfe_read_proc;
			vpfe_stats->write_proc = vpfe_write_proc;
			vpfe_stats->owner = THIS_MODULE;
			printk(KERN_NOTICE "successfully created proc file\n");
		} else {
			printk(KERN_ERR "Error in creating proc file\n");
		}
	} else {
		printk(KERN_ERR "Error in creating proc dir\n");
	}
#endif

	ccdc_init();

	if (device_type == TVP5146) {
		fbuf_size = VPFE_TVP5146_MAX_FBUF_SIZE;
		vpfe_device = vpfe_device_ycbcr;
		frame_format = ccdc_get_frame_format(CCDC_YCBCR);
	}

	else if (device_type == MT9T001 || device_type == MT9T031) {
		fbuf_size = VPFE_MT9T001_MAX_FBUF_SIZE;
		vpfe_device = vpfe_device_raw;
		frame_format = ccdc_get_frame_format(CCDC_RAW);
	} else {
		return -1;
	}
	/* allocate memory at initialization time to guarentee availability */
	for (i = 0; i < VPFE_DEFNUM_FBUFS; i++) {
		mem =
		    (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
					     get_order(fbuf_size));
		if (mem) {
			unsigned long adr = (unsigned long)mem;
			u32 size = PAGE_SIZE << (get_order(fbuf_size));
			while (size > 0) {
				/* make sure the frame buffers
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			vpfe_device.fbuffers[i] = (u8 *) mem;
		} else {
			while (--i >= 0) {
				free_reserved_pages((unsigned long)
						    vpfe_device.
						    fbuffers[i], fbuf_size);
			}
			dev_err(vpfe_dev,
				"frame buffer memory allocation failed.\n");
			return -ENOMEM;
		}
	}
	if (driver_register(&vpfe_driver) != 0) {
		dev_err(vpfe_dev, "driver registration failed\n");
		return -1;
	}
	if (platform_device_register(&_vpfe_device) != 0) {
		driver_unregister(&vpfe_driver);
		dev_err(vpfe_dev, "device registration failed\n");
		return -1;
	}

	ccdc_reset();

	if (device_type == TVP5146) {
		ret = tvp5146_ctrl(TVP5146_INIT, NULL);
		if (ret >= 0) {
			ret = tvp5146_ctrl(TVP5146_RESET, NULL);
			/* configure the tvp5146 to default parameters */
			ret |=
			    tvp5146_ctrl(TVP5146_CONFIG,
					 &vpfe_device.tvp5146_params);
		}
		if (ret < 0) {
			tvp5146_ctrl(TVP5146_CLEANUP, NULL);
		}
	} else if (device_type == MT9T001 || device_type == MT9T031) {
		/* enable video port in case of raw capture */
		ccdc_enable_vport();
		vpfe_device.config_dev_fxn = mt9t001_ctrl;
		/* enable the i2c switch on the MT9T031 head board */
		if (device_type == MT9T031)
			vpfe_device.config_dev_fxn(MT9T001_ENABLE_I2C_SWITCH,
						   NULL, NULL);
		ret =
		    vpfe_device.config_dev_fxn(MT9T001_INIT,
					       &vpfe_device.std,
					       &vpfe_device.device_params);
	}

	if (ret < 0) {
		platform_device_unregister(&_vpfe_device);
		driver_unregister(&vpfe_driver);
		/* Free memory for all image buffers */
		for (i = 0; i < VPFE_DEFNUM_FBUFS; i++) {
			free_reserved_pages((unsigned long)
					    vpfe_device.fbuffers[i], fbuf_size);
		}
		return -1;
	}

	/* setup interrupt handling */
	/* request VDINT1 if progressive format */
	if (frame_format == CCDC_FRMFMT_PROGRESSIVE) {
		ret = request_irq(IRQ_VDINT1, vdint1_isr, SA_INTERRUPT,
				  "vpfe_capture", (void *)&vpfe_device);
		if (ret < 0) {
			platform_device_unregister(&_vpfe_device);
			driver_unregister(&vpfe_driver);
			/* Free memory for all image buffers */
			for (i = 0; i < VPFE_DEFNUM_FBUFS; i++) {
				free_reserved_pages((unsigned long)
						    vpfe_device.
						    fbuffers[i], fbuf_size);
			}
			return -1;
		}
	}
	ret = request_irq(IRQ_VDINT0, vpfe_isr, SA_INTERRUPT,
			  "vpfe_capture", (void *)&vpfe_device);
	if (ret < 0) {
		platform_device_unregister(&_vpfe_device);
		driver_unregister(&vpfe_driver);
		/* Free memory for all image buffers */
		for (i = 0; i < VPFE_DEFNUM_FBUFS; i++) {
			free_reserved_pages((unsigned long)
					    vpfe_device.fbuffers[i], fbuf_size);
		}
		free_irq(IRQ_VDINT1, &vpfe_device);
		return -1;
	}

	dev_err(vpfe_dev, "DaVinci v4l2 capture driver V1.0 loaded\n");
	return 0;
}

/*
 * ======== vpfe_cleanup ========
 */
 /*This function unregisters the driver and free the allocated pages */
static void vpfe_cleanup(void)
{
	int i = vpfe_device.numbuffers;
	int buf_size;
	ccdc_frmfmt frame_format;
	//unsigned int *fpc_physaddr = NULL, *fpc_virtaddr = NULL;

	if (device_type == TVP5146) {
		tvp5146_ctrl(TVP5146_CLEANUP, NULL);
		buf_size = VPFE_TVP5146_MAX_FBUF_SIZE;
		frame_format = ccdc_get_frame_format(CCDC_YCBCR);
	} else {
		buf_size = VPFE_MT9T001_MAX_FBUF_SIZE;
		/* Free mt9t001 object memory */
		vpfe_device.config_dev_fxn(MT9T001_CLEANUP, NULL,
					   vpfe_device.device_params);
		frame_format = ccdc_get_frame_format(CCDC_RAW);
	}

	platform_device_unregister(&_vpfe_device);
	driver_unregister(&vpfe_driver);
	/* disable interrupt */
	free_irq(IRQ_VDINT0, &vpfe_device);
	/* Free VDINT1 if progressive format */
	if (frame_format == CCDC_FRMFMT_PROGRESSIVE) {
		free_irq(IRQ_VDINT1, &vpfe_device);
	}

	ccdc_cleanup();

	/* Free memory for all image buffers */
	while (--i >= 0) {
		free_reserved_pages((unsigned long)vpfe_device.
				    fbuffers[i], buf_size);
	}

#ifdef VPFE_DEBUG
	remove_proc_entry("stats", vpfe_stats_dir);
	remove_proc_entry(PROCNAME, NULL);
#endif
}

module_init(vpfe_init);
module_exit(vpfe_cleanup);
