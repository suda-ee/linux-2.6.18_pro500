/*
 *  linux/arch/x86_64/kernel/vsyscall.c
 *
 *  Copyright (C) 2001 Andrea Arcangeli <andrea@suse.de> SuSE
 *  Copyright 2003 Andi Kleen, SuSE Labs.
 *
 *  Thanks to hpa@transmeta.com for some useful hint.
 *  Special thanks to Ingo Molnar for his early experience with
 *  a different vsyscall implementation for Linux/IA32 and for the name.
 *
 *  vsyscall 1 is located at -10Mbyte, vsyscall 2 is located
 *  at virtual address -10Mbyte+1024bytes etc... There are at max 4
 *  vsyscalls. One vsyscall can reserve more than 1 slot to avoid
 *  jumping out of line if necessary. We cannot add more with this
 *  mechanism because older kernels won't return -ENOSYS.
 *  If we want more than four we need a vDSO.
 *
 *  Note: the concept clashes with user mode linux. If you use UML and
 *  want per guest time just set the kernel.vsyscall64 sysctl to 0.
 */

#include <linux/time.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/seqlock.h>
#include <linux/jiffies.h>
#include <linux/sysctl.h>
#include <linux/clocksource.h>

#include <asm/vsyscall.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/fixmap.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/unistd.h>

#define __vsyscall(nr) __attribute__ ((unused,__section__(".vsyscall_" #nr))) notrace

struct vsyscall_gtod_data_t {
	raw_seqlock_t lock;
	int sysctl_enabled;
	struct timeval wall_time_tv;
	struct timezone sys_tz;
	cycle_t offset_base;
	struct clocksource clock;
};

struct vsyscall_gtod_data_t __vsyscall_gtod_data __section_vsyscall_gtod_data =  {
	.lock = __RAW_SEQLOCK_UNLOCKED(__vsyscall_gtod_data.lock),
	.sysctl_enabled = 1,
};

void update_vsyscall(struct timespec* wall_time, struct clocksource* clock)
{
	unsigned long flags;

	write_seqlock_irqsave(&vsyscall_gtod_data.lock, flags);
	/* copy vsyscall data */
	vsyscall_gtod_data.clock = *clock;
	vsyscall_gtod_data.wall_time_tv.tv_sec = wall_time->tv_sec;
	vsyscall_gtod_data.wall_time_tv.tv_usec = wall_time->tv_nsec/1000;
	vsyscall_gtod_data.sys_tz = sys_tz;

	write_sequnlock_irqrestore(&vsyscall_gtod_data.lock, flags);
}

/* RED-PEN may want to readd seq locking, but then the variable should be write-once. */
static __always_inline void do_get_tz(struct timezone * tz)
{
	*tz = __vsyscall_gtod_data.sys_tz;
}

static __always_inline int gettimeofday(struct timeval *tv, struct timezone *tz)
{
	int ret;
	asm volatile("vsysc2: syscall"
		: "=a" (ret)
		: "0" (__NR_gettimeofday),"D" (tv),"S" (tz) : __syscall_clobber );
	return ret;
}

static __always_inline long time_syscall(long *t)
{
	long secs;
	asm volatile("vsysc1: syscall"
		: "=a" (secs)
		: "0" (__NR_time),"D" (t) : __syscall_clobber);
	return secs;
}

static __always_inline void do_vgettimeofday(struct timeval * tv)
{
	cycle_t now, base, mask, cycle_delta;
	unsigned long seq, mult, shift, nsec_delta;
	cycle_t (*vread)(void);
	do {
		seq = read_seqbegin(&__vsyscall_gtod_data.lock);

		vread = __vsyscall_gtod_data.clock.vread;
		if (unlikely(!__vsyscall_gtod_data.sysctl_enabled || !vread)) {
			gettimeofday(tv,0);
			return;
		}
		now = vread();
		base = __vsyscall_gtod_data.clock.cycle_last;
		mask = __vsyscall_gtod_data.clock.mask;
		mult = __vsyscall_gtod_data.clock.mult;
		shift = __vsyscall_gtod_data.clock.shift;

		*tv = __vsyscall_gtod_data.wall_time_tv;

	} while (read_seqretry(&__vsyscall_gtod_data.lock, seq));

	/* calculate interval: */
	cycle_delta = (now - base) & mask;
	/* convert to nsecs: */
	nsec_delta = (cycle_delta * mult) >> shift;

	/* convert to usecs and add to timespec: */
	tv->tv_usec += nsec_delta / NSEC_PER_USEC;
	while (tv->tv_usec > USEC_PER_SEC) {
		tv->tv_sec += 1;
		tv->tv_usec -= USEC_PER_SEC;
	}
}

int __vsyscall(0) vgettimeofday(struct timeval * tv, struct timezone * tz)
{
	if (tv)
		do_vgettimeofday(tv);
	if (tz)
		do_get_tz(tz);
	return 0;
}

/* This will break when the xtime seconds get inaccurate, but that is
 * unlikely */
time_t __vsyscall(1) vtime(time_t *t)
{
	if (unlikely(!__vsyscall_gtod_data.sysctl_enabled))
		return time_syscall(t);
	else if (t)
		*t = __vsyscall_gtod_data.wall_time_tv.tv_sec;
	return __vsyscall_gtod_data.wall_time_tv.tv_sec;
}

long __vsyscall(2) venosys_0(void)
{
	return -ENOSYS;
}

long __vsyscall(3) venosys_1(void)
{
	return -ENOSYS;
}

#ifdef CONFIG_SYSCTL

#define SYSCALL 0x050f
#define NOP2    0x9090

/*
 * NOP out syscall in vsyscall page when not needed.
 */
static int vsyscall_sysctl_change(ctl_table *ctl, int write, struct file * filp,
                        void __user *buffer, size_t *lenp, loff_t *ppos)
{
	extern u16 vsysc1, vsysc2;
	u16 *map1, *map2;
	int ret = proc_dointvec(ctl, write, filp, buffer, lenp, ppos);
	if (!write)
		return ret;
	/* gcc has some trouble with __va(__pa()), so just do it this
	   way. */
	map1 = ioremap(__pa_symbol(&vsysc1), 2);
	if (!map1)
		return -ENOMEM;
	map2 = ioremap(__pa_symbol(&vsysc2), 2);
	if (!map2) {
		ret = -ENOMEM;
		goto out;
	}
	if (!vsyscall_gtod_data.sysctl_enabled) {
		*map1 = SYSCALL;
		*map2 = SYSCALL;
	} else {
		*map1 = NOP2;
		*map2 = NOP2;
	}
	iounmap(map2);
out:
	iounmap(map1);
	return ret;
}

static int vsyscall_sysctl_nostrat(ctl_table *t, int __user *name, int nlen,
				void __user *oldval, size_t __user *oldlenp,
				void __user *newval, size_t newlen,
				void **context)
{
	return -ENOSYS;
}

static ctl_table kernel_table2[] = {
	{ .ctl_name = 99, .procname = "vsyscall64",
	  .data = &vsyscall_gtod_data.sysctl_enabled, .maxlen = sizeof(int), .mode = 0644,
	  .strategy = vsyscall_sysctl_nostrat,
	  .proc_handler = vsyscall_sysctl_change },
	{ 0, }
};

static ctl_table kernel_root_table2[] = {
	{ .ctl_name = CTL_KERN, .procname = "kernel", .mode = 0555,
	  .child = kernel_table2 },
	{ 0 },
};

#endif

static void __init map_vsyscall(void)
{
	extern char __vsyscall_0;
	unsigned long physaddr_page0 = __pa_symbol(&__vsyscall_0);

	__set_fixmap(VSYSCALL_FIRST_PAGE, physaddr_page0, PAGE_KERNEL_VSYSCALL);
}

static int __init vsyscall_init(void)
{
	BUG_ON(((unsigned long) &vgettimeofday !=
			VSYSCALL_ADDR(__NR_vgettimeofday)));
	BUG_ON((unsigned long) &vtime != VSYSCALL_ADDR(__NR_vtime));
	BUG_ON((VSYSCALL_ADDR(0) != __fix_to_virt(VSYSCALL_FIRST_PAGE)));
	map_vsyscall();
#ifdef CONFIG_SYSCTL
	register_sysctl_table(kernel_root_table2, 0);
#endif
	return 0;
}

__initcall(vsyscall_init);
