/*
 * Common time routines among all ppc machines.
 *
 * Written by Cort Dougan (cort@cs.nmt.edu) to merge
 * Paul Mackerras' version and mine for PReP and Pmac.
 * MPC8xx/MBX changes by Dan Malek (dmalek@jlc.net).
 * Converted for 64-bit by Mike Corrigan (mikejc@us.ibm.com)
 *
 * First round of bugfixes by Gabriel Paubert (paubert@iram.es)
 * to make clock more stable (2.4.0-test5). The only thing
 * that this code assumes is that the timebases have been synchronized
 * by firmware on SMP and are never stopped (never do sleep
 * on SMP then, nap and doze are OK).
 *
 * Speeded up do_gettimeofday by getting rid of references to
 * xtime (which required locks for consistency). (mikejc@us.ibm.com)
 *
 * TODO (not necessarily in this file):
 * - improve precision and reproducibility of timebase frequency
 * measurement at boot time. (for iSeries, we calibrate the timebase
 * against the Titan chip's clock.)
 * - for astronomical applications: add a new function to get
 * non ambiguous timestamps even around leap seconds. This needs
 * a new timestamp format and a good name.
 *
 * 1997-09-10  Updated NTP code according to technical memorandum Jan '96
 *             "A Kernel Model for Precision Timekeeping" by Dave Mills
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/kernel_stat.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/profile.h>
#include <linux/cpu.h>
#include <linux/security.h>
#include <linux/percpu.h>
#include <linux/rtc.h>
#include <linux/jiffies.h>
#include <linux/posix-timers.h>
#include <linux/clockchips.h>

#include <asm/io.h>
#include <asm/processor.h>
#include <asm/nvram.h>
#include <asm/cache.h>
#include <asm/machdep.h>
#include <asm/uaccess.h>
#include <asm/time.h>
#include <asm/prom.h>
#include <asm/irq.h>
#include <asm/div64.h>
#include <asm/smp.h>
#include <asm/vdso_datapage.h>
#ifdef CONFIG_PPC64
#include <asm/firmware.h>
#endif
#ifdef CONFIG_PPC_ISERIES
#include <asm/iseries/it_lp_queue.h>
#include <asm/iseries/hv_call_xm.h>
#endif
#include <asm/smp.h>

unsigned long cpu_khz;	/* Detected as we calibrate the TSC */
EXPORT_SYMBOL(cpu_khz);

/* keep track of when we need to update the rtc */
time_t last_rtc_update;
#ifdef CONFIG_PPC_ISERIES
unsigned long iSeries_recal_titan = 0;
unsigned long iSeries_recal_tb = 0;
static unsigned long first_settimeofday = 1;
#endif

/* The decrementer counts down by 128 every 128ns on a 601. */
#define DECREMENTER_COUNT_601	(1000000000 / HZ)

#define XSEC_PER_SEC (1024*1024)

#ifdef CONFIG_PPC64
#define SCALE_XSEC(xsec, max)	(((xsec) * max) / XSEC_PER_SEC)
#else
/* compute ((xsec << 12) * max) >> 32 */
#define SCALE_XSEC(xsec, max)	mulhwu((xsec) << 12, max)
#endif

unsigned long tb_ticks_per_jiffy;
unsigned long tb_ticks_per_usec = 100; /* sane default */
EXPORT_SYMBOL(tb_ticks_per_usec);
unsigned long tb_ticks_per_sec;
EXPORT_SYMBOL(tb_ticks_per_sec);	/* for cputime_t conversions */
u64 tb_to_xs;
unsigned tb_to_us;

#define TICKLEN_SCALE	TICK_LENGTH_SHIFT
u64 last_tick_len;	/* units are ns / 2^TICKLEN_SCALE */
u64 ticklen_to_xs;	/* 0.64 fraction */

/* If last_tick_len corresponds to about 1/HZ seconds, then
   last_tick_len << TICKLEN_SHIFT will be about 2^63. */
#define TICKLEN_SHIFT	(63 - 30 - TICKLEN_SCALE + SHIFT_HZ)

DEFINE_SPINLOCK(rtc_lock);
EXPORT_SYMBOL_GPL(rtc_lock);

u64 tb_to_ns_scale;
unsigned tb_to_ns_shift;

extern struct timezone sys_tz;
static long timezone_offset;

unsigned long ppc_proc_freq;
unsigned long ppc_tb_freq;

static u64 tb_last_jiffy __cacheline_aligned_in_smp;
static DEFINE_PER_CPU(u64, last_jiffy);

#ifdef CONFIG_GENERIC_CLOCKEVENTS

#if defined(CONFIG_40x) || defined(CONFIG_BOOKE)
#define DECREMENTER_MAX 0xffffffff
#else
#define DECREMENTER_MAX 0x7fffffff /* setting MSB triggers an interrupt */
#endif

struct decrementer_device {
       struct clock_event_device device;
       int mode;
};

static void decrementer_set_next_event(unsigned long evt,
				       struct clock_event_device *dev)
{
#if defined(CONFIG_40x)
	mtspr(SPRN_PIT, evt);	/* 40x has a hidden PIT auto-reload register */
#elif defined(CONFIG_BOOKE)
	mtspr(SPRN_DECAR, evt); /* Book E  has separate auto-reload register */
	set_dec(evt);
#else
	set_dec(evt - 1);	/* Classic decrementer interrupts at -1 */
#endif
}

static void decrementer_set_mode(enum	clock_event_mode mode,
				 struct clock_event_device *dev)
{
	struct decrementer_device *decrementer;
#if defined(CONFIG_40x) || defined(CONFIG_BOOKE)
	u32 tcr = mfspr(SPRN_TCR);

	if (mode == CLOCK_EVT_PERIODIC)
		tcr |=  TCR_ARE;
	else
		tcr &= ~TCR_ARE;

	mtspr(SPRN_TCR, tcr);
#endif
	decrementer = container_of(dev, struct decrementer_device, device);
	decrementer->mode = mode;

	if (mode == CLOCK_EVT_PERIODIC)
		decrementer_set_next_event(tb_ticks_per_jiffy, dev);
}

static struct clock_event_device decrementer_template = {
	.name		= "decrementer",
	.capabilities	= CLOCK_CAP_PROFILE | CLOCK_CAP_UPDATE |
			  CLOCK_CAP_NEXTEVT,
	.shift		= 32,
	.set_mode	= decrementer_set_mode,
	.set_next_event	= decrementer_set_next_event,
};

static DEFINE_PER_CPU(struct decrementer_device, decrementers);

static void register_decrementer(void)
{
	int cpu = smp_processor_id();
	struct decrementer_device *decrementer = &per_cpu(decrementers, cpu);

	decrementer->device = decrementer_template;

	/* We only want do_timer() to be called on a boot CPU. */
	if (cpu == boot_cpuid)
		decrementer->device.capabilities |= CLOCK_CAP_TICK;

	register_local_clockevent(&decrementer->device);
}

#endif /* CONFIG_GENERIC_CLOCKEVENTS */

#ifdef CONFIG_VIRT_CPU_ACCOUNTING
/*
 * Factors for converting from cputime_t (timebase ticks) to
 * jiffies, milliseconds, seconds, and clock_t (1/USER_HZ seconds).
 * These are all stored as 0.64 fixed-point binary fractions.
 */
u64 __cputime_jiffies_factor;
EXPORT_SYMBOL(__cputime_jiffies_factor);
u64 __cputime_msec_factor;
EXPORT_SYMBOL(__cputime_msec_factor);
u64 __cputime_sec_factor;
EXPORT_SYMBOL(__cputime_sec_factor);
u64 __cputime_clockt_factor;
EXPORT_SYMBOL(__cputime_clockt_factor);

static void calc_cputime_factors(void)
{
	struct div_result res;

	div128_by_32(HZ, 0, tb_ticks_per_sec, &res);
	__cputime_jiffies_factor = res.result_low;
	div128_by_32(1000, 0, tb_ticks_per_sec, &res);
	__cputime_msec_factor = res.result_low;
	div128_by_32(1, 0, tb_ticks_per_sec, &res);
	__cputime_sec_factor = res.result_low;
	div128_by_32(USER_HZ, 0, tb_ticks_per_sec, &res);
	__cputime_clockt_factor = res.result_low;
}

/*
 * Read the PURR on systems that have it, otherwise the timebase.
 */
static u64 read_purr(void)
{
	if (cpu_has_feature(CPU_FTR_PURR))
		return mfspr(SPRN_PURR);
	return mftb();
}

/*
 * Account time for a transition between system, hard irq
 * or soft irq state.
 */
void account_system_vtime(struct task_struct *tsk)
{
	u64 now, delta;
	unsigned long flags;

	local_irq_save(flags);
	now = read_purr();
	delta = now - get_paca()->startpurr;
	get_paca()->startpurr = now;
	if (!in_interrupt()) {
		delta += get_paca()->system_time;
		get_paca()->system_time = 0;
	}
	account_system_time(tsk, 0, delta);
	local_irq_restore(flags);
}

/*
 * Transfer the user and system times accumulated in the paca
 * by the exception entry and exit code to the generic process
 * user and system time records.
 * Must be called with interrupts disabled.
 */
void account_process_vtime(struct task_struct *tsk)
{
	cputime_t utime;

	utime = get_paca()->user_time;
	get_paca()->user_time = 0;
	account_user_time(tsk, utime);
}

static void account_process_time(struct pt_regs *regs)
{
	int cpu = smp_processor_id();

	account_process_vtime(current);
	run_local_timers();
	if (rcu_pending(cpu))
		rcu_check_callbacks(cpu, user_mode(regs));
	scheduler_tick();
 	run_posix_cpu_timers(current);
}

#ifdef CONFIG_PPC_SPLPAR
/*
 * Stuff for accounting stolen time.
 */
struct cpu_purr_data {
	int	initialized;			/* thread is running */
	u64	tb;			/* last TB value read */
	u64	purr;			/* last PURR value read */
	raw_spinlock_t lock;
};

static DEFINE_PER_CPU(struct cpu_purr_data, cpu_purr_data);

static void snapshot_tb_and_purr(void *data)
{
	struct cpu_purr_data *p = &__get_cpu_var(cpu_purr_data);

	p->tb = mftb();
	p->purr = mfspr(SPRN_PURR);
	wmb();
	p->initialized = 1;
}

/*
 * Called during boot when all cpus have come up.
 */
void snapshot_timebases(void)
{
	int cpu;

	if (!cpu_has_feature(CPU_FTR_PURR))
		return;
	for_each_possible_cpu(cpu)
		spin_lock_init(&per_cpu(cpu_purr_data, cpu).lock);
	on_each_cpu(snapshot_tb_and_purr, NULL, 0, 1);
}

void calculate_steal_time(void)
{
	u64 tb, purr;
	s64 stolen;
	struct cpu_purr_data *pme;

	if (!cpu_has_feature(CPU_FTR_PURR))
		return;
	pme = &per_cpu(cpu_purr_data, smp_processor_id());
	if (!pme->initialized)
		return;		/* this can happen in early boot */
	spin_lock(&pme->lock);
	tb = mftb();
	purr = mfspr(SPRN_PURR);
	stolen = (tb - pme->tb) - (purr - pme->purr);
	if (stolen > 0)
		account_steal_time(current, stolen);
	pme->tb = tb;
	pme->purr = purr;
	spin_unlock(&pme->lock);
}

/*
 * Must be called before the cpu is added to the online map when
 * a cpu is being brought up at runtime.
 */
static void snapshot_purr(void)
{
	struct cpu_purr_data *pme;
	unsigned long flags;

	if (!cpu_has_feature(CPU_FTR_PURR))
		return;
	pme = &per_cpu(cpu_purr_data, smp_processor_id());
	spin_lock_irqsave(&pme->lock, flags);
	pme->tb = mftb();
	pme->purr = mfspr(SPRN_PURR);
	pme->initialized = 1;
	spin_unlock_irqrestore(&pme->lock, flags);
}

#endif /* CONFIG_PPC_SPLPAR */

#else /* ! CONFIG_VIRT_CPU_ACCOUNTING */
#define calc_cputime_factors()
#define account_process_time(regs)	update_process_times(user_mode(regs))
#define calculate_steal_time()		do { } while (0)
#endif

#if !(defined(CONFIG_VIRT_CPU_ACCOUNTING) && defined(CONFIG_PPC_SPLPAR))
#define snapshot_purr()			do { } while (0)
#endif

/*
 * Called when a cpu comes up after the system has finished booting,
 * i.e. as a result of a hotplug cpu action.
 */
void snapshot_timebase(void)
{
	__get_cpu_var(last_jiffy) = get_tb();
	snapshot_purr();
#ifdef CONFIG_GENERIC_CLOCKEVENTS
	register_decrementer();
#endif
}

void __delay(unsigned long loops)
{
	unsigned long start;
	int diff;

	if (__USE_RTC()) {
		start = get_rtcl();
		do {
			/* the RTCL register wraps at 1000000000 */
			diff = get_rtcl() - start;
			if (diff < 0)
				diff += 1000000000;
		} while (diff < loops);
	} else {
		start = get_tbl();
		while (get_tbl() - start < loops)
			HMT_low();
		HMT_medium();
	}
}
EXPORT_SYMBOL(__delay);

void udelay(unsigned long usecs)
{
	__delay(tb_ticks_per_usec * usecs);
}
EXPORT_SYMBOL(udelay);

static __inline__ void timer_check_rtc(void)
{
        /*
         * update the rtc when needed, this should be performed on the
         * right fraction of a second. Half or full second ?
         * Full second works on mk48t59 clocks, others need testing.
         * Note that this update is basically only used through
         * the adjtimex system calls. Setting the HW clock in
         * any other way is a /dev/rtc and userland business.
         * This is still wrong by -0.5/+1.5 jiffies because of the
         * timer interrupt resolution and possible delay, but here we
         * hit a quantization limit which can only be solved by higher
         * resolution timers and decoupling time management from timer
         * interrupts. This is also wrong on the clocks
         * which require being written at the half second boundary.
         * We should have an rtc call that only sets the minutes and
         * seconds like on Intel to avoid problems with non UTC clocks.
         */
        if (ppc_md.set_rtc_time && ntp_synced() &&
	    xtime.tv_sec - last_rtc_update >= 659 &&
	    abs((xtime.tv_nsec/1000) - (1000000-1000000/HZ)) < 500000/HZ) {
		struct rtc_time tm;
		to_tm(xtime.tv_sec + 1 + timezone_offset, &tm);
		tm.tm_year -= 1900;
		tm.tm_mon -= 1;
		if (ppc_md.set_rtc_time(&tm) == 0)
			last_rtc_update = xtime.tv_sec + 1;
		else
			/* Try again one minute later */
			last_rtc_update += 60;
        }
}

#ifdef CONFIG_SMP
unsigned long notrace profile_pc(struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);

	if (in_lock_functions(pc))
		return regs->link;

	return pc;
}
EXPORT_SYMBOL(profile_pc);
#endif

#ifdef CONFIG_PPC_ISERIES

/*
 * This function recalibrates the timebase based on the 49-bit time-of-day
 * value in the Titan chip.  The Titan is much more accurate than the value
 * returned by the service processor for the timebase frequency.
 */

static void iSeries_tb_recal(void)
{
	struct div_result divres;
	unsigned long titan, tb;
	tb = get_tb();
	titan = HvCallXm_loadTod();
	if ( iSeries_recal_titan ) {
		unsigned long tb_ticks = tb - iSeries_recal_tb;
		unsigned long titan_usec = (titan - iSeries_recal_titan) >> 12;
		unsigned long new_tb_ticks_per_sec   = (tb_ticks * USEC_PER_SEC)/titan_usec;
		unsigned long new_tb_ticks_per_jiffy = (new_tb_ticks_per_sec+(HZ/2))/HZ;
		long tick_diff = new_tb_ticks_per_jiffy - tb_ticks_per_jiffy;
		char sign = '+';
		/* make sure tb_ticks_per_sec and tb_ticks_per_jiffy are consistent */
		new_tb_ticks_per_sec = new_tb_ticks_per_jiffy * HZ;

		if ( tick_diff < 0 ) {
			tick_diff = -tick_diff;
			sign = '-';
		}
		if ( tick_diff ) {
			if ( tick_diff < tb_ticks_per_jiffy/25 ) {
				printk( "Titan recalibrate: new tb_ticks_per_jiffy = %lu (%c%ld)\n",
						new_tb_ticks_per_jiffy, sign, tick_diff );
				tb_ticks_per_jiffy = new_tb_ticks_per_jiffy;
				tb_ticks_per_sec   = new_tb_ticks_per_sec;
				calc_cputime_factors();
				div128_by_32( XSEC_PER_SEC, 0, tb_ticks_per_sec, &divres );
				tb_to_xs = divres.result_low;
			}
			else {
				printk( "Titan recalibrate: FAILED (difference > 4 percent)\n"
					"                   new tb_ticks_per_jiffy = %lu\n"
					"                   old tb_ticks_per_jiffy = %lu\n",
					new_tb_ticks_per_jiffy, tb_ticks_per_jiffy );
			}
		}
	}
	iSeries_recal_titan = titan;
	iSeries_recal_tb = tb;
}
#endif

/*
 * For iSeries shared processors, we have to let the hypervisor
 * set the hardware decrementer.  We set a virtual decrementer
 * in the lppaca and call the hypervisor if the virtual
 * decrementer is less than the current value in the hardware
 * decrementer. (almost always the new decrementer value will
 * be greater than the current hardware decementer so the hypervisor
 * call will not be needed)
 */

/*
 * timer_interrupt - gets called when the decrementer overflows,
 * with interrupts disabled.
 */
void timer_interrupt(struct pt_regs * regs)
{
	int next_dec;
	int cpu = smp_processor_id();
	unsigned long ticks;
	u64 tb_next_jiffy;

	MARK(kernel_trap_entry, "%ld struct pt_regs %p", regs->trap, regs);

#ifdef CONFIG_PPC32
	if (atomic_read(&ppc_n_lost_interrupts) != 0)
		do_IRQ(regs);
#endif

	irq_enter();

#ifdef CONFIG_GENERIC_CLOCKEVENTS
#ifdef CONFIG_PPC_MULTIPLATFORM
	/*
	 * We must write a positive value to the decrementer to clear
	 * the interrupt on the IBM 970 CPU series.  In periodic mode,
	 * this happens when the decrementer gets reloaded later, but
	 * in one-shot mode, we have to do it here since an event handler
	 * may skip loading the new value...
	 */
	if (per_cpu(decrementers, cpu).mode != CLOCK_EVT_PERIODIC)
		set_dec(DECREMENTER_MAX);
#endif
	/*
	 * We can't disable the decrementer, so in the period between
	 * CPU being marked offline and calling stop-self, it's taking
	 * timer interrupts...
	 */
	if (!cpu_is_offline(cpu))
		per_cpu(decrementers, cpu).device.event_handler(regs);
#else
	profile_tick(CPU_PROFILING, regs);
#endif
	calculate_steal_time();

#ifdef CONFIG_PPC_ISERIES
	get_lppaca()->int_dword.fields.decr_int = 0;
#endif

	while ((ticks = tb_ticks_since(per_cpu(last_jiffy, cpu)))
	       >= tb_ticks_per_jiffy) {
		/* Update last_jiffy */
		per_cpu(last_jiffy, cpu) += tb_ticks_per_jiffy;
		/* Handle RTCL overflow on 601 */
		if (__USE_RTC() && per_cpu(last_jiffy, cpu) >= 1000000000)
			per_cpu(last_jiffy, cpu) -= 1000000000;

#ifndef CONFIG_GENERIC_CLOCKEVENTS
		/*
		 * We cannot disable the decrementer, so in the period
		 * between this cpu's being marked offline in cpu_online_map
		 * and calling stop-self, it is taking timer interrupts.
		 * Avoid calling into the scheduler rebalancing code if this
		 * is the case.
		 */
		if (!cpu_is_offline(cpu))
			account_process_time(regs);
#endif

		/*
		 * No need to check whether cpu is offline here; boot_cpuid
		 * should have been fixed up by now.
		 */
		if (cpu != boot_cpuid)
			continue;

		write_seqlock(&xtime_lock);
		tb_next_jiffy = tb_last_jiffy + tb_ticks_per_jiffy;
		if (per_cpu(last_jiffy, cpu) >= tb_next_jiffy) {
			tb_last_jiffy = tb_next_jiffy;
#ifndef CONFIG_GENERIC_CLOCKEVENTS
			do_timer(1);
#endif
			timer_check_rtc();
		}
		write_sequnlock(&xtime_lock);
	}

	next_dec = tb_ticks_per_jiffy - ticks;
#ifdef CONFIG_GENERIC_CLOCKEVENTS
#if !defined(CONFIG_40x) && !defined(CONFIG_BOOKE)
	if (per_cpu(decrementers, cpu).mode == CLOCK_EVT_PERIODIC)
		set_dec(next_dec - 1);
#endif
#else
	set_dec(next_dec);
#endif

#ifdef CONFIG_PPC_ISERIES
	if (hvlpevent_is_pending())
		process_hvlpevents(regs);
#endif

#ifdef CONFIG_PPC64
	/* collect purr register values often, for accurate calculations */
	if (firmware_has_feature(FW_FEATURE_SPLPAR)) {
		struct cpu_usage *cu = &__get_cpu_var(cpu_usage_array);
		cu->current_tb = mfspr(SPRN_PURR);
	}
#endif

	irq_exit();
 	MARK(kernel_trap_exit, MARK_NOARGS);
}

void wakeup_decrementer(void)
{
	unsigned long ticks;

	/*
	 * The timebase gets saved on sleep and restored on wakeup,
	 * so all we need to do is to reset the decrementer.
	 */
	ticks = tb_ticks_since(__get_cpu_var(last_jiffy));
	if (ticks < tb_ticks_per_jiffy)
		ticks = tb_ticks_per_jiffy - ticks;
	else
		ticks = 1;
	set_dec(ticks);
}

#ifdef CONFIG_SMP
void __init smp_space_timers(unsigned int max_cpus)
{
	int i;
	unsigned long half = tb_ticks_per_jiffy / 2;
	unsigned long offset = tb_ticks_per_jiffy / max_cpus;
	u64 previous_tb = per_cpu(last_jiffy, boot_cpuid);

	/* make sure tb > per_cpu(last_jiffy, cpu) for all cpus always */
	previous_tb -= tb_ticks_per_jiffy;
	/*
	 * The stolen time calculation for POWER5 shared-processor LPAR
	 * systems works better if the two threads' timebase interrupts
	 * are staggered by half a jiffy with respect to each other.
	 */
	for_each_possible_cpu(i) {
		if (i == boot_cpuid)
			continue;
		if (i == (boot_cpuid ^ 1))
			per_cpu(last_jiffy, i) =
				per_cpu(last_jiffy, boot_cpuid) - half;
		else if (i & 1)
			per_cpu(last_jiffy, i) =
				per_cpu(last_jiffy, i ^ 1) + half;
		else {
			previous_tb += offset;
			per_cpu(last_jiffy, i) = previous_tb;
		}
	}
}
#endif

/*
 * Scheduler clock - returns current time in nanosec units.
 *
 * Note: mulhdu(a, b) (multiply high double unsigned) returns
 * the high 64 bits of a * b, i.e. (a * b) >> 64, where a and b
 * are 64-bit unsigned numbers.
 */
unsigned long long sched_clock(void)
{
	if (__USE_RTC())
		return get_rtc();
	return mulhdu(get_tb(), tb_to_ns_scale) << tb_to_ns_shift;
}

static int __init get_freq(char *name, int cells, unsigned long *val)
{
	struct device_node *cpu;
	const unsigned int *fp;
	int found = 0;

	/* The cpu node should have timebase and clock frequency properties */
	cpu = of_find_node_by_type(NULL, "cpu");

	if (cpu) {
		fp = of_get_property(cpu, name, NULL);
		if (fp) {
			found = 1;
			*val = of_read_ulong(fp, cells);
		}

		of_node_put(cpu);
	}

	return found;
}

void __init generic_calibrate_decr(void)
{
	ppc_tb_freq = DEFAULT_TB_FREQ;		/* hardcoded default */

	if (!get_freq("ibm,extended-timebase-frequency", 2, &ppc_tb_freq) &&
	    !get_freq("timebase-frequency", 1, &ppc_tb_freq)) {

		printk(KERN_ERR "WARNING: Estimating decrementer frequency "
				"(not found)\n");
	}

	ppc_proc_freq = DEFAULT_PROC_FREQ;	/* hardcoded default */

	if (!get_freq("ibm,extended-clock-frequency", 2, &ppc_proc_freq) &&
	    !get_freq("clock-frequency", 1, &ppc_proc_freq)) {

		printk(KERN_ERR "WARNING: Estimating processor frequency "
				"(not found)\n");
	}

#if defined(CONFIG_BOOKE) || defined(CONFIG_40x)
	/* Set the time base to zero */
	mtspr(SPRN_TBWL, 0);
	mtspr(SPRN_TBWU, 0);

	/* Clear any pending timer interrupts */
	mtspr(SPRN_TSR, TSR_ENW | TSR_WIS | TSR_DIS | TSR_FIS);

	/* Enable decrementer interrupt */
	mtspr(SPRN_TCR, TCR_DIE);
#endif
}

unsigned long read_persistent_clock(void)
{
	unsigned long time = 0;
	static int first = 1;

	if (first && ppc_md.time_init) {
		timezone_offset = ppc_md.time_init();

		/* If platform provided a timezone (pmac), we correct the time */
		if (timezone_offset) {
			sys_tz.tz_minuteswest = -timezone_offset / 60;
			sys_tz.tz_dsttime = 0;
		}
	}

	if (ppc_md.get_boot_time)
		time = ppc_md.get_boot_time();
	else if (ppc_md.get_rtc_time) {
		struct rtc_time tm;

		ppc_md.get_rtc_time(&tm);
		time = mktime(tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday,
			      tm.tm_hour, tm.tm_min, tm.tm_sec);
	}
	time -= timezone_offset;

	if (first) {
		last_rtc_update = time;
		first = 0;
	}
	return time;
}

/* This function is only called on the boot processor */
void __init time_init(void)
{
	struct div_result res;
	u64 scale, x;
	unsigned shift;

	if (__USE_RTC()) {
		/* 601 processor: dec counts down by 128 every 128ns */
		ppc_tb_freq = 1000000000;
		tb_last_jiffy = get_rtcl();
	} else {
		/* Normal PowerPC with timebase register */
		ppc_md.calibrate_decr();
		printk(KERN_DEBUG "time_init: decrementer frequency = %lu.%.6lu MHz\n",
		       ppc_tb_freq / 1000000, ppc_tb_freq % 1000000);
		printk(KERN_DEBUG "time_init: processor frequency   = %lu.%.6lu MHz\n",
		       ppc_proc_freq / 1000000, ppc_proc_freq % 1000000);
		tb_last_jiffy = get_tb();
	}

	tb_ticks_per_jiffy = ppc_tb_freq / HZ;
	tb_ticks_per_sec = ppc_tb_freq;
	tb_ticks_per_usec = ppc_tb_freq / 1000000;
	cpu_khz  = ppc_tb_freq / 1000;
	tb_to_us = mulhwu_scale_factor(ppc_tb_freq, 1000000);
	calc_cputime_factors();

	/*
	 * Calculate the length of each tick in ns.  It will not be
	 * exactly 1e9/HZ unless ppc_tb_freq is divisible by HZ.
	 * We compute 1e9 * tb_ticks_per_jiffy / ppc_tb_freq,
	 * rounded up.
	 */
	x = (u64) NSEC_PER_SEC * tb_ticks_per_jiffy + ppc_tb_freq - 1;
	do_div(x, ppc_tb_freq);
	tick_nsec = x;
	last_tick_len = x << TICKLEN_SCALE;

	/*
	 * Compute ticklen_to_xs, which is a factor which gets multiplied
	 * by (last_tick_len << TICKLEN_SHIFT) to get a tb_to_xs value.
	 * It is computed as:
	 * ticklen_to_xs = 2^N / (tb_ticks_per_jiffy * 1e9)
	 * where N = 64 + 20 - TICKLEN_SCALE - TICKLEN_SHIFT
	 * which turns out to be N = 51 - SHIFT_HZ.
	 * This gives the result as a 0.64 fixed-point fraction.
	 * That value is reduced by an offset amounting to 1 xsec per
	 * 2^31 timebase ticks to avoid problems with time going backwards
	 * by 1 xsec when we do timer_recalc_offset due to losing the
	 * fractional xsec.  That offset is equal to ppc_tb_freq/2^51
	 * since there are 2^20 xsec in a second.
	 */
	div128_by_32((1ULL << 51) - ppc_tb_freq, 0,
		     tb_ticks_per_jiffy << SHIFT_HZ, &res);
	div128_by_32(res.result_high, res.result_low, NSEC_PER_SEC, &res);
	ticklen_to_xs = res.result_low;

	/* Compute tb_to_xs from tick_nsec */
	tb_to_xs = mulhdu(last_tick_len << TICKLEN_SHIFT, ticklen_to_xs);

	/*
	 * Compute scale factor for sched_clock.
	 * The calibrate_decr() function has set tb_ticks_per_sec,
	 * which is the timebase frequency.
	 * We compute 1e9 * 2^64 / tb_ticks_per_sec and interpret
	 * the 128-bit result as a 64.64 fixed-point number.
	 * We then shift that number right until it is less than 1.0,
	 * giving us the scale factor and shift count to use in
	 * sched_clock().
	 */
	div128_by_32(1000000000, 0, tb_ticks_per_sec, &res);
	scale = res.result_low;
	for (shift = 0; res.result_high != 0; ++shift) {
		scale = (scale >> 1) | (res.result_high << 63);
		res.result_high >>= 1;
	}
	tb_to_ns_scale = scale;
	tb_to_ns_shift = shift;

#ifdef CONFIG_GENERIC_CLOCKEVENTS
	decrementer_template.mult = div_sc(ppc_tb_freq, NSEC_PER_SEC,
					   decrementer_template.shift);
	decrementer_template.max_delta_ns =
		clockevent_delta2ns(DECREMENTER_MAX, &decrementer_template);
	decrementer_template.min_delta_ns =
		clockevent_delta2ns(0xf, &decrementer_template);

	register_decrementer();
#else
	/* Not exact, but the timer interrupt takes care of this */
	set_dec(tb_ticks_per_jiffy);
#endif
}


#define FEBRUARY	2
#define	STARTOFTIME	1970
#define SECDAY		86400L
#define SECYR		(SECDAY * 365)
#define	leapyear(year)		((year) % 4 == 0 && \
				 ((year) % 100 != 0 || (year) % 400 == 0))
#define	days_in_year(a) 	(leapyear(a) ? 366 : 365)
#define	days_in_month(a) 	(month_days[(a) - 1])

static int month_days[12] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/*
 * This only works for the Gregorian calendar - i.e. after 1752 (in the UK)
 */
void GregorianDay(struct rtc_time * tm)
{
	int leapsToDate;
	int lastYear;
	int day;
	int MonthOffset[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

	lastYear = tm->tm_year - 1;

	/*
	 * Number of leap corrections to apply up to end of last year
	 */
	leapsToDate = lastYear / 4 - lastYear / 100 + lastYear / 400;

	/*
	 * This year is a leap year if it is divisible by 4 except when it is
	 * divisible by 100 unless it is divisible by 400
	 *
	 * e.g. 1904 was a leap year, 1900 was not, 1996 is, and 2000 was
	 */
	day = tm->tm_mon > 2 && leapyear(tm->tm_year);

	day += lastYear*365 + leapsToDate + MonthOffset[tm->tm_mon-1] +
		   tm->tm_mday;

	tm->tm_wday = day % 7;
}

void to_tm(int tim, struct rtc_time * tm)
{
	register int    i;
	register long   hms, day;

	day = tim / SECDAY;
	hms = tim % SECDAY;

	/* Hours, minutes, seconds are easy */
	tm->tm_hour = hms / 3600;
	tm->tm_min = (hms % 3600) / 60;
	tm->tm_sec = (hms % 3600) % 60;

	/* Number of years in days */
	for (i = STARTOFTIME; day >= days_in_year(i); i++)
		day -= days_in_year(i);
	tm->tm_year = i;

	/* Number of months in days left */
	if (leapyear(tm->tm_year))
		days_in_month(FEBRUARY) = 29;
	for (i = 1; day >= days_in_month(i); i++)
		day -= days_in_month(i);
	days_in_month(FEBRUARY) = 28;
	tm->tm_mon = i;

	/* Days are what is left over (+1) from all that. */
	tm->tm_mday = day + 1;

	/*
	 * Determine the day of week
	 */
	GregorianDay(tm);
}

/* Auxiliary function to compute scaling factors */
/* Actually the choice of a timebase running at 1/4 the of the bus
 * frequency giving resolution of a few tens of nanoseconds is quite nice.
 * It makes this computation very precise (27-28 bits typically) which
 * is optimistic considering the stability of most processor clock
 * oscillators and the precision with which the timebase frequency
 * is measured but does not harm.
 */
unsigned mulhwu_scale_factor(unsigned inscale, unsigned outscale)
{
        unsigned mlt=0, tmp, err;
        /* No concern for performance, it's done once: use a stupid
         * but safe and compact method to find the multiplier.
         */

        for (tmp = 1U<<31; tmp != 0; tmp >>= 1) {
                if (mulhwu(inscale, mlt|tmp) < outscale)
			mlt |= tmp;
        }

        /* We might still be off by 1 for the best approximation.
         * A side effect of this is that if outscale is too large
         * the returned value will be zero.
         * Many corner cases have been checked and seem to work,
         * some might have been forgotten in the test however.
         */

        err = inscale * (mlt+1);
        if (err <= inscale/2)
		mlt++;
        return mlt;
}

/*
 * Divide a 128-bit dividend by a 32-bit divisor, leaving a 128 bit
 * result.
 */
void div128_by_32(u64 dividend_high, u64 dividend_low,
		  unsigned divisor, struct div_result *dr)
{
	unsigned long a, b, c, d;
	unsigned long w, x, y, z;
	u64 ra, rb, rc;

	a = dividend_high >> 32;
	b = dividend_high & 0xffffffff;
	c = dividend_low >> 32;
	d = dividend_low & 0xffffffff;

	w = a / divisor;
	ra = ((u64)(a - (w * divisor)) << 32) + b;

	rb = ((u64) do_div(ra, divisor) << 32) + c;
	x = ra;

	rc = ((u64) do_div(rb, divisor) << 32) + d;
	y = rb;

	do_div(rc, divisor);
	z = rc;

	dr->result_high = ((u64)w << 32) + x;
	dr->result_low  = ((u64)y << 32) + z;

}

/* PowerPC clocksource code */

#include <linux/clocksource.h>

static cycle_t timebase_read(void)
{
	return (cycle_t)get_tb();
}

struct clocksource clocksource_timebase = {
	.name		= "timebase",
	.rating		= 200,
	.read		= timebase_read,
	.mask		= (cycle_t)-1,
	.mult		= 0,
	.shift		= 22,
	.is_continuous	= 1,
};


/* XXX - this should be calculated or properly externed! */
static int __init init_timebase_clocksource(void)
{
	if (__USE_RTC())
		return -ENODEV;

	clocksource_timebase.mult = clocksource_hz2mult(tb_ticks_per_sec,
					clocksource_timebase.shift);
	return clocksource_register(&clocksource_timebase);
}

module_init(init_timebase_clocksource);
