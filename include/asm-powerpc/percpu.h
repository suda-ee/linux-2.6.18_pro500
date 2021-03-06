#ifndef _ASM_POWERPC_PERCPU_H_
#define _ASM_POWERPC_PERCPU_H_
#ifdef __powerpc64__
#include <linux/compiler.h>

/*
 * Same as asm-generic/percpu.h, except that we store the per cpu offset
 * in the paca. Based on the x86-64 implementation.
 */

#ifdef CONFIG_SMP

#include <asm/paca.h>

#define __per_cpu_offset(cpu) (paca[cpu].data_offset)
#define __my_cpu_offset() get_paca()->data_offset
#define per_cpu_offset(x) (__per_cpu_offset(x))

/* Separate out the type, so (int[3], foo) works. */
#define DEFINE_PER_CPU(type, name) \
    __attribute__((__section__(".data.percpu"))) __typeof__(type) per_cpu__##name
#define DEFINE_PER_CPU_LOCKED(type, name) \
    __attribute__((__section__(".data.percpu"))) __DEFINE_SPINLOCK(per_cpu_lock__##name##_locked); \
    __attribute__((__section__(".data.percpu"))) __typeof__(type) per_cpu__##name##_locked

/* var is in discarded region: offset to particular copy we want */
#define per_cpu(var, cpu) (*RELOC_HIDE(&per_cpu__##var, __per_cpu_offset(cpu)))
#define __get_cpu_var(var) (*RELOC_HIDE(&per_cpu__##var, __my_cpu_offset()))
#define __raw_get_cpu_var(var) (*RELOC_HIDE(&per_cpu__##var, __my_cpu_offset()))

#define per_cpu_lock(var, cpu) \
    (*RELOC_HIDE(&per_cpu_lock__##var##_locked, __per_cpu_offset(cpu)))
#define per_cpu_var_locked(var, cpu) \
    (*RELOC_HIDE(&per_cpu__##var##_locked, __per_cpu_offset(cpu)))
#define __get_cpu_lock(var, cpu) \
    per_cpu_lock(var, cpu)
#define __get_cpu_var_locked(var, cpu) \
    per_cpu_var_locked(var, cpu)

/* A macro to avoid #include hell... */
#define percpu_modcopy(pcpudst, src, size)			\
do {								\
	unsigned int __i;					\
	for_each_possible_cpu(__i)				\
		memcpy((pcpudst)+__per_cpu_offset(__i),		\
		       (src), (size));				\
} while (0)

extern void setup_per_cpu_areas(void);

#else /* ! SMP */

#define DEFINE_PER_CPU(type, name) \
    __typeof__(type) per_cpu__##name
#define DEFINE_PER_CPU_LOCKED(type, name) \
    __DEFINE_SPINLOCK(per_cpu_lock__##name##_locked); \
    __typeof__(type) per_cpu__##name##_locked

#define per_cpu(var, cpu)			(*((void)(cpu), &per_cpu__##var))
#define per_cpu_var_locked(var, cpu)		(*((void)(cpu), &per_cpu__##var##_locked))

#define __get_cpu_var(var)			per_cpu__##var
#define __raw_get_cpu_var(var)			per_cpu__##var

#endif	/* SMP */

#define DECLARE_PER_CPU(type, name) extern __typeof__(type) per_cpu__##name
#define DECLARE_PER_CPU_LOCKED(type, name) \
    extern spinlock_t per_cpu_lock__##name##_locked; \
    extern __typeof__(type) per_cpu__##name##_locked

#define EXPORT_PER_CPU_SYMBOL(var) EXPORT_SYMBOL(per_cpu__##var)
#define EXPORT_PER_CPU_SYMBOL_GPL(var) EXPORT_SYMBOL_GPL(per_cpu__##var)
#define EXPORT_PER_CPU_LOCKED_SYMBOL(var) EXPORT_SYMBOL(per_cpu_lock__##var##_locked); EXPORT_SYMBOL(per_cpu__##var##_locked)
#define EXPORT_PER_CPU_LOCKED_SYMBOL_GPL(var) EXPORT_SYMBOL_GPL(per_cpu_lock__##var##_locked); EXPORT_SYMBOL_GPL(per_cpu__##var##_locked)

#else
#include <asm-generic/percpu.h>
#endif

#endif /* _ASM_POWERPC_PERCPU_H_ */
