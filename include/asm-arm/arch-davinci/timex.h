/*
 * DaVinci timer defines
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_TIMEX_H
#define __ASM_ARCH_TIMEX_H

#include <asm/arch/cpu.h>

/* The source frequency for the timers is the 27MHz clock */
#define CLOCK_TICK_RATE			24000000
#define DM644X_CLOCK_TICK_RATE		27000000
#define DM646X_CLOCK_TICK_RATE		148500000
#define DM355_CLOCK_TICK_RATE		24000000

#define DAVINCI_CLOCK_TICK_RATE ((cpu_is_davinci_dm6467()) ?		\
		DM646X_CLOCK_TICK_RATE : ((cpu_is_davinci_dm644x()) ?	\
		DM644X_CLOCK_TICK_RATE : DM355_CLOCK_TICK_RATE))

#endif /* __ASM_ARCH_TIMEX_H__ */
