/*
 * Copyright (C) Paul Mackerras 1997.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * NOTE: this code runs in 32 bit mode and is packaged as ELF32.
 */

#include "ppc_asm.h"

	.text
	/* a procedure descriptor used when booting this as a COFF file */
_zimage_start_opd:
	.long	_zimage_start, 0, 0, 0

	.weak	_zimage_start
	.globl	_zimage_start
_zimage_start:
	.globl	_zimage_start_lib
_zimage_start_lib:
	/* Work out the offset between the address we were linked at
	   and the address where we're running. */
	bl	1f
1:	mflr	r0
	lis	r9,1b@ha
	addi	r9,r9,1b@l
	subf.	r0,r9,r0
	beq	3f		/* if running at same address as linked */

	/* The .got2 section contains a list of addresses, so add
	   the address offset onto each entry. */
	lis	r9,__got2_start@ha
	addi	r9,r9,__got2_start@l
	lis	r8,__got2_end@ha
	addi	r8,r8,__got2_end@l
	subf.	r8,r9,r8
	beq	3f
	srwi.	r8,r8,2
	mtctr	r8
	add	r9,r0,r9
2:	lwz	r8,0(r9)
	add	r8,r8,r0
	stw	r8,0(r9)
	addi	r9,r9,4
	bdnz	2b

	/* Do a cache flush for our text, in case the loader didn't */
3:	lis	r9,_start@ha
	addi	r9,r9,_start@l
	add	r9,r0,r9
	lis	r8,_etext@ha
	addi	r8,r8,_etext@l
	add	r8,r0,r8
4:	dcbf	r0,r9
	icbi	r0,r9
	addi	r9,r9,0x20
	cmplw	cr0,r9,r8
	blt	4b
	sync
	isync

	/* Clear the BSS */
	lis	r9,__bss_start@ha
	addi	r9,r9,__bss_start@l
	add	r9,r0,r9
	lis	r8,_end@ha
	addi	r8,r8,_end@l
	add	r8,r0,r8
	li	r10,0
5:	stw	r10,0(r9)
	addi	r9,r9,4
	cmplw	cr0,r9,r8
	blt	5b

	/* Possibly set up a custom stack */
.weak	_platform_stack_top
	lis	r8,_platform_stack_top@ha
	addi	r8,r8,_platform_stack_top@l
	cmpwi	r8,0
	beq	6f
	add	r8,r0,r8
	lwz	r1,0(r8)
	add	r1,r0,r1
	li	r0,0
	stwu	r0,-16(r1)	/* establish a stack frame */
6:

	/* Call platform_init() */
	bl	platform_init

	/* Call start */
	b	start
