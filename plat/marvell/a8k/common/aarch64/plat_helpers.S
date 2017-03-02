/*
 * ***************************************************************************
 * Copyright (C) 2016 Marvell International Ltd.
 * ***************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Marvell nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************
 */

#include <asm_macros.S>
#include <arm_def.h>
#include <platform_def.h>
#include <marvell_pm.h>

#define CCU_WIN_CR(x)		(MVEBU_CCU_BASE + (0x10 * x))
#define	CCU_SRAM_WIN_CR		CCU_WIN_CR(1)

#define	LLC_CR			(MVEBU_LLC_BASE + 0x100)
#define LLC_TC0_LOCK		(MVEBU_LLC_BASE + 0x920)
#define	LLC_MNT_IW		(MVEBU_LLC_BASE + 0x77c)
#define LLC_WAYS		8
#define	LLC_WAY_MASK		((1 << LLC_WAYS) - 1)

	.globl	plat_secondary_cold_boot_setup
	.globl	plat_get_my_entrypoint
	.globl	plat_is_my_cpu_primary
	.globl  plat_reset_handler
	.globl disable_mmu_dcache
	.globl invalidate_dcache_all
	.globl invalidate_tlb_all
	.globl platform_unmap_sram
	.globl disable_sram
	.globl disable_icache
	.globl invalidate_icache_all
	.globl exit_bootrom

	/* -----------------------------------------------------
	 * void plat_secondary_cold_boot_setup (void);
	 *
	 * This function performs any platform specific actions
	 * needed for a secondary cpu after a cold reset. Right
	 * now this is a stub function.
	 * -----------------------------------------------------
	 */
func plat_secondary_cold_boot_setup
	mov	x0, #0
	ret
endfunc plat_secondary_cold_boot_setup

	/* ---------------------------------------------------------------------
	 * unsigned long plat_get_my_entrypoint (void);
	 *
	 * Main job of this routine is to distinguish
	 * between a cold and warm boot
	 * For a cold boot, return 0.
	 * For a warm boot, read the mailbox and return the address it contains.
	 *
	 * ---------------------------------------------------------------------
	 */
func plat_get_my_entrypoint
	mov_imm x0, PLAT_MARVELL_MAILBOX_BASE	/* Read first word and compare it with magic num */
	ldr     x1, [x0]
	mov_imm x2, MVEBU_MAILBOX_MAGIC_NUM
	cmp     x1, x2
	beq     warm_boot			/* If compare failed, return 0, i.e. cold boot */
	mov     x0, #0
	ret
warm_boot:
	mov_imm x1, MBOX_IDX_SEC_ADDR		/* Get the jump address */
	subs	x1, x1, #1
	mov	x2, #(MBOX_IDX_SEC_ADDR * 8)
	lsl	x3, x2, x1
	add     x0, x0, x3
	ldr     x0, [x0]
	ret
endfunc plat_get_my_entrypoint

	/* -----------------------------------------------------
	 * unsigned int plat_is_my_cpu_primary (void);
	 *
	 * Find out whether the current cpu is the primary
	 * cpu.
	 * -----------------------------------------------------
	 */
func plat_is_my_cpu_primary
	mrs	x0, mpidr_el1
	and	x0, x0, #(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	cmp	x0, #MVEBU_PRIMARY_CPU
	cset	w0, eq
	ret
endfunc plat_is_my_cpu_primary

        /* -----------------------------------------------------
	 * void plat_reset_handler (void);
         *
	 * Platform specific configuration right after cpu is
	 * is our of reset.
	 *
         * The plat_reset_handler can clobber x0 - x18, x30.
         * -----------------------------------------------------
         */
func plat_reset_handler
	/*
	 * Note: the configurations below  should be done before MMU,
	 *	  I Cache and L2are enabled.
	 *	  The reset handler is executed right after reset
	 * 	  and before Caches are enabled.
	 */

	/* Enable L1/L2 ECC and Parity */
	mrs x5, s3_1_c11_c0_2  /* L2 Ctrl */
	orr x5, x5, #(1 << 21) /* Enable L1/L2 cache ECC & Parity */
	msr s3_1_c11_c0_2, x5  /* L2 Ctrl */

#if !LLC_DISABLE
	/*
	 * Enable L2 UniqueClean evictions
	 *  Note: this configuration assumes that LLC is configured
	 *	  in exclusive mode.
	 *	  Later on in the code this assumption will be validated
	 */
	mrs x5, s3_1_c15_c0_0  /* L2 Ctrl */
	orr x5, x5, #(1 << 14) /* Enable UniqueClean evictions with data */
	msr s3_1_c15_c0_0, x5  /* L2 Ctrl */
#endif

	/* Instruction Barrier to allow msr command completion */
	isb

        ret
endfunc plat_reset_handler

/*
 ***************************************************************************
 * PM helpers
 ***************************************************************************
 */

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func disable_mmu_dcache
	/* Disable icache, dcache, and MMU */
	mrs	x0, sctlr_el3
	bic	x0, x0, 0x1		/* M bit - MMU */
	bic	x0, x0, 0x4		/* C bit - Dcache L1 & L2 */
	msr	sctlr_el3, x0
	isb
	b	mmu_off
mmu_off:
	ret
endfunc disable_mmu_dcache

/*
 * void dcache_maint_level(level)
 *
 * clean and invalidate one level cache.
 *
 * x0: cache level
 * x1: 0 flush & invalidate, 1 invalidate only
 * x2~x9: clobbered
 */
func dcache_maint_level
	lsl	x12, x0, #1
	msr	csselr_el1, x12		/* select cache level */
	isb				/* sync change of cssidr_el1 */
	mrs	x6, ccsidr_el1		/* read the new cssidr_el1 */
	and	x2, x6, #7		/* x2 <- log2(cache line size)-4 */
	add	x2, x2, #4		/* x2 <- log2(cache line size) */
	mov	x3, #0x3ff
	and	x3, x3, x6, lsr #3	/* x3 <- max number of #ways */
	clz	w5, w3			/* bit position of #ways */
	mov	x4, #0x7fff
	and	x4, x4, x6, lsr #13	/* x4 <- max number of #sets */
	/* x12 <- cache level << 1 */
	/* x2 <- line length offset */
	/* x3 <- number of cache ways - 1 */
	/* x4 <- number of cache sets - 1 */
	/* x5 <- bit position of #ways */

loop_set:
	mov	x6, x3			/* x6 <- working copy of #ways */
loop_way:
	lsl	x7, x6, x5
	orr	x9, x12, x7		/* map way and level to cisw value */
	lsl	x7, x4, x2
	orr	x9, x9, x7		/* map set number to cisw value */
	tbz	w1, #0, 1f
	dc	isw, x9
	b	2f
1:	dc	cisw, x9		/* clean & invalidate by set/way */
2:	subs	x6, x6, #1		/* decrement the way */
	b.ge	loop_way
	subs	x4, x4, #1		/* decrement the set */
	b.ge	loop_set

	ret
endfunc dcache_maint_level

/*
 * void __asm_flush_dcache_all(int invalidate_only)
 *
 * x0: 0 flush & invalidate, 1 invalidate only
 *
 * clean and invalidate all data cache by SET/WAY.
 */
func dcache_maint_all
	mov	x1, x0
	dsb	sy
	mrs	x10, clidr_el1		/* read clidr_el1 */
	lsr	x11, x10, #24
	and	x11, x11, #0x7		/* x11 <- loc */
	cbz	x11, finished		/* if loc is 0, exit */
	mov	x15, x30
	mov	x0, #0			/* start flush at cache level 0 */
	/* x0  <- cache level */
	/* x10 <- clidr_el1 */
	/* x11 <- loc */
	/* x15 <- return address */

loop_level:
	lsl	x12, x0, #1
	add	x12, x12, x0		/* x0 <- tripled cache level */
	lsr	x12, x10, x12
	and	x12, x12, #7		/* x12 <- cache type */
	cmp	x12, #2
	b.lt	skip			/* skip if no cache or icache */
	bl	dcache_maint_level	/* x1 = 0 flush, 1 invalidate */
skip:
	add	x0, x0, #1		/* increment cache level */
	cmp	x11, x0
	b.gt	loop_level

	mov	x0, #0
	msr	csselr_el1, x0		/* resotre csselr_el1 */
	dsb	sy
	isb
	mov	x30, x15

finished:
	ret
endfunc dcache_maint_all

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func invalidate_dcache_all
	mov	x16, x30
	mov	x0, #0xffff
	bl	dcache_maint_all
	mov	x30, x16
	ret
endfunc invalidate_dcache_all

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func invalidate_tlb_all
	tlbi	alle3
	dsb	sy
	isb
	ret
endfunc invalidate_tlb_all

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func platform_unmap_sram
	/* Just need to clear the enable bit */
	ldr     x0, =CCU_SRAM_WIN_CR
	str     wzr, [x0]
	ret
endfunc platform_unmap_sram

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func disable_sram
	/* Disable the line lockings. They must be disabled expictly
	 * or the OS will have problems using the cache */
	ldr     x1, =LLC_TC0_LOCK
	str     wzr, [x1]

	/* Invalidate all ways */
	ldr	w1, =LLC_WAY_MASK
	ldr	x0, =LLC_MNT_IW
	str     w1, [x0]

	/* Finally disable LLC */
	ldr     x0, =LLC_CR
	str     wzr, [x0]

	ret
endfunc disable_sram

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func disable_icache
	mrs 	x0, sctlr_el3
	bic	x0, x0, 0x1000	/* I bit - Icache L1 & L2 */
	msr	sctlr_el3, x0
	isb
	ret
endfunc disable_icache

	/* -----------------------------------------------------
	 * -----------------------------------------------------
	 */
func invalidate_icache_all
	ic	ialluis
	isb	sy
	ret
endfunc invalidate_icache_all

func exit_bootrom
	/* Save the system restore address */
	mov	x28, x0

	/* close the caches and MMU */
	bl	disable_mmu_dcache

	/*
	 * There is nothing important in the caches now,
	 * so invalidate them instead of cleaning.
	 */
	bl	invalidate_dcache_all
	bl	invalidate_tlb_all

	/*
	 * Clean the memory mapping of SRAM
	 * the DDR mapping will remain to enable boot image to execute
	 */
	bl	platform_unmap_sram

	/* Disable the SRAM */
	bl	disable_sram

	/* Disable and invalidate icache */
	bl	disable_icache
	bl	invalidate_icache_all

	mov	x0, x28
	br	x0
endfunc exit_bootrom
