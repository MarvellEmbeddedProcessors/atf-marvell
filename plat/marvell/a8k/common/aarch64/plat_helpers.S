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
#include <platform_def.h>

	.globl	plat_secondary_cold_boot_setup
	.globl	plat_get_my_entrypoint
	.globl	plat_is_my_cpu_primary
	.globl  plat_reset_handler

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
	mov_imm x0, PLAT_MARVELL_MAILBOX_BASE
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
