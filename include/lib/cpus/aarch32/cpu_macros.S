/*
 * Copyright (c) 2016-2017, ARM Limited and Contributors. All rights reserved.
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
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __CPU_MACROS_S__
#define __CPU_MACROS_S__

#include <arch.h>
#include <errata_report.h>

#define CPU_IMPL_PN_MASK	(MIDR_IMPL_MASK << MIDR_IMPL_SHIFT) | \
				(MIDR_PN_MASK << MIDR_PN_SHIFT)

/* The number of CPU operations allowed */
#define CPU_MAX_PWR_DWN_OPS		2

/* Special constant to specify that CPU has no reset function */
#define CPU_NO_RESET_FUNC		0

/* Word size for 32-bit CPUs */
#define CPU_WORD_SIZE			4

/*
 * Whether errata status needs reporting. Errata status is printed in debug
 * builds for both BL1 and BL32 images.
 */
#if (defined(IMAGE_BL1) || defined(IMAGE_BL32)) && DEBUG
# define REPORT_ERRATA	1
#else
# define REPORT_ERRATA	0
#endif

	/*
	 * Define the offsets to the fields in cpu_ops structure.
	 */
	.struct 0
CPU_MIDR: /* cpu_ops midr */
	.space  4
/* Reset fn is needed during reset */
#if IMAGE_BL1 || IMAGE_BL32
CPU_RESET_FUNC: /* cpu_ops reset_func */
	.space  4
#endif
#if IMAGE_BL32 /* The power down core and cluster is needed only in BL32 */
CPU_PWR_DWN_CORE: /* cpu_ops core_pwr_dwn */
	.space  4
CPU_PWR_DWN_CLUSTER: /* cpu_ops cluster_pwr_dwn */
	.space  4
#endif

/*
 * Fields required to print errata status. Only in BL32 that the printing
 * require mutual exclusion and printed flag.
 */
#if REPORT_ERRATA
CPU_ERRATA_FUNC: /* CPU errata status printing function */
	.space  4
#ifdef IMAGE_BL32
CPU_ERRATA_LOCK:
	.space	4
CPU_ERRATA_PRINTED:
	.space	4
#endif
#endif

CPU_OPS_SIZE = .

	/*
	 * Convenience macro to declare cpu_ops structure.
	 * Make sure the structure fields are as per the offsets
	 * defined above.
	 */
	.macro declare_cpu_ops _name:req, _midr:req, _noresetfunc = 0
	.section cpu_ops, "a"
	.align 2
	.type cpu_ops_\_name, %object
	.word \_midr
#if IMAGE_BL1 || IMAGE_BL32
	.if \_noresetfunc
	.word 0
	.else
	.word \_name\()_reset_func
	.endif
#endif
#if IMAGE_BL32
	.word \_name\()_core_pwr_dwn
	.word \_name\()_cluster_pwr_dwn
#endif

#if REPORT_ERRATA
	.ifndef \_name\()_cpu_str
	  /*
	   * Place errata reported flag, and the spinlock to arbitrate access to
	   * it in the data section.
	   */
	  .pushsection .data
	  define_asm_spinlock \_name\()_errata_lock
	  \_name\()_errata_reported:
	  .word	0
	  .popsection

	  /* Place CPU string in rodata */
	  .pushsection .rodata
	  \_name\()_cpu_str:
	  .asciz "\_name"
	  .popsection
	.endif

	/*
	 * Weakly-bound, optional errata status printing function for CPUs of
	 * this class.
	 */
	.weak \_name\()_errata_report
	.word \_name\()_errata_report

#ifdef IMAGE_BL32
	/* Pointers to errata lock and reported flag */
	.word \_name\()_errata_lock
	.word \_name\()_errata_reported
#endif
#endif
	.endm

#if REPORT_ERRATA
	/*
	 * Print status of a CPU errata
	 *
	 * _chosen:
	 *	Identifier indicating whether or not a CPU errata has been
	 *	compiled in.
	 * _cpu:
	 *	Name of the CPU
	 * _id:
	 *	Errata identifier
	 * _rev_var:
	 *	Register containing the combined value CPU revision and variant
	 *	- typically the return value of cpu_get_rev_var
	 */
	.macro report_errata _chosen, _cpu, _id, _rev_var=r4
	/* Stash a string with errata ID */
	.pushsection .rodata
	\_cpu\()_errata_\_id\()_str:
	.asciz	"\_id"
	.popsection

	/* Check whether errata applies */
	mov	r0, \_rev_var
	bl	check_errata_\_id

	.ifeq \_chosen
	/*
	 * Errata workaround has not been compiled in. If the errata would have
	 * applied had it been compiled in, print its status as missing.
	 */
	cmp	r0, #0
	movne	r0, #ERRATA_MISSING
	.endif
	ldr	r1, =\_cpu\()_cpu_str
	ldr	r2, =\_cpu\()_errata_\_id\()_str
	bl	errata_print_msg
	.endm
#endif

#endif /* __CPU_MACROS_S__ */
