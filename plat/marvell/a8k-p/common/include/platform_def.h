/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <board_marvell_def.h>
#include <plat_def.h>
#ifndef __ASSEMBLY__
#include <stdio.h>
#endif /* __ASSEMBLY__ */

/*
 * Most platform porting definitions provided by included headers
 */

/*
 * DRAM Memory layout:
 *		+-----------------------+
 *		:			:
 *		:	Linux		:
 * 0x04000000-->+-----------------------+
 *		|	BL3-3(u-boot)	|>>}>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *		|-----------------------|  }				       |
 *		|	BL3-[0,1, 2]	|  }---------------------------------> |
 *		|-----------------------|  }				||     |
 *		|	BL2		|  }->FIP (loaded by            ||     |
 *		|-----------------------|  }       BootROM to DRAM)     ||     |
 *		|	FIP_TOC		|  }                            ||     |
 * 0x04120000-->|-----------------------|				||     |
 *		|	BL1 (RO)	|				||     |
 * 0x04100000-->+-----------------------+				||     |
 *		:			:				||     |
 *		: Trusted SRAM section	:				\/     |
 * 0x04040000-->+-----------------------+  Replaced by BL2  +----------------+ |
 *		|	BL1 (RW)	|  <<<<<<<<<<<<<<<< | BL3-1 NOBITS   | |
 * 0x04037000-->|-----------------------|  <<<<<<<<<<<<<<<< |----------------| |
 *		|			|  <<<<<<<<<<<<<<<< | BL3-1 PROGBITS | |
 * 0x04023000-->|-----------------------|		    +----------------+ |
 *		|	BL2		|				       |
 *		|-----------------------|				       |
 *		|			|				       |
 * 0x04001000-->|-----------------------|				       |
 *		|	Shared		|				       |
 * 0x04000000-->+-----------------------+				       |
 *		:			:				       |
 *		:	Linux		:				       |
 *		:			:				       |
 *		|-----------------------|				       |
 *		|			|	U-Boot(BL3-3) Loaded by BL2    |
 *		|	U-Boot		|	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 * 0x00000000-->+-----------------------+
 *
 * Trusted SRAM section 0x4000000..0x4200000:
 * ----------------------------------------
 * SRAM_BASE		= 0x4001000
 * BL2_BASE		= 0x4006000
 * BL2_LIMIT		= BL31_BASE
 * BL31_BASE		= 0x4023000 = (64MB + 256KB - 0x1D000)
 * BL31_PROGBITS_LIMIT	= BL1_RW_BASE
 * BL1_RW_BASE		= 0x4037000 = (64MB + 256KB - 0x9000)
 * BL1_RW_LIMIT		= BL31_LIMIT = 0x4040000
 *
 *
 * PLAT_MARVELL_FIP_BASE	= 0x4120000
 */

/*
 * Since BL33 is loaded by BL2 (and validated by BL31) to DRAM offset 0,
 * it is allowed to load/copy images to 'NULL' pointers
 */
#if defined(IMAGE_BL2) || defined(IMAGE_BL31)
#define PLAT_ALLOW_ZERO_ADDR_COPY
#endif

/*
 * ******************************************************
 *              SRAM Memory map - offsets only          *
 *                                                      *
 *  0x000000 0x003FFF  16K  Page tables                 *
 *  0x004000 0x007FFF  16K  AP global memory            *
 *  0x008000 0x00BFFF  16K  CP global memory            *
 *  0x00B000 0x01AFFF  64K  IOROM copy space            *
 *  0x01C000 0x3FBFFF  3968K Extended header copy       *
 *  0x3FC000 0x3FFFFC  16K  Stack                       *
 *                                                      *
 * ******************************************************
 */
#define PLAT_MARVELL_SRAM_BASE			0xFFA1C048
#define PLAT_MARVELL_SRAM_END			0xFFDFC000

#define PLAT_MARVELL_ATF_BASE			0x4000000
#define PLAT_MARVELL_ATF_LOAD_ADDR		(PLAT_MARVELL_ATF_BASE + \
						0x100000)

#define PLAT_MARVELL_FIP_BASE			(PLAT_MARVELL_ATF_LOAD_ADDR + \
						0x20000)
#define PLAT_MARVELL_FIP_MAX_SIZE		0x4000000

/* AP810 SoC parameters */
#define PLAT_MARVELL_NORTHB_COUNT		4	/* Number of interconnected APs */
#define PLAT_MARVELL_SOUTHB_COUNT		4	/* Number of CPs connected to each AP */
#define PLAT_MARVELL_CLUSTER_PER_NB		4
#define PLAT_MARVELL_CLUSTER_CORE_COUNT		2

/* AP810 support 4 AP dies, every die include 4 clusters */
#define PLAT_MARVELL_CLUSTER_COUNT		(PLAT_MARVELL_NORTHB_COUNT * \
						PLAT_MARVELL_CLUSTER_PER_NB)
/* AP810 suppprt 4 AP dies, every die include 8 cores */
#define PLAT_MARVELL_CORE_COUNT			(PLAT_MARVELL_CLUSTER_COUNT * \
						PLAT_MARVELL_CLUSTER_CORE_COUNT)

#define MPIDR_AP_ID_GET(mpidr)			MPIDR_AFFLVL2_VAL((mpidr))
#define MPIDR_CLUSTER_ID_GET(mpidr)		MPIDR_AFFLVL1_VAL((mpidr))
#define MPIDR_CPU_ID_GET(mpidr)			MPIDR_AFFLVL0_VAL((mpidr))

/* DRAM[2MB..66MB] is used as Trusted ROM */
#define PLAT_MARVELL_TRUSTED_ROM_BASE		PLAT_MARVELL_ATF_LOAD_ADDR
/* 64 MB TODO: reduce this to minimum needed according to fip image size */
#define PLAT_MARVELL_TRUSTED_ROM_SIZE		0x04000000
/* Reserve 16M for SCP (Secure PayLoad) Trusted DRAM */
#define PLAT_MARVELL_TRUSTED_DRAM_BASE		0x04400000
#define PLAT_MARVELL_TRUSTED_DRAM_SIZE		0x01000000	/* 16 MB */

/*
 * PLAT_ARM_MAX_BL1_RW_SIZE is calculated using the current BL1 RW debug size
 * plus a little space for growth.
 */
#define PLAT_MARVELL_MAX_BL1_RW_SIZE		0xA000

/*
 * PLAT_ARM_MAX_BL2_SIZE is calculated using the current BL2 debug size plus a
 * little space for growth.
 */
#define PLAT_MARVELL_MAX_BL2_SIZE		0xF000

/*
 * PLAT_ARM_MAX_BL31_SIZE is calculated using the current BL31 debug size plus a
 * little space for growth.
 */
#define PLAT_MARVEL_MAX_BL31_SIZE		0x5D000

#define PLAT_MARVELL_CPU_ENTRY_ADDR		BL1_RO_BASE

/* GIC related definitions */
#define PLAT_MARVELL_GICD_BASE			(MVEBU_REGS_BASE + MVEBU_GICD_BASE)
#define PLAT_MARVELL_GICR_BASE			(MVEBU_REGS_BASE + MVEBU_GICR_BASE)

#define PLAT_MARVELL_G0_IRQS			MARVELL_G0_IRQS
#define PLAT_MARVELL_G1S_IRQS			MARVELL_G1S_IRQS

#define PLAT_MARVELL_SHARED_RAM_CACHED		1

/*
 * Load address of BL3-3 for this platform port
 */
#if PALLADIUM
#define PLAT_MARVELL_NS_IMAGE_OFFSET		0x1000
#else
#define PLAT_MARVELL_NS_IMAGE_OFFSET		0x0
#endif

/* System Reference Clock*/
#define PLAT_REF_CLK_IN_HZ			COUNTER_FREQUENCY

/*
 * PL011 related constants
 */
#define PLAT_MARVELL_BOOT_UART_BASE		MVEBU_AP_UART_BASE(0)
#if PALLADIUM
#define PLAT_MARVELL_BOOT_UART_CLK_IN_HZ	384000
#else
#define PLAT_MARVELL_BOOT_UART_CLK_IN_HZ	200000000
#endif

#define PLAT_MARVELL_CRASH_UART_BASE		PLAT_MARVELL_BOOT_UART_BASE
#define PLAT_MARVELL_CRASH_UART_CLK_IN_HZ	PLAT_MARVELL_BOOT_UART_CLK_IN_HZ

#define PLAT_MARVELL_BL31_RUN_UART_BASE		PLAT_MARVELL_BOOT_UART_BASE
#define PLAT_MARVELL_BL31_RUN_UART_CLK_IN_HZ	PLAT_MARVELL_BOOT_UART_CLK_IN_HZ

/* Required platform porting definitions */
#define PLAT_MAX_PWR_LVL			MPIDR_AFFLVL1

/* System timer related constants */
#define PLAT_MARVELL_NSTIMER_FRAME_ID		1

/* Mailbox base address (note the lower memory space are reserved for BLE data) */
#define PLAT_MARVELL_MAILBOX_BASE		(MARVELL_TRUSTED_SRAM_BASE + 0x400)
#define PLAT_MARVELL_MAILBOX_SIZE		0x100
#define PLAT_MARVELL_MAILBOX_MAGIC_NUM		0x6D72766C	/* mrvl */

/* Securities */
#define IRQ_SEC_OS_TICK_INT			MARVELL_IRQ_SEC_PHY_TIMER

#define TRUSTED_DRAM_BASE			PLAT_MARVELL_TRUSTED_DRAM_BASE
#define TRUSTED_DRAM_SIZE			PLAT_MARVELL_TRUSTED_DRAM_SIZE

#define BL32_BASE				TRUSTED_DRAM_BASE

/* CCU registers */
#define CCU_WIN_CR_OFFSET(ap, win)	(MVEBU_CCU_BASE(ap) + 0x0 + (0x10 * win))
#define CCU_TARGET_ID_OFFSET		(8)
#define CCU_TARGET_ID_MASK		(0x7F)

#define CCU_WIN_SCR_OFFSET(ap, win)	(MVEBU_CCU_BASE(ap) + 0x4 + (0x10 * win))
#define CCU_WIN_ENA_WRITE_SECURE	(0x1)
#define CCU_WIN_ENA_READ_SECURE		(0x2)

#define CCU_WIN_ALR_OFFSET(ap, win)	(MVEBU_CCU_BASE(ap) + 0x8 + (0x10 * win))
#define CCU_WIN_AHR_OFFSET(ap, win)	(MVEBU_CCU_BASE(ap) + 0xC + (0x10 * win))

#define CCU_WIN_GCR_OFFSET(ap)		(MVEBU_CCU_BASE(ap) + 0xD0)
#define CCU_GCR_TARGET_OFFSET		(8)
#define CCU_GCR_TARGET_MASK		(0xF)

#define CCU_MC_RCR_OFFSET(ap, iface)	(MVEBU_REGS_BASE_AP(ap) + \
					0x1700 + (0x1400 * (iface)))
#define CCU_MC_RSBR_OFFSET(ap, iface)	(CCU_MC_RCR_OFFSET(ap, iface) + 0x4)
#define CCU_MC_RTBR_OFFSET(ap, iface)	(CCU_MC_RCR_OFFSET(ap, iface) + 0x8)

#define	CCU_SRAM_WIN_CR			CCU_WIN_CR_OFFSET(MVEBU_AP0, 1)

#endif /* __PLATFORM_DEF_H__ */
