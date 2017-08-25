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
/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
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
#ifndef __MARVELL_DEF_H__
#define __MARVELL_DEF_H__

#include <arch.h>
#include <common_def.h>
#include <platform_def.h>
#include <tbbr_img_def.h>
#include <xlat_tables.h>


/******************************************************************************
 * Definitions common to all MARVELL standard platforms
 *****************************************************************************/

/* Special value used to verify platform parameters from BL2 to BL31 */
#define MARVELL_BL31_PLAT_PARAM_VAL		0x0f1e2d3c4b5a6978ULL

#define PLAT_MARVELL_NORTHB_COUNT	1

#define PLAT_MARVELL_CLUSTER_COUNT	1

#define MARVELL_CACHE_WRITEBACK_SHIFT	6

/*
 * Macros mapping the MPIDR Affinity levels to MARVELL Platform Power levels. The
 * power levels have a 1:1 mapping with the MPIDR affinity levels.
 */
#define MARVELL_PWR_LVL0		MPIDR_AFFLVL0
#define MARVELL_PWR_LVL1		MPIDR_AFFLVL1
#define MARVELL_PWR_LVL2		MPIDR_AFFLVL2

/*
 *  Macros for local power states in Marvell platforms encoded by State-ID field
 *  within the power-state parameter.
 */
/* Local power state for power domains in Run state. */
#define MARVELL_LOCAL_STATE_RUN	0
/* Local power state for retention. Valid only for CPU power domains */
#define MARVELL_LOCAL_STATE_RET	1
/* Local power state for OFF/power-down. Valid for CPU and cluster power
   domains */
#define MARVELL_LOCAL_STATE_OFF	2

/* The first 4KB of Trusted SRAM are used as shared memory */
#define MARVELL_TRUSTED_SRAM_BASE	PLAT_MARVELL_ATF_BASE
#define MARVELL_SHARED_RAM_BASE		MARVELL_TRUSTED_SRAM_BASE
#define MARVELL_SHARED_RAM_SIZE		0x00001000	/* 4 KB */

/* The remaining Trusted SRAM is used to load the BL images */
#define MARVELL_BL_RAM_BASE		(MARVELL_SHARED_RAM_BASE +	\
					 MARVELL_SHARED_RAM_SIZE)
#define MARVELL_BL_RAM_SIZE		(PLAT_MARVELL_TRUSTED_SRAM_SIZE -	\
					 MARVELL_SHARED_RAM_SIZE)

#define MARVELL_NS_DRAM1_BASE		MARVELL_DRAM1_BASE
#define MARVELL_NS_DRAM1_SIZE		MARVELL_DRAM1_SIZE
#define MARVELL_NS_DRAM1_END		(MARVELL_NS_DRAM1_BASE +		\
					 MARVELL_NS_DRAM1_SIZE - 1)

#define MARVELL_DRAM1_BASE			MAKE_ULL(0x0)
#define MARVELL_DRAM1_SIZE			MAKE_ULL(0x20000000)
#define MARVELL_DRAM1_END			(MARVELL_DRAM1_BASE +		\
					 MARVELL_DRAM1_SIZE - 1)

#define MARVELL_IRQ_SEC_PHY_TIMER		29

#define MARVELL_IRQ_SEC_SGI_0		8
#define MARVELL_IRQ_SEC_SGI_1		9
#define MARVELL_IRQ_SEC_SGI_2		10
#define MARVELL_IRQ_SEC_SGI_3		11
#define MARVELL_IRQ_SEC_SGI_4		12
#define MARVELL_IRQ_SEC_SGI_5		13
#define MARVELL_IRQ_SEC_SGI_6		14
#define MARVELL_IRQ_SEC_SGI_7		15

/*
 * Define a list of Group 1 Secure and Group 0 interrupts as per GICv3
 * terminology. On a GICv2 system or mode, the lists will be merged and treated
 * as Group 0 interrupts.
 */
#define MARVELL_G1S_IRQS			MARVELL_IRQ_SEC_PHY_TIMER,		\
					MARVELL_IRQ_SEC_SGI_1,		\
					MARVELL_IRQ_SEC_SGI_2,		\
					MARVELL_IRQ_SEC_SGI_3,		\
					MARVELL_IRQ_SEC_SGI_4,		\
					MARVELL_IRQ_SEC_SGI_5,		\
					MARVELL_IRQ_SEC_SGI_7

#define MARVELL_G0_IRQS			MARVELL_IRQ_SEC_SGI_0,		\
					MARVELL_IRQ_SEC_SGI_6

#define MARVELL_MAP_SHARED_RAM		MAP_REGION_FLAT(		\
						MARVELL_SHARED_RAM_BASE,	\
						MARVELL_SHARED_RAM_SIZE,	\
						MT_MEMORY | MT_RW | MT_SECURE)

#define MARVELL_MAP_NS_DRAM1		MAP_REGION_FLAT(		\
						MARVELL_NS_DRAM1_BASE,	\
						MARVELL_NS_DRAM1_SIZE,	\
						MT_MEMORY | MT_RW | MT_NS)


/*
 * The number of regions like RO(code), coherent and data required by
 * different BL stages which need to be mapped in the MMU.
 */
#if USE_COHERENT_MEM
#define MARVELL_BL_REGIONS			3
#else
#define MARVELL_BL_REGIONS			2
#endif

#define MAX_MMAP_REGIONS		(PLAT_MARVELL_MMAP_ENTRIES +	\
					 MARVELL_BL_REGIONS)

#define MARVELL_CONSOLE_BAUDRATE	115200

/******************************************************************************
 * Required platform porting definitions common to all MARVELL standard platforms
 *****************************************************************************/

#define ADDR_SPACE_SIZE			(1ull << 32)

/*
 * This macro defines the deepest retention state possible. A higher state
 * id will represent an invalid or a power down state.
 */
#define PLAT_MAX_RET_STATE		MARVELL_LOCAL_STATE_RET

/*
 * This macro defines the deepest power down states possible. Any state ID
 * higher than this is invalid.
 */
#define PLAT_MAX_OFF_STATE		MARVELL_LOCAL_STATE_OFF


#define PLATFORM_CORE_COUNT		PLAT_MARVELL_CLUSTER_CORE_COUNT

/*
 * Some data must be aligned on the biggest cache line size in the platform.
 * This is known only to the platform as it might have a combination of
 * integrated and external caches.
 */
#define CACHE_WRITEBACK_GRANULE		(1 << MARVELL_CACHE_WRITEBACK_SHIFT)


/*******************************************************************************
 * BL1 specific defines.
 * BL1 RW data is relocated from ROM to RAM at runtime so we need 2 sets of
 * addresses.
 ******************************************************************************/
#define BL1_RO_BASE			PLAT_MARVELL_TRUSTED_ROM_BASE
#define BL1_RO_LIMIT		(PLAT_MARVELL_TRUSTED_ROM_BASE	\
					 + PLAT_MARVELL_TRUSTED_ROM_SIZE)
/*
 * Put BL1 RW at the top of the Trusted SRAM.
 */
#define BL1_RW_BASE			(MARVELL_BL_RAM_BASE +		\
						MARVELL_BL_RAM_SIZE -	\
						0x6000)
#define BL1_RW_LIMIT		(MARVELL_BL_RAM_BASE + MARVELL_BL_RAM_SIZE)

/*******************************************************************************
 * BL2 specific defines.
 ******************************************************************************/
/*
 * Put BL2 just below BL31.
 */
#define BL2_BASE			(BL31_BASE - 0xC000)
#define BL2_LIMIT			BL31_BASE

/*******************************************************************************
 * BL31 specific defines.
 ******************************************************************************/
/*
 * Put BL31 at the top of the Trusted SRAM.
 */
#define BL31_BASE			(MARVELL_BL_RAM_BASE +		\
						MARVELL_BL_RAM_SIZE -	\
						0x5D000)
#define BL31_PROGBITS_LIMIT		BL1_RW_BASE
#define BL31_LIMIT			(MARVELL_BL_RAM_BASE + MARVELL_BL_RAM_SIZE)


#endif /* __MARVELL_DEF_H__ */
