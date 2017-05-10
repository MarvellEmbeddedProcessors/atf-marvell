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

#ifndef __MVEBU_A8K_DEF_H__
#define __MVEBU_A8K_DEF_H__

#include <arm_def.h>

#define MVEBU_PRIMARY_CPU		0x0

#if PALLADIUM
#define COUNTER_FREQUENCY		48000
#else
#define COUNTER_FREQUENCY		25000000
#endif

#define MVEBU_REGS_BASE			0xF0000000
#define MVEBU_REGS_BASE_MASK		0xF0000000
#define MVEBU_CP_REGS_BASE(cp_index)	(0xF2000000 + (cp_index) * 0x2000000)
#define MVEBU_RFU_BASE			(MVEBU_REGS_BASE + 0x6F0000)
#define MVEBU_CCU_BASE			(MVEBU_REGS_BASE + 0x4000)
#define MVEBU_LLC_BASE			(MVEBU_REGS_BASE + 0x8000)
#define MVEBU_IOB_BASE(cp_index)	(MVEBU_CP_REGS_BASE(cp_index) + \
								0x190000)
#define MVEBU_DRAM_MAC_BASE		(MVEBU_REGS_BASE + 0x20000)
#define MVEBU_DRAM_PHY_BASE		(MVEBU_REGS_BASE + 0x20000)
#define MVEBU_AMB_IP_BASE(cp_index)	(MVEBU_CP_REGS_BASE(cp_index) + \
								0x13ff00)
#define MVEBU_AMB_ADEC_BASE(cp_index)	(MVEBU_CP_REGS_BASE(cp_index) + \
								0x70ff00)
#define MVEBU_SMMU_BASE			(MVEBU_REGS_BASE + 0x100000)
#define MVEBU_CP_MPP_REGS(cp_index, n)	(MVEBU_CP_REGS_BASE(cp_index) + \
							0x440000 + ((n) << 2))
#define MVEBU_CP_GPIO_DATA_IN(cp_index, n) (MVEBU_CP_REGS_BASE(cp_index) + \
					0x440110 + ((n > 32) ? 0x40 : 0x00))
#define MVEBU_ICU_REG_BASE(cp_index)	(MVEBU_CP_REGS_BASE(cp_index) + \
								0x1E0000)
#define MVEBU_AP_MPP_REGS(n)		(MVEBU_RFU_BASE + 0x4000 + ((n) << 2))
#define MVEBU_AP_GPIO_REGS		(MVEBU_RFU_BASE + 0x5040)
#define MVEBU_AP_GPIO_DATA_IN		(MVEBU_AP_GPIO_REGS + 0x10)
#define MVEBU_AP_I2C_BASE		(MVEBU_REGS_BASE + 0x511000)
#define MVEBU_CP0_I2C_BASE		(MVEBU_CP_REGS_BASE(0) + 0x701000)

#define MVEBU_MCI_REG_BASE_REMAP(index) (0xFD000000 + ((index) * 0x1000000))

#define MVEBU_PCIE_X4_MAC_BASE(x)	(MVEBU_CP_REGS_BASE(x) + 0x600000)
#define MVEBU_COMPHY_BASE(x)		(MVEBU_CP_REGS_BASE(x) + 0x441000)
#define MVEBU_HPIPE_BASE(x)		(MVEBU_CP_REGS_BASE(x) + 0x120000)
#define MVEBU_CP_DFX_BASE(x)		(MVEBU_CP_REGS_BASE(x) + 0x400200)

/*******************************************************************************
 * MVEBU memory map related constants
 ******************************************************************************/
#define MVEBU_SAMPLE_AT_RESET_REG(x)	(MVEBU_CP_REGS_BASE(x) + 0x440600)
#define SAR_PCIE1_CLK_CFG_OFFSET	31
#define SAR_PCIE1_CLK_CFG_MASK		(0x1 << SAR_PCIE1_CLK_CFG_OFFSET)
#define SAR_PCIE0_CLK_CFG_OFFSET	30
#define SAR_PCIE0_CLK_CFG_MASK		(0x1 << SAR_PCIE0_CLK_CFG_OFFSET)


/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE			MVEBU_REGS_BASE
#define DEVICE0_SIZE			0x10000000

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
/* Base MVEBU compatible GIC memory map */
#define MVEBU_GICD_BASE			0x210000
#define MVEBU_GICC_BASE			0x220000


/*******************************************************************************
 * AXI Configuration
 ******************************************************************************/
#define MVEBU_AXI_ATTR_ARCACHE_OFFSET		4
#define MVEBU_AXI_ATTR_ARCACHE_MASK		(0xF << \
						 MVEBU_AXI_ATTR_ARCACHE_OFFSET)
#define MVEBU_AXI_ATTR_ARDOMAIN_OFFSET		12
#define MVEBU_AXI_ATTR_ARDOMAIN_MASK		(0x3 << \
						 MVEBU_AXI_ATTR_ARDOMAIN_OFFSET)
#define MVEBU_AXI_ATTR_AWCACHE_OFFSET		20
#define MVEBU_AXI_ATTR_AWCACHE_MASK		(0xF << \
						 MVEBU_AXI_ATTR_AWCACHE_OFFSET)
#define MVEBU_AXI_ATTR_AWDOMAIN_OFFSET		28
#define MVEBU_AXI_ATTR_AWDOMAIN_MASK		(0x3 << \
						 MVEBU_AXI_ATTR_AWDOMAIN_OFFSET)

/* SATA MBUS to AXI configuration */
#define MVEBU_SATA_M2A_AXI_ARCACHE_OFFSET	1
#define MVEBU_SATA_M2A_AXI_ARCACHE_MASK		(0xF << \
					MVEBU_SATA_M2A_AXI_ARCACHE_OFFSET)
#define MVEBU_SATA_M2A_AXI_AWCACHE_OFFSET	5
#define MVEBU_SATA_M2A_AXI_AWCACHE_MASK		(0xF << \
					MVEBU_SATA_M2A_AXI_AWCACHE_OFFSET)

/* ARM cache attributes */
#define CACHE_ATTR_BUFFERABLE			0x1
#define CACHE_ATTR_CACHEABLE			0x2
#define CACHE_ATTR_READ_ALLOC			0x4
#define CACHE_ATTR_WRITE_ALLOC			0x8
/* Domain */
#define DOMAIN_NON_SHAREABLE			0x0
#define DOMAIN_INNER_SHAREABLE			0x1
#define DOMAIN_OUTER_SHAREABLE			0x2
#define DOMAIN_SYSTEM_SHAREABLE			0x3

/*******************************************************************************
 * MSS Device Push Set Register
 ******************************************************************************/
#define MVEBU_CP_MSS_DPSHSR_REG(x)	(MVEBU_CP_REGS_BASE(x) + 0x280040)
#define MSS_DPSHSR_REG_PCIE_CLK_SEL	0x8

/*******************************************************************************
 * PCIE clock buffer control
 ******************************************************************************/
#define MVEBU_PCIE_REF_CLK_BUF_CTRL(x)	(MVEBU_CP_REGS_BASE(x) + 0x4404F0)
#define PCIE1_REFCLK_BUFF_SOURCE	0x800
#define PCIE0_REFCLK_BUFF_SOURCE	0x400

/*************************************************************************
 * Required platform porting definitions common to all
 * Mangement Compute SubSystems (MSS)
 ************************************************************************
 */
/*
 * Load address of SCP_BL2
 * SCP_BL2 is loaded to the same place as BL31.
 * Once SCP_BL2 is transferred to the SCP,
 * it is discarded and BL31 is loaded over the top.
 */
#ifdef SCP_IMAGE
#define SCP_BL2_BASE                    BL31_BASE
#endif

#endif /* __MVEBU_A8K_DEF_H__ */
