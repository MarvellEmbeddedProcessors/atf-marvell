/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __MVEBU_A8K_DEF_H__
#define __MVEBU_A8K_DEF_H__

#include <arm_def.h>

#define MVEBU_PRIMARY_CPU			0x0

#if PALLADIUM
#define COUNTER_FREQUENCY			7000
#else
#define COUNTER_FREQUENCY			25000000
#endif

#define MVEBU_REGS_BASE_LOCAL_AP		0xEC000000
#define MVEBU_REGS_BASE_AP0			0xE8000000
#define MVEBU_REGS_SIZE_PER_AP			0x04000000
#define MVEBU_REGS_BASE_AP(ap)			(MVEBU_REGS_BASE_AP0 - \
						((ap) * MVEBU_REGS_SIZE_PER_AP))

#define MVEBU_REGS_BASE				MVEBU_REGS_BASE_LOCAL_AP
#define MVEBU_REGS_BASE_MASK			0xFF000000

#define MVEBU_CCU_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x4000)
#define MVEBU_CCU_ADD_DEC_BASE(ap)		(MVEBU_CCU_BASE(ap))
#define MVEBU_CCU_HOME_CNTL_BASE(ap)		(MVEBU_CCU_BASE(ap) + 0x200)
#define MVEBU_CCU_LOCL_CNTL_BASE(ap)		(MVEBU_CCU_BASE(ap) + 0x300)

#define MVEBU_LLC_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x8000)

#define MVEBU_AP_UART_BASE			(MVEBU_REGS_BASE + 0x512000)
#define MVEBU_AR_RFU_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x6F0000)
#define MVEBU_IO_WIN_BASE(ap)			(MVEBU_AR_RFU_BASE(ap))
#define MVEBU_IO_WIN_GCR_OFFSET			(0xF0)

#define MVEBU_DFX_SR_BASE(ap)			(MVEBU_AR_RFU_BASE(ap) + 0x8000)
#define MVEBU_DFX_SAR_REG(ap, sar)		(MVEBU_DFX_SR_BASE(ap) + 0x200 + 0x4 * sar)
#define MVEBU_SAR_0_COHERENT_EN_OFFSET		15
#define MVEBU_SAR_0_COHERENT_EN_MASK		0xf

#define MVEBU_AP_MPP_REGS(ap, n)		(MVEBU_AR_RFU_BASE(ap) + 0x4000 + ((n) << 2))
#define MVEBU_AP_MISC_SOC_BASE(ap)		(MVEBU_AR_RFU_BASE(ap) + 0x4300)
#define MVEBU_AP_AXI_ATTR_REGS(ap)		(MVEBU_AR_RFU_BASE(ap) + 0x4580)
#define MVEBU_AP_GPIO_REGS(ap)			(MVEBU_AR_RFU_BASE(ap) + 0x5040)
#define MVEBU_AP_GPIO_DATA_IN(ap)		(MVEBU_AP_GPIO_REGS(ap) + 0x10)

#define MVEBU_SMMU_BASE(ap)			(MVEBU_AR_RFU_BASE(ap) + 0x100000)

#define MVEBU_DRAM_MAC_BASE			(MVEBU_REGS_BASE + 0x20000)
#define MVEBU_DRAM_PHY_BASE			(MVEBU_REGS_BASE + 0x20000)

#define MVEBU_CP_REGS_BASE(cp)			(0xF2000000 + (cp) * 0x2000000)

#define MVEBU_HPIPE_BASE(cp)			(MVEBU_CP_REGS_BASE(cp) + 0x120000)
#define MVEBU_AMB_IP_BASE(cp)			(MVEBU_CP_REGS_BASE(cp) + 0x13ff00)
#define MVEBU_IOB_BASE(cp)			(MVEBU_CP_REGS_BASE(cp) + 0x190000)
#define MVEBU_ICU_REG_BASE(cp)			(MVEBU_CP_REGS_BASE(cp) + 0x1E0000)

#define MVEBU_CP_MPP_REGS(cp, n)		(MVEBU_CP_REGS_BASE(cp) + 0x440000 + ((n) << 2))
#define MVEBU_CP_GPIO_DATA_OUT(cp, n)		(MVEBU_CP_REGS_BASE(cp) + 0x440100 + ((n > 32) ? 0x40 : 0x00))
#define MVEBU_CP_GPIO_DATA_OUT_EN(cp, n)	(MVEBU_CP_REGS_BASE(cp) + 0x440104 + ((n > 32) ? 0x40 : 0x00))
#define MVEBU_CP_GPIO_DATA_IN(cp, n)		(MVEBU_CP_REGS_BASE(cp) + 0x440110 + ((n > 32) ? 0x40 : 0x00))

#define MVEBU_CP_DFX_BASE(cp)			(MVEBU_CP_REGS_BASE(cp) + 0x400200)
#define MVEBU_AMB_ADEC_BASE(cp)			(MVEBU_CP_REGS_BASE(cp) + 0x70ff00)

/*******************************************************************************
 * MVEBU memory map related constants
 ******************************************************************************/
#define MVEBU_SAMPLE_AT_RESET_REG(x)		(MVEBU_CP_REGS_BASE(x) + 0x440600)
#define SAR_PCIE1_CLK_CFG_OFFSET		31
#define SAR_PCIE1_CLK_CFG_MASK			(0x1 << SAR_PCIE1_CLK_CFG_OFFSET)
#define SAR_PCIE0_CLK_CFG_OFFSET		30
#define SAR_PCIE0_CLK_CFG_MASK			(0x1 << SAR_PCIE0_CLK_CFG_OFFSET)
#define SAR_I2C_INIT_EN_OFFSET			24
#define SAR_I2C_INIT_EN_MASK			(1 << SAR_I2C_INIT_EN_OFFSET)

/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE				MVEBU_REGS_BASE_AP(3)
#define DEVICE0_SIZE				0x18000000

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
/* Base MVEBU compatible GIC memory map */
#define MVEBU_GICD_BASE				0x3000000
#define MVEBU_GICR_BASE				(MVEBU_GICD_BASE + 0x60000)

/*******************************************************************************
 * AXI Configuration
 ******************************************************************************/
#define MVEBU_AP_AXI_ATTR_REG(ap, index)	(MVEBU_AP_AXI_ATTR_REGS(ap) + 0x4 * index)

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
#define MVEBU_CP_MSS_DPSHSR_REG(x)		(MVEBU_CP_REGS_BASE(x) + 0x280040)
#define MSS_DPSHSR_REG_PCIE_CLK_SEL		0x8

/*******************************************************************************
 * PCIE clock buffer control
 ******************************************************************************/
#define MVEBU_PCIE_REF_CLK_BUF_CTRL(x)		(MVEBU_CP_REGS_BASE(x) + 0x4404F0)
#define PCIE1_REFCLK_BUFF_SOURCE		0x800
#define PCIE0_REFCLK_BUFF_SOURCE		0x400

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
#define SCP_BL2_BASE				BL31_BASE
#endif

#endif /* __MVEBU_A8K_DEF_H__ */
