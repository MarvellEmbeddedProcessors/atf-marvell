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

#define MVEBU_A2_BANKED_STOP_BASE(ap, stop)	(MVEBU_REGS_BASE_AP(ap) + 0x800 + 0x400 * stop)

#define MVEBU_CCU_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x4000)
#define MVEBU_CCU_MAX_WINS			(5)
#define MVEBU_CCU_GRU_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x5200)

#define MVEBU_GWIN_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x5400)
#define MVEBU_GWIN_MAX_WINS			(16)

#define MVEBU_CCU_HOME_CNTL_BASE(ap)		(MVEBU_CCU_BASE(ap) + 0x200)
#define CCU_HTC_ACR(ap, m)			(MVEBU_CCU_HOME_CNTL_BASE(ap) + 0x10 + (m) * 0x4)
#define CCU_HTC_ASET(ap)			(MVEBU_CCU_HOME_CNTL_BASE(ap) + 0x64)
#define CCU_HTC_ACR_CLUSTERx_OFFSET(x)		(0x1 << (x))
#define CCU_HTC_GACR(ap, m)			(MVEBU_CCU_HOME_CNTL_BASE(ap) + 0x80 + (m) * 0x4)
#define CCU_HTC_GASET(ap)			(MVEBU_CCU_HOME_CNTL_BASE(ap) + 0xD4)

#define MVEBU_CCU_LOCL_CNTL_BASE(ap)		(MVEBU_CCU_BASE(ap) + 0x300)

#define MVEBU_LLC_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x8000)

#define MVEBU_AP_UART_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x512000)
#define MVEBU_MRI_XBAR_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x6A0000)
#define MVEBU_AR_RFU_BASE(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x6F0000)
#define MVEBU_IO_WIN_BASE(ap)			(MVEBU_AR_RFU_BASE(ap))
#define MVEBU_IO_WIN_GCR_OFFSET			(0xF0)
#define MVEBU_IO_WIN_MAX_WINS			(11)


#define MVEBU_DFX_SR_BASE(ap)			(MVEBU_AR_RFU_BASE(ap) + 0x8000)
#define MVEBU_DFX_SAR_REG(ap, sar)		(MVEBU_DFX_SR_BASE(ap) + 0x200 + 0x4 * sar)
#define MVEBU_SAR_0_COHERENT_EN_OFFSET		15
#define MVEBU_SAR_0_COHERENT_EN_MASK		0xf

#define MVEBU_AP_MPP_REGS(ap, n)		(MVEBU_AR_RFU_BASE(ap) + 0x4000 + ((n) << 2))
#define MVEBU_AP_IHBX4_CNTRL_BASE(ap, mci)	(MVEBU_AR_RFU_BASE(ap) + 0x4200 + 0x20 * mci)
#define MVEBU_AP_MISC_SOC_BASE(ap)		(MVEBU_AR_RFU_BASE(ap) + 0x4300)
#define MVEBU_AP_AXI_ATTR_REGS(ap)		(MVEBU_AR_RFU_BASE(ap) + 0x4580)
#define MVEBU_AP_GPIO_REGS(ap)			(MVEBU_AR_RFU_BASE(ap) + 0x5040)
#define MVEBU_AP_GPIO_DATA_OUT_VAL(ap)		(MVEBU_AP_GPIO_REGS(ap))
#define MVEBU_AP_GPIO_DATA_OUT_EN(ap)		(MVEBU_AP_GPIO_REGS(ap) + 0x4)
#define MVEBU_AP_GPIO_DATA_IN(ap)		(MVEBU_AP_GPIO_REGS(ap) + 0x10)

#define MVEBU_MCI_PHY(ap, n)			(MVEBU_REGS_BASE_AP(ap) + 0x6C0000 + \
						(((n) / 2 + 1) * 0x1000) + 0x1000 * (n))

#define MVEBU_SMMU_BASE(ap)			(MVEBU_AR_RFU_BASE(ap) + 0x100000)

#define MVEBU_DRAM_MAC_BASE			(MVEBU_REGS_BASE + 0x20000)
#define MVEBU_DRAM_PHY_BASE			(MVEBU_REGS_BASE + 0x20000)

#define MVEBU_CP_DEFAULT_BASE_ADDR		0xF2000000
#define MVEBU_CP_DEFAULT_BASE_SIZE		0x2000000

#define MVEBU_CP_REGS_BASE(ap, cp)			(0xF2000000 + (cp) * 0x2000000)
#define MVEBU_CP_DFX_OFFSET			(0x400200)

#define MCI_MAX_UNIT_ID				8
#define MVEBU_MCI_REG_BASE_REMAP(index)		(0xFD000000)
#define MVEBU_MCI_REG_SIZE_REMAP		(0x1000000)

/*******************************************************************************
 * MVEBU memory map related constants
 ******************************************************************************/
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

#ifndef __ASSEMBLER__
enum io_win_target_ids {
	MCI_0_TID	 = 0x0,
	MCI_1_TID	 = 0x1,
	MCI_2_TID	 = 0x2,
	MCI_3_TID	 = 0x3,
	MCI_4_TID	 = 0x4,
	MCI_5_TID	 = 0x5,
	MCI_6_TID	 = 0x6,
	MCI_7_TID	 = 0x7,
	PIDI_TID	 = 0x8,
	SPI_TID		 = 0x9,
	STM_TID		 = 0xa,
	BOOTROM_TID	 = 0xb,
	IO_WIN_MAX_TID
};

enum ccu_target_ids {
	IO_0_TID        = 0x00,
	DRAM_0_TID      = 0x03,
	DRAM_1_TID      = 0x08,
	GLOBAL_TID	= 0x09,
	CFG_REG_TID     = 0x10,
	RAR_TID         = 0x20,
	SRAM_TID        = 0x40,
	CCU_MAX_TID,
	INVALID_TID     = 0xFF
};
#endif /* __ASSEMBLER__ */

#endif /* __MVEBU_A8K_DEF_H__ */
