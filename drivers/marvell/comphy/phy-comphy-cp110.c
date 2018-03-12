/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <mmio.h>
#include <plat_def.h>
#include "mvebu.h"
#include "comphy-cp110.h"

/* #define DEBUG_COMPHY */
#ifdef DEBUG_COMPHY
#define debug(format...) NOTICE(format)
#else
#define debug(format, arg...)
#endif

/* A lane is described by 4 fields:
 *      - bit 1~0 represent comphy polarity invert
 *      - bit 7~2 represent comphy speed
 *      - bit 11~8 represent unit index
 *      - bit 16~12 represent mode
 *      - bit 17 represent comphy indication of clock source
 *      - bit 31~18 reserved
 */

#define COMPHY_INVERT_OFFSET	0
#define COMPHY_INVERT_LEN	2
#define COMPHY_INVERT_MASK	COMPHY_MASK(COMPHY_INVERT_OFFSET, COMPHY_INVERT_LEN)
#define COMPHY_SPEED_OFFSET	(COMPHY_INVERT_OFFSET + COMPHY_INVERT_LEN)
#define COMPHY_SPEED_LEN	6
#define COMPHY_SPEED_MASK	COMPHY_MASK(COMPHY_SPEED_OFFSET, COMPHY_SPEED_LEN)
#define COMPHY_UNIT_ID_OFFSET	(COMPHY_SPEED_OFFSET + COMPHY_SPEED_LEN)
#define COMPHY_UNIT_ID_LEN	4
#define COMPHY_UNIT_ID_MASK	COMPHY_MASK(COMPHY_UNIT_ID_OFFSET, COMPHY_UNIT_ID_LEN)
#define COMPHY_MODE_OFFSET	(COMPHY_UNIT_ID_OFFSET + COMPHY_UNIT_ID_LEN)
#define COMPHY_MODE_LEN		5
#define COMPHY_MODE_MASK	COMPHY_MASK(COMPHY_MODE_OFFSET, COMPHY_MODE_LEN)
#define COMPHY_CLK_SRC_OFFSET	(COMPHY_MODE_OFFSET + COMPHY_MODE_LEN)
#define COMPHY_CLK_SRC_LEN	1
#define COMPHY_CLK_SRC_MASK	COMPHY_MASK(COMPHY_CLK_SRC_OFFSET, COMPHY_CLK_SRC_LEN)

#define COMPHY_MASK(offset, len)	(((1 << (len)) - 1) << (offset))

/* Macro the extract the mode from lane description */
#define COMPHY_GET_MODE(x)		(((x) & COMPHY_MODE_MASK) >> COMPHY_MODE_OFFSET)
/* Macro the extract the unit index from lane description */
#define COMPHY_GET_ID(x)		(((x) & COMPHY_UNIT_ID_MASK) >> COMPHY_UNIT_ID_OFFSET)
/* Macro the extract the speed from lane description */
#define COMPHY_GET_SPEED(x)		(((x) & COMPHY_SPEED_MASK) >> COMPHY_SPEED_OFFSET)

#define COMPHY_SATA_MODE	0x1
#define COMPHY_SGMII_MODE	0x2	/* SGMII 1G */
#define COMPHY_HS_SGMII_MODE	0x3	/* SGMII 2.5G */
#define COMPHY_USB3H_MODE	0x4
#define COMPHY_USB3D_MODE	0x5
#define COMPHY_PCIE_MODE	0x6
#define COMPHY_RXAUI_MODE	0x7
#define COMPHY_XFI_MODE		0x8
#define COMPHY_SFI_MODE		0x9
#define COMPHY_USB3_MODE	0xa
#define COMPHY_AP_MODE		0xb

#define COMPHY_PIPE_FROM_COMPHY_ADDR(x)	((x & ~0xffffff) + 0x120000)

enum reg_width_type {
	REG_16BIT = 0,
	REG_32BIT,
};

static inline uint32_t polling_with_timeout(uintptr_t addr, uint32_t val,
		  uint32_t mask, uint32_t usec_timout, enum reg_width_type type)
{
	uint32_t data;

	do {
		udelay(1);
		if (type == REG_16BIT)
			data = mmio_read_16(addr) & mask;
		else
			data = mmio_read_32(addr) & mask;
	} while (data != val  && --usec_timout > 0);

	if (usec_timout == 0)
		return data;

	return 0;
}

static inline void reg_set(uintptr_t addr, uint32_t data, uint32_t mask)
{
	debug("Write to addr = 0x%lx, data = 0x%x (mask = 0x%x) - old val = 0x%x",
	      addr, data, mask, mmio_read_32(addr));
	mmio_clrsetbits_32(addr, mask, data);

	debug("new val 0x%x\n", mmio_read_32(addr));
}

/* Clear PHY selector - avoid collision with previous configuration */
static void mvebu_cp110_comphy_clr_phy_selector(uint64_t comphy_base, uint8_t comphy_index)
{
	uint32_t reg, mask, field;
	uint32_t comphy_offset = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;

	mask = COMMON_SELECTOR_COMPHY_MASK << comphy_offset;
	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET);
	field = reg & mask;

	/* Clear comphy selector - if it was already configured.
	 * (might be that this comphy was configured as PCIe/USB,
	 * in such case, no need to clear comphy selector because PCIe/USB
	 * are controlled by hpipe selector).
	 */
	if (field) {
		reg &= ~mask;
		mmio_write_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET, reg);
	}
}

/* PHY selector configures SATA and Network modes */
static void mvebu_cp110_comphy_set_phy_selector(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uint32_t reg, mask;
	uint32_t comphy_offset = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;
	int mode;

	/* Comphy mode (compound of the IO mode and id). Here, only the IO mode
	 * is required to distinguish between SATA and network modes.
	 */
	mode = COMPHY_GET_MODE(comphy_mode);

	mask = COMMON_SELECTOR_COMPHY_MASK << comphy_offset;
	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET);
	reg &= ~mask;

	/* SATA port 0/1 require the same configuration */
	if (mode == COMPHY_SATA_MODE) {
		/* SATA selector values is always 4 */
		reg |= COMMON_SELECTOR_COMPHYN_SATA << comphy_offset;
	} else {
		switch (comphy_index) {
		case(0):
		case(1):
		case(2):
			/* For comphy 0,1, and 2:
			 * Network selector value is always 1.
			 */
			reg |= COMMON_SELECTOR_COMPHY0_1_2_NETWORK << comphy_offset;
			break;
		case(3):
			/* For comphy 3:
			 * 0x1 = RXAUI_Lane1
			 * 0x2 = SGMII/HS-SGMII Port1
			 */
			if (mode == COMPHY_RXAUI_MODE)
				reg |= COMMON_SELECTOR_COMPHY3_RXAUI << comphy_offset;
			else
				reg |= COMMON_SELECTOR_COMPHY3_SGMII << comphy_offset;
			break;
		case(4):
			 /* For comphy 4:
			  * 0x1 = SGMII/HS-SGMII Port2
			  * 0x2 = SGMII/HS-SGMII Port0: XFI/SFI, RXAUI_Lane0
			  *
			  * We want to check if SGMII1/HS_SGMII1 is the requested mode in order to
			  * determine which value should be set (all other modes use the same value)
			  * so we need to strip the mode, and check the ID because we might handle
			  * SGMII0/HS_SGMII0 too.
			  */
			if ((mode == COMPHY_SGMII_MODE || mode == COMPHY_HS_SGMII_MODE) &&
			    COMPHY_GET_ID(comphy_mode) == 2)
				reg |= COMMON_SELECTOR_COMPHY4_SGMII2 << comphy_offset;
			else
				reg |= COMMON_SELECTOR_COMPHY4_ALL_OTHERS << comphy_offset;
			break;
		case(5):
			/* For comphy 5:
			 * 0x1 = SGMII/HS-SGMII Port2
			 * 0x2 = RXAUI Lane1
			 */
			if (mode == COMPHY_RXAUI_MODE)
				reg |= COMMON_SELECTOR_COMPHY5_RXAUI << comphy_offset;
			else
				reg |= COMMON_SELECTOR_COMPHY5_SGMII << comphy_offset;
			break;
		}
	}

	mmio_write_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET, reg);
}

/* Clear PIPE selector - avoid collision with privious configuration */
void mvebu_cp110_comphy_clr_pipe_selector(uint64_t comphy_base, uint8_t comphy_index)
{
	uint32_t reg, mask, field;
	uint32_t comphy_offset = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;

	mask = COMMON_SELECTOR_COMPHY_MASK << comphy_offset;
	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PIPE_REG_OFFSET);
	field = reg & mask;

	if (field) {
		reg &= ~mask;
		mmio_write_32(comphy_base + COMMON_SELECTOR_PIPE_REG_OFFSET, reg);
	}
}

int mvebu_cp110_comphy_is_pll_locked(uint64_t comphy_base, uint8_t comphy_index)
{

	uintptr_t sd_ip_addr, addr;
	uint32_t mask, data;
	int ret = 0;

	debug_enter();

	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);

	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_PLL_TX_MASK &
		SD_EXTERNAL_STATUS0_PLL_RX_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, PLL_LOCK_TIMEOUT, REG_32BIT);
	if (data != 0) {
		if (data & SD_EXTERNAL_STATUS0_PLL_RX_MASK)
			ERROR("RX PLL is not locked\n");
		if (data & SD_EXTERNAL_STATUS0_PLL_TX_MASK)
			ERROR("TX PLL is not locked\n");

		ret = -ETIMEDOUT;
	}

	debug_exit();

	return ret;
}

static int mvebu_cp110_comphy_sata_power_on(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uintptr_t hpipe_addr, sd_ip_addr, comphy_addr;
	uint32_t mask, data;
	int ret = 0;

	debug_enter();

	/* configure phy selector for SATA */
	mvebu_cp110_comphy_set_phy_selector(comphy_base, comphy_index, comphy_mode);

	hpipe_addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	comphy_addr = COMPHY_ADDR(comphy_base, comphy_index);

	debug(" add hpipe 0x%lx, sd 0x%lx, comphy 0x%lx\n",
					   hpipe_addr, sd_ip_addr, comphy_addr);
	debug("stage: RFU configurations - hard reset comphy\n");
	/* RFU configurations - hard reset comphy */
	mask = COMMON_PHY_CFG1_PWR_UP_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_UP_OFFSET;
	mask |= COMMON_PHY_CFG1_PIPE_SELECT_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PIPE_SELECT_OFFSET;
	mask |= COMMON_PHY_CFG1_PWR_ON_RESET_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PWR_ON_RESET_OFFSET;
	mask |= COMMON_PHY_CFG1_CORE_RSTN_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_CORE_RSTN_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Set select data  width 40Bit - SATA mode only */
	reg_set(comphy_addr + COMMON_PHY_CFG6_REG,
		0x1 << COMMON_PHY_CFG6_IF_40_SEL_OFFSET, COMMON_PHY_CFG6_IF_40_SEL_MASK);

	/* release from hard reset in SD external */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Wait 1ms - until band gap and ref clock ready */
	mdelay(1);

	debug("stage: Comphy configuration\n");
	/* Start comphy Configuration */
	/* Set reference clock to comes from group 1 - choose 25Mhz */
	reg_set(hpipe_addr + HPIPE_MISC_REG,
		0x0 << HPIPE_MISC_REFCLK_SEL_OFFSET, HPIPE_MISC_REFCLK_SEL_MASK);
	/* Reference frequency select set 1 (for SATA = 25Mhz) */
	mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
	data = 0x1 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	/* PHY mode select (set SATA = 0x0 */
	mask |= HPIPE_PWR_PLL_PHY_MODE_MASK;
	data |= 0x0 << HPIPE_PWR_PLL_PHY_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_PLL_REG, data, mask);
	/* Set max PHY generation setting - 6Gbps */
	reg_set(hpipe_addr + HPIPE_INTERFACE_REG,
		0x2 << HPIPE_INTERFACE_GEN_MAX_OFFSET, HPIPE_INTERFACE_GEN_MAX_MASK);
	/* Set select data  width 40Bit (SEL_BITS[2:0]) */
	reg_set(hpipe_addr + HPIPE_LOOPBACK_REG,
		0x2 << HPIPE_LOOPBACK_SEL_OFFSET, HPIPE_LOOPBACK_SEL_MASK);

	debug("stage: Analog parameters from ETP(HW)\n");
	/* G1 settings */
	mask = HPIPE_G1_SET_1_G1_RX_SELMUPI_MASK;
	data = 0x0 << HPIPE_G1_SET_1_G1_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUPP_MASK;
	data |= 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUPP_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUFI_MASK;
	data |= 0x0 << HPIPE_G1_SET_1_G1_RX_SELMUFI_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUFF_MASK;
	data |= 0x3 << HPIPE_G1_SET_1_G1_RX_SELMUFF_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_DIGCK_DIV_MASK;
	data |= 0x1 << HPIPE_G1_SET_1_G1_RX_DIGCK_DIV_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SET_1_REG, data, mask);

	mask = HPIPE_G1_SETTINGS_3_G1_FFE_CAP_SEL_MASK;
	data = 0xf << HPIPE_G1_SETTINGS_3_G1_FFE_CAP_SEL_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_RES_SEL_MASK;
	data |= 0x2 << HPIPE_G1_SETTINGS_3_G1_FFE_RES_SEL_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_SETTING_FORCE_MASK;
	data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_SETTING_FORCE_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_DEG_RES_LEVEL_MASK;
	data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_DEG_RES_LEVEL_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_LOAD_RES_LEVEL_MASK;
	data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_LOAD_RES_LEVEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SETTINGS_3_REG, data, mask);

	/* G2 settings */
	mask = HPIPE_G2_SET_1_G2_RX_SELMUPI_MASK;
	data = 0x0 << HPIPE_G2_SET_1_G2_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUPP_MASK;
	data |= 0x1 << HPIPE_G2_SET_1_G2_RX_SELMUPP_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUFI_MASK;
	data |= 0x0 << HPIPE_G2_SET_1_G2_RX_SELMUFI_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUFF_MASK;
	data |= 0x3 << HPIPE_G2_SET_1_G2_RX_SELMUFF_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_DIGCK_DIV_MASK;
	data |= 0x1 << HPIPE_G2_SET_1_G2_RX_DIGCK_DIV_OFFSET;
	reg_set(hpipe_addr + HPIPE_G2_SET_1_REG, data, mask);

	/* G3 settings */
	mask = HPIPE_G3_SET_1_G3_RX_SELMUPI_MASK;
	data = 0x2 << HPIPE_G3_SET_1_G3_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_SELMUPF_MASK;
	data |= 0x2 << HPIPE_G3_SET_1_G3_RX_SELMUPF_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_SELMUFI_MASK;
	data |= 0x3 << HPIPE_G3_SET_1_G3_RX_SELMUFI_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_SELMUFF_MASK;
	data |= 0x3 << HPIPE_G3_SET_1_G3_RX_SELMUFF_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_DFE_EN_MASK;
	data |= 0x1 << HPIPE_G3_SET_1_G3_RX_DFE_EN_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_DIGCK_DIV_MASK;
	data |= 0x2 << HPIPE_G3_SET_1_G3_RX_DIGCK_DIV_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_SAMPLER_INPAIRX2_EN_MASK;
	data |= 0x0 << HPIPE_G3_SET_1_G3_SAMPLER_INPAIRX2_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SET_1_REG, data, mask);

	/* DTL Control */
	mask = HPIPE_PWR_CTR_DTL_SQ_DET_EN_MASK;
	data = 0x1 << HPIPE_PWR_CTR_DTL_SQ_DET_EN_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_SQ_PLOOP_EN_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_SQ_PLOOP_EN_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_FLOOP_EN_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_FLOOP_EN_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_CLAMPING_SEL_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_CLAMPING_SEL_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_INTPCLK_DIV_FORCE_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_INTPCLK_DIV_FORCE_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_CLK_MODE_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_CLK_MODE_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_CLK_MODE_FORCE_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_CLK_MODE_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_CTR_DTL_REG, data, mask);

	/* Trigger sampler enable pulse */
	mask = HPIPE_SMAPLER_MASK;
	data = 0x1 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);
	mask = HPIPE_SMAPLER_MASK;
	data = 0x0 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);

	/* VDD Calibration Control 3 */
	mask = HPIPE_EXT_SELLV_RXSAMPL_MASK;
	data = 0x10 << HPIPE_EXT_SELLV_RXSAMPL_OFFSET;
	reg_set(hpipe_addr + HPIPE_VDD_CAL_CTRL_REG, data, mask);

	/* DFE Resolution Control */
	mask = HPIPE_DFE_RES_FORCE_MASK;
	data = 0x1 << HPIPE_DFE_RES_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_REG0, data, mask);

	/* DFE F3-F5 Coefficient Control */
	mask = HPIPE_DFE_F3_F5_DFE_EN_MASK;
	data = 0x0 << HPIPE_DFE_F3_F5_DFE_EN_OFFSET;
	mask |= HPIPE_DFE_F3_F5_DFE_CTRL_MASK;
	data = 0x0 << HPIPE_DFE_F3_F5_DFE_CTRL_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_F3_F5_REG, data, mask);

	/* G3 Setting 3 */
	mask = HPIPE_G3_FFE_CAP_SEL_MASK;
	data = 0xf << HPIPE_G3_FFE_CAP_SEL_OFFSET;
	mask |= HPIPE_G3_FFE_RES_SEL_MASK;
	data |= 0x4 << HPIPE_G3_FFE_RES_SEL_OFFSET;
	mask |= HPIPE_G3_FFE_SETTING_FORCE_MASK;
	data |= 0x1 << HPIPE_G3_FFE_SETTING_FORCE_OFFSET;
	mask |= HPIPE_G3_FFE_DEG_RES_LEVEL_MASK;
	data |= 0x1 << HPIPE_G3_FFE_DEG_RES_LEVEL_OFFSET;
	mask |= HPIPE_G3_FFE_LOAD_RES_LEVEL_MASK;
	data |= 0x3 << HPIPE_G3_FFE_LOAD_RES_LEVEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_3_REG, data, mask);

	/* G3 Setting 4 */
	mask = HPIPE_G3_DFE_RES_MASK;
	data = 0x1 << HPIPE_G3_DFE_RES_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_4_REG, data, mask);

	/* Offset Phase Control */
	mask = HPIPE_OS_PH_OFFSET_MASK;
	data = 0x61 << HPIPE_OS_PH_OFFSET_OFFSET;
	mask |= HPIPE_OS_PH_OFFSET_FORCE_MASK;
	data |= 0x1 << HPIPE_OS_PH_OFFSET_FORCE_OFFSET;
	mask |= HPIPE_OS_PH_VALID_MASK;
	data |= 0x0 << HPIPE_OS_PH_VALID_OFFSET;
	reg_set(hpipe_addr + HPIPE_PHASE_CONTROL_REG, data, mask);
	mask = HPIPE_OS_PH_VALID_MASK;
	data = 0x1 << HPIPE_OS_PH_VALID_OFFSET;
	reg_set(hpipe_addr + HPIPE_PHASE_CONTROL_REG, data, mask);
	mask = HPIPE_OS_PH_VALID_MASK;
	data = 0x0 << HPIPE_OS_PH_VALID_OFFSET;
	reg_set(hpipe_addr + HPIPE_PHASE_CONTROL_REG, data, mask);

	/* Set G1 TX amplitude and TX post emphasis value */
	mask = HPIPE_G1_SET_0_G1_TX_AMP_MASK;
	data = 0x8 << HPIPE_G1_SET_0_G1_TX_AMP_OFFSET;
	mask |= HPIPE_G1_SET_0_G1_TX_AMP_ADJ_MASK;
	data |= 0x1 << HPIPE_G1_SET_0_G1_TX_AMP_ADJ_OFFSET;
	mask |= HPIPE_G1_SET_0_G1_TX_EMPH1_MASK;
	data |= 0x1 << HPIPE_G1_SET_0_G1_TX_EMPH1_OFFSET;
	mask |= HPIPE_G1_SET_0_G1_TX_EMPH1_EN_MASK;
	data |= 0x1 << HPIPE_G1_SET_0_G1_TX_EMPH1_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SET_0_REG, data, mask);

	/* Set G2 TX amplitude and TX post emphasis value */
	mask = HPIPE_G2_SET_0_G2_TX_AMP_MASK;
	data = 0xa << HPIPE_G2_SET_0_G2_TX_AMP_OFFSET;
	mask |= HPIPE_G2_SET_0_G2_TX_AMP_ADJ_MASK;
	data |= 0x1 << HPIPE_G2_SET_0_G2_TX_AMP_ADJ_OFFSET;
	mask |= HPIPE_G2_SET_0_G2_TX_EMPH1_MASK;
	data |= 0x2 << HPIPE_G2_SET_0_G2_TX_EMPH1_OFFSET;
	mask |= HPIPE_G2_SET_0_G2_TX_EMPH1_EN_MASK;
	data |= 0x1 << HPIPE_G2_SET_0_G2_TX_EMPH1_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G2_SET_0_REG, data, mask);

	/* Set G3 TX amplitude and TX post emphasis value */
	mask = HPIPE_G3_SET_0_G3_TX_AMP_MASK;
	data = 0x1e << HPIPE_G3_SET_0_G3_TX_AMP_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_AMP_ADJ_MASK;
	data |= 0x1 << HPIPE_G3_SET_0_G3_TX_AMP_ADJ_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_EMPH1_MASK;
	data |= 0xe << HPIPE_G3_SET_0_G3_TX_EMPH1_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_EMPH1_EN_MASK;
	data |= 0x1 << HPIPE_G3_SET_0_G3_TX_EMPH1_EN_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_SLEW_RATE_SEL_MASK;
	data |= 0x4 << HPIPE_G3_SET_0_G3_TX_SLEW_RATE_SEL_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_SLEW_CTRL_EN_MASK;
	data |= 0x0 << HPIPE_G3_SET_0_G3_TX_SLEW_CTRL_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SET_0_REG, data, mask);

	/* SERDES External Configuration 2 register */
	mask = SD_EXTERNAL_CONFIG2_SSC_ENABLE_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG2_SSC_ENABLE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG2_REG, data, mask);

	/* DFE reset sequence */
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x1 << HPIPE_PWR_CTR_RST_DFE_OFFSET, HPIPE_PWR_CTR_RST_DFE_MASK);
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x0 << HPIPE_PWR_CTR_RST_DFE_OFFSET, HPIPE_PWR_CTR_RST_DFE_MASK);
	/* SW reset for interrupt logic */
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x1 << HPIPE_PWR_CTR_SFT_RST_OFFSET, HPIPE_PWR_CTR_SFT_RST_MASK);
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x0 << HPIPE_PWR_CTR_SFT_RST_OFFSET, HPIPE_PWR_CTR_SFT_RST_MASK);

	debug_exit();

	return ret;
}

int mvebu_cp110_comphy_power_on(uint64_t comphy_base, uint64_t comphy_index, uint64_t comphy_mode)
{
	int mode = COMPHY_GET_MODE(comphy_mode);
	int err = 0;

	debug_enter();

	switch (mode) {
	case(COMPHY_SATA_MODE):
		err = mvebu_cp110_comphy_sata_power_on(comphy_base, comphy_index, comphy_mode);
		break;
	default:
		ERROR("comphy%ld: unsupported comphy mode\n", comphy_index);
		err = -EINVAL;
		break;
	}

	debug_exit();

	return err;
}

int mvebu_cp110_comphy_power_off(uint64_t comphy_base, uint64_t comphy_index)
{
	uintptr_t sd_ip_addr, comphy_ip_addr;
	uint32_t mask, data;

	debug_enter();

	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	comphy_ip_addr = COMPHY_ADDR(comphy_base, comphy_index);

	/* Hard reset the comphy, for Ethernet modes and Sata */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Hard reset the comphy, for PCIe and usb3 */
	mask = COMMON_PHY_CFG1_PWR_ON_RESET_MASK;
	data = 0x0 << COMMON_PHY_CFG1_PWR_ON_RESET_OFFSET;
	mask |= COMMON_PHY_CFG1_CORE_RSTN_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_CORE_RSTN_OFFSET;
	reg_set(comphy_ip_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Clear comphy PHY and PIPE selector, can't rely on previous config. */
	mvebu_cp110_comphy_clr_phy_selector(comphy_base, comphy_index);
	mvebu_cp110_comphy_clr_pipe_selector(comphy_base, comphy_index);

	debug_exit();

	return 0;
}