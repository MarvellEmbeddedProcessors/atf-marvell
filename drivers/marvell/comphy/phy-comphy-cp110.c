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
	int ret = 0;

	debug_enter();

	debug_exit();

	return ret;
}

int mvebu_cp110_comphy_power_on(uint64_t comphy_base, uint64_t comphy_index, uint64_t comphy_mode)
{
	int err = 0;

	debug_enter();

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
