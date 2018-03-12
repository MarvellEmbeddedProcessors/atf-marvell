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
	debug_enter();

	debug_exit();

	return 0;
}
