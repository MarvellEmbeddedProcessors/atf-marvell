/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __MVEBU_DEF_H__
#define __MVEBU_DEF_H__

#include <a8kp_plat_def.h>

#if (CP_NUM > 8) || (CP_NUM < 0)
#error "Supported CP_NUM values are 0 to 8."
#else
#define CP110_DIE_NUM			CP_NUM
#endif
#define MPP_MCI_RELEASE_FROM_RESET	16

/* DB-88F8160-MODULAR has 4 DIMMs on board that are connected to
 * AP2 I2C bus-0 at the following addresses:
 * AP0 DIMM0 - 0x53
 * AP0 DIMM1 - 0x54
 * AP1 DIMM0 - 0x55
 * AP1 DIMM1 - 0x56
 */
#define I2C_SPD_BASE_ADDR		0x53
#define I2C_SPD_DATA_ADDR(iface_num)	(I2C_SPD_BASE_ADDR + (iface_num))
#define I2C_SPD_P0_SEL_ADDR		0x36	/* Select SPD data page 0 */

#endif /* __MVEBU_DEF_H__ */
