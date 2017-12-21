/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <plat_marvell.h>
#include <plat_def.h>
#include <ap810_setup.h>

int ble_plat_setup(int *skip)
{
	int ret = 0;

	ap810_ble_init();

	/* TODO: need to check if need early cpu powerdown */

	/* TODO: check if recovery feature is needed */

	/* TODO: check if SVC is needed */

	/* TODO: Add DRAM initialization call */

	return ret;
}
