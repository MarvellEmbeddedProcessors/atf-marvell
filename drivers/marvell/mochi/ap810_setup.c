/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <platform.h>
#include <debug.h>

/* This function initialize the AP810 SoC configuration */
void ap810_init(void)
{
	INFO("%s: initialization for AP810 is not implemented\n", __func__);

	/* TODO: Call Aurora init */
	/* TODO: Go over APs and init:
	**	- IO_WIN
	**	- CCU
	**	- SMMU
	**	- AXI ATTR
	**	- TIMER */

	return;
}
