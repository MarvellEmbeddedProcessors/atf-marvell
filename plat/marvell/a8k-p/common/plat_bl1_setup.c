/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <plat_marvell.h>
#include <mmio.h>
#include <ap810_setup.h>

void marvell_bl1_setup_mpps(void)
{
	/* Enable UART MPPs.
	 ** In a normal system, this is done by Bootrom.
	 */
	mmio_write_32(MVEBU_AP_MPP_REGS(0, 1), 0x3000);
	mmio_write_32(MVEBU_AP_MPP_REGS(0, 2), 0x30000);
}
