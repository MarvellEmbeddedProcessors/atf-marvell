/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __PLAT_DRAM_H__
#define __PLAT_DRAM_H__

#include <mv_ddr_if.h>

/* TODO: move plat_dram_iface_set, plat_dram_ap_ifaces_get to ddr header file */
void plat_dram_iface_set(struct mv_ddr_iface *iface);
int plat_dram_ap_ifaces_get(int ap_id, struct mv_ddr_iface **ifaces, uint32_t *size);
int plat_dram_init(void);
uint32_t dram_rar_interleave(void);

#endif /* __PLAT_DRAM_H__ */
