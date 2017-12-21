/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __APN810_SETUP_H__
#define __APN810_SETUP_H__

void ap810_init(void);
void ap810_ble_init(void);
void ap810_addr_decode_init(void);
int get_ap_count(void);
int get_connected_cp_per_ap(int);

#endif /* __APN810_SETUP_H__ */
