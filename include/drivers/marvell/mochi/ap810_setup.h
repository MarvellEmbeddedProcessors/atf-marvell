/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __APN810_SETUP_H__
#define __APN810_SETUP_H__

void ap810_enumeration_algo(void);
int get_ap_count(void);
int get_connected_cp_per_ap(int);
int get_static_cp_per_ap(int);
int ap810_rev_id_get(int ap_index);
void ap810_setup_banked_rgf(int ap_id);

#endif /* __APN810_SETUP_H__ */
