/*
 * Copyright (C) 2017, 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef _GWIN_H_
#define _GWIN_H_

#include <addr_map.h>

int init_gwin(int);
void gwin_temp_win_insert(int ap_index, struct addr_map_win *win, int size);
void gwin_temp_win_remove(int ap_index, struct addr_map_win *win, int size);

#endif /* _GWIN_H_ */
