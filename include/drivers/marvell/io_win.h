/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef _IO_WIN_H_
#define _IO_WIN_H_

#include <addr_map.h>

int init_io_win(int);
void iow_temp_win_insert(int ap_index, struct addr_map_win *win, int size);
void iow_temp_win_remove(int ap_index, struct addr_map_win *win, int size);
void iow_save_win_all(int ap_id);
void iow_restore_win_all(int ap_id);

#endif /* _IO_WIN_H_ */
