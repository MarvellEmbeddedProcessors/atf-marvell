/*
 * Copyright (C) 2016 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */
#ifndef __PLAT_CONFIG_H__
#define __PLAT_CONFIG_H__

#include <stdint.h>

int marvell_get_io_dec_win_conf(struct dec_win_config **win, uint32_t *size);

#endif /* __PLAT_CONFIG_H__ */
