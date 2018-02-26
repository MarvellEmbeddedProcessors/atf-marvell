/*
 * Copyright (C) 2017, 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef _ADDR_MAP_H_
#define _ADDR_MAP_H_

#include <stdint.h>

struct addr_map_win {
	uint64_t base_addr;
	uint64_t win_size;
	uint32_t target_id;
};

#endif /* _ADDR_MAP_H_ */
