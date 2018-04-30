/*
 * Copyright (C) 2016 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */
#ifndef __MVEBU_A3700_DRAM_CS_H__
#define __MVEBU_A3700_DRAM_CS_H__

#include <stdint.h>
/*
 * marvell_get_dram_cs_base_size
 *
 * This function is used to get DRAM CS's memory base address
 * and memory block length from DRAM CS memory map registers,
 * the block length unit is MB.
 *
 * @input:
 *     - cs_num: DRAM CS number
 *
 * @output:
 *     - base_low: DRAM CS's memory base address in low 4 bytes
 *     - base_high: DRAM CS's memory base address in high 4 bytes
 *     - size_mbytes: DRAM CS's memory length in unit of MB
 *
 * @return:  N/A
 */
int marvell_get_dram_cs_base_size(uint32_t cs_num,
				  uint32_t *base_low,
				  uint32_t *base_high,
				  uint32_t *size_mbytes);

#endif /* __MVEBU_A3700_DRAM_CS_H__ */
