/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
 
#ifndef _SYS_INFO_H_
#define _SYS_INFO_H_

#define SYSTEM_INFO_ADDRESS	0x4000000

enum sys_info_type {
	ARRAY_SIZE,
	DRAM_CS0_SIZE,
	DRAM_CS1_SIZE,
	DRAM_CS2_SIZE,
	DRAM_CS3_SIZE,
	DRAM_BUS_WIDTH,
	DRAM_ECC,
	DRAM_CS0,
	DRAM_CS1,
	DRAM_CS2,
	DRAM_CS3,
	RECOVERY_MODE,
	BOOT_MODE,
	CPU_DEC_WIN0_BASE,
	CPU_DEC_WIN1_BASE,
	CPU_DEC_WIN2_BASE,
	CPU_DEC_WIN3_BASE,
	CPU_DEC_WIN4_BASE,
	CPU_DEC_WIN0_SIZE,
	CPU_DEC_WIN1_SIZE,
	CPU_DEC_WIN2_SIZE,
	CPU_DEC_WIN3_SIZE,
	CPU_DEC_WIN4_SIZE,
	MAX_OPTION,
};

struct sys_info {
	enum sys_info_type field_id;
	uint32_t value;
};

void set_info(enum sys_info_type field, uint32_t value);

#endif /* _SYS_INFO_H_ */
