/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
 
#include <arch_helpers.h>
#include <plat_marvell.h>
#include <debug.h>
#include <sys_info.h>

struct sys_info *sys_info_in_ptr = (struct sys_info *)SYSTEM_INFO_ADDRESS;
/* skip index 0 because this index in dedicated to array system-info size*/
int sys_info_size = 1;

void set_info(enum sys_info_type field, unsigned int value)
{
	if (sys_info_size == 1)
		sys_info_in_ptr[ARRAY_SIZE].field_id = ARRAY_SIZE;
	sys_info_in_ptr[sys_info_size].field_id = field;
	sys_info_in_ptr[sys_info_size].value = value;
	sys_info_in_ptr[ARRAY_SIZE].value = ++sys_info_size;
}
