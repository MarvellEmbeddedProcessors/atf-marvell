/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __MVEBU_DEF_H__
#define __MVEBU_DEF_H__

#include <a8kp_plat_def.h>

/* For now we'll have North bridge count to
** configure the number of APs, later we'll
** add function to read AP count */
#if PLAT_MARVELL_NORTHB_COUNT
#undef PLAT_MARVELL_NORTHB_COUNT
#define PLAT_MARVELL_NORTHB_COUNT	1
#endif

#endif /* __MVEBU_DEF_H__ */
