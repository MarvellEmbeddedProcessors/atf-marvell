/*
 * Copyright (C) 2015 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
 
#include <assert.h>
#include <cache_llc.h>
#include <mmio.h>
#include <plat_def.h>

void llc_cache_sync(int ap_index)
{
	mmio_write_32(LLC_CACHE_SYNC(ap_index), 0);
	/* Atumic write no need to wait */
}

void llc_flush_all(int ap_index)
{
	mmio_write_32(L2X0_CLEAN_INV_WAY(ap_index), LLC_WAY_MASK);
	llc_cache_sync(ap_index);
}

void llc_clean_all(int ap_index)
{
	mmio_write_32(L2X0_CLEAN_WAY(ap_index), LLC_WAY_MASK);
	llc_cache_sync(ap_index);
}

void llc_inv_all(int ap_index)
{
	mmio_write_32(L2X0_INV_WAY(ap_index), LLC_WAY_MASK);
	llc_cache_sync(ap_index);
}

void llc_disable(int ap_index)
{
	llc_flush_all(ap_index);
	mmio_write_32(LLC_CTRL(ap_index), 0);
	__asm__ volatile("dsb st");
}

void llc_enable(int ap_index, int excl_mode)
{
	uint32_t val;

	__asm__ volatile("dsb sy");
	llc_inv_all(ap_index);
	__asm__ volatile("dsb sy");

	val = LLC_CTRL_EN;
	if (excl_mode)
		val |= LLC_EXCLUSIVE_EN;

	mmio_write_32(LLC_CTRL(ap_index), val);
	__asm__ volatile("dsb sy");
}

int llc_is_exclusive(int ap_index)
{
	uint32_t reg;

	reg = mmio_read_32(LLC_CTRL(ap_index));

	if ((reg & (LLC_CTRL_EN | LLC_EXCLUSIVE_EN)) == (LLC_CTRL_EN | LLC_EXCLUSIVE_EN))
		return 1;
	return 0;
}

void llc_save(int ap_index)
{
	/* TBD */
}

void llc_resume(int ap_index)
{
	/* TBD */
}
