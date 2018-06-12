/*
 * Copyright (C) 2015 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <assert.h>
#include <cache_llc.h>
#include <ccu.h>
#include <mmio.h>
#include <plat_def.h>

#define CCU_HTC_CR(ap_index)		(MVEBU_CCU_BASE(ap_index) + 0x200)
#define CCU_SET_POC_OFFSET		5

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

void llc_runtime_enable(int ap_index)
{
	uint32_t reg;

	reg = mmio_read_32(LLC_CTRL(ap_index));
	if (reg & LLC_CTRL_EN)
		return;

	INFO("Enabling LLC\n");

	/*
	 * Enable L2 UniqueClean evictions
	 *  Note: this configuration assumes that LLC is configured
	 *	  in exclusive mode.
	 *	  Later on in the code this assumption will be validated
	 */
	__asm__ volatile ("mrs %0, s3_1_c15_c0_0" : "=r" (reg));
	reg |=  (1 << 14);
	__asm__ volatile ("msr s3_1_c15_c0_0, %0" : : "r" (reg));

	llc_enable(ap_index, 1);

	/* Set point of coherency to DDR.
	 * This is required by units which have SW cache coherency
	 */
	reg = mmio_read_32(CCU_HTC_CR(ap_index));
	reg |= (0x1 << CCU_SET_POC_OFFSET);
	mmio_write_32(CCU_HTC_CR(ap_index), reg);
}
