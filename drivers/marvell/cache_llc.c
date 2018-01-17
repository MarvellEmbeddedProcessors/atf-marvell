/*
 * ***************************************************************************
 * Copyright (C) 2015 Marvell International Ltd.
 * ***************************************************************************
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ***************************************************************************
 */

#include <assert.h>
#include <mmio.h>
#include <plat_def.h>
#include <cache_llc.h>

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
