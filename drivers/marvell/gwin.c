/*
* ***************************************************************************
* Copyright (C) 2017 Marvell International Ltd.
* ***************************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* Neither the name of Marvell nor the names of its contributors may be used
* to endorse or promote products derived from this software without specific
* prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************
*/

#include <plat_def.h>
#include <debug.h>
#include <mmio.h>
#include <mvebu.h>
#include <gwin.h>
#include <plat_config.h>

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define DEBUG_ADDR_MAP
#endif

/* common defines */
#define WIN_ENABLE_BIT			(0x1)
#define WIN_TARGET_MASK			(0xF)
#define WIN_TARGET(tgt)			(((tgt) & WIN_TARGET_MASK) << 8)

/* Bits[43:26] of the physical address are the window base,
 * which is aligned to 64MB
 */
#define ADDRESS_RSHIFT			(26)
#define ADDRESS_LSHIFT			(10)
#define GWIN_ALIGNMENT_64M		(0x4000000)

/* AP registers */
#define GWIN_CR_OFFSET(ap, win)		(MVEBU_GWIN_BASE(ap) + 0x0 + (0x10 * (win)))
#define GWIN_ALR_OFFSET(ap, win)	(MVEBU_GWIN_BASE(ap) + 0x8 + (0x10 * (win)))
#define GWIN_AHR_OFFSET(ap, win)	(MVEBU_GWIN_BASE(ap) + 0xc + (0x10 * (win)))

#define CCU_GRU_CR_OFFSET(ap)		(MVEBU_CCU_GRU_BASE(ap))
#define CCR_GRU_CR_GWIN_MBYPASS		(1 << 1)

static void gwin_check(struct addr_map_win *win)
{
	/* The base is always 64M aligned */
	if (IS_NOT_ALIGN(win->base_addr, GWIN_ALIGNMENT_64M)) {
		win->base_addr = ALIGN_UP(win->base_addr, GWIN_ALIGNMENT_64M);
		NOTICE("Align up the base address to 0x%lx\n", win->base_addr);
	}

	/* size parameter validity check */
	if (IS_NOT_ALIGN(win->win_size, GWIN_ALIGNMENT_64M)) {
		win->win_size = ALIGN_UP(win->win_size, GWIN_ALIGNMENT_64M);
		NOTICE("Aligning window size to 0x%lx\n", win->win_size);
	}
}

static void gwin_enable_window(int ap_index, struct addr_map_win *win, uint32_t win_num)
{
	uint32_t alr, ahr;
	uint64_t end_addr;

	if ((win->target_id & WIN_TARGET_MASK) != win->target_id) {
		ERROR("target ID = %d, is invalid\n", win->target_id);
		return;
	}

	/* calculate 64bit end-address */
	end_addr = (win->base_addr + win->win_size - 1);

	alr = (uint32_t)((win->base_addr >> ADDRESS_RSHIFT) << ADDRESS_LSHIFT);
	ahr = (uint32_t)((end_addr >> ADDRESS_RSHIFT) << ADDRESS_LSHIFT);

	/* write start address and end address for GWIN */
	mmio_write_32(GWIN_ALR_OFFSET(ap_index, win_num), alr);
	mmio_write_32(GWIN_AHR_OFFSET(ap_index, win_num), ahr);

	/* write the target ID and enable the window */
	mmio_write_32(GWIN_CR_OFFSET(ap_index, win_num),
		      WIN_TARGET(win->target_id) | WIN_ENABLE_BIT);
}

static void gwin_disable_window(int ap_index, uint32_t win_num)
{
	uint32_t win_reg;

	win_reg = mmio_read_32(GWIN_CR_OFFSET(ap_index, win_num));
	win_reg &= ~WIN_ENABLE_BIT;
	mmio_write_32(GWIN_CR_OFFSET(ap_index, win_num), win_reg);
}

#ifdef DEBUG_ADDR_MAP
static void dump_gwin(int ap_index)
{
	uint32_t win_num;

	/* Dump all GWIN windows */
	printf("GWIN for AP%d (0x%lx)\n\n", ap_index, base);
	printf("win\ttarget\tstart\t\t\tend\n");
	printf("----------------------------------------------------\n");
	for (win_num = 0; win_num < MVEBU_GWIN_MAX_WINS; win_num++) {
		uint32_t cr;
		uint64_t alr, ahr;

		cr  = mmio_read_32(GWIN_CR_OFFSET(ap_index, win_num));
		printf("%02d\t ", win_num);
		/* Window enabled */
		if (cr & WIN_ENABLE_BIT) {
			printf("%02d\t ", (cr >> 8) & 0xF);
			alr = mmio_read_32(GWIN_ALR_OFFSET(ap_index, win_num));
			alr = (alr >> ADDRESS_LSHIFT) << ADDRESS_RSHIFT;
			ahr = mmio_read_32(GWIN_AHR_OFFSET(ap_index, win_num));
			ahr = (ahr >> ADDRESS_LSHIFT) << ADDRESS_RSHIFT;
			printf("0x%016lx 0x%016lx\n", alr, ahr);
		} else
			printf("\t Disabled\n");
	}
	return;
}
#endif

int init_gwin(int ap_index)
{
	struct addr_map_win *win;
	uint32_t win_id;
	uint32_t win_count;
	uint32_t win_reg;

	INFO("Initializing GWIN Address decoding\n");

	/* Get the array of the windows and its size */
	marvell_get_gwin_memory_map(ap_index, &win, &win_count);
	if (win_count <= 0) {
		INFO("no windows configurations found\n");
		return 0;
	}

	if (win_count > MVEBU_GWIN_MAX_WINS) {
		ERROR("number of windows is bigger than %d\n", MVEBU_GWIN_MAX_WINS);
		return 0;
	}

	/* disable all windows */
	for (win_id = 0; win_id < MVEBU_GWIN_MAX_WINS; win_id++)
		gwin_disable_window(ap_index, win_id);

	/* enable relevant windows */
	for (win_id = 0; win_id < win_count; win_id++, win++) {
		gwin_check(win);
		gwin_enable_window(ap_index, win, win_id);
	}

	/* GWIN Miss feature has not verified, therefore any access towards
	 * remote AP should be accompanied with proper configuration to
	 * GWIN registers group and therefore the GWIN Miss feature
	 * should be set into Bypass mode, need to make sure all GWIN regions
	 * are defined correctly that will assure no GWIN miss occurrance
	 * JIRA-AURORA2-1630
	 */
	INFO("Update GWIN miss bypass\n");
	win_reg = mmio_read_32(CCU_GRU_CR_OFFSET(ap_index));
	win_reg |= CCR_GRU_CR_GWIN_MBYPASS;
	mmio_write_32(CCU_GRU_CR_OFFSET(ap_index), win_reg);

#ifdef DEBUG_ADDR_MAP
	dump_gwin(ap_index);
#endif

	INFO("Done GWIN Address decoding Initializing\n");

	return 0;
}
