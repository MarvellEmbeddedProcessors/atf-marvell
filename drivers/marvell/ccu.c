/*
* ***************************************************************************
* Copyright (C) 2016 Marvell International Ltd.
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
#include <ccu.h>
#include <plat_config.h>

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define DEBUG_ADDR_MAP
#endif

/* common defines */
#define WIN_ENABLE_BIT			(0x1)
/* Physical address of the base of the window = {AddrLow[19:0],20’h0} */
#define ADDRESS_SHIFT			(20 - 4)
#define ADDRESS_MASK			(0xFFFFFFF0)
#define CCU_WIN_ALIGNMENT		(0x100000)

/* AP registers */
#define CCU_WIN_CR_OFFSET(win)		(ccu_base + 0x0 + (0x10 * win))
#define CCU_TARGET_ID_OFFSET		(8)
#define CCU_TARGET_ID_MASK		(0x7F)

#define CCU_WIN_SCR_OFFSET(win)		(ccu_base + 0x4 + (0x10 * win))
#define CCU_WIN_ENA_WRITE_SECURE	(0x1)
#define CCU_WIN_ENA_READ_SECURE		(0x2)

#define CCU_WIN_ALR_OFFSET(win)		(ccu_base + 0x8 + (0x10 * win))
#define CCU_WIN_AHR_OFFSET(win)		(ccu_base + 0xC + (0x10 * win))

#define CCU_WIN_GCR_OFFSET		(ccu_base + 0xD0)
#define CCU_GCR_TARGET_OFFSET		(8)
#define CCU_GCR_TARGET_MASK		(0xF)

uintptr_t ccu_base;

#ifdef DEBUG_ADDR_MAP
struct ccu_target_name_map {
	enum ccu_target_ids trgt_id;
	char name[10];
};

struct ccu_target_name_map ccu_target_name_table[] = {
	{IO_0_TID,	"IO-0	"},
	{DRAM_0_TID,	"DRAM-0 "},
	{IO_1_TID,	"IO-1	"},
	{CFG_REG_TID,	"CFG-REG"},
	{RAR_TID,	"RAR	"},
	{SRAM_TID,	"SRAM	"},
	{DRAM_1_TID,	"DRAM-1 "},
	{INVALID_TID,	"INVALID"},
};

static char *ccu_target_name_get(enum ccu_target_ids trgt_id)
{
	int i;

	for (i = 0; i < CCU_MAX_TID; i++)
		if (ccu_target_name_table[i].trgt_id == trgt_id)
			return ccu_target_name_table[i].name;
	return ccu_target_name_get(INVALID_TID);
}

static void dump_ccu(void)
{
	uint32_t win_id, win_cr, alr, ahr;
	uint8_t target_id;
	uint64_t start, end;

	/* Dump all AP windows */
	printf("bank  id target   start		     end\n");
	printf("----------------------------------------------------\n");
	for (win_id = 0; win_id < MVEBU_CCU_MAX_WINS; win_id++) {
		win_cr = mmio_read_32(CCU_WIN_CR_OFFSET(win_id));
		printf("Win %d: 0x%lx: 0x%x\n", win_id, CCU_WIN_CR_OFFSET(win_id), win_cr);
		if (win_cr & WIN_ENABLE_BIT) {
			printf("\tWin %d: Enabled\n", win_id);
			target_id = (win_cr >> CCU_TARGET_ID_OFFSET) & CCU_TARGET_ID_MASK;
			alr = mmio_read_32(CCU_WIN_ALR_OFFSET(win_id));
			ahr = mmio_read_32(CCU_WIN_AHR_OFFSET(win_id));
			start = ((uint64_t)alr << ADDRESS_SHIFT);
			end = (((uint64_t)ahr + 0x10) << ADDRESS_SHIFT);
			printf("ccu   %02x %s  0x%016lx 0x%016lx\n"
				, win_id, ccu_target_name_get(target_id), start, end);
		}
	}
	win_cr = mmio_read_32(CCU_WIN_GCR_OFFSET);
	target_id = (win_cr >> CCU_GCR_TARGET_OFFSET) & CCU_GCR_TARGET_MASK;
	printf("ccu   GCR %s - all other transactions\n", ccu_target_name_get(target_id));

	return;
}
#endif

void ccu_win_check(struct addr_map_win *win, uint32_t win_num)
{
	/* check if address is aligned to 1M */
	if (IS_NOT_ALIGN(win->base_addr, CCU_WIN_ALIGNMENT)) {
		win->base_addr = ALIGN_UP(win->base_addr, CCU_WIN_ALIGNMENT);
		ERROR("Window %d: base address unaligned to 0x%x\n", win_num, CCU_WIN_ALIGNMENT);
		printf("Align up the base address to 0x%lx\n", win->base_addr);
	}

	/* size parameter validity check */
	if (IS_NOT_ALIGN(win->win_size, CCU_WIN_ALIGNMENT)) {
		win->win_size = ALIGN_UP(win->win_size, CCU_WIN_ALIGNMENT);
		ERROR("Window %d: window size unaligned to 0x%x\n", win_num, CCU_WIN_ALIGNMENT);
		printf("Aligning size to 0x%lx\n", win->win_size);
	}
}

void ccu_enable_win(struct addr_map_win *win, uint32_t win_id)
{
	uint32_t ccu_win_reg;
	uint32_t alr, ahr;
	uint64_t end_addr;

	end_addr = (win->base_addr + win->win_size - 1);
	alr = (uint32_t)((win->base_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);
	ahr = (uint32_t)((end_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);

	mmio_write_32(CCU_WIN_ALR_OFFSET(win_id), alr);
	mmio_write_32(CCU_WIN_AHR_OFFSET(win_id), ahr);

	ccu_win_reg = WIN_ENABLE_BIT;
	ccu_win_reg |= (win->target_id & CCU_TARGET_ID_MASK) << CCU_TARGET_ID_OFFSET;
	mmio_write_32(CCU_WIN_CR_OFFSET(win_id), ccu_win_reg);
}

int init_ccu(int ap_index)
{
	struct addr_map_win *win;
	uint32_t win_id, win_reg;
	uint32_t win_count, array_id;

	INFO("Initializing CCU Address decoding\n");

	/* Get the base address of the address decoding CCU */
	ccu_base = MVEBU_CCU_BASE(ap_index);

	/* Get the array of the windows and fill the map data */
	marvell_get_ccu_memory_map(ap_index, &win, &win_count);
	if (win_count <= 0) {
		INFO("no windows configurations found\n");
		return 0;
	} else if (win_count > (MVEBU_CCU_MAX_WINS - 1)) {
		ERROR("CCU memory map array greater than max available windows, set win_count to max %d\n",
				MVEBU_CCU_MAX_WINS);
		win_count = MVEBU_CCU_MAX_WINS;
	}

	/* Get & set the default target according board topology */
	win_reg = (marvell_get_ccu_gcr_target(ap_index) & CCU_GCR_TARGET_MASK) << CCU_GCR_TARGET_OFFSET;
	mmio_write_32(CCU_WIN_GCR_OFFSET, win_reg);

	/* disable all AP windows, start from 1 to avoid overriding internal registers */
	for (win_id = 1; win_id < MVEBU_CCU_MAX_WINS; win_id++) {
		win_reg = mmio_read_32(CCU_WIN_CR_OFFSET(win_id));

		win_reg &= ~WIN_ENABLE_BIT;
		mmio_write_32(CCU_WIN_CR_OFFSET(win_id), win_reg);

		/* enable write secure (and clear read secure) */
		win_reg = CCU_WIN_ENA_WRITE_SECURE;
		mmio_write_32(CCU_WIN_SCR_OFFSET(win_id), win_reg);
	}

	/* win_id is the index of the current ccu window
	** array_id is the index of the current memory map window entry */
	for (win_id = 1, array_id = 0;
		  ((win_id < MVEBU_CCU_MAX_WINS) && (array_id < win_count)); win_id++) {
		ccu_win_check(win, win_id);
		ccu_enable_win(win, win_id);

		win++;
		array_id++;
	}

#ifdef DEBUG_ADDR_MAP
	dump_ccu();
#endif

	INFO("Done CCU Address decoding Initializing\n");

	return 0;
}
