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
#define REMAP_ADDR_OFFSET		10
#define REMAP_ADDR_MASK			0xfffff
#define REMAP_SIZE_OFFSET		20
#define REMAP_SIZE_MASK			0xfff
#define REMAP_ENABLE_MASK		0x1

#define IS_DRAM_TARGET(tgt)		((((tgt) == DRAM_0_TID) || \
					((tgt) == DRAM_1_TID) || \
					((tgt) == RAR_TID)) ? 1 : 0)
#ifdef DEBUG_ADDR_MAP
static void dump_ccu(int ap_index)
{
	uint32_t win_id, win_cr, alr, ahr;
	uint8_t target_id;
	uint64_t start, end;

	/* Dump all AP windows */
	printf("bank  id target   start		     end\n");
	printf("----------------------------------------------------\n");
	for (win_id = 0; win_id < MVEBU_CCU_MAX_WINS; win_id++) {
		win_cr = mmio_read_32(CCU_WIN_CR_OFFSET(ap_index, win_id));
		printf("Win %d: 0x%x: 0x%x\n", win_id, CCU_WIN_CR_OFFSET(ap_index, win_id), win_cr);
		if (win_cr & WIN_ENABLE_BIT) {
			printf("\tWin %d: Enabled\n", win_id);
			target_id = (win_cr >> CCU_TARGET_ID_OFFSET) & CCU_TARGET_ID_MASK;
			alr = mmio_read_32(CCU_WIN_ALR_OFFSET(ap_index, win_id));
			ahr = mmio_read_32(CCU_WIN_AHR_OFFSET(ap_index, win_id));
			start = ((uint64_t)alr << ADDRESS_SHIFT);
			end = (((uint64_t)ahr + 0x10) << ADDRESS_SHIFT);
			printf("ccu   %02x %x  0x%016lx 0x%016lx\n", win_id, target_id, start, end);
		}
	}
	win_cr = mmio_read_32(CCU_WIN_GCR_OFFSET(ap_index));
	target_id = (win_cr >> CCU_GCR_TARGET_OFFSET) & CCU_GCR_TARGET_MASK;
	printf("ccu   GCR %d - all other transactions\n", target_id);

	return;
}
#endif

void ccu_win_check(struct addr_map_win *win)
{
	/* check if address is aligned to 1M */
	if (IS_NOT_ALIGN(win->base_addr, CCU_WIN_ALIGNMENT)) {
		win->base_addr = ALIGN_UP(win->base_addr, CCU_WIN_ALIGNMENT);
		NOTICE("%s: Align up the base address to 0x%lx\n", __func__, win->base_addr);
	}

	/* size parameter validity check */
	if (IS_NOT_ALIGN(win->win_size, CCU_WIN_ALIGNMENT)) {
		win->win_size = ALIGN_UP(win->win_size, CCU_WIN_ALIGNMENT);
		NOTICE("%s: Aligning size to 0x%lx\n", __func__, win->win_size);
	}
}

void ccu_enable_win(int ap_index, struct addr_map_win *win, uint32_t win_id)
{
	uint32_t ccu_win_reg;
	uint32_t alr, ahr;
	uint64_t end_addr;

	if ((win_id == 0) || (win_id > MVEBU_CCU_MAX_WINS)) {
		ERROR("Enabling wrong CCU window %d!\n", win_id);
		return;
	}

	end_addr = (win->base_addr + win->win_size - 1);
	alr = (uint32_t)((win->base_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);
	ahr = (uint32_t)((end_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);

	mmio_write_32(CCU_WIN_ALR_OFFSET(ap_index, win_id), alr);
	mmio_write_32(CCU_WIN_AHR_OFFSET(ap_index, win_id), ahr);

	ccu_win_reg = WIN_ENABLE_BIT;
	ccu_win_reg |= (win->target_id & CCU_TARGET_ID_MASK) << CCU_TARGET_ID_OFFSET;
	mmio_write_32(CCU_WIN_CR_OFFSET(ap_index, win_id), ccu_win_reg);
}

static void ccu_disable_win(int ap_index, uint32_t win_id)
{
	uint32_t win_reg;

	if ((win_id == 0) || (win_id > MVEBU_CCU_MAX_WINS)) {
		ERROR("Disabling wrong CCU window %d!\n", win_id);
		return;
	}

	win_reg = mmio_read_32(CCU_WIN_CR_OFFSET(ap_index, win_id));
	win_reg &= ~WIN_ENABLE_BIT;
	mmio_write_32(CCU_WIN_CR_OFFSET(ap_index, win_id), win_reg);
}

/* Insert/Remove temporary window for using the out-of reset default
 * CPx base address to access the CP configuration space prior to
 * the further base address update in accordance with address mapping
 * design.
 *
 * NOTE: Use the same window array for insertion and removal of
 *       temporary windows.
 */
void ccu_temp_win_insert(int ap_index, struct addr_map_win *win, int size)
{
	uint32_t win_id;

	for (int i = 0; i < size; i++) {
		win_id = MVEBU_CCU_MAX_WINS - 1 - i;
		ccu_win_check(win);
		ccu_enable_win(ap_index, win, win_id);
		win++;
	}
}

/*
 * NOTE: Use the same window array for insertion and removal of
 *       temporary windows.
 */
void ccu_temp_win_remove(int ap_index, struct addr_map_win *win, int size)
{
	uint32_t win_id;

	for (int i = 0; i < size; i++) {
		uint64_t base;
		uint32_t target;
		win_id = MVEBU_CCU_MAX_WINS - 1 - i;

		target = mmio_read_32(CCU_WIN_CR_OFFSET(ap_index, win_id));
		target >>= CCU_TARGET_ID_OFFSET;
		target &= CCU_TARGET_ID_MASK;

		base = mmio_read_32(CCU_WIN_ALR_OFFSET(ap_index, win_id));
		base <<= ADDRESS_SHIFT;

		if ((win->target_id != target) || (win->base_addr != base)) {
			ERROR("%s: Trying to remove bad window-%d!\n", __func__, win_id);
			continue;
		}
		ccu_disable_win(ap_index, win_id);
		win++;
	}
}

/* Returns current DRAM window target (DRAM_0_TID, DRAM_1_TID, RAR_TID)
 * NOTE: Call only once for each AP.
 * The AP0 DRAM window is located at index 2 only at the BL31 execution start.
 * Then it relocated to index 1 for matching the rest of APs DRAM settings.
 * Calling this function after relocation will produce wrong results on AP0
 */
static uint32_t ccu_dram_target_get(int ap_index)
{
#if PALLADIUM
	/* no BLE for Palladium */
	return DRAM_0_TID;
#endif

	/* On BLE stage the AP0 DRAM window is opened by the BootROM at index 2.
	 * All the rest of detected APs will use window at index 1.
	 * The AP0 DRAM window is moved from index 2 to 1 during init_ccu() execution.
	 */
	const uint32_t win_id = (ap_index == 0) ? 2 : 1;
	uint32_t target;

	target = mmio_read_32(CCU_WIN_CR_OFFSET(ap_index, win_id));
	target >>= CCU_TARGET_ID_OFFSET;
	target &= CCU_TARGET_ID_MASK;

	return target;
}

void ccu_dram_target_set(int ap_index, uint32_t target)
{
	/* On BLE stage the AP0 DRAM window is opened by the BootROM at index 2.
	 * All the rest of detected APs will use window at index 1.
	 * The AP0 DRAM window is moved from index 2 to 1 during init_ccu() execution.
	 */
	const uint32_t win_id = (ap_index == 0) ? 2 : 1;
	uint32_t dram_cr;

	dram_cr = mmio_read_32(CCU_WIN_CR_OFFSET(ap_index, win_id));
	dram_cr &= ~(CCU_TARGET_ID_MASK << CCU_TARGET_ID_OFFSET);
	dram_cr |= (target & CCU_TARGET_ID_MASK) << CCU_TARGET_ID_OFFSET;
	mmio_write_32(CCU_WIN_CR_OFFSET(ap_index, win_id), dram_cr);
}

/* Setup CCU DRAM window and enable it */
void ccu_dram_win_config(int ap_index, struct addr_map_win *win)
{
#if IMAGE_BLE /* BLE */
	/* On BLE stage the AP0 DRAM window is opened by the BootROM at index 2.
	 * Since the BootROM is not accessing DRAM at BLE stage,
	 * the DRAM window can be temporarely disabled.
	 */
	const uint32_t win_id = (ap_index == 0) ? 2 : 1;
#else /* end of BLE */
	/* At the ccu_init() execution stage, DRAM windows of all APs are arranged at index 1.
	 * The AP0 still has the old window BootROM DRAM at index 2, so the window-1 can be safely
	 * disabled without breaking the DRAM access.
	 */
	const uint32_t win_id = 1;
#endif

	ccu_disable_win(ap_index, win_id);
	/* enable write secure (and clear read secure) */
	mmio_write_32(CCU_WIN_SCR_OFFSET(ap_index, win_id), CCU_WIN_ENA_WRITE_SECURE);
	ccu_win_check(win);
	ccu_enable_win(ap_index, win, win_id);

	return;
}

/* Remap Physical address range to Memory Controller addrress range (PA->MCA) */
void ccu_dram_mca_remap(int ap_index, int dram_tgt, uint64_t from, uint64_t to, uint64_t size)
{
	uint8_t dram_if, if_start, if_stop;

	switch (dram_tgt) {
	case RAR_TID:
		if_start = 0;
		if_stop = 1;
		break;
	case DRAM_0_TID:
		if_start = 0;
		if_stop = 0;
		break;
	case DRAM_1_TID:
		if_start = 1;
		if_stop = 1;
		break;
	default:
		/* Bad target */
		ERROR("Invalid remap target %x\n", dram_tgt);
		return;
	}

	/* Size should be non-zero, up to 4GB and multiple of 1MB */
	if (!size || (size >> 32) || (size % (1 << 20))) {
		ERROR("Invalid remap size %lx\n", size);
		return;
	}

	/* Remap addresses must be multiple of remap size */
	if ((from % size) || (to % size)) {
		ERROR("Invalid remap address %lx -> %lx\n", from, to);
		return;
	}

	from >>= 20; /* bit[39:20] */
	to   >>= 20; /* bit[39:20] */
	size >>= 20; /* Size is in 1MB chunks */
	for (dram_if = if_start; dram_if <= if_stop; dram_if++) {
		uint32_t val;
		/* set mc remap source base to the top of dram */
		val = (from & REMAP_ADDR_MASK) << REMAP_ADDR_OFFSET;
		mmio_write_32(CCU_MC_RSBR_OFFSET(ap_index, dram_if), val);

		/* set mc remap target base to the overlapped dram region */
		val = (to & REMAP_ADDR_MASK) << REMAP_ADDR_OFFSET;
		mmio_write_32(CCU_MC_RTBR_OFFSET(ap_index, dram_if), val);

		/* set mc remap size to the size of the overlapped dram region */
		/* up to 4GB region for remapping */
		val = ((size - 1) & REMAP_SIZE_MASK) << REMAP_SIZE_OFFSET;
		/* enable remapping */
		val |= REMAP_ENABLE_MASK;
		mmio_write_32(CCU_MC_RCR_OFFSET(ap_index, dram_if), val);
	}
}

int init_ccu(int ap_index)
{
	struct addr_map_win *win, *dram_win;
	uint32_t win_id, win_reg;
	uint32_t win_count, array_id;
	uint32_t dram_target;

	INFO("Initializing CCU Address decoding\n");

	/* Get the array of the windows and fill the map data */
	marvell_get_ccu_memory_map(ap_index, &win, &win_count);
	if (win_count <= 0) {
		INFO("No windows configurations found\n");
	} else if (win_count > (MVEBU_CCU_MAX_WINS - 1)) {
		ERROR("CCU memory map array greater than max available windows, set win_count to max %d\n",
				MVEBU_CCU_MAX_WINS);
		win_count = MVEBU_CCU_MAX_WINS;
	}

	/* Need to set GCR to DRAM before all CCU windows are disabled for securing the normal access
	 * to DRAM location, which the ATF is running from. Once all CCU windows are set, which have to
	 * include the dedicated DRAM window as well, the GCR can be switched to the target defined
	 * by the platform configuration.
	 */
	dram_target = ccu_dram_target_get(ap_index);
	win_reg = (dram_target & CCU_GCR_TARGET_MASK) << CCU_GCR_TARGET_OFFSET;
	mmio_write_32(CCU_WIN_GCR_OFFSET(ap_index), win_reg);

	/* If the DRAM window was already configured at the BLE stage,
	 * only the window target considered valid, the address range should be updated
	 * according to the platform configuration.
	 */
	for (dram_win = win, array_id = 0; array_id < win_count; array_id++, dram_win++) {
		if (IS_DRAM_TARGET(dram_win->target_id)) {
			dram_win->target_id = dram_target;
			break;
		}
	}

	/* Disable all AP CCU windows
	 * Window-0 is always bypassed since it already contains
	 * data allowing the internal configuration space access
	 */
	for (win_id = 1; win_id < MVEBU_CCU_MAX_WINS; win_id++) {
		ccu_disable_win(ap_index, win_id);
		/* enable write secure (and clear read secure) */
		mmio_write_32(CCU_WIN_SCR_OFFSET(ap_index, win_id), CCU_WIN_ENA_WRITE_SECURE);
	}

	/* win_id is the index of the current ccu window
	 * array_id is the index of the current memory map window entry
	 */
	for (win_id = 1, array_id = 0;
	    ((win_id < MVEBU_CCU_MAX_WINS) && (array_id < win_count)); win_id++) {
		ccu_win_check(win);
		ccu_enable_win(ap_index, win, win_id);
		win++;
		array_id++;
	}

	/* Get & set the default target according to board topology */
	win_reg = (marvell_get_ccu_gcr_target(ap_index) & CCU_GCR_TARGET_MASK) << CCU_GCR_TARGET_OFFSET;
	mmio_write_32(CCU_WIN_GCR_OFFSET(ap_index), win_reg);

#ifdef DEBUG_ADDR_MAP
	dump_ccu(ap_index);
#endif

	INFO("Done CCU Address decoding Initializing\n");

	return 0;
}
