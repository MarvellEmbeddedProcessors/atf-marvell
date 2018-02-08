/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <ap810_setup.h>
#include <a8kp_plat_def.h>
#include <debug.h>
#include <addr_map.h>
#include <ccu.h>
#include <gwin.h>
#include <plat_marvell.h>
#include <plat_def.h>
#include <plat_dram.h>
#include <mv_ddr_if.h>
#include <mvebu.h>
#include <ap810_init_clocks.h>

/* The whole DRAM is mapped to the first 512GB of the address space strarting 0x0
 * For setups with up to 2 interconnected APs each AP maps 256GB of the physical DRAM
 * When 3 or 4 APs are connected together, each AP maps 128GB pf physical DRAM.
 */
#define AP_DRAM_SIZE(ap_cnt)		((ap_cnt) < 3 ? (256 * _1GB_) : (128 * _1GB_))
#define AP_DRAM_BASE_ADDR(ap, ap_cnt)	((ap) * AP_DRAM_SIZE(ap_cnt))
#define AP0_BOOTROM_DRAM_SIZE		0xec000000

static int ble_dram_config(void)
{
	const int ap_cnt = get_ap_count();
	int  iface_id, iface_cnt, iface_mode, ap_id, ap_dram_tgt;
	uint64_t ap_dram_size, iface_size[DDR_MAX_UNIT_PER_AP];
	struct addr_map_win gwin_temp_win, ccu_dram_win;

	/* Walk through interconnected APs for the attached memory
	 * detection and configuration
	 */
	for (ap_id = 0; ap_id < ap_cnt; ap_id++) {
		/* Test every interface of the AP memory controller
		 * for a DIMM presence and its size
		 */
		for (iface_id = 0, iface_cnt = 0; iface_id < DDR_MAX_UNIT_PER_AP; iface_id++) {
			iface_size[iface_id] = ap_dram_iface_info_get(ap_id, iface_id);
			if (iface_size[iface_id])
				iface_cnt++;
		}

		/* Bypass the rest if current AP has no atatched DRAM */
		if (!iface_cnt)
			continue;

		/* Add a single GWIN entry to AP0 for enabling remote APs access
		 * There is no need to open GWIN on other APs, since only AP0
		 * is involved at this stage.
		 */
		if (ap_id != 0) {
			gwin_temp_win.base_addr = AP_DRAM_BASE_ADDR(ap_id, ap_cnt);
			gwin_temp_win.win_size = AP_DRAM_SIZE(ap_cnt);
			gwin_temp_win.target_id = ap_id;
			gwin_temp_win_insert(0, &gwin_temp_win, 1);
		}

		/* Add CCU window for DRAM access:
		 * Single DIMM on this AP, CCU target = DRAM 0/1
		 * Multiple DIMMs on this AP, CCU target = RAR
		 * The RAR target allows access to both DRAM interfaces
		 * in parallel, increasing the total memory bandwidth.
		 */
		if (iface_cnt == 1) {
			if (iface_size[0]) {
				ap_dram_tgt = DRAM_0_TID;
				iface_mode = SINGLE_DRAM_0;
				ap_dram_size = iface_size[0];
			} else {
				ap_dram_tgt = DRAM_1_TID;
				iface_mode = SINGLE_DRAM_1;
				ap_dram_size = iface_size[1];
			}
		} else {
			ap_dram_tgt = RAR_TID;
			iface_mode = DUAL_DRAM;
			ap_dram_size = iface_size[0] + iface_size[1];
		}
		ccu_dram_win.base_addr = AP_DRAM_BASE_ADDR(ap_id, ap_cnt);
		ccu_dram_win.win_size = AP_DRAM_SIZE(ap_cnt);
		ccu_dram_win.target_id = ap_dram_tgt;

		/* Create a memory window with the approriate target in CCU */
		ccu_dram_win_config(ap_id, &ccu_dram_win);

		/* Do the proper memory controller configuration for the choosen interface mode */
		dram_mmap_config(ap_id, iface_mode, AP_DRAM_BASE_ADDR(ap_id, ap_cnt), ap_dram_size);

		/* Remap the physical memory shadowed by the internal registers configration
		 * address space to the top of the detected memory area.
		 * Regardless the fact that all APs are reserving extra 1GB for this purpose,
		 * only the AP0 overlaps this configuration area with the DRAM, so only its memory
		 * controller has to remap the overlapped region to the upper memory.
		 * With less than 3GB of DRAM the internal registers space remapping is not needed
		 * since there is no overlap between DRAM and the configuration address spaces
		 */
		if ((ap_id == 0)  && (ap_dram_size > (3 * _1GB_)))
			ccu_dram_mca_remap(0, ap_dram_tgt, ap_dram_size, 3 * _1GB_, _1GB_);

		/* Scrub the DRAM for ECC support */
		dram_scrubbing(ap_id, AP_DRAM_BASE_ADDR(ap_id, ap_cnt), ap_dram_size);

		/* Restore the original DRAM size on AP0 before returning to the BootROM.
		 * Access to entire DRAM is required only during DDR initialization and scrubbing.
		 * The correct DRAM size will be set back by ccu_init() at BL31 stage.
		 */
		if (ap_id == 0) {
			ccu_dram_win.win_size = AP0_BOOTROM_DRAM_SIZE;
			ccu_dram_win_config(0, &ccu_dram_win);
		} else {
			/* Remove the earlier configured GWIN entry from AP0 */
			gwin_temp_win_remove(0, &gwin_temp_win, 1);
		}
	} /* for every inerconnected AP */

	return 0;
}


/* Read Frequency Value from MPPS 15-17 and save
 * to scratch-pad Register as a temporary solution
 * in AP810 A0 revision to cover the bug in sampled-at-reset
 * register.
*/
static void ble_read_cpu_freq(void)
{
	unsigned int mpp_address, val;

	if (ap810_rev_id_get(0))
		return;

	/* TODO: add errata for this WA, we can't read from sample at reset
	 * register.
	 */
	mpp_address = MVEBU_AP_GPIO_DATA_IN(0);
	val = mmio_read_32(mpp_address);
	val = (val >> 15) & 0x7;
	INFO("sar option read from MPPs = 0x%x\n", val);
	mmio_write_32(SCRATCH_PAD_ADDR(0, 1), val);
}

int ble_plat_setup(int *skip)
{
	int ret = 0;

#if !PALLADIUM
	/* SW WA for AP link bring-up over JTAG connection */
	if ((get_ap_count() != 1) &&
	    (ap810_rev_id_get(0) == 0))
		jtag_init_ihb_dual_ap();
#endif

	ble_read_cpu_freq();

	ap810_ble_init();

	/* init clocks for single AP */
	ap810_clocks_init(get_ap_count());

	/* TODO: need to check if need early cpu powerdown */

	/* TODO: check if recovery feature is needed */

	/* TODO: check if SVC is needed */

	/* Trigger DRAM driver initialization */
	ret = plat_dram_init();
	if (!ret)
		ret = ble_dram_config();

	return ret;
}
