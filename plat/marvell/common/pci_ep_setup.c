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

#include <arch_helpers.h>
#include <plat_marvell.h>
#include <debug.h>
#include <mmio.h>
#include <pci_ep.h>

/* Set a weak stub for platforms that don't use PCIe end point */
#pragma weak plat_get_pcie_hw_data
struct pci_hw_cfg *plat_get_pcie_hw_data(void)
{
	return 0;
}

void ble_plat_pcie_ep_setup(void)
{
	int ret, lane = 0;
	struct pci_hw_cfg *hw_cfg;

	hw_cfg = (struct pci_hw_cfg *)plat_get_pcie_hw_data();
	if (!hw_cfg)
		return;

	INFO("Setting up PCI as end point\n");

	/* First setup all COMPHY lanes */
	while (lane < hw_cfg->lane_width) {
		ret = comphy_pcie_power_up(hw_cfg->lane_ids[lane], hw_cfg);
		if (ret == 0) {
			ERROR(" Failed to setup SERDES lane %d\n",
			      hw_cfg->lane_ids[lane]);
			return;
		}
		lane++;
	}

	/* Now setup the MAC */
	dw_pcie_ep_init(hw_cfg->mac_base, hw_cfg->delay_cfg, hw_cfg->master_en);
}
