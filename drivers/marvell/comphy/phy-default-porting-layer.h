/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __PHY_DEFAULT_PORTING_LAYER_H
#define __PHY_DEFAULT_PORTING_LAYER_H


#define MAX_LANE_NR		6

#warning "Using default comphy parameters - it may be need to suit them for your board"

static const struct xfi_params xfi_static_values_tab[AP_NUM][CP_NUM][MAX_LANE_NR] = {
	[0 ... AP_NUM-1][0 ... CP_NUM-1][0 ... MAX_LANE_NR-1] = {
		.g1_ffe_res_sel = 0x3, .g1_ffe_cap_sel = 0xf, .align90 = 0x5f,
		.g1_dfe_res = 0x2, .g1_amp = 0x1c, .g1_emph = 0xe,
		.g1_emph_en = 1, .g1_tx_amp_adj = 1, .g1_tx_emph_en = 1,
		.g1_tx_emph = 0, .g1_rx_selmuff = 2, .g1_rx_selmufi = 0x3,
		.g1_rx_selmupf = 0x1, .g1_rx_selmupi = 6, .valid = 1
	}
};

static const struct sata_params sata_static_values_tab[AP_NUM][CP_NUM][MAX_LANE_NR] = {
	[0 ... AP_NUM-1][0 ... CP_NUM-1][0 ... MAX_LANE_NR-1] = {
		.g1_amp = 0x8, .g2_amp = 0xa, .g3_amp = 0x1e,
		.g1_emph = 0x1, .g2_emph = 0x2, .g3_emph = 0x6,
		.g1_emph_en = 0x1, .g2_emph_en = 0x1, .g3_emph_en = 0x1,
		.g1_tx_amp_adj = 0x1, .g2_tx_amp_adj = 0x1, .g3_tx_amp_adj = 0x1,
		.g1_tx_emph_en = 0x0, .g2_tx_emph_en = 0x0, .g3_tx_emph_en = 0x0,
		.g1_tx_emph = 0x1, .g2_tx_emph = 0x1, .g3_tx_emph = 0x1,
		.g3_dfe_res = 0x1, .g3_ffe_res_sel = 0x4, .g3_ffe_cap_sel = 0xf,
		.allign_90 = 0x1c,
		.g1_rx_selmuff = 0x3, .g2_rx_selmuff = 0x3, .g3_rx_selmuff = 0x3,
		.g1_rx_selmufi = 0x0, .g2_rx_selmufi = 0x0, .g3_rx_selmufi = 0x3,
		.g1_rx_selmupf = 0x1, .g2_rx_selmupf = 0x1, .g3_rx_selmupf = 0x2,
		.g1_rx_selmupi = 0x0, .g2_rx_selmupi = 0x0, .g3_rx_selmupi = 0x2,
		.valid = 0x1
	},
};
#endif /* __PHY_DEFAULT_PORTING_LAYER_H */
