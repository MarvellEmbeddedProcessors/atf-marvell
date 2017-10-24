/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <arch_helpers.h>
#include <platform.h>
#include <plat_def.h>
#include <mmio.h>
#include <ap810_setup.h>
#include <io_win.h>
#include <ccu.h>
#include <cache_llc.h>
#include <debug.h>
#include <types.h>

#define AP810_MAX_AP_NUM			4
#define AP810_MAX_AP_MASK			0xf

#define SMMU_S_ACR(ap)				(MVEBU_SMMU_BASE(ap) + 0x10)
#define SMMU_S_ACR_PG_64K			(1 << 16)

#define MVEBU_CCU_GSPMU_CR(ap)			(MVEBU_CCU_LOCL_CNTL_BASE(ap) + 0x3F0)
#define GSPMU_CPU_CONTROL			(0x1 << 0)

#define CCU_HTC_CR(ap)				(MVEBU_CCU_BASE(ap) + 0x200)
#define CCU_SET_POC_OFFSET			5

/* SYSRST_OUTn Config definitions */
#define MVEBU_SYSRST_OUT_CONFIG_REG(ap)		(MVEBU_AP_MISC_SOC_BASE(ap) + 0x4)
#define WD_MASK_SYS_RST_OUT			(1 << 2)

/* Used for Units of AP-810 (e.g. SDIO and etc) */
enum axi_attr {
	AXI_SDIO_ATTR = 0,
	AXI_DFX_ATTR,
	AXI_EIP197_ATTR,
	AXI_MAX_ATTR,
};

/* Global AP count */
int g_ap_count = -1;

/* Get AP count, by read how many coherent
 * ports connected to AP0. For now assume
 * that AP0 is connected to all the APs in the system
 */
int get_ap_count(void)
{
	uint32_t reg;
	int count;

	if (g_ap_count != -1)
		return g_ap_count;

	count = 1; /* start with the local AP */
	reg = mmio_read_32(MVEBU_DFX_SAR_REG(0, 0));
	reg = (reg >> MVEBU_SAR_0_COHERENT_EN_OFFSET) & MVEBU_SAR_0_COHERENT_EN_MASK;

	/* Count the coherent ports that enabled */
	while (reg) {
		count += reg & 1;
		reg >>= 1;
	}

	g_ap_count = count;
	INFO("Found %d APs\n", g_ap_count);
	return g_ap_count;
}

static void ap810_enumeration_algo(void)
{
	INFO("place holder to implement %s\n", __func__);
}

static void ap810_dvm_affinity(int ap_id)
{
	INFO("place holder to implement %s\n", __func__);
}

static void ap810_init_aurora2(int ap_id)
{
	unsigned int reg;

	/* Enable CPU control over SPMU registers */
	reg = mmio_read_32(MVEBU_CCU_GSPMU_CR(ap_id));
	reg |= GSPMU_CPU_CONTROL;
	mmio_write_32(MVEBU_CCU_GSPMU_CR(ap_id), reg);

#if !LLC_DISABLE
	/* Enable LLC in exclusive mode */
	llc_enable(ap_id, 1);
#endif /* !LLC_DISABLE */

	/* Set point of coherency to DDR. This is
	 * required by units which have SW cache coherency
	 */
	reg = mmio_read_32(CCU_HTC_CR(ap_id));
	reg |= (0x1 << CCU_SET_POC_OFFSET);
	mmio_write_32(CCU_HTC_CR(ap_id), reg);

	ap810_dvm_affinity(ap_id);
}

static void ap810_setup_smmu(int ap)
{
	uint32_t reg;

	/* Set the SMMU page size to 64 KB */
	reg = mmio_read_32(SMMU_S_ACR(ap));
	reg |= SMMU_S_ACR_PG_64K;
	mmio_write_32(SMMU_S_ACR(ap), reg);
}

static void ap810_sec_masters_access_en(int ap, uint32_t enable)
{
	INFO("place holder to implement %s\n", __func__);
}

static void ap810_axi_attr_init(int ap)
{
	uint32_t index, data;

	/* Initialize AXI attributes for AP810
	 * Go over the AXI attributes and set
	 * Ax-Cache and Ax-Domain
	 */
	for (index = 0; index < AXI_MAX_ATTR; index++) {
		switch (index) {
		/* DFX works with no coherent only -
		 * there's no option to configure the
		 * Ax-Cache and Ax-Domain
		 */
		case AXI_DFX_ATTR:
			continue;
		default:
			/* Set Ax-Cache as cacheable, no allocate, modifiable,
			 * bufferable the values are different because Read & Write
			 * definition is different in Ax-Cache
			 */
			data = mmio_read_32(MVEBU_AP_AXI_ATTR_REG(ap, index));
			data &= ~MVEBU_AXI_ATTR_ARCACHE_MASK;
			data |= (CACHE_ATTR_WRITE_ALLOC | CACHE_ATTR_CACHEABLE | CACHE_ATTR_BUFFERABLE)
				<< MVEBU_AXI_ATTR_ARCACHE_OFFSET;
			data &= ~MVEBU_AXI_ATTR_AWCACHE_MASK;
			data |= (CACHE_ATTR_READ_ALLOC | CACHE_ATTR_CACHEABLE | CACHE_ATTR_BUFFERABLE)
				<< MVEBU_AXI_ATTR_AWCACHE_OFFSET;
			/* Set Ax-Domain as Outer domain */
			data &= ~MVEBU_AXI_ATTR_ARDOMAIN_MASK;
			data |= DOMAIN_OUTER_SHAREABLE << MVEBU_AXI_ATTR_ARDOMAIN_OFFSET;
			data &= ~MVEBU_AXI_ATTR_AWDOMAIN_MASK;
			data |= DOMAIN_OUTER_SHAREABLE << MVEBU_AXI_ATTR_AWDOMAIN_OFFSET;
			mmio_write_32(MVEBU_AP_AXI_ATTR_REG(ap, index), data);
		}
	}

	return;
}

void ap810_setup_events(int ap_id)
{
	INFO("place holder to implement %s\n", __func__);
}

static void ap810_stream_id_init(int ap_id)
{
	INFO("place holder to implement %s\n", __func__);
}

static void ap810_soc_misc_configurations(int ap)
{
	uint32_t reg;

	/* Un-mask Watchdog reset from influencing the SYSRST_OUTn.
	 * Otherwise, upon WD timeout, the WD reset singal won't trigger reset
	 */
	reg = mmio_read_32(MVEBU_SYSRST_OUT_CONFIG_REG(ap));
	reg &= ~(WD_MASK_SYS_RST_OUT);
	mmio_write_32(MVEBU_SYSRST_OUT_CONFIG_REG(ap), reg);
}

void ap810_generic_timer_init(void)
{
	INFO("place holder to implement %s\n", __func__);
}

void ap810_init(void)
{
	int ap_id;

	ap810_enumeration_algo();

	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		/* Setup Aurora2. */
		ap810_init_aurora2(ap_id);
		/* configure RFU windows */
		init_io_win(ap_id);
		/* configure CCU windows */
		init_ccu(ap_id);
		/* configure the SMMU */
		ap810_setup_smmu(ap_id);
		/* Open AP incoming access for all masters */
		ap810_sec_masters_access_en(ap_id, 1);
		/* configure axi for AP */
		ap810_axi_attr_init(ap_id);
		/* Setup events */
		ap810_setup_events(ap_id);
		/* Setup stream-id */
		ap810_stream_id_init(ap_id);
		/* misc configuration of the SoC */
		ap810_soc_misc_configurations(ap_id);
	}

	ap810_generic_timer_init();
}
