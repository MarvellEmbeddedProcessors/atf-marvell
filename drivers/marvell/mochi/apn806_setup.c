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
#include <apn806_setup.h>
#include <rfu.h>
#include <ccu.h>
#include <mci.h>
#include <cache_llc.h>
#include <debug.h>

#define SMMU_sACR				(MVEBU_SMMU_BASE + 0x10)
#define SMMU_sACR_PG_64K			(1 << 16)

#define CCU_GSPMU_CR				(MVEBU_CCU_BASE + 0x3F0)
#define GSPMU_CPU_CONTROL			(0x1 << 0)

#define CCU_HTC_CR				(MVEBU_CCU_BASE + 0x200)
#define CCU_SET_POC_OFFSET			5

#define CCU_LTC_CR				(MVEBU_CCU_BASE + 0x300)
#define CCU_CLEAN_INV_WRITE_OFFSET		8

#define DSS_CR0					(MVEBU_RFU_BASE + 0x100)
#define DVM_48BIT_VA_ENABLE			(1 << 21)

/* Secure MoChi incoming access */
#define SEC_MOCHI_IN_ACC_REG			(MVEBU_RFU_BASE + 0x4738)
#define SEC_MOCHI_IN_ACC_IHB0_EN		(1)
#define SEC_MOCHI_IN_ACC_IHB1_EN		(1 << 3)
#define SEC_MOCHI_IN_ACC_IHB2_EN		(1 << 6)
#define SEC_MOCHI_IN_ACC_PIDI_EN		(1 << 9)
#define SEC_IN_ACCESS_ENA_ALL_MASTERS		(SEC_MOCHI_IN_ACC_IHB0_EN | \
						 SEC_MOCHI_IN_ACC_IHB1_EN | \
						 SEC_MOCHI_IN_ACC_IHB2_EN | \
						 SEC_MOCHI_IN_ACC_PIDI_EN)

/* Misc SoC configurations Base*/
#define MVEBU_MISC_SOC_BASE			(MVEBU_REGS_BASE + 0x6F4300)

/* SYSRST_OUTn Config definitions */
#define MVEBU_SYSRST_OUT_CONFIG_REG		(MVEBU_MISC_SOC_BASE + 0x4)
#define WD_MASK_SYS_RST_OUT			(1 << 2)

/*
 * AXI Configuration.
 */

/* Used for Units of AP-806 (e.g. SDIO and etc) */
#define MVEBU_AXI_ATTR_BASE			(MVEBU_REGS_BASE + 0x6F4580)
#define MVEBU_AXI_ATTR_REG(index)		(MVEBU_AXI_ATTR_BASE + 0x4 * index)

enum axi_attr {
	AXI_SDIO_ATTR = 0,
	AXI_DFX_ATTR,
	AXI_MAX_ATTR,
};

uint32_t apn806_sar_get_bootsrc()
{
	uint32_t reg;

	reg = mmio_read_32(AP806_SAR0_REG_BASE);
	return (reg >> AP806_SAR0_BOOT_SOURCE_OFFSET) & AP806_SAR0_BOOT_SOURCE_MASK;
}

static void apn_sec_masters_access_en(uint32_t enable)
{
	uint32_t reg;

	/* Open/Close incoming access for all masters.
	   The access is disabled in trusted boot mode
	   Could only be done in EL3
	 */
	reg = mmio_read_32(SEC_MOCHI_IN_ACC_REG);
	if (enable)
		mmio_write_32(SEC_MOCHI_IN_ACC_REG, reg | SEC_IN_ACCESS_ENA_ALL_MASTERS);
	else
		mmio_write_32(SEC_MOCHI_IN_ACC_REG, reg & ~SEC_IN_ACCESS_ENA_ALL_MASTERS);
}

void setup_smmu(void)
{
	uint32_t reg;

	/* Set the SMMU page size to 64 KB */
	reg = mmio_read_32(SMMU_sACR);
	reg |= SMMU_sACR_PG_64K;
	mmio_write_32(SMMU_sACR, reg);
}

void init_aurora2(void)
{
	uint32_t reg;

	/* Enable GSPMU control by CPU */
	reg = mmio_read_32(CCU_GSPMU_CR);
	reg |= GSPMU_CPU_CONTROL;
	mmio_write_32(CCU_GSPMU_CR, reg);

#if !LLC_DISABLE
	/* Enable LLC in exclusive mode */
	llc_enable(1);

	/* Set point of coherency to DDR.
	   This is required by units which have
	   SW cache coherency */
	reg = mmio_read_32(CCU_HTC_CR);
	reg |= (0x1 << CCU_SET_POC_OFFSET);
	mmio_write_32(CCU_HTC_CR, reg);

	/* A0 Only: cache line clean & invalidate instead of)
	** cache line invalidate only - to avoid system hang
	** due to memory coherency issue */
	if (apn806_rev_id_get() == APN806_REV_ID_A0) {
		reg = mmio_read_32(CCU_LTC_CR);
		reg |= (0x1 << CCU_CLEAN_INV_WRITE_OFFSET);
		mmio_write_32(CCU_LTC_CR, reg);
	}
#endif /* !LLC_DISABLE */
}

void apn806_axi_attr_init(void)
{
	uint32_t index, data;

	/* Initialize AXI attributes for APN806 */

	/* Go over the AXI attributes and set Ax-Cache and Ax-Domain */
	for (index = 0; index < AXI_MAX_ATTR; index++) {
		switch (index) {
		/* DFX works with no coherent only -
		   there's no option to configure the Ax-Cache and Ax-Domain */
		case AXI_DFX_ATTR:
			continue;
		default:
			/* Set Ax-Cache as cacheable, no allocate, modifiable, bufferable
			 The values are different because Read & Write definition
			 is different in Ax-Cache */
			data = mmio_read_32(MVEBU_AXI_ATTR_REG(index));
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
			mmio_write_32(MVEBU_AXI_ATTR_REG(index), data);
		}
	}

	return;
}

void dss_setup(void)
{
	/* Enable 48-bit VA */
	mmio_setbits_32(DSS_CR0, DVM_48BIT_VA_ENABLE);
}

void misc_soc_configurations(void)
{
	uint32_t reg;

	/* Un-mask Watchdog reset from influencing the SYSRST_OUTn.
	 * Otherwise, upon WD timeout, the WD reset singal won't trigger reset
	 */
	reg = mmio_read_32(MVEBU_SYSRST_OUT_CONFIG_REG);
	reg &= ~(WD_MASK_SYS_RST_OUT);
	mmio_write_32(MVEBU_SYSRST_OUT_CONFIG_REG, reg);
}

void apn806_init(void)
{
	/* Setup Aurora2. */
	init_aurora2();

	/* configure MCI mapping */
	mci_remap_indirect_access_base();

	/* configure RFU windows */
	init_rfu();

	/* configure CCU windows */
	init_ccu();

	/* configure DSS */
	dss_setup();

	/* configure the SMMU */
	setup_smmu();

	/* Open APN incoming access for all masters */
	apn_sec_masters_access_en(1);

	/* configure axi for APN*/
	apn806_axi_attr_init();

	/* misc configuration of the SoC */
	misc_soc_configurations();
}
