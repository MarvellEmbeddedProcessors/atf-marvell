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
#include <amb_adec.h>
#include <iob.h>
#include <icu.h>
#include <mmio.h>
#include <delay_timer.h>
#include <cp110_setup.h>

/*
  ICU configuration
 */
/* Multi instance sources, multipllied in dual CP mode */
static const struct icu_irq irq_map_ns_multi[] = {
	{22, 0, 0}, /* PCIx4 INT A interrupt */
	{23, 1, 0}, /* PCIx1 INT A interrupt */
	{24, 2, 0}, /* PCIx1 INT A interrupt */

	{33, 3, 0}, /* PPv2 DBG AXI monitor */
	{34, 3, 0}, /* HB1      AXI monitor */
	{35, 3, 0}, /* AP       AXI monitor */
	{36, 3, 0}, /* PPv2     AXI monitor */

	{38,  4, 0}, /* PPv2 Misc */

	{39,  5, 0}, /* PPv2 irq */
	{40,  6, 0}, /* PPv2 irq */
	{41,  7, 0}, /* PPv2 irq */
	{42,  8, 0}, /* PPv2 irq */
	{43,  9, 0}, /* PPv2 irq */
	{44, 10, 0}, /* PPv2 irq */
	{45, 11, 0}, /* PPv2 irq */
	{46, 12, 0}, /* PPv2 irq */
	{47, 13, 0}, /* PPv2 irq */
	{48, 14, 0}, /* PPv2 irq */
	{49, 15, 0}, /* PPv2 irq */
	{50, 16, 0}, /* PPv2 irq */
	{51, 17, 0}, /* PPv2 irq */
	{52, 18, 0}, /* PPv2 irq */
	{53, 19, 0}, /* PPv2 irq */
	{54, 20, 0}, /* PPv2 irq */

	{78, 21, 0}, /* MG irq */
	{88, 22, 0}, /* EIP-197 ring-0 */
	{89, 23, 0}, /* EIP-197 ring-1 */
	{90, 24, 0}, /* EIP-197 ring-2 */
	{91, 25, 0}, /* EIP-197 ring-3 */
	{92, 26, 0}, /* EIP-197 int */
	{95, 27, 0}, /* EIP-150 irq */
	{102, 28, 0}, /* USB3 Device irq */
	{105, 29, 0}, /* USB3 Host-1 irq */
	{106, 30, 0}, /* USB3 Host-0 irq */
	{107, 31, 0}, /* SATA Host-1 irq */
	{109, 31, 0}, /* SATA Host-0 irq */
	{126, 33, 0}, /* PTP irq */
	{127, 34, 0}, /* GOP-3 irq */
	{128, 35, 0}, /* GOP-2 irq */
	{129, 36, 0}, /* GOP-0 irq */

	/* PPv2 interrupts which originally assigned as single interrupts.
	 * To avoid updating Device tree, left original SPI assignment 60-63,
	 * hence there is a gap between previous multi interrupts */
	{55, 60, 0}, /* PPv2 irq */
	{56, 61, 0}, /* PPv2 irq */
	{57, 62, 0}, /* PPv2 irq */
	{58, 63, 0}, /* PPv2 irq */
};

/* Single instance sources, not multiplies in dual CP mode */
static const struct icu_irq irq_map_ns_single[] = {
	{27, 37, 0}, /* SD/MMC */
	{76, 38, 0}, /* Audio */
	{77, 39, 0}, /* MSS RTC */
	{79, 40, 0}, /* GPIO 56-63 */
	{80, 41, 0}, /* GPIO 48-55 */
	{81, 42, 0}, /* GPIO 40-47 */
	{82, 43, 0}, /* GPIO 32-39 */
	{83, 44, 0}, /* GPIO 24-31 */
	{84, 45, 0}, /* GPIO 16-23 */
	{85, 46, 0}, /* GPIO  8-15 */
	{86, 47, 0}, /* GPIO  0-7  */
	{111, 48, 0}, /* TDM-MC func 1 */
	{112, 49, 0}, /* TDM-MC func 0 */
	{113, 50, 0}, /* TDM-MC irq */
	{115, 51, 0}, /* NAND irq */
	{117, 52, 0}, /* SPI-1 irq */
	{118, 53, 0}, /* SPI-0 irq */
	{120, 54, 0}, /* I2C 0 irq */
	{121, 55, 0}, /* I2C 1 irq */
	{122, 56, 0}, /* UART 0 irq */
	{123, 57, 0}, /* UART 1 irq */
	{124, 58, 0}, /* UART 2 irq */
	{125, 59, 0}, /* UART 3 irq */
};

/* SEI - System Error Interrupts */
/* Note: SPI ID 0-20 are reserved for North-Bridge */
static struct icu_irq irq_map_sei[] = {
	{11, 21, 0}, /* SEI error CP-2-CP */
	{15, 22, 0}, /* PIDI-64 SOC */
	{16, 23, 0}, /* D2D error irq */
	{17, 24, 0}, /* D2D irq */
	{18, 25, 0}, /* NAND error */
	{19, 26, 0}, /* PCIx4 error */
	{20, 27, 0}, /* PCIx1_0 error */
	{21, 28, 0}, /* PCIx1_1 error */
	{25, 29, 0}, /* SDIO reg error */
	{75, 30, 0}, /* IOB error */
	{94, 31, 0}, /* EIP150 error */
	{97, 32, 0}, /* XOR-1 system error */
	{99, 33, 0}, /* XOR-0 system error */
	{108, 34, 0}, /* SATA-1 error */
	{110, 35, 0}, /* SATA-0 error */
	{114, 36, 0}, /* TDM-MC error */
	{116, 37, 0}, /* DFX server irq */
	{117, 38, 0}, /* Device bus error */
	{147, 39, 0}, /* Audio error */
	{171, 40, 0}, /* PIDI Sync error */
};

/* REI - RAM Error Interrupts */
static const struct icu_irq irq_map_rei[] = {
	{12, 0, 0}, /* REI error CP-2-CP */
	{26, 1, 0}, /* SDIO memory error */
	{87, 2, 0}, /* EIP-197 ECC error */
	{93, 3, 1}, /* EIP-150 RAM error */
	{96, 4, 0}, /* XOR-1 memory irq */
	{98, 5, 0}, /* XOR-0 memory irq */
	{100, 6, 1}, /* USB3 device tx parity */
	{101, 7, 1}, /* USB3 device rq parity */
	{103, 8, 1}, /* USB3H-1 RAM error */
	{104, 9, 1}, /* USB3H-0 RAM error */
};

static const struct icu_config icu_config = {
	.ns_multi = { irq_map_ns_multi, ARRAY_SIZE(irq_map_ns_multi) },
	.ns_single = { irq_map_ns_single, ARRAY_SIZE(irq_map_ns_single) },
	.sei = { irq_map_sei, ARRAY_SIZE(irq_map_sei) },
	.rei = { irq_map_rei, ARRAY_SIZE(irq_map_rei) },
};

/*
 * AXI Configuration.
 */

 /* Used for Units of CP-110 (e.g. USB device, USB Host, and etc) */
#define MVEBU_AXI_ATTR_BASE(cp_index)		(MVEBU_CP_REGS_BASE(cp_index) + 0x441300)
#define MVEBU_AXI_ATTR_REG(cp_index, index)	(MVEBU_AXI_ATTR_BASE(cp_index) + 0x4 * index)

/* AXI Protection bits */
#define MVEBU_AXI_PROT_BASE(cp_index)		(MVEBU_CP_REGS_BASE(cp_index) + 0x441200)

/* AXI Protection regs for A0 revision */
#define MVEBU_AXI_PROT_REG_A0(cp_index, index)	((index <= 6) ? (MVEBU_AXI_PROT_BASE(cp_index) + 0x4 * index) : \
						(MVEBU_AXI_PROT_BASE(cp_index) + 0x20 + 0x4 * (index - 7)))
#define MVEBU_AXI_PROT_REGS_NUM_A0		(9)

/* AXI Protection regs for A1 revision */
#define MVEBU_AXI_PROT_REG(cp_index, index)	((index <= 4) ? (MVEBU_AXI_PROT_BASE(cp_index) + 0x4 * index) : \
						(MVEBU_AXI_PROT_BASE(cp_index) + 0x18))
#define MVEBU_AXI_PROT_REGS_NUM			(6)

#define MVEBU_SOC_CFGS_BASE(cp_index)		(MVEBU_CP_REGS_BASE(cp_index) + 0x441900)
#define MVEBU_SOC_CFG_REG(cp_index, index)	(MVEBU_SOC_CFGS_BASE(cp_index) + 0x4 * index)
#define MVEBU_SOC_CFG_REG_NUM			(0)
#define MVEBU_SOC_CFG_GLOG_SECURE_EN_MASK	(0xE)

/* SATA3 MBUS to AXI regs */
#define MVEBU_SATA_M2A_AXI_PORT_CTRL_REG(cp_index)	(MVEBU_CP_REGS_BASE(cp_index) + 0x54ff04)

/* AXI to MBUS bridge registers */
#define MVEBU_AMB_IP_BRIDGE_WIN_REG(cp_index, win)	(MVEBU_AMB_IP_BASE(cp_index) + (win * 0x8))
#define MVEBU_AMB_IP_BRIDGE_WIN_EN_OFFSET		0
#define MVEBU_AMB_IP_BRIDGE_WIN_EN_MASK			(0x1 << MVEBU_AMB_IP_BRIDGE_WIN_EN_OFFSET)
#define MVEBU_AMB_IP_BRIDGE_WIN_SIZE_OFFSET		16
#define MVEBU_AMB_IP_BRIDGE_WIN_SIZE_MASK		(0xffff << MVEBU_AMB_IP_BRIDGE_WIN_SIZE_OFFSET)

enum axi_attr {
	AXI_ADUNIT_ATTR = 0,
	AXI_COMUNIT_ATTR,
	AXI_EIP197_ATTR,
	AXI_USB3D_ATTR,
	AXI_USB3H0_ATTR,
	AXI_USB3H1_ATTR,
	AXI_SATA0_ATTR,
	AXI_SATA1_ATTR,
	AXI_DAP_ATTR,
	AXI_DFX_ATTR,
	AXI_DBG_TRC_ATTR = 12,
	AXI_SDIO_ATTR,
	AXI_MSS_ATTR,
	AXI_MAX_ATTR,
};

/* Most stream IDS are configured centrally in the CP-110 RFU
 * but some are configured inside the unit registers
 */
#define RFU_STREAM_ID_BASE	(0x450000)
#define AUDIO_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x0)
#define TDM_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x4)
#define USB3D_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x8)
#define USB3H_0_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0xC)
#define USB3H_1_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x10)
#define SATA_0_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x14)
#define SATA_1_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x18)
#define DBG_TRC_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x24)
#define SDIO_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x28)
#define DAP_STREAM_ID_REG	(RFU_STREAM_ID_BASE + 0x2C)

#define CP_DMA_0_STREAM_ID_REG  (0x6B0010)
#define CP_DMA_1_STREAM_ID_REG  (0x6D0010)

/* We allocate IDs 0-127 for PCI since
 * SR-IOV devices can generate many functions
 * that need a unique Stream-ID.
 */
#define MAX_PCIE_STREAM_ID	(0x80)

uintptr_t stream_id_reg[] = {
	AUDIO_STREAM_ID_REG,
	TDM_STREAM_ID_REG,
	USB3D_STREAM_ID_REG,
	USB3H_0_STREAM_ID_REG,
	USB3H_1_STREAM_ID_REG,
	SATA_0_STREAM_ID_REG,
	SATA_1_STREAM_ID_REG,
	DBG_TRC_STREAM_ID_REG,
	SDIO_STREAM_ID_REG,
	DAP_STREAM_ID_REG,
	CP_DMA_0_STREAM_ID_REG,
	CP_DMA_1_STREAM_ID_REG,
	0
};

void cp110_errata_wa_init(int cp_index)
{
	uint32_t data;

	/* ERRATA GL-4076863 (STORM-916):
	 * Reset value for global_secure_enable inputs must be changed from '1' to '0'.
	 * When asserted, only "secured" transactions can enter IHB configuration space.
	 * However, blocking AXI transactions is performed by IOB.
	 * Performing it also at IHB/HB complicates programming model.
	 *
	 * Enable non-secure access in SOC configuration register */
	data = mmio_read_32(MVEBU_SOC_CFG_REG(cp_index, MVEBU_SOC_CFG_REG_NUM));
	data &= ~MVEBU_SOC_CFG_GLOG_SECURE_EN_MASK;
	mmio_write_32(MVEBU_SOC_CFG_REG(cp_index, MVEBU_SOC_CFG_REG_NUM), data);
}

/* Set a unique stream id for all DMA capable devices */
void cp110_stream_id_init(uintptr_t cp110_base)
{
	int i = 0;
	uint32_t stream_id = MAX_PCIE_STREAM_ID;

	while (stream_id_reg[i]) {
		/* SATA port 0/1 are in the same SATA unit, and they should use
		** the same STREAM ID number */
		if (stream_id_reg[i] == SATA_0_STREAM_ID_REG)
			mmio_write_32(cp110_base + stream_id_reg[i++], stream_id);
		else
			mmio_write_32(cp110_base + stream_id_reg[i++], stream_id++);
	}
}

void cp110_axi_attr_init(int cp_index)
{
	uint32_t index, data;

	/* Initialize AXI attributes for Armada-7K/8K SoC */

	/* Go over the AXI attributes and set Ax-Cache and Ax-Domain */
	for (index = 0; index < AXI_MAX_ATTR; index++) {
		switch (index) {
		/* DFX and MSS unit works with no coherent only -
		   there's no option to configure the Ax-Cache and Ax-Domain */
		case AXI_DFX_ATTR:
		case AXI_MSS_ATTR:
			continue;
		default:
			/* Set Ax-Cache as cacheable, no allocate, modifiable, bufferable
			 The values are different because Read & Write definition
			 is different in Ax-Cache */
			data = mmio_read_32(MVEBU_AXI_ATTR_REG(cp_index, index));
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
			mmio_write_32(MVEBU_AXI_ATTR_REG(cp_index, index), data);
		}
	}

	if (apn806_rev_id_get() == APN806_REV_ID_A1) {
		/* SATA IOCC supported in A1 rev only:
		 * Cache attributes for SATA MBUS to AXI configuration */
		data = mmio_read_32(MVEBU_SATA_M2A_AXI_PORT_CTRL_REG(cp_index));
		data &= ~MVEBU_SATA_M2A_AXI_AWCACHE_MASK;
		data |= (CACHE_ATTR_WRITE_ALLOC | CACHE_ATTR_CACHEABLE | CACHE_ATTR_BUFFERABLE)
			<< MVEBU_SATA_M2A_AXI_AWCACHE_OFFSET;
		data &= ~MVEBU_SATA_M2A_AXI_ARCACHE_MASK;
		data |= (CACHE_ATTR_READ_ALLOC | CACHE_ATTR_CACHEABLE | CACHE_ATTR_BUFFERABLE)
			<< MVEBU_SATA_M2A_AXI_ARCACHE_OFFSET;
		mmio_write_32(MVEBU_SATA_M2A_AXI_PORT_CTRL_REG(cp_index), data);

	/* Set all IO's AXI attribute to non-secure access. */
		for (index = 0; index < MVEBU_AXI_PROT_REGS_NUM; index++)
			mmio_write_32(MVEBU_AXI_PROT_REG(cp_index, index), DOMAIN_SYSTEM_SHAREABLE);
	} else {
		/* for A0 rev only set AXI attribute to non-secure access
		 * (different AXI_PROT reg mapping vs A1) */
		for (index = 0; index < MVEBU_AXI_PROT_REGS_NUM_A0; index++)
			mmio_write_32(MVEBU_AXI_PROT_REG_A0(cp_index, index), DOMAIN_SYSTEM_SHAREABLE);
	}

	return;
}

void amb_bridge_init(int cp_index)
{
	uint32_t reg;

	/* Open AMB bridge Window to Access COMPHY/MDIO registers */
	reg = mmio_read_32(MVEBU_AMB_IP_BRIDGE_WIN_REG(cp_index, 0));
	reg &= ~(MVEBU_AMB_IP_BRIDGE_WIN_SIZE_MASK | MVEBU_AMB_IP_BRIDGE_WIN_EN_MASK);
	reg |= (0x7ff << MVEBU_AMB_IP_BRIDGE_WIN_SIZE_OFFSET | 0x1 << MVEBU_AMB_IP_BRIDGE_WIN_EN_OFFSET);
	mmio_write_32(MVEBU_AMB_IP_BRIDGE_WIN_REG(cp_index, 0), reg);
}

void cp110_rtc_init(int cp_index)
{
	/* Update MBus timing parameters before accessing RTC registers */
	mmio_clrsetbits_32(MVEBU_RTC_BASE(cp_index) +
			   MVEBU_RTC_BRIDGE_TIMING_CTRL0_REG_OFFS,
			   MVEBU_RTC_WRCLK_PERIOD_MASK,
			   MVEBU_RTC_WRCLK_PERIOD_DEFAULT);

	mmio_clrsetbits_32(MVEBU_RTC_BASE(cp_index) +
			   MVEBU_RTC_BRIDGE_TIMING_CTRL0_REG_OFFS,
			   MVEBU_RTC_WRCLK_SETUP_MASK,
			   MVEBU_RTC_WRCLK_SETUP_DEFAULT << MVEBU_RTC_WRCLK_SETUP_OFFS);

	mmio_clrsetbits_32(MVEBU_RTC_BASE(cp_index) +
			   MVEBU_RTC_BRIDGE_TIMING_CTRL1_REG_OFFS,
			   MVEBU_RTC_READ_OUTPUT_DELAY_MASK,
			   MVEBU_RTC_READ_OUTPUT_DELAY_DEFAULT);

	/*
	 * Issue reset to the RTC if Clock Correction register
	 * contents did not sustain the reboot/power-on.
	 */
	if ((mmio_read_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_CCR_REG) &
	    MVEBU_RTC_NOMINAL_TIMING_MASK) != MVEBU_RTC_NOMINAL_TIMING) {
		/* Reset Test register */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_TEST_CONFIG_REG, 0);
		udelay(500000);

		/* Reset Time register */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_TIME_REG, 0);
		udelay(62);

		/* Reset Status register */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_STATUS_REG,
			      (MVEBU_RTC_STATUS_ALARM1_MASK | MVEBU_RTC_STATUS_ALARM2_MASK));
		udelay(62);

		/* Turn off Int1 and Int2 sources & clear the Alarm count */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_IRQ_1_CONFIG_REG, 0);
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_IRQ_2_CONFIG_REG, 0);
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_ALARM_1_REG, 0);
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_ALARM_2_REG, 0);

		/* Setup nominal register access timing */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_CCR_REG,
			      MVEBU_RTC_NOMINAL_TIMING);

		/* Reset Time register */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_TIME_REG, 0);
		udelay(10);

		/* Reset Status register */
		mmio_write_32(MVEBU_RTC_BASE(cp_index) + MVEBU_RTC_STATUS_REG,
			      (MVEBU_RTC_STATUS_ALARM1_MASK | MVEBU_RTC_STATUS_ALARM2_MASK));
		udelay(50);
	}
}

void cp110_init(int cp_index)
{
	/* configure AXI-MBUS windows for CP0*/
	init_amb_adec(cp_index);

	/* configure IOB windows for CP0*/
	init_iob(cp_index);

	/* configure axi for CP0*/
	cp110_axi_attr_init(cp_index);

	/* Execute SW WA for erratas */
	cp110_errata_wa_init(cp_index);

	/* configure icu for CP0 */
	/* ICU - Interrupt Consolidation unit
	 * CP0: interrupt 0..63 mapped to ID 64..127 in AP
	 * CP1: interrupt 64..127 mapped to ID 288..351 in AP */
	icu_init(cp_index, 0, cp_index * 64, &icu_config);

	/* configure stream id for CP0 */
	cp110_stream_id_init(MVEBU_CP_REGS_BASE(cp_index));

	/* Open AMB bridge for comphy for CP0 & CP1*/
	amb_bridge_init(cp_index);

	/* Reset RTC if needed */
	cp110_rtc_init(cp_index);
}

/* Do the minimal setup required to configure the CP in BLE */
void cp110_ble_init(int cp_index)
{
	amb_bridge_init(cp_index);
}
