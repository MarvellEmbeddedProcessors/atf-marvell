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
#include <mvebu.h>

#define AP810_MAX_AP_NUM			4
#define AP810_MAX_AP_MASK			0xf

#define CCU_B_GIDACR(ap, stop)			(MVEBU_A2_BANKED_STOP_BASE(ap, stop) + 0x34)

#define CCU_B_LTC_CR(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x344)
#define LTC_MULTI_CHIP_TRAIN_MODE_EN		(1 << 15)

#define MRI_XBAR_PORTx_ROUTING0(ap, port)	(MVEBU_MRI_XBAR_BASE(ap) + 0x10 + 0x8 * port)

#define MVEBU_CCU_GUID(ap)			(MVEBU_REGS_BASE_AP(ap) + 0x4808)

#define SMMU_S_ACR(ap)				(MVEBU_SMMU_BASE(ap) + 0x10)
#define SMMU_S_ACR_PG_64K			(1 << 16)

#define MVEBU_CCU_GSPMU_CR(ap)			(MVEBU_CCU_LOCL_CNTL_BASE(ap) + 0x3F0)
#define GSPMU_CPU_CONTROL			(0x1 << 0)

#define CCU_HTC_CR(ap)				(MVEBU_CCU_BASE(ap) + 0x200)
#define CCU_SET_POC_OFFSET			5

#define GEVENT_CR_PORTx_EVENT_MASK(ap, port)	(MVEBU_AR_RFU_BASE(ap) + 0x500 + port * 0x4)

/* SYSRST_OUTn Config definitions */
#define MVEBU_SYSRST_OUT_CONFIG_REG(ap)		(MVEBU_AP_MISC_SOC_BASE(ap) + 0x4)
#define WD_MASK_SYS_RST_OUT			(1 << 2)

/* define AP810 stops */
enum ap810_stations {
	AP810_S0_SMC0 = 0,	/* Stop memory contoler 0 */
	AP810_S0_SIO0,		/* Stop IO 0 */
	AP810_S0_SIO1,		/* Stop IO 1 */
	AP810_S0_SMC1,		/* Stop memory contoler 1 */
	AP810_S0_SP0,		/* Stop proccessor 0 */
	AP810_S0_SP1,		/* Stop proccessor 1 */
	AP810_S0_SP2,		/* Stop proccessor 2 */
	AP810_S0_SP3,		/* Stop proccessor 3 */
	AP810_S0_SMC2,		/* Stop memory contoler 2 */
	AP810_S0_SG,		/* Stop general */
	AP810_S0_SIO2,		/* Stop IO 2 */
	AP810_S_END,
};

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

	debug_enter();
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

	debug_exit();
	return g_ap_count;
}

/* function to open access RGF to access another ring*/
static void setup_banked_rgf(int ap_id)
{
	int val, stop;

	debug_enter();
	/* Open access for all the banked RGF
	 * (remote ring access registers)
	 * 0xf for QUAD - 0x3 for DUAL - 0x1 for single (default)
	 * Open access for all IO & proccess stops, because MC & SG
	 * stop can't start transcations to another ring
	 */
	val = AP810_MAX_AP_MASK >> (AP810_MAX_AP_NUM - get_ap_count());
	for (stop = 0; stop < AP810_S_END; stop++) {
		switch (stop) {
		case AP810_S0_SMC0:
		case AP810_S0_SMC1:
		case AP810_S0_SMC2:
		case AP810_S0_SG:
			continue;
		default:
			mmio_write_32(CCU_B_GIDACR(ap_id, stop), val);
		}
	}
	debug_exit();
}


/* Configure access between AP, use static configuration */
static void ap810_enumeration_algo(void)
{
	uint32_t reg;
	int ap_id;

	/* In case of single AP, no need to configure MRI-xbar */
	if (get_ap_count() == 1)
		return;

	debug_enter();
	setup_banked_rgf(0);

	/* Enable training bit - for AP0 */
	reg = mmio_read_32(CCU_B_LTC_CR(0));
	mmio_write_32(CCU_B_LTC_CR(0), reg | LTC_MULTI_CHIP_TRAIN_MODE_EN);

	/* Configure MRI XBar
	 * MRI XBAR include access configuration for other APs.
	 * MRI XBAR have 5 ports, port 0 connected to global stop
	 * in the Aurora, and 4 other ports connected to other APs
	 * one other port is not connected.
	 * For every port there's register PORT_%x_ROUTING0, that mean
	 * every transaction come from port %x with AP-ID 0/1/2/3
	 * will exit on port number %y that written in register.
	 * e.g.:
	 *     AP0.PORT0_ROUTING0: {1,3,4,0}:
	 *     all transaction comes from port 0, with AP-ID 0 goes to
	 *     port 0 (return to AP0), AP-ID 1 goes to port 4,
	 *     AP-ID 2 goes to port 3, AP-ID 3 goes to port 1.
	 *
	 * QUAD AP Clique (all AP dies connected to each other)
	 * AP0: port 1 -> AP3, port 2 -> NA, port 3 -> AP2, port 4 -> AP3
	 * AP1: port 1 -> AP3, port 2 -> AP0, port 3 -> AP2, port 4 -> NA
	 * AP2: port 1 -> AP1, port 2 -> NA, port 3 -> AP0, port 4 -> AP3
	 * AP3: port 1 -> AP2, port 2 -> AP3, port 3 -> AP0, port 4 -> NA
	 * mri-xbar configurations:
	 *      AP0.PORT0_ROUTING0: {1,3,4,0}
	 *      AP1.PORT0_ROUTING0: {1,3,0,2}
	 *      AP2.PORT0_ROUTING0: {4,0,1,3}
	 *      AP3.PORT0_ROUTING0: {0,2,1,3}
	 *
	 * AP0: port 1 -> NA, port 2 -> NA, port 3 -> AP1, port 4 -> NA
	 * AP1: port 1 -> AP0, port 2 -> NA, port 3 -> NA, port 4 -> NA
	 * DUAL AP:
	 *      AP0.PORT0_ROUTING0: {3,0}
	 *      AP1.PORT0_ROUTING0: {0,1}
	 */
	if (get_ap_count() == 2) {
		mmio_write_32(MRI_XBAR_PORTx_ROUTING0(0, 0), 0x30);
		mmio_write_32(MRI_XBAR_PORTx_ROUTING0(1, 0), 0x1);
	} else if (get_ap_count() == 4) {
		mmio_write_32(MRI_XBAR_PORTx_ROUTING0(0, 0), 0x1340);
		mmio_write_32(MRI_XBAR_PORTx_ROUTING0(1, 0), 0x1302);
		mmio_write_32(MRI_XBAR_PORTx_ROUTING0(2, 0), 0x4013);
		mmio_write_32(MRI_XBAR_PORTx_ROUTING0(3, 0), 0x0213);
	}

	/* Disable training bit */
	mmio_write_32(CCU_B_LTC_CR(0), reg);

	/* Test AP access */
	if (get_ap_count() == 2) {
		/* Read status from AP1 */
		INFO("Test AP1: 0x%x\n", mmio_read_32(MRI_XBAR_PORTx_ROUTING0(1, 0)));
	} else if (get_ap_count() == 4) {
		/* Read status from AP1 */
		INFO("Test AP1: 0x%x\n", mmio_read_32(MRI_XBAR_PORTx_ROUTING0(1, 0)));
		/* Read status from AP2 */
		INFO("Test AP2: 0x%x\n", mmio_read_32(MRI_XBAR_PORTx_ROUTING0(2, 0)));
		/* Read status from AP3 */
		INFO("Test AP3: 0x%x\n", mmio_read_32(MRI_XBAR_PORTx_ROUTING0(3, 0)));
	}

	/* Update AP-ID of every AP die in the system */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++)
		mmio_write_32(MVEBU_CCU_GUID(ap_id), ap_id);
	debug_exit();
}

static void ap810_dvm_affinity(int ap_id)
{
	debug_enter();
	INFO("place holder to implement %s\n", __func__);
	debug_exit();
}

static void ap810_init_aurora2(int ap_id)
{
	unsigned int reg;

	debug_enter();

	/* Open access to another AP configuration */
	setup_banked_rgf(ap_id);

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

	debug_exit();
}

static void ap810_setup_smmu(int ap)
{
	uint32_t reg;

	debug_enter();

	/* Set the SMMU page size to 64 KB */
	reg = mmio_read_32(SMMU_S_ACR(ap));
	reg |= SMMU_S_ACR_PG_64K;
	mmio_write_32(SMMU_S_ACR(ap), reg);

	debug_exit();
}

static void ap810_sec_masters_access_en(int ap, uint32_t enable)
{
	debug_enter();
	INFO("place holder to implement %s\n", __func__);
	debug_exit();
}

static void ap810_axi_attr_init(int ap)
{
	uint32_t index, data;

	/* Initialize AXI attributes for AP810
	 * Go over the AXI attributes and set
	 * Ax-Cache and Ax-Domain
	 */
	debug_enter();
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

	debug_exit();
	return;
}

/* Setup events that controls the propagation
 * of CPU event between dies.
 */
void ap810_setup_events(int ap_id)
{
	debug_enter();
	INFO("Event propegation setup for AP%d\n", ap_id);
	/* The index of the register represents the destination port.
	 * The bit number represents the source to be masked.
	 * All sources which are unmasked will be ORed and sent to the
	 * destination port.
	 *
	 * For Quad AP the connectios is:
	 *  AP0: port 0 -> AP3, port 1 -> NC, port 2 -> AP2, port 3 -> AP1
	 *  AP1: port 0 -> AP3, port 1 -> AP0, port 2 -> AP2, port 3 -> NC
	 *  AP2: port 0 -> AP1, port 1 -> NC, port 2 -> AP0, port 3 -> AP3
	 *  AP3: port 0 -> AP2, port 1 -> AP2, port 2 -> AP0, port 3 -> NC
	 *
	 * For Dual AP the connection is:
	 *  AP0: port 2 -> AP1
	 *  AP1: port 0 -> AP0
	 */
	switch (ap_id) {
	case 0:
		if (get_ap_count() == 2) {
			/* Port 2 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 2), 0x2f);
			/* Port 4 (Local) - unmask Port 2 */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 4), 0x3b);
		} else {
			/* Port 0 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 0), 0x2f);
			/* Port 2 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 2), 0x2f);
			/* Port 3 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 3), 0x2f);
			/* Port 4 (Local) - unmask Port 0/2/3 */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 4), 0x32);
		}
		break;
	case 1:
		if (get_ap_count() == 2) {
			/* Port 0 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 0), 0x2f);
			/* Port 4 (Local) - unmask Port 0 */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 4), 0x3e);
		} else {
			/* Port 0 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 0), 0x2f);
			/* Port 1 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 1), 0x2f);
			/* Port 2 - unmask local GEvent  */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 2), 0x2f);
			/* Port 4 (Local) - unmask Port 0/1/2 */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 4), 0x38);
		}
		break;
	case 2:
			/* Port 0 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 0), 0x2f);
			/* Port 2 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 2), 0x2f);
			/* Port 3 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 3), 0x2f);
			/* Port 4 (Local) - unmask Port 0/2/3 */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 4), 0x32);
		break;
	case 3:
			/* Port 0 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 0), 0x2f);
			/* Port 1 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 1), 0x2f);
			/* Port 2 - unmask local GEvent */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 2), 0x2f);
			/* Port 4 (Local) - unmask Port 0/1/2 */
			mmio_write_32(GEVENT_CR_PORTx_EVENT_MASK(ap_id, 4), 0x38);
		break;
	}
	debug_exit();
}

static void ap810_stream_id_init(int ap_id)
{
	debug_enter();
	INFO("place holder to implement %s\n", __func__);
	debug_exit();
}

static void ap810_soc_misc_configurations(int ap)
{
	uint32_t reg;

	debug_enter();
	/* Un-mask Watchdog reset from influencing the SYSRST_OUTn.
	 * Otherwise, upon WD timeout, the WD reset singal won't trigger reset
	 */
	reg = mmio_read_32(MVEBU_SYSRST_OUT_CONFIG_REG(ap));
	reg &= ~(WD_MASK_SYS_RST_OUT);
	mmio_write_32(MVEBU_SYSRST_OUT_CONFIG_REG(ap), reg);
	debug_exit();
}

void ap810_generic_timer_init(void)
{
	debug_enter();
	INFO("place holder to implement %s\n", __func__);
	debug_exit();
}

void ap810_init(void)
{
	int ap_id;

	debug_enter();
	ap810_enumeration_algo();

	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		INFO("Initialize AP-%d\n", ap_id);
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
	debug_exit();
}
