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

#include <plat_marvell.h>
#include <platform.h>
#include <console.h>
#include <gicv2.h>
#include <mmio.h>
#include <assert.h>
#include <debug.h>
#include <delay_timer.h>
#include <cache_llc.h>
#include <marvell_pm.h>
#include <plat_config.h>

#ifdef SCP_IMAGE
#include <bakery_lock.h>
#include <platform.h>
#include <mss_pm_ipc.h>
#include <plat_pm_trace.h>
#endif

#define MVEBU_PRIVATE_UID_REG		0x30
#define MVEBU_RFU_GLOBL_SW_RST		0x84
#define MVEBU_CCU_RVBAR(i)		(MVEBU_REGS_BASE + 0x640 + (i * 4))
#define MVEBU_CCU_CPU_UN_RESET		(MVEBU_REGS_BASE + 0x650)

#define MPIDR_CPU_GET(mpidr)		((mpidr) & MPIDR_CPU_MASK)
#define MPIDR_CLUSTER_GET(mpidr)	MPIDR_AFFLVL1_VAL((mpidr))

#define MVEBU_GPIO_MASK(index)		(1 << (index % 32))
#define MVEBU_MPP_MASK(index)		(0xF << (4 * (index % 8)))
#define MVEBU_GPIO_VALUE(index, value)	(value << (index % 32))

#define MVEBU_USER_CMD_0_REG		(MVEBU_DRAM_MAC_BASE + 0x20)
#define MVEBU_USER_CMD_CH0_OFFSET	28
#define MVEBU_USER_CMD_CH0_MASK		(1 << MVEBU_USER_CMD_CH0_OFFSET)
#define MVEBU_USER_CMD_CH0_EN		(1 << MVEBU_USER_CMD_CH0_OFFSET)
#define MVEBU_USER_CMD_CS_OFFSET	24
#define MVEBU_USER_CMD_CS_MASK		(0xF << MVEBU_USER_CMD_CS_OFFSET)
#define MVEBU_USER_CMD_CS_ALL		(0xF << MVEBU_USER_CMD_CS_OFFSET)
#define MVEBU_USER_CMD_SR_OFFSET	6
#define MVEBU_USER_CMD_SR_MASK		(0x3 << MVEBU_USER_CMD_SR_OFFSET)
#define MVEBU_USER_CMD_SR_ENTER		(0x1 << MVEBU_USER_CMD_SR_OFFSET)
#define MVEBU_MC_PWR_CTRL_REG		(MVEBU_DRAM_MAC_BASE + 0x54)
#define MVEBU_MC_AC_ON_DLY_OFFSET	8
#define MVEBU_MC_AC_ON_DLY_MASK		(0xF << MVEBU_MC_AC_ON_DLY_OFFSET)
#define MVEBU_MC_AC_ON_DLY_DEF_VAR	(8 << MVEBU_MC_AC_ON_DLY_OFFSET)
#define MVEBU_MC_AC_OFF_DLY_OFFSET	4
#define MVEBU_MC_AC_OFF_DLY_MASK	(0xF << MVEBU_MC_AC_OFF_DLY_OFFSET)
#define MVEBU_MC_AC_OFF_DLY_DEF_VAR	(0xC << MVEBU_MC_AC_OFF_DLY_OFFSET)
#define MVEBU_MC_PHY_AUTO_OFF_OFFSET	0
#define MVEBU_MC_PHY_AUTO_OFF_MASK	(1 << MVEBU_MC_PHY_AUTO_OFF_OFFSET)
#define MVEBU_MC_PHY_AUTO_OFF_EN	(1 << MVEBU_MC_PHY_AUTO_OFF_OFFSET)

#ifdef SCP_IMAGE
/* this lock synchronize AP multiple cores execution with MSS */
DEFINE_BAKERY_LOCK(pm_sys_lock);
#endif

/* Weak definitions may be overridden in specific board */
#pragma weak plat_get_pm_cfg

/* AP806 CPU power down /power up definitions */
enum CPU_ID {
	CPU0,
	CPU1,
	CPU2,
	CPU3
};

#define CPUS_PER_CLUSTER		2
#define REG_WR_VALIDATE_TIMEOUT		(2000)

#define FEATURE_DISABLE_STATUS_REG			(MVEBU_REGS_BASE + 0x6F8230)
#define FEATURE_DISABLE_STATUS_CPU_CLUSTER_OFFSET	4
#define FEATURE_DISABLE_STATUS_CPU_CLUSTER_MASK		(0x1 << FEATURE_DISABLE_STATUS_CPU_CLUSTER_OFFSET)

#define PWRC_CPUN_CR_REG(cpu_id)		(MVEBU_REGS_BASE + 0x680000 + (cpu_id * 0x10))
#define PWRC_CPUN_CR_PWR_DN_RQ_OFFSET		0
#define PWRC_CPUN_CR_PWR_DN_RQ_MASK		(0x1 << PWRC_CPUN_CR_PWR_DN_RQ_OFFSET)
#define PWRC_CPUN_CR_ISO_ENABLE_OFFSET		16
#define PWRC_CPUN_CR_ISO_ENABLE_MASK		(0x1 << PWRC_CPUN_CR_ISO_ENABLE_OFFSET)
#define PWRC_CPUN_CR_LDO_BYPASS_RDY_OFFSET	31
#define PWRC_CPUN_CR_LDO_BYPASS_RDY_MASK	(0x1 << PWRC_CPUN_CR_LDO_BYPASS_RDY_OFFSET)

#define CCU_B_PRCRN_REG(cpu_id)			(MVEBU_REGS_BASE + 0x1A50 + \
						((cpu_id / 2) * (0x400)) + ((cpu_id % 2) * 4))
#define CCU_B_PRCRN_CPUPORESET_STATIC_OFFSET	0
#define CCU_B_PRCRN_CPUPORESET_STATIC_MASK	(0x1 << CCU_B_PRCRN_CPUPORESET_STATIC_OFFSET)

/*
 * Power down CPU:
 * Used to reduce power consumption, and avoid SoC unnecessary temperature rise.
 */
int plat_marvell_cpu_powerdown(int cpu_id)
{
	uint32_t	reg_val;
	int		exit_loop = REG_WR_VALIDATE_TIMEOUT;

	INFO("Powering down CPU%d\n", cpu_id);

	/* 1. Isolation enable */
	reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
	reg_val |= 0x1 << PWRC_CPUN_CR_ISO_ENABLE_OFFSET;
	mmio_write_32(PWRC_CPUN_CR_REG(cpu_id), reg_val);

	/* 2. Read and check Isolation enabled - verify bit set to 1 */
	do {
		reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
		exit_loop--;
	} while (!(reg_val & (0x1 << PWRC_CPUN_CR_ISO_ENABLE_OFFSET)) && exit_loop > 0);

	/* 3. Switch off CPU power */
	reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
	reg_val &= ~PWRC_CPUN_CR_PWR_DN_RQ_MASK;
	mmio_write_32(PWRC_CPUN_CR_REG(cpu_id), reg_val);

	/* 4. Read and check Switch Off - verify bit set to 0 */
	exit_loop = REG_WR_VALIDATE_TIMEOUT;
	do {
		reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
		exit_loop--;
	} while (reg_val & PWRC_CPUN_CR_PWR_DN_RQ_MASK && exit_loop > 0);

	if (exit_loop <= 0)
		goto cpu_poweroff_error;

	/* 5. De-Assert power ready */
	reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
	reg_val &= ~PWRC_CPUN_CR_LDO_BYPASS_RDY_MASK;
	mmio_write_32(PWRC_CPUN_CR_REG(cpu_id), reg_val);

	/* 6. Assert CPU POR reset */
	reg_val = mmio_read_32(CCU_B_PRCRN_REG(cpu_id));
	reg_val &= ~CCU_B_PRCRN_CPUPORESET_STATIC_MASK;
	mmio_write_32(CCU_B_PRCRN_REG(cpu_id), reg_val);

	/* 7. Read and poll on Validate the CPU is out of reset */
	exit_loop = REG_WR_VALIDATE_TIMEOUT;
	do {
		reg_val = mmio_read_32(CCU_B_PRCRN_REG(cpu_id));
		exit_loop--;
	} while (reg_val & CCU_B_PRCRN_CPUPORESET_STATIC_MASK && exit_loop > 0);

	if (exit_loop <= 0)
		goto cpu_poweroff_error;

	INFO("Successfully powered down CPU%d\n", cpu_id);

	return 0;

cpu_poweroff_error:
	ERROR("ERROR: Can't power down CPU%d\n" , cpu_id);
	return -1;
}

/*
 * Power down CPUs 1-3 at early boot stage,
 * to reduce power consumption and SoC temperature.
 * This is triggered by BLE prior to DDR initialization.
 *
 * Note:
 * All CPUs will be powered up by plat_marvell_cpu_powerup on Linux boot stage,
 * which is triggered by PSCI ops (pwr_domain_on).
 */
int plat_marvell_early_cpu_powerdown(void)
{
	uint32_t cpu_cluster_status = mmio_read_32(FEATURE_DISABLE_STATUS_REG)
						& FEATURE_DISABLE_STATUS_CPU_CLUSTER_MASK;
	/* if cpu_cluster_status bit is set, that means we have only single cluster */
	int cluster_count = cpu_cluster_status ? 1 : 2;

	INFO("Powering off unused CPUs\n");

	/* CPU1 is in AP806 cluster-0, which always exists - so power it down */
	if (plat_marvell_cpu_powerdown(CPU1) == -1)
		return -1;

	/*
	 * CPU2-3 are in AP806 2nd cluster (cluster-1), which doesn't exists in dual-core systems.
	 * so need to check if we have dual-core (single cluster) or quad-code (2 clusters)
	 */
	if (cluster_count == 2) {
		/* CPU2-3 are part of 2nd cluster */
		if (plat_marvell_cpu_powerdown(CPU2) == -1)
			return -1;
		if (plat_marvell_cpu_powerdown(CPU3) == -1)
			return -1;
	}

	return 0;
}

/*
 * Power up CPU - part of Linux boot stage
 */
int plat_marvell_cpu_powerup(u_register_t mpidr)
{
	uint32_t	reg_val;
	int	cpu_id = MPIDR_CPU_GET(mpidr), cluster = MPIDR_CLUSTER_GET(mpidr);
	int	exit_loop = REG_WR_VALIDATE_TIMEOUT;

	/* calculate absolute CPU ID */
	cpu_id = cluster * CPUS_PER_CLUSTER + cpu_id;

	INFO("Powering on CPU%d\n", cpu_id);

	/* 1. Switch CPU power ON */
	reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
	reg_val |= 0x1 << PWRC_CPUN_CR_PWR_DN_RQ_OFFSET;
	mmio_write_32(PWRC_CPUN_CR_REG(cpu_id), reg_val);

	/* 2. Wait for CPU on, up to 100 uSec: */
	udelay(100);

	/* 3. Assert power ready */
	reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
	reg_val |= 0x1 << PWRC_CPUN_CR_LDO_BYPASS_RDY_OFFSET;
	mmio_write_32(PWRC_CPUN_CR_REG(cpu_id), reg_val);

	/* 4. Read & Validate power ready - used in order to generate 16 Host CPU cycles */
	do {
		reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
		exit_loop--;
	} while (!(reg_val & (0x1 << PWRC_CPUN_CR_LDO_BYPASS_RDY_OFFSET)) && exit_loop > 0);

	if (exit_loop <= 0)
		goto cpu_poweron_error;

	/* 5. Isolation disable */
	reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
	reg_val &= ~PWRC_CPUN_CR_ISO_ENABLE_MASK;
	mmio_write_32(PWRC_CPUN_CR_REG(cpu_id), reg_val);

	/* 6. Read and check Isolation enabled - verify bit set to 1 */
	exit_loop = REG_WR_VALIDATE_TIMEOUT;
	do {
		reg_val = mmio_read_32(PWRC_CPUN_CR_REG(cpu_id));
		exit_loop--;
	} while ((reg_val & (0x1 << PWRC_CPUN_CR_ISO_ENABLE_OFFSET)) && exit_loop > 0);

	/* 7. De Assert CPU POR reset & Core reset */
	reg_val = mmio_read_32(CCU_B_PRCRN_REG(cpu_id));
	reg_val |= 0x1 << CCU_B_PRCRN_CPUPORESET_STATIC_OFFSET;
	mmio_write_32(CCU_B_PRCRN_REG(cpu_id), reg_val);

	/* 8. Read & Validate CPU POR reset */
	exit_loop = REG_WR_VALIDATE_TIMEOUT;
	do {
		reg_val = mmio_read_32(CCU_B_PRCRN_REG(cpu_id));
		exit_loop--;
	} while (!(reg_val & (0x1 << CCU_B_PRCRN_CPUPORESET_STATIC_OFFSET)) && exit_loop > 0);

	if (exit_loop <= 0)
		goto cpu_poweron_error;

	INFO("Successfully powered on CPU%d\n", cpu_id);

	return 0;

cpu_poweron_error:
	ERROR("ERROR: Can't power up CPU%d\n" , cpu_id);
	return -1;
}


int plat_marvell_cpu_on(u_register_t mpidr)
{
	int cpu_id;
	int cluster;

	/* Set barierr */
	__asm__ volatile("dsb     sy");

	/* Get cpu number - use CPU ID */
	cpu_id =  MPIDR_CPU_GET(mpidr);

	/* Get cluster number - use affinity level 1 */
	cluster = MPIDR_CLUSTER_GET(mpidr);

	/* Set CPU private UID */
	mmio_write_32(MVEBU_REGS_BASE + MVEBU_PRIVATE_UID_REG, cluster + 0x4);

	/* Set the cpu start address to BL1 entry point (align to 0x10000) */
	mmio_write_32(MVEBU_CCU_RVBAR(0) + (cpu_id << 2),
		      PLAT_MARVELL_CPU_ENTRY_ADDR >> 16);

	/* Get the cpu out of reset */
	mmio_write_32(MVEBU_CCU_CPU_UN_RESET + (cpu_id << 2), 0x10001);

	return 0;
}

/*******************************************************************************
 * A8K handler called to check the validity of the power state
 * parameter.
 ******************************************************************************/
int a8k_validate_power_state(unsigned int power_state,
			    psci_power_state_t *req_state)
{
	int pstate = psci_get_pstate_type(power_state);
	int pwr_lvl = psci_get_pstate_pwrlvl(power_state);
	int i;

	if (pwr_lvl > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	/* Sanity check the requested state */
	if (pstate == PSTATE_TYPE_STANDBY) {
		/*
		 * It's possible to enter standby only on power level 0
		 * Ignore any other power level.
		 */
		if (pwr_lvl != MARVELL_PWR_LVL0)
			return PSCI_E_INVALID_PARAMS;

		req_state->pwr_domain_state[MARVELL_PWR_LVL0] =
					MARVELL_LOCAL_STATE_RET;
	} else {
		for (i = MARVELL_PWR_LVL0; i <= pwr_lvl; i++)
			req_state->pwr_domain_state[i] =
					MARVELL_LOCAL_STATE_OFF;
	}

	/*
	 * We expect the 'state id' to be zero.
	 */
	if (psci_get_pstate_id(power_state))
		return PSCI_E_INVALID_PARAMS;

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * A8K handler called when a CPU is about to enter standby.
 ******************************************************************************/
void a8k_cpu_standby(plat_local_state_t cpu_state)
{
	ERROR("a8k_cpu_standby needs to be implemented\n");
	panic();
}

/*******************************************************************************
 * A8K handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
int a8k_pwr_domain_on(u_register_t mpidr)
{
	/* Power up CPU (CPUs 1-3 are powered off at start of BLE) */
	plat_marvell_cpu_powerup(mpidr);

#ifdef SCP_IMAGE
	unsigned int target = ((mpidr & 0xFF) + (((mpidr >> 8) & 0xFF) * 2));

	/*
	 * pm system synchronization - used to synchronize
	 * multiple core access to MSS
	 */
	bakery_lock_get(&pm_sys_lock);

	/* send CPU ON IPC Message to MSS */
	mss_pm_ipc_msg_send(target, PM_IPC_MSG_CPU_ON, 0);

	/* trigger IPC message to MSS */
	mss_pm_ipc_msg_trigger();

	/* pm system synchronization */
	bakery_lock_release(&pm_sys_lock);

	/* trace message */
	PM_TRACE(TRACE_PWR_DOMAIN_ON | target);
#else
	/* proprietary CPU ON exection flow */
	plat_marvell_cpu_on(mpidr);
#endif /* SCP_IMAGE */

	return 0;
}

/*******************************************************************************
 * A8K handler called to validate the entry point.
 ******************************************************************************/
int a8k_validate_ns_entrypoint(uintptr_t entrypoint)
{
	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * A8K handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void a8k_pwr_domain_off(const psci_power_state_t *target_state)
{
#ifdef SCP_IMAGE
	unsigned int idx = plat_my_core_pos();

	/* Prevent interrupts from spuriously waking up this cpu */
	gicv2_cpuif_disable();

	/* pm system synchronization - used to synchronize multiple core access to MSS */
	bakery_lock_get(&pm_sys_lock);

	/* send CPU OFF IPC Message to MSS */
	mss_pm_ipc_msg_send(idx, PM_IPC_MSG_CPU_OFF, target_state);

	/* trigger IPC message to MSS */
	mss_pm_ipc_msg_trigger();

	/* pm system synchronization */
	bakery_lock_release(&pm_sys_lock);

	/* trace message */
	PM_TRACE(TRACE_PWR_DOMAIN_OFF);
#else
	INFO("a8k_pwr_domain_off is not supported without SCP\n");
	return;
#endif /* SCP_IMAGE */
}

#ifndef SCP_IMAGE
/*
 * This function should be called on restore from
 * "suspend to RAM" state when the execution flow
 * has to bypass BootROM image to RAM copy and speed up
 * the system recovery
 *
 */
static void plat_exit_bootrom(void)
{
	exit_bootrom(PLAT_MARVELL_TRUSTED_ROM_BASE);
}
#endif

/* Get PM config to power off the SoC */
void *plat_get_pm_cfg(void)
{
	return NULL;
}

/*
 * Prepare for the power off of the system via GPIO
 */
static void plat_marvell_power_off_gpio(struct power_off_method *pm_cfg)
{
	unsigned int gpio;
	unsigned int idx;
	unsigned int shift;
	unsigned int reg;
	unsigned int addr;
	gpio_info_t *info;
	unsigned int tog_bits;

	assert((pm_cfg->cfg.gpio.pin_count < PMIC_GPIO_MAX_NUMBER) &&
	       (pm_cfg->cfg.gpio.step_count < PMIC_GPIO_MAX_TOGGLE_STEP));

	/* Prepare GPIOs for PMIC */
	for (gpio = 0; gpio < pm_cfg->cfg.gpio.pin_count; gpio++) {
		info = &pm_cfg->cfg.gpio.info[gpio];
		/* Set PMIC GPIO to output mode */
		reg = mmio_read_32(MVEBU_CP_GPIO_DATA_OUT_EN(info->cp_index, info->gpio_index));
		mmio_write_32(MVEBU_CP_GPIO_DATA_OUT_EN(info->cp_index, info->gpio_index),
			      reg & ~MVEBU_GPIO_MASK(info->gpio_index));

		/* Set the appropriate MPP to GPIO mode */
		reg = mmio_read_32(MVEBU_PM_MPP_REGS(info->cp_index, info->gpio_index));
		mmio_write_32(MVEBU_PM_MPP_REGS(info->cp_index, info->gpio_index),
			reg & ~MVEBU_MPP_MASK(info->gpio_index));
	}

	/* Wait for MPP & GPIO pre-configurations done */
	mdelay(pm_cfg->cfg.gpio.delay_ms);

	/* Toggle the GPIO values, and leave final step to be triggered after  DDR self-refresh is enabled */
	for (idx = 0; idx < pm_cfg->cfg.gpio.step_count; idx++) {
		tog_bits = pm_cfg->cfg.gpio.seq[idx];

		/* The GPIOs must be within same GPIO register, thus could get the original value by first GPIO */
		info = &pm_cfg->cfg.gpio.info[0];
		reg = mmio_read_32(MVEBU_CP_GPIO_DATA_OUT(info->cp_index, info->gpio_index));
		addr = MVEBU_CP_GPIO_DATA_OUT(info->cp_index, info->gpio_index);

		for (gpio = 0; gpio < pm_cfg->cfg.gpio.pin_count; gpio++) {
			shift = pm_cfg->cfg.gpio.info[gpio].gpio_index % 32;
			if (GPIO_LOW == (tog_bits & (1 << gpio)))
				reg &= ~(1 << shift);
			else
				reg |= (1 << shift);
		}

		/* Set the GPIO register, for last step just store register address and values to system registers */
		if (idx < pm_cfg->cfg.gpio.step_count - 1) {
			mmio_write_32(MVEBU_CP_GPIO_DATA_OUT(info->cp_index, info->gpio_index), reg);
			mdelay(pm_cfg->cfg.gpio.delay_ms);
		} else {
			/* Save GPIO register value to X17, and address to X18 */
			__asm__ volatile (
				"mov	x17, %0\n\t"
				"mov	x18, %1\n\t"
				: : "r" (reg), "r" (addr));
		}
	}
}

/*
 * Prepare for the power off of the system
 */
static void plat_marvell_power_off_prepare(struct power_off_method *pm_cfg)
{
	switch (pm_cfg->type) {
	case PMIC_GPIO:
		plat_marvell_power_off_gpio(pm_cfg);
	default:
		break;
	}
}

/*
 * Enable DDR self-refresh
 */
static inline void plat_marvell_ddr_self_refresh_en(void)
{
	dsb();

	/* Put DRAM in self-refresh state */
	mmio_clrsetbits_32(MVEBU_MC_PWR_CTRL_REG,
			MVEBU_MC_AC_ON_DLY_MASK | MVEBU_MC_AC_OFF_DLY_MASK | MVEBU_MC_PHY_AUTO_OFF_MASK,
			MVEBU_MC_AC_ON_DLY_DEF_VAR | MVEBU_MC_AC_OFF_DLY_DEF_VAR | MVEBU_MC_PHY_AUTO_OFF_EN);

	mmio_clrsetbits_32(MVEBU_USER_CMD_0_REG,
			MVEBU_USER_CMD_CH0_MASK | MVEBU_USER_CMD_CS_MASK | MVEBU_USER_CMD_SR_MASK,
			MVEBU_USER_CMD_CH0_EN | MVEBU_USER_CMD_CS_ALL | MVEBU_USER_CMD_SR_ENTER);

	isb();

	/*
	 * Wait for DRAM is done using registers access only.
	 * At this stage any access to DRAM (procedure call) will
	 * release it from the self-refresh mode
	 */
	__asm__ volatile (
		/* Align to a cache line */
		"	.balign 64\n\t"
		/*
		* Wait 100 cycles for DDR to enter self refresh, by
		* doing 50 times two instructions.
		*/
		"	mov	x1, #50\n\t"
		"1:	subs	x1, x1, #1\n\t"
		"	bne	1b\n\t"
		: : : "x1");
}

/*
 * Trigger the power off of the system, no DRAM access is allowed in this routine.
 * It should be inline function so that no return at the end of the routine and
 * it can be adjacent to previous handling such as enabling the DDR self-refresh,
 * which make sure they are executed in the cache and no DRAM access is needed.
 * The X17 stores GPIO output value while X18 stores GPIO register address
 */
static inline void plat_marvell_power_off_trigger(void)
{
	__asm__ volatile ("str	w17, [x18]\n\t");
}

/* Trigger the power off of the system */
void plat_marvell_system_power_off(void)
{
	struct power_off_method *pm_cfg;

	/* Check if there is valid PM config */
	pm_cfg = (struct power_off_method *)plat_get_pm_cfg();
	if (!pm_cfg)
		return;

	/* Prepare for power off */
	plat_marvell_power_off_prepare(pm_cfg);

	/* Enable DDR self-refresh to keep the data during suspend */
	plat_marvell_ddr_self_refresh_en();

	/* Issue the power off */
	plat_marvell_power_off_trigger();
}

/*******************************************************************************
 * A8K handler called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void a8k_pwr_domain_suspend(const psci_power_state_t *target_state)
{
#ifdef SCP_IMAGE
	unsigned int idx;

	/* Prevent interrupts from spuriously waking up this cpu */
	gicv2_cpuif_disable();

	idx = plat_my_core_pos();

	/* pm system synchronization -used to synchronize multiple core access to MSS */
	bakery_lock_get(&pm_sys_lock);

	/* send CPU Suspend IPC Message to MSS */
	mss_pm_ipc_msg_send(idx, PM_IPC_MSG_CPU_SUSPEND, target_state);

	/* trigger IPC message to MSS */
	mss_pm_ipc_msg_trigger();

	/* pm system synchronization */
	bakery_lock_release(&pm_sys_lock);

	/* trace message */
	PM_TRACE(TRACE_PWR_DOMAIN_SUSPEND);
#else
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	INFO("Suspending to RAM\n");

	/* Prevent interrupts from spuriously waking up this cpu */
	gicv2_cpuif_disable();

	mailbox[MBOX_IDX_SUSPEND_MAGIC] = MVEBU_MAILBOX_SUSPEND_STATE;
	mailbox[MBOX_IDX_ROM_EXIT_ADDR] = (uintptr_t)&plat_exit_bootrom;

#if PLAT_MARVELL_SHARED_RAM_CACHED
	flush_dcache_range(PLAT_MARVELL_MAILBOX_BASE +
			   MBOX_IDX_SUSPEND_MAGIC * sizeof(uintptr_t),
			   2 * sizeof(uintptr_t));
#endif

	/* Flush and disable LLC before going off-power */
	llc_disable();

	/*
	 * Power off whole system, it should be guaranteed that CPU has enough time to finish
	 * remained tasks before the power off takes effect.
	 */
	plat_marvell_system_power_off();

	isb();
	/*
	 * Do not halt here!
	 * The function must return for allowing the caller function
	 * psci_power_up_finish() to do the proper context saving and
	 * to release the CPU lock.
	*/

#endif /* SCP_IMAGE */
}

/*******************************************************************************
 * A8K handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void a8k_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	/* arch specific configuration */
	psci_arch_init();

	/* Interrupt initialization */
	gicv2_pcpu_distif_init();
	gicv2_cpuif_enable();

#ifdef SCP_IMAGE
	/* trace message */
	PM_TRACE(TRACE_PWR_DOMAIN_ON_FINISH);
#endif /* SCP_IMAGE */
}

/*******************************************************************************
 * A8K handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
void a8k_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
#ifdef SCP_IMAGE
	/* arch specific configuration */
	psci_arch_init();

	/* Interrupt initialization */
	gicv2_cpuif_enable();

	/* trace message */
	PM_TRACE(TRACE_PWR_DOMAIN_SUSPEND_FINISH);
#else
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	/* Only primary CPU requres platform init */
	if (!plat_my_core_pos()) {
		/* Initialize the console to provide early debug support */
		console_init(PLAT_MARVELL_BOOT_UART_BASE,
			     PLAT_MARVELL_BOOT_UART_CLK_IN_HZ,
			     MARVELL_CONSOLE_BAUDRATE);
		bl31_plat_arch_setup();
		marvell_bl31_platform_setup();
		/*
		 * Remove suspend to RAM marker from the mailbox
		 * for treating a regular reset as a cold boot
		 */
		mailbox[MBOX_IDX_SUSPEND_MAGIC] = 0;
		mailbox[MBOX_IDX_ROM_EXIT_ADDR] = 0;
#if PLAT_MARVELL_SHARED_RAM_CACHED
		flush_dcache_range(PLAT_MARVELL_MAILBOX_BASE +
				   MBOX_IDX_SUSPEND_MAGIC * sizeof(uintptr_t),
				   2 * sizeof(uintptr_t));
#endif
	}
#endif /* SCP_IMAGE */
}

/*******************************************************************************
 * This handler is called by the PSCI implementation during the `SYSTEM_SUSPEND`
 * call to get the `power_state` parameter. This allows the platform to encode
 * the appropriate State-ID field within the `power_state` parameter which can
 * be utilized in `pwr_domain_suspend()` to suspend to system affinity level.
 ******************************************************************************/
void a8k_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	/* lower affinities use PLAT_MAX_OFF_STATE */
	for (int i = MPIDR_AFFLVL0; i <= PLAT_MAX_PWR_LVL; i++)
		req_state->pwr_domain_state[i] = PLAT_MAX_OFF_STATE;
}

/*******************************************************************************
 * A8K handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 a8k_system_off(void)
{
	ERROR("a8k_system_off needs to be implemented\n");
	panic();
	wfi();
	ERROR("A8K System Off: operation not handled.\n");
	panic();
}

void plat_marvell_system_reset(void)
{
	mmio_write_32(MVEBU_RFU_BASE + MVEBU_RFU_GLOBL_SW_RST, 0x0);
}

static void __dead2 a8k_system_reset(void)
{
	plat_marvell_system_reset();

	/* we shouldn't get to this point */
	panic();
}

/*******************************************************************************
 * Export the platform handlers via plat_arm_psci_pm_ops. The ARM Standard
 * platform layer will take care of registering the handlers with PSCI.
 ******************************************************************************/
const plat_psci_ops_t plat_arm_psci_pm_ops = {
	.cpu_standby = a8k_cpu_standby,
	.pwr_domain_on = a8k_pwr_domain_on,
	.pwr_domain_off = a8k_pwr_domain_off,
	.pwr_domain_suspend = a8k_pwr_domain_suspend,
	.pwr_domain_on_finish = a8k_pwr_domain_on_finish,
	.get_sys_suspend_power_state = a8k_get_sys_suspend_power_state,
	.pwr_domain_suspend_finish = a8k_pwr_domain_suspend_finish,
	.system_off = a8k_system_off,
	.system_reset = a8k_system_reset,
	.validate_power_state = a8k_validate_power_state,
	.validate_ns_entrypoint = a8k_validate_ns_entrypoint
};
