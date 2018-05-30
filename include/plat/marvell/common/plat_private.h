/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
 
#ifndef __PLAT_PRIVATE_H__
#define __PLAT_PRIVATE_H__

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/
void plat_delay_timer_init(void);

/*
 * GIC operation, mandatory functions required in Marvell standard platforms
 */
void plat_marvell_gic_driver_init(void);
void plat_marvell_gic_init(void);
void plat_marvell_gic_cpuif_enable(void);
void plat_marvell_gic_cpuif_disable(void);
void plat_marvell_gic_pcpu_init(void);
void plat_marvell_gic_irq_save(void);
void plat_marvell_gic_irq_restore(void);
void plat_marvell_gic_irq_pcpu_save(void);
void plat_marvell_gic_irq_pcpu_restore(void);

#endif /* __PLAT_PRIVATE_H__ */
