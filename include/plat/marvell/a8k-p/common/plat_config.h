/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include <amb_adec.h>
#include <io_win.h>
#include <iob.h>
#include <ccu.h>
#include <pci_ep.h>

/*
 * This struct supports skip image request
 * detection_method: the method used to detect the request "signal".
 * info:
 *	GPIO:
 *		detection_method: HIGH (pressed button), LOW (unpressed button),
 *		num (button mpp number).
 *	i2c:
 *		i2c_addr: the address of the i2c chosen.
 *		i2d_reg: the i2c register chosen.
 *	test:
 *		choose the DIE you picked the button in (AP or CP).
 *		in case of CP(cp_index = 0 if CP0, cp_index = 1 if CP1)
 */
struct skip_image {
	enum {
		GPIO,
		I2C,
		USER_DEFINED
	} detection_method;

	struct {
		struct {
			int num;
			enum {
				HIGH,
				LOW
			} button_state;

		} gpio;

		struct {
			int i2c_addr;
			int i2c_reg;
		} i2c;

		struct {
			enum {
				CP,
				AP
			} cp_ap;
			int cp_index;
		} test;
	} info;
};

uintptr_t marvell_get_amb_reg_offs(int cp_index);
uintptr_t marvell_get_io_win_reg_offs(int);
uint32_t marvell_get_io_win_gcr_target(int);
uintptr_t marvell_get_iob_reg_offs(int cp_index);
int marvell_get_iob_max_win(void);
uintptr_t marvell_get_ccu_reg_offs(int ap);
uint32_t marvell_get_ccu_gcr_target(int);
int marvell_get_ccu_max_win(void);

/*
 * The functions below are defined as Weak and may be overridden
 * in specific Marvell standard platform
 */
int marvell_get_amb_memory_map(struct amb_win **win, uint32_t *size);
int marvell_get_io_win_memory_map(int, struct io_win **win, uint32_t *size);
int marvell_get_iob_memory_map(struct iob_win **win,
			       uint32_t *size, int cp_index);
int marvell_get_ccu_memory_map(int ap, struct ccu_win **win, uint32_t *size);

#endif /* __BOARD_CONFIG_H__ */
