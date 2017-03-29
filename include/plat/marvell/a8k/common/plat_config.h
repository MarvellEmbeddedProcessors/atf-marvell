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
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include <amb_adec.h>
#include <rfu.h>
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
uintptr_t marvell_get_rfu_reg_offs(void);
uintptr_t marvell_get_iob_reg_offs(int cp_index);
int marvell_get_iob_max_win(void);
uintptr_t marvell_get_ccu_reg_offs(void);
int marvell_get_ccu_max_win(void);


/*
 * The functions below are defined as Weak and may be overridden
 * in specific Marvell standard platform
 */
int marvell_get_amb_memory_map(struct amb_win **win, uint32_t *size);
int marvell_get_rfu_memory_map(struct rfu_win **win, uint32_t *size);
int marvell_get_iob_memory_map(struct iob_win **win,
			       uint32_t *size, int cp_index);
int marvell_get_ccu_memory_map(struct ccu_win **win, uint32_t *size);

#endif /* __BOARD_CONFIG_H__ */
