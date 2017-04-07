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

#ifndef __MVEBU_A3700_PM_H__
#define __MVEBU_A3700_PM_H__

/* supported wake up sources */
enum pm_wake_up_src_type {
	WAKE_UP_SRC_GPIO,
	/* FOLLOWING SRC NOT SUPPORTED YET */
	WAKE_UP_SRC_TIMER,
	WAKE_UP_SRC_UART0,
	WAKE_UP_SRC_UART1,
	WAKE_UP_SRC_MAX,
};

struct pm_gpio_data {
	/*
	 * bank 0: North bridge GPIO
	 * bank 1: South bridge GPIO
	 */
	uint32_t bank_num;
	uint32_t gpio_num;
};

union pm_wake_up_src_data {
	struct pm_gpio_data gpio_data;
	/* delay in seconds */
	uint32_t timer_delay;
};

struct pm_wake_up_src {
	enum pm_wake_up_src_type wake_up_src_type;

	union pm_wake_up_src_data wake_up_data;
};

struct pm_wake_up_src_config {
	uint32_t	wake_up_src_num;
	struct pm_wake_up_src wake_up_src[WAKE_UP_SRC_MAX];
};

struct pm_wake_up_src_config *mv_wake_up_src_config_get(void);


#endif /* __MVEBU_A3700_PM_H__ */
