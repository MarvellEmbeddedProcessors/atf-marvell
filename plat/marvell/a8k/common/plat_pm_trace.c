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

#include <mmio.h>
#include <platform.h>
#include <plat_pm_trace.h>
#include <mss_mem.h>

#ifdef PM_TRACE_ENABLE

/* core trace APIs */
core_trace_func funcTbl[PLATFORM_CORE_COUNT] = {
	pm_core_0_trace,
	pm_core_1_trace,
	pm_core_2_trace,
	pm_core_3_trace};

/*******************************************************************************
 * pm_core0_trace
 * pm_core1_trace
 * pm_core2_trace
 * pm_core_3trace
 *
 * This functions set trace info into core cyclic trace queue in MSS SRAM
 * memory space
 ******************************************************************************/
void pm_core_0_trace(unsigned int trace)
{
	unsigned int current_position_core_0 =
			mmio_read_32(AP_MSS_ATF_CORE_0_CTRL_BASE);
	mmio_write_32((AP_MSS_ATF_CORE_0_INFO_BASE  +
		     (current_position_core_0 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     mmio_read_32(AP_MSS_TIMER_BASE));
	mmio_write_32((AP_MSS_ATF_CORE_0_INFO_TRACE +
		     (current_position_core_0 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     trace);
	mmio_write_32(AP_MSS_ATF_CORE_0_CTRL_BASE,
		     ((current_position_core_0 + 1) &
		     AP_MSS_ATF_TRACE_SIZE_MASK));
}

void pm_core_1_trace(unsigned int trace)
{
	unsigned int current_position_core_1 =
			mmio_read_32(AP_MSS_ATF_CORE_1_CTRL_BASE);
	mmio_write_32((AP_MSS_ATF_CORE_1_INFO_BASE +
		     (current_position_core_1 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     mmio_read_32(AP_MSS_TIMER_BASE));
	mmio_write_32((AP_MSS_ATF_CORE_1_INFO_TRACE +
		     (current_position_core_1 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     trace);
	mmio_write_32(AP_MSS_ATF_CORE_1_CTRL_BASE,
		     ((current_position_core_1 + 1) &
		     AP_MSS_ATF_TRACE_SIZE_MASK));
}

void pm_core_2_trace(unsigned int trace)
{
	unsigned int current_position_core_2 =
			mmio_read_32(AP_MSS_ATF_CORE_2_CTRL_BASE);
	mmio_write_32((AP_MSS_ATF_CORE_2_INFO_BASE +
		     (current_position_core_2 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     mmio_read_32(AP_MSS_TIMER_BASE));
	mmio_write_32((AP_MSS_ATF_CORE_2_INFO_TRACE +
		     (current_position_core_2 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     trace);
	mmio_write_32(AP_MSS_ATF_CORE_2_CTRL_BASE,
		     ((current_position_core_2 + 1) &
		     AP_MSS_ATF_TRACE_SIZE_MASK));
}

void pm_core_3_trace(unsigned int trace)
{
	unsigned int current_position_core_3 =
			mmio_read_32(AP_MSS_ATF_CORE_3_CTRL_BASE);
	mmio_write_32((AP_MSS_ATF_CORE_3_INFO_BASE +
		     (current_position_core_3 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     mmio_read_32(AP_MSS_TIMER_BASE));
	mmio_write_32((AP_MSS_ATF_CORE_3_INFO_TRACE +
		     (current_position_core_3 * AP_MSS_ATF_CORE_ENTRY_SIZE)),
		     trace);
	mmio_write_32(AP_MSS_ATF_CORE_3_CTRL_BASE,
		     ((current_position_core_3 + 1) &
		     AP_MSS_ATF_TRACE_SIZE_MASK));
}
#endif /* PM_TRACE_ENABLE */
