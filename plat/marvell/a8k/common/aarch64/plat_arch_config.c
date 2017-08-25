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

#include <platform.h>
#include <arch_helpers.h>
#include <mmio.h>
#include <debug.h>
#include <cache_llc.h>


#define CCU_HTC_ASET			(MVEBU_CCU_BASE + 0x264)
#define MVEBU_IO_AFFINITY		(0xF00)


void plat_enable_affinity(void)
{
	int cluster_id;
	int affinity;

	/* set CPU Affinity */
	cluster_id = plat_my_core_pos() / PLAT_MARVELL_CLUSTER_CORE_COUNT;
	affinity = (MVEBU_IO_AFFINITY | (1 << cluster_id));
	mmio_write_32(CCU_HTC_ASET, affinity);

	/* set barier */
	__asm__ volatile("isb");
}

void psci_arch_init(int die_index)
{
#if !LLC_DISABLE
	/* check if LLC is in exclusive mode
	 * as L2 is configured to UniqueClean eviction
	 * (in a8k reset handler)
	 */
	if (llc_is_exclusive(0) == 0)
		ERROR("LLC should be configured to exclusice mode\n");
#endif

	/* Enable Affinity */
	plat_enable_affinity();
}
