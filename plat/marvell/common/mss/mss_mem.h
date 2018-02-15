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

#ifndef __MSS_PM_MEM_H
#define __MSS_PM_MEM_H

/* MSS SRAM Memory base */
#define MSS_SRAM_PM_CONTROL_BASE		(MVEBU_REGS_BASE + 0x520000)

enum mss_pm_ctrl_handshake {
	MSS_UN_INITIALIZED	= 0,
	MSS_COMPATIBILITY_ERROR	= 1,
	MSS_ACKNOWLEDGEMENT	= 2,
	HOST_ACKNOWLEDGEMENT	= 3
};

enum mss_pm_ctrl_rtos_env {
	MSS_MULTI_PROCESS_ENV	= 0,
	MSS_SINGLE_PROCESS_ENV	= 1,
	MSS_MAX_PROCESS_ENV
};

struct mss_pm_ctrl_block {
	/* This field is used to synchronize the Host
	 * and MSS initialization sequence
	 * Valid Values
	 * 0 - Un-Initialized
	 * 1 - Compatibility Error
	 * 2 - MSS Acknowledgment
	 * 3 - Host Acknowledgment
	 */
	unsigned int handshake;

	/*
	 * This field include Host IPC version. Once received by the MSS
	 * It will be compared to MSS IPC version and set MSS Acknowledge to
	 * "compatibility error" in case there is no match
	 */
	unsigned int ipc_version;
	unsigned int ipc_base_address;
	unsigned int ipc_state;

	/* Following fields defines firmware core architecture */
	unsigned int num_of_cores;
	unsigned int num_of_clusters;
	unsigned int num_of_cores_per_cluster;

	/* Following fields define pm trace debug base address */
	unsigned int pm_trace_ctrl_base_address;
	unsigned int pm_trace_info_base_address;
	unsigned int pm_trace_info_core_size;

	unsigned int ctrl_blk_size;
};

#endif /* __MSS_PM_MEM_H */
