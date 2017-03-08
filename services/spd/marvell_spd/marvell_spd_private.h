/*
 * ***************************************************************************
 * Copyright (C) 2017 Marvell International Ltd.
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

#ifndef __MARVELL_SPD_PRIVATE_H__
#define __MARVELL_SPD_PRIVATE_H__

#include <arch.h>
#include <context.h>
#include <interrupt_mgmt.h>
#include <platform_def.h>

/*******************************************************************************
 * Secure Payload execution state information i.e. aarch32 or aarch64
 ******************************************************************************/
#define MARVELL_SPD_AARCH32		MODE_RW_32
#define MARVELL_SPD_AARCH64		MODE_RW_64

/*******************************************************************************
 * Number of cpus that the present on this platform. TODO: Rely on a topology
 * tree to determine this in the future to avoid assumptions about mpidr
 * allocation
 ******************************************************************************/
#define MARVELL_SPD_CORE_COUNT		PLATFORM_CORE_COUNT

#define MARVELL_SPD_C_RT_CTX_X19		0x0
#define MARVELL_SPD_C_RT_CTX_X20		0x8
#define MARVELL_SPD_C_RT_CTX_X21		0x10
#define MARVELL_SPD_C_RT_CTX_X22		0x18
#define MARVELL_SPD_C_RT_CTX_X23		0x20
#define MARVELL_SPD_C_RT_CTX_X24		0x28
#define MARVELL_SPD_C_RT_CTX_X25		0x30
#define MARVELL_SPD_C_RT_CTX_X26		0x38
#define MARVELL_SPD_C_RT_CTX_X27		0x40
#define MARVELL_SPD_C_RT_CTX_X28		0x48
#define MARVELL_SPD_C_RT_CTX_X29		0x50
#define MARVELL_SPD_C_RT_CTX_X30		0x58
#define MARVELL_SPD_C_RT_CTX_SIZE		0x60
#define MARVELL_SPD_C_RT_CTX_ENTRIES		(MARVELL_SPD_C_RT_CTX_SIZE >> DWORD_SHIFT)

#ifndef __ASSEMBLY__

/*******************************************************************************
 * Structure which helps the SPD to maintain the per-cpu state of the SP.
 * 'mpidr'          - mpidr to associate a context with a cpu
 * 'c_rt_ctx' - spaces to restore C runtime context from after returning
 *              from a synchronous entry into the SP.
 * 'cpu_ctx'  - space to maintain SP architectural state
 ******************************************************************************/
struct marvell_spd_context {
	uint32_t state;
	uint64_t mpidr;
	uint64_t c_rt_ctx;
	cpu_context_t cpu_ctx;
};


/*******************************************************************************
 * Function & Data prototypes
 ******************************************************************************/
uint64_t marvell_spd_enter_sp(uint64_t *c_rt_ctx);
void __dead2 marvell_spd_exit_sp(uint64_t c_rt_ctx, uint64_t ret);
uint64_t marvell_spd_synchronous_sp_entry(struct marvell_spd_context *marvell_spd_ctx);
void __dead2 marvell_spd_synchronous_sp_exit(struct marvell_spd_context *marvell_spd_ctx, uint64_t ret);
void marvell_spd_init_marvell_spd_ep_state(struct entry_point_info *marvell_spd_entry_point,
				uint32_t rw,
				uint64_t pc,
				struct marvell_spd_context *marvell_spd_ctx);
uint64_t marvell_spd_smc_handler(uint32_t smc_fid, uint64_t x1, uint64_t x2,
			 uint64_t x3, uint64_t x4, void *cookie, void *handle,
			 uint64_t flags);
void marvell_spd_switch_to(uint64_t dst_world);
struct marvell_spd_context marvell_spd_sp_context;
#endif /*__ASSEMBLY__*/

#endif /* __MARVELL_SPD_PRIVATE_H__ */
