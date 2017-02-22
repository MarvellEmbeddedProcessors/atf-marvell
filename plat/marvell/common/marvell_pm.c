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
/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
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
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch_helpers.h>
#include <assert.h>
#include <psci.h>
#include <marvell_pm.h>

/* Standard ARM platforms are expected to export plat_arm_psci_pm_ops */
extern const plat_psci_ops_t plat_arm_psci_pm_ops;

/*******************************************************************************
 * Private function to program the mailbox for a cpu before it is released
 * from reset. This function assumes that the mail box base is within
 * the MARVELL_SHARED_RAM region
 ******************************************************************************/
void marvell_program_mailbox(uintptr_t address)
{
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	/*
	 * Ensure that the PLAT_MARVELL_MAILBOX_BASE is within
	 * MARVELL_SHARED_RAM region.
	 */
	assert((PLAT_MARVELL_MAILBOX_BASE >= MARVELL_SHARED_RAM_BASE) &&
	       ((PLAT_MARVELL_MAILBOX_BASE + sizeof(*mailbox)) <=
	       (MARVELL_SHARED_RAM_BASE + MARVELL_SHARED_RAM_SIZE)));

	mailbox[MBOX_IDX_MAGIC] = MVEBU_MAILBOX_MAGIC_NUM;
	mailbox[MBOX_IDX_SEC_ADDR] = address;

	/* Flush data cache if the mail box shared RAM is cached */
#if PLAT_MARVELL_SHARED_RAM_CACHED
	flush_dcache_range((uintptr_t)PLAT_MARVELL_MAILBOX_BASE +
			   8 * MBOX_IDX_MAGIC,
			   2 * sizeof(uint64_t));
#endif
}

/*******************************************************************************
 * The ARM Standard platform definition of platform porting API
 * `plat_setup_psci_ops`.
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	*psci_ops = &plat_arm_psci_pm_ops;

	/* Setup mailbox with entry point. */
	marvell_program_mailbox(sec_entrypoint);
	return 0;
}
