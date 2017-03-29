#
# ***************************************************************************
# Copyright (C) 2016 Marvell International Ltd.
# ***************************************************************************
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of Marvell nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

MARVELL_PLAT_BASE		:= plat/marvell
MARVELL_PLAT_INCLUDE_BASE	:= include/plat/marvell

PLAT_INCLUDES		+=	-I.	-Iinclude/common/tbbr		       \
				-I$(MARVELL_PLAT_INCLUDE_BASE)/common	       \
				-I$(MARVELL_PLAT_INCLUDE_BASE)/common/aarch64  \
				-I$(MARVELL_PLAT_INCLUDE_BASE)/common/board


PLAT_BL_COMMON_SOURCES  +=      lib/xlat_tables/xlat_tables_common.c			\
				lib/xlat_tables/aarch64/xlat_tables.c			\
				$(MARVELL_PLAT_BASE)/common/aarch64/marvell_common.c	\
				$(MARVELL_PLAT_BASE)/common/aarch64/marvell_helpers.S	\
				plat/common/aarch64/plat_common.c

BL1_SOURCES		+=	drivers/io/io_fip.c					\
				drivers/io/io_memmap.c					\
				drivers/io/io_storage.c					\
				$(MARVELL_PLAT_BASE)/common/marvell_bl1_setup.c		\
				$(MARVELL_PLAT_BASE)/common/marvell_io_storage.c	\
				plat/common/aarch64/platform_up_stack.S
ifdef EL3_PAYLOAD_BASE
# Need the arm_program_trusted_mailbox() function to release secondary CPUs from
# their holding pen
endif

BL2_SOURCES		+=	drivers/io/io_fip.c					\
				drivers/io/io_memmap.c					\
				drivers/io/io_storage.c					\
				$(MARVELL_PLAT_BASE)/common/marvell_bl2_setup.c		\
				$(MARVELL_PLAT_BASE)/common/marvell_io_storage.c	\
				plat/common/aarch64/platform_up_stack.S

BL31_SOURCES		+=	$(MARVELL_PLAT_BASE)/common/marvell_bl31_setup.c	\
				$(MARVELL_PLAT_BASE)/common/marvell_pm.c		\
				$(MARVELL_PLAT_BASE)/common/marvell_topology.c		\
				plat/common/aarch64/platform_mp_stack.S			\
				plat/common/plat_psci_common.c				\
				$(MARVELL_PLAT_BASE)/common/plat_delay_timer.c		\
				drivers/delay_timer/delay_timer.c

# PSCI functionality
$(eval $(call add_define,CONFIG_ARM64))
