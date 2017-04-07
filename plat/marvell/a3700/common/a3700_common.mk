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

PLAT_FAMILY		:= a3700
PLAT_FAMILY_BASE	:= plat/marvell/$(PLAT_FAMILY)
PLAT_INCLUDE_BASE	:= include/plat/marvell/$(PLAT_FAMILY)
PLAT_COMMON_BASE	:= $(PLAT_FAMILY_BASE)/common
MARVELL_DRV_BASE	:= drivers/marvell

CALL_DOIMAGE		:= y

# GICV3
$(eval $(call add_define,CONFIG_GICV3))

# CCI-400
$(eval $(call add_define,USE_CCI))

MARVELL_GIC_SOURCES	:=	drivers/arm/gic/common/gic_common.c	\
				drivers/arm/gic/v3/gicv3_main.c		\
				drivers/arm/gic/v3/gicv3_helpers.c	\
				plat/common/plat_gicv3.c

ATF_INCLUDES		:=	-Iinclude/common/tbbr		\
				-Iinclude/drivers

PLAT_INCLUDES		:=	-I$(PLAT_FAMILY_BASE)/$(PLAT)			\
				-I$(PLAT_COMMON_BASE)/include			\
				-I$(PLAT_INCLUDE_BASE)/common			\
				-I$(MARVELL_DRV_BASE)/uart			\
				$(ATF_INCLUDES)

PLAT_BL_COMMON_SOURCES	:=	$(PLAT_COMMON_BASE)/aarch64/a3700_common.c	\
				drivers/console/aarch64/console.S		\
				plat/marvell/common/marvell_cci.c		\
				$(MARVELL_DRV_BASE)/uart/a3700_console.S

BL1_SOURCES		+=	$(PLAT_COMMON_BASE)/aarch64/plat_helpers.S	\
				lib/cpus/aarch64/cortex_a53.S

BL31_PORTING_SOURCES	:=	$(PLAT_FAMILY_BASE)/$(PLAT)/board/pm_src.c

BL31_SOURCES		+=	lib/cpus/aarch64/cortex_a53.S		\
				$(PLAT_COMMON_BASE)/aarch64/plat_helpers.S	\
				$(PLAT_COMMON_BASE)/plat_pm.c		\
				$(PLAT_COMMON_BASE)/a3700_dram_cs.c	\
				$(PLAT_COMMON_BASE)/dram_win.c		\
				$(PLAT_COMMON_BASE)/io_addr_dec.c	\
				$(PLAT_COMMON_BASE)/marvell_plat_config.c	\
				$(PLAT_FAMILY_BASE)/$(PLAT)/plat_bl31_setup.c	\
				plat/marvell/common/sys_info.c		\
				plat/marvell/common/marvell_gicv3.c	\
				$(MARVELL_GIC_SOURCES)			\
				drivers/arm/cci/cci.c			\
				$(BL31_PORTING_SOURCES)

# Disable the PSCI platform compatibility layer (allows porting from Old Platform APIs
# to the new APIs).
# It is not needed since Marvell platform already used the new platform APIs.
ENABLE_PLAT_COMPAT	:= 	0
