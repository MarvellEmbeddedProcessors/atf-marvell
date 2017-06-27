
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

PLAT_FAMILY		:= a8k
PLAT_FAMILY_BASE	:= plat/marvell/$(PLAT_FAMILY)
PLAT_INCLUDE_BASE	:= include/plat/marvell/$(PLAT_FAMILY)
PLAT_COMMON_BASE	:= $(PLAT_FAMILY_BASE)/common
MARVELL_DRV_BASE	:= drivers/marvell

CALL_DOIMAGE		:= y

# This define specifies DDR type for BLE
$(eval $(call add_define,CONFIG_DDR4))

MARVELL_GIC_SOURCES	:=	drivers/arm/gic/common/gic_common.c	\
				drivers/arm/gic/v2/gicv2_main.c		\
				drivers/arm/gic/v2/gicv2_helpers.c	\
				plat/common/plat_gicv2.c

ATF_INCLUDES		:=	-Iinclude/common/tbbr

PLAT_INCLUDES		:=	-I$(PLAT_FAMILY_BASE)/$(PLAT)		\
				-I$(PLAT_COMMON_BASE)/include		\
				-I$(PLAT_INCLUDE_BASE)/common		\
				-Iinclude/drivers/marvell		\
				-Iinclude/drivers/marvell/mochi		\
				$(ATF_INCLUDES)

PLAT_BL_COMMON_SOURCES	:=	$(PLAT_COMMON_BASE)/aarch64/a8k_common.c \
				drivers/console/aarch64/console.S	 \
				drivers/ti/uart/aarch64/16550_console.S

BLE_PORTING_SOURCES	:=	$(PLAT_FAMILY_BASE)/$(PLAT)/board/dram_port.c \
			$(PLAT_FAMILY_BASE)/$(PLAT)/board/marvell_plat_config.c

BLE_SOURCES		:=	plat/marvell/common/sys_info.c		 \
				plat/marvell/a8k/common/plat_ble_setup.c \
				$(MARVELL_DRV_BASE)/mochi/cp110_setup.c	 \
				$(MARVELL_DRV_BASE)/i2c/a8k_i2c.c	 \
				$(PLAT_COMMON_BASE)/plat_pm.c		 \
				$(BLE_PORTING_SOURCES)
ifeq (${PCI_EP_SUPPORT}, 1)
BLE_SOURCES		+=	plat/marvell/common/pci_ep_setup.c	 \
				$(MARVELL_DRV_BASE)/dw-pcie-ep.c	 \
				$(MARVELL_DRV_BASE)/pcie-comphy-cp110.c
endif

ifeq (${PALLADIUM}, 1)
BL1_SOURCES		+=	$(PLAT_COMMON_BASE)/plat_bl1_setup.c
endif

BL1_SOURCES		+=	$(PLAT_COMMON_BASE)/aarch64/plat_helpers.S \
				lib/cpus/aarch64/cortex_a72.S

MARVELL_DRV		:= 	$(MARVELL_DRV_BASE)/rfu.c	\
				$(MARVELL_DRV_BASE)/iob.c	\
				$(MARVELL_DRV_BASE)/mci.c	\
				$(MARVELL_DRV_BASE)/amb_adec.c	\
				$(MARVELL_DRV_BASE)/ccu.c	\
				$(MARVELL_DRV_BASE)/icu.c	\
				$(MARVELL_DRV_BASE)/cache_llc.c

MARVELL_MOCHI_DRV	:=	$(MARVELL_DRV_BASE)/mochi/apn806_setup.c \
				$(MARVELL_DRV_BASE)/mochi/cp110_setup.c

BL31_PORTING_SOURCES	:=	$(PLAT_FAMILY_BASE)/$(PLAT)/board/marvell_plat_config.c

BL31_SOURCES		+=	lib/cpus/aarch64/cortex_a72.S		       \
				$(PLAT_COMMON_BASE)/aarch64/plat_helpers.S     \
				$(PLAT_COMMON_BASE)/aarch64/plat_arch_config.c \
				$(PLAT_COMMON_BASE)/plat_pm.c		       \
				$(PLAT_COMMON_BASE)/plat_bl31_setup.c	       \
				plat/marvell/common/marvell_gicv2.c	       \
				$(BL31_PORTING_SOURCES)			       \
				$(MARVELL_DRV)				       \
				$(MARVELL_MOCHI_DRV)			       \
				$(MARVELL_GIC_SOURCES)

# Add trace functionality for PM
ifneq (${SCP_BL2},)
BL31_SOURCES		+=	$(PLAT_COMMON_BASE)/plat_pm_trace.c
endif

# Disable the PSCI platform compatibility layer (allows porting
# from Old Platform APIs to the new APIs).
# It is not needed since Marvell platform already used the new platform APIs.
ENABLE_PLAT_COMPAT	:= 	0

# MSS (SCP) build
ifneq (${SCP_BL2},)
include plat/marvell/a8k/common/mss/mss_common.mk
endif
