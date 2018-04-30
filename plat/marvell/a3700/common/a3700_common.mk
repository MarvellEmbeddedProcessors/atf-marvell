#
# Copyright (C) 2016 Marvell International Ltd.
#
# SPDX-License-Identifier:	BSD-3-Clause
# https://spdx.org/licenses
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
				drivers/arm/gic/v3/arm_gicv3_common.c	\
				plat/common/plat_gicv3.c		\
				drivers/arm/gic/v3/gic500.c

ATF_INCLUDES		:=	-Iinclude/common/tbbr		\
				-Iinclude/drivers

PLAT_INCLUDES		:=	-I$(PLAT_FAMILY_BASE)/$(PLAT)			\
				-I$(PLAT_COMMON_BASE)/include			\
				-I$(PLAT_INCLUDE_BASE)/common			\
				-I$(MARVELL_DRV_BASE)/uart			\
				-I$/drivers/arm/gic/common/			\
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
