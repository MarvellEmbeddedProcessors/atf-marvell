#
# Copyright (C) 2016 - 2018 Marvell International Ltd.
#
# SPDX-License-Identifier:     BSD-3-Clause
# https://spdx.org/licenses

CP_NUM			:= 2
$(eval $(call add_define,CP_NUM))

include plat/marvell/a8k/common/a8k_common.mk

include plat/marvell/common/marvell_common.mk
