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
#ifndef __BOARD_MARVELL_DEF_H__
#define __BOARD_MARVELL_DEF_H__

/*
 * Required platform porting definitions common to all ARM
 * development platforms
 */

/* Size of cacheable stacks */
#if DEBUG_XLAT_TABLE
# define PLATFORM_STACK_SIZE 0x800
#elif IMAGE_BL1
#if TRUSTED_BOARD_BOOT
# define PLATFORM_STACK_SIZE 0x1000
#else
# define PLATFORM_STACK_SIZE 0x440
#endif
#elif IMAGE_BL2
# if TRUSTED_BOARD_BOOT
#  define PLATFORM_STACK_SIZE 0x1000
# else
#  define PLATFORM_STACK_SIZE 0x400
# endif
#elif IMAGE_BL31
# define PLATFORM_STACK_SIZE 0x400
#elif IMAGE_BL32
# define PLATFORM_STACK_SIZE 0x440
#endif

/*
 * PLAT_MARVELL_MMAP_ENTRIES depends on the number of entries in the
 * plat_arm_mmap array defined for each BL stage.
 */
#if IMAGE_BLE
#  define PLAT_MARVELL_MMAP_ENTRIES	3
#endif
#if IMAGE_BL1
#  if TRUSTED_BOARD_BOOT
#   define PLAT_MARVELL_MMAP_ENTRIES	7
#  else
#   define PLAT_MARVELL_MMAP_ENTRIES	6
#  endif	/* TRUSTED_BOARD_BOOT */
#endif
#if IMAGE_BL2
#  define PLAT_MARVELL_MMAP_ENTRIES		8
#endif
#if IMAGE_BL31
#define PLAT_MARVELL_MMAP_ENTRIES		5
#endif

/*
 * Platform specific page table and MMU setup constants
 */
#if IMAGE_BL1
#define MAX_XLAT_TABLES			4
#elif IMAGE_BLE
#  define MAX_XLAT_TABLES		4
#elif IMAGE_BL2
#  define MAX_XLAT_TABLES		4
#elif IMAGE_BL31
# define MAX_XLAT_TABLES		4
#elif IMAGE_BL32
#  define MAX_XLAT_TABLES               4
#endif

#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4

#define PLAT_MARVELL_TRUSTED_SRAM_SIZE	0x80000	/* 512 KB */


#endif /* __BOARD_MARVELL_DEF_H__ */
