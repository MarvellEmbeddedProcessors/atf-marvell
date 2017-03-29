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

#ifndef _MVEBU_H_
#define _MVEBU_H_

/* Bit definitions */
#define NO_BIT		0x00000000
#define BIT0		0x00000001
#define BIT1		0x00000002
#define BIT2		0x00000004
#define BIT3		0x00000008
#define BIT4		0x00000010
#define BIT5		0x00000020
#define BIT6		0x00000040
#define BIT7		0x00000080
#define BIT8		0x00000100
#define BIT9		0x00000200
#define BIT10		0x00000400
#define BIT11		0x00000800
#define BIT12		0x00001000
#define BIT13		0x00002000
#define BIT14		0x00004000
#define BIT15		0x00008000
#define BIT16		0x00010000
#define BIT17		0x00020000
#define BIT18		0x00040000
#define BIT19		0x00080000
#define BIT20		0x00100000
#define BIT21		0x00200000
#define BIT22		0x00400000
#define BIT23		0x00800000
#define BIT24		0x01000000
#define BIT25		0x02000000
#define BIT26		0x04000000
#define BIT27		0x08000000
#define BIT28		0x10000000
#define BIT29		0x20000000
#define BIT30		0x40000000
#define BIT31		0x80000000
#define BIT(x)		(1 << (x))

/* Use this functions only when printf is allowed */
#if defined(CONFIG_MVEBU_DEBUG_FUNC_IN_OUT) || defined(DEBUG)
#define debug_enter()	printf("----> Enter %s\n", __func__)
#define debug_exit()  printf("<---- Exit  %s\n", __func__)
#else
#define debug_enter()
#define debug_exit()
#endif

/* Macro for testing alignment. Positive if number is NOT aligned */
#define IS_NOT_ALIGN(number, align)	((number) & ((align) - 1))

/* Macro for alignment up. For example, ALIGN_UP(0x0330, 0x20) = 0x0340 */
#define ALIGN_UP(number, align) (((number) & ((align) - 1)) ? \
		(((number) + (align)) & ~((align)-1)) : (number))

/* Macro for testing whether a number is a power of 2. Positive if so */
#define IS_POWER_OF_2(number)	(number != 0 && ((number & (number - 1)) == 0))

/*
 * Macro for ronding up to next power of 2
 * it is done by count leading 0 (clz assembly opcode) and see msb set bit.
 * then you can shift it left and get number which power of 2
 * Note: this Macro is for 32 bit number
 */
#define ROUND_UP_TO_POW_OF_2(number)	(1 << (32 - __builtin_clz(number - 1)))

#endif	/* MVEBU_H */
