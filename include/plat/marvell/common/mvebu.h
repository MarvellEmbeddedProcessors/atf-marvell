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
