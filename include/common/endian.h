/* Copyright 2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

/**
 * SWAP_UINT16(val) - Swaps the byte order of a given 16-bit value.
 * @val:         the 16-bit value to swap
 * @return:      the byte-swapped value
 *
 * Caution: the given value is evaluated multiple times by this macro. For
 * calculated expressions or expressions that contain function calls it
 * is recommended to use the SwapUint16() routine.
 */
#define SWAP_UINT16(val) \
	((uint16_t)((((val) & 0x00FF) << 8) | (((val) & 0xFF00) >> 8)))

/**
 * SWAP_UINT32(val) - Swaps the byte order of a given 32-bit value.
 * @val:         the 32-bit value to swap
 * @return:      the byte-swapped value
 *
 * Caution: the given value is evaluated multiple times by this macro. For
 * calculated expressions or expressions that contain function calls it
 * is recommended to use the SwapUint32() routine.
 */
#define SWAP_UINT32(val) \
	((uint32_t)((((val) & 0x000000FF) << 24) | \
		(((val) & 0x0000FF00) <<  8) | \
		(((val) & 0x00FF0000) >>  8) | \
		(((val) & 0xFF000000) >> 24)))
