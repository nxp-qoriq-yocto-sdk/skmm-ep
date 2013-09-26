/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * Neither the name of Freescale Semiconductor nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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
#ifndef __SKMM_CACHE_H__
#define __SKMM_CACHE_H__

#include <common.h>

#define L2_CTRL_OFFSET		0x20000
#define L2_CACHE_SIZE		0x80000

#define L2CR_L2FI		0x40000000	/* L2 flash invalidate */
#define L2CR_L2IO		0x00200000	/* L2 instruction only */
#define L2CR_SRAM_ZERO		0x00000000	/* L2SRAM zero size */
#define L2CR_SRAM_FULL		0x00010000	/* L2SRAM full size */
#define L2CR_SRAM_HALF		0x00020000	/* L2SRAM half size */
#define L2CR_SRAM_TWO_HALFS	0x00030000	/* L2SRAM two half sizes */
#define L2CR_SRAM_QUART		0x00040000	/* L2SRAM one quarter size */
#define L2CR_SRAM_TWO_QUARTS	0x00050000	/* L2SRAM two quarter size */
#define L2CR_SRAM_EIGHTH	0x00060000	/* L2SRAM one eighth size */
#define L2CR_SRAM_TWO_EIGHTH	0x00070000	/* L2SRAM two eighth size */

#define L2SRAM_OPTIMAL_SZ_SHIFT	0x00000003	/* Optimum size for L2SRAM */

#define L2SRAM_BAR_MSK_LO18	0xFFFFC000	/* Lower 18 bits */
#define L2SRAM_BARE_MSK_HI4	0x0000000F	/* Upper 4 bits */
#define L2CR_L2E		0x80000000

enum cache_sram_lock_ways {
	LOCK_WAYS_ZERO,
	LOCK_WAYS_EIGHTH,
	LOCK_WAYS_TWO_EIGHTH,
	LOCK_WAYS_HALF = 4,
	LOCK_WAYS_FULL = 8,
};

struct mpc85xx_l2ctlr {
	u32	ctl;		/* 0x000 - L2 control */
	u8	res1[0xC];
	u32	ewar0;		/* 0x010 - External write address 0 */
	u32	ewarea0;	/* 0x014 - External write address extended 0 */
	u32	ewcr0;		/* 0x018 - External write ctrl */
	u8	res2[4];
	u32	ewar1;		/* 0x020 - External write address 1 */
	u32	ewarea1;	/* 0x024 - External write address extended 1 */
	u32	ewcr1;		/* 0x028 - External write ctrl 1 */
	u8	res3[4];
	u32	ewar2;		/* 0x030 - External write address 2 */
	u32	ewarea2;	/* 0x034 - External write address extended 2 */
	u32	ewcr2;		/* 0x038 - External write ctrl 2 */
	u8	res4[4];
	u32	ewar3;		/* 0x040 - External write address 3 */
	u32	ewarea3;	/* 0x044 - External write address extended 3 */
	u32	ewcr3;		/* 0x048 - External write ctrl 3 */
	u8	res5[0xB4];
	u32	srbar0;		/* 0x100 - SRAM base address 0 */
	u32	srbarea0;	/* 0x104 - SRAM base addr reg ext address 0 */
	u32	srbar1;		/* 0x108 - SRAM base address 1 */
	u32	srbarea1;	/* 0x10C - SRAM base addr reg ext address 1 */
	u8	res6[0xCF0];
	u32	errinjhi;	/* 0xE00 - Error injection mask high */
	u32	errinjlo;	/* 0xE04 - Error injection mask low */
	u32	errinjctl;	/* 0xE08 - Error injection tag/ecc control */
	u8	res7[0x14];
	u32	captdatahi;	/* 0xE20 - Error data high capture */
	u32	captdatalo;	/* 0xE24 - Error data low capture */
	u32	captecc;	/* 0xE28 - Error syndrome */
	u8	res8[0x14];
	u32	errdet;		/* 0xE40 - Error detect */
	u32	errdis;		/* 0xE44 - Error disable */
	u32	errinten;	/* 0xE48 - Error interrupt enable */
	u32	errattr;	/* 0xE4c - Error attribute capture */
	u32	erradrrl;	/* 0xE50 - Error address capture low */
	u32	erradrrh;	/* 0xE54 - Error address capture high */
	u32	errctl;		/* 0xE58 - Error control */
	u8	res9[0x1A4];
};

/**
 *  * upper_32_bits - return bits 32-63 of a number
 *   * @n: the number we're accessing
 *    *
 *     * A basic shift-right of a 64- or 32-bit quantity.  Use this to suppress
 *      * the "right shift count >= width of type" warning when that quantity is
 *       * 32-bits.
 *        */
#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))

/**
 *  * lower_32_bits - return bits 0-31 of a number
 *   * @n: the number we're accessing
 *    */
#define lower_32_bits(n) ((u32)(n))

int fsl_init_l2ctrl(phys_addr_t sram_addr, u32 sram_size);

#endif /*  __SKMM_CACHE_H__ */
