/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
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

#include <common.h>

#define CONFIG_SYS_HZ 		1000

#define mfspr(reg) \
({ \
	register_t ret; \
	asm volatile("mfspr %0, %1" : "=r" (ret) : "i" (reg) : "memory"); \
	ret; \
})

#define CCSR_PHYS_ADDR 		0xfffe00000ull

phys_addr_t get_ccsr_phys_addr(void)
{
	return CCSR_PHYS_ADDR;
}

inline u64 getticks(void)
{
	u32 l = 0, h = 0;

	asm volatile ("mfspr %0, 526 " : "=r" (l));
	asm volatile ("mfspr %0, 527 " : "=r" (h));

	return ((u64) h << 32) | l;
}

unsigned long get_tbclk(void)
{
	unsigned long tbclk;

	tbclk = CONFIG_SYS_HZ;
	return tbclk;
}

/*
  * This function is intended for SHORT delays only.
  * It will overflow at around 10 seconds @ 400MHz,
  * or 20 seconds @ 200MHz.
  */
unsigned long usec2ticks(unsigned long usec)
{
	unsigned long ticks;

	if (usec < 1000)
		ticks = ((usec * (get_tbclk() / 1000)) + 500) / 1000;

	else
		ticks = ((usec / 10) * (get_tbclk() / 100000));

	return ticks;
}
