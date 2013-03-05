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
 * iowrite8be() - Set the value of an 8-bit big-endian memory mapped register.
 */
static inline void iowrite8be(uint8_t val, uint8_t *addr)
{
	asm volatile("sync; stb%U0%X0 %1, %0" : "=m"
			(*addr) : "r" (val) : "memory");
}

/**
 * iowrite16be() - Set the value of a 16-bit big-endian memory mapped register
 */
static inline void iowrite16be(uint16_t val, uint16_t *addr)
{
	asm volatile("sync; sth%U0%X0 %1, %0" : "=m"
			(*addr) : "r" (val) : "memory");
}

/**
 * iowrite32be() - Set the value of a 32-bit big-endian memory mapped register
 */
static inline void iowrite32be(uint32_t val, uint32_t *addr)
{
	asm volatile("sync; stw%U0%X0 %1,%0" : "=m"
			(*addr) : "r" (val) : "memory");
}

/**
 * ioread8be() - Get the value of an 8-bit big-endian memory mapped register
 */
static inline uint8_t ioread8be(const uint8_t *addr)
{
	uint8_t ret;
	asm volatile("lbz%U1%X1 %0, %1" : "=r"
			(ret) : "m" (*addr) : "memory");
	return ret;
}

/**
 * ioread16be() - Get the value of a 16-bit big-endian memory mapped register
 */
static inline uint16_t ioread16be(const uint16_t *addr)
{
	uint16_t ret;
	asm volatile("lhz%U1%X1 %0, %1" : "=r"
			(ret) : "m" (*addr) : "memory");
	return ret;
}

/**
 * ioread32be() - Get the value of a 32-bit big-endian memory mapped register
 */
static inline uint32_t ioread32be(const uint32_t *addr)
{
	uint32_t ret;
	asm volatile("lwz%U1%X1 %0, %1" : "=r"
			(ret) : "m" (*addr) : "memory");
	return ret;
}
