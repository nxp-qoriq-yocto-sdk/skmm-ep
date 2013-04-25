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
#ifndef	__COMMON_H__
#define	__COMMON_H__

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <limits.h>
#include <error.h>
#include <sys/mman.h>
#include <fcntl.h>

#ifdef PRINT_DEBUG
#define _DEBUG  1
#else
#define _DEBUG  0
#endif

#define debug_cond(cond, fmt, args...)	\
	do {				\
		if (cond)		\
		printf(fmt, ##args);	\
	} while (0)

#define print_debug(fmt, args...)	\
	debug_cond(_DEBUG, fmt, ##args)

#ifdef PRINT_ERROR
#define print_error	printf
#else
#define print_error
#endif

typedef unsigned char		u8;
typedef unsigned short		u16;
typedef unsigned int		u32;
typedef unsigned long long	u64;

typedef char			i8;
typedef short			i16;
typedef int			i32;
typedef unsigned long		i64;

#define __packed	__attribute__((__packed__))

#define DEV_VIRT_ADDR_32BIT
#ifdef DEV_VIRT_ADDR_32BIT
typedef u32	va_addr_t;

#else
typedef u64	va_addr_t;

#endif

#ifdef	DEV_PHYS_ADDR_32BIT
typedef	u32	dma_addr_t;
typedef u32	phys_addr_t;
#else
typedef u64     dma_addr_t;
typedef u64	phys_addr_t;
#endif

static inline u32 __read_reg(u32 *p)
{
	u32 ret;

	ret = *(volatile u32 *)(p);
	__sync_synchronize();
	return ret;
}

static inline void __write_reg(u32 *p, u32 v)
{
	*(volatile u32 *)(p) = v;
	__sync_synchronize();
}

#define read_reg	__read_reg
#define write_reg	__write_reg

inline u64 getticks(void);
unsigned long get_tbclk(void);
unsigned long usec2ticks(unsigned long usec);
phys_addr_t get_ccsr_phys_addr(void);

#endif /* __COMMON_H__ */
