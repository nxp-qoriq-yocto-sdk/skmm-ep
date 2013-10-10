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

#ifndef __SKMM_MEMMGR_H__
#define __SKMM_MEMMGR_H__

#include <common.h>

#if (__WORDSIZE == 64)
/*
 * Making sure that metadata is min 32 bytes
 * so that actual buf mem will be aligned to 32 bytes for better performance.
 */
struct buffer_header {
	struct buffer_header *prev_link;
	struct buffer_header *next_link;

	u32 len;
	u8 in_use;
	u8 flag;
	u8 pad[2];

	unsigned long priv;
} __packed;
#endif

#if (__WORDSIZE == 32)
struct buffer_header {
	struct buffer_header *prev_link;
	struct buffer_header *next_link;

	u32 len;
	u8 in_use;
	u8 flag;
	u8 pad1[2];

	unsigned long priv;
	u32 pad[3];
} __packed;
#endif

typedef struct buffer_header bh;

typedef struct buffer_pool {
	u32 tot_free_mem;
	bh *free_list;

	void *buff;
	u32 len;
	pthread_spinlock_t mem_lock;
} bp;

void *reg_mem_pool(void *buf, u32 len);
void *get_buffer(u32 size);
void put_buffer(void *buffer);
void reset_pool(void);

struct key_info {
	int name;
	u32 len;
	u8 *data;
	phys_addr_t p_data;
};

void reset_pool(void);
void *reg_mem_pool(void *buf, u32 len);
void *get_buffer(u32 len);
void put_buffer(void *buffer);
void store_priv_data(void *id, void *buffer, unsigned long priv);
unsigned long get_priv_data(void *id, void *buffer);

#endif /* __SKMM_MEMMGR_H__ */
