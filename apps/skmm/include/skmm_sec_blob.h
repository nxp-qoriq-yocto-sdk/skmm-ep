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

#ifndef	__SKMM_SEC_BLOB_H__
#define	__SKMM_SEC_BLOB_H__

#include <common.h>
#include <skmm.h>
#include <skmm_memmgr.h>

struct blob_desc {
	u32 head;
	u32 key_cmd;
	dma_addr_t key_addr;
	u32 in_cmd;
	dma_addr_t in_addr;
	u32 out_cmd;
	dma_addr_t out_addr;
	u32 opt_cmd;
} __packed;

struct blob_param {
	void *hw_desc;
	dma_addr_t key_idnfr;
	u32 key_idnfr_len;
	dma_addr_t input;
	u32 input_len;
	dma_addr_t output;
	u32 output_len;
};

int encrypt_priv_key_to_blob(sec_engine_t *ccsr_sec, const char *key_file,
				char *key, int len);
int decrypt_priv_key_from_blob(sec_engine_t *ccsr_sec, int type);

#define BLOB_RSA 0
#define BLOB_DSA 1

#define RSA_KEY_FILE ".key/rsa.key"
#define DSA_KEY_FILE ".key/dsa.key"

void assign_rsa_key(void *buf);
void assign_dsa_key(void *buf);


#endif /* __SKMM_SEC_BLOB_H__ */
