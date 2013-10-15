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
#include <skmm.h>
#include <skmm_memmgr.h>
#include <abstract_req.h>
#include <skmm_sec_blob.h>

#define PRIV_KEY_LEN 32
#define SUPPORT_KEY_NUM 4

static u8 dsa_key[SUPPORT_KEY_NUM][PRIV_KEY_LEN];

enum {
	dsa_sign_s_512,
	dsa_sign_s_1k,
	dsa_sign_s_2k,
	dsa_sign_s_4k,
};

static u8 PRIV_KEY_512[] = {
	0x65, 0xe5, 0xc7, 0x38, 0x60, 0x24, 0xb5, 0x89, 0xd4, 0x9c, 0xeb, 0x4c,
	0x9c, 0x1d, 0x7a, 0x22, 0xbd, 0xd1, 0xc2, 0xd2,
};

static u8 PRIV_KEY_1024[] = {
	0x7d, 0x21, 0xda, 0xbb, 0x62, 0x15, 0x47, 0x36, 0x07, 0x67, 0x12, 0xe8,
	0x8c, 0xaa, 0x1c, 0xcd, 0x38, 0x12, 0x61, 0x18,
};

static u8 PRIV_KEY_2048[] = {
	0x32, 0x67, 0x92, 0xf6, 0xc4, 0xe2, 0xe2, 0xe8, 0xa0, 0x8b, 0x6b, 0x45,
	0x0c, 0x8a, 0x76, 0xb0, 0xee, 0xcf, 0x91, 0xa7,
};

static u8 PRIV_KEY_4096[] = {
	0x99, 0xc8, 0xf2, 0x7f, 0x21, 0xe1, 0x2e, 0x83, 0x25, 0x38, 0x9d, 0xbb,
	0xae, 0x4a,
	0x12, 0x70, 0xde, 0x38, 0x2f, 0xac, 0xd7, 0x8a, 0x97, 0xc7, 0x60, 0xdc,
	0xf3, 0x55, 0x32,
	0x02, 0x0f, 0x61
};

static struct key_info dsa_key_info[SUPPORT_KEY_NUM] = {
	{
		.len = PRIV_KEY_LEN,
		.data = PRIV_KEY_512,
	},
	{
		.data = PRIV_KEY_1024,
		.len = PRIV_KEY_LEN,
	},
	{
		.data = PRIV_KEY_2048,
		.len = PRIV_KEY_LEN,
	},
	{
		.data = PRIV_KEY_4096,
		.len = PRIV_KEY_LEN,
	}
};

phys_addr_t get_dsa_s_addr(int type)
{
	switch (type) {
	case SKMM_DSA_SIGN_512:
		return dsa_key_info[dsa_sign_s_512].p_data;
	case SKMM_DSA_SIGN_1K:
		return dsa_key_info[dsa_sign_s_1k].p_data;
	case SKMM_DSA_SIGN_2K:
		return dsa_key_info[dsa_sign_s_2k].p_data;
	case SKMM_DSA_SIGN_4K:
		return dsa_key_info[dsa_sign_s_4k].p_data;
	}

	return 0;
}

static void init_key_info(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dsa_key_info); i++)
		dsa_key_info[i].len = PRIV_KEY_LEN;
}

void dsa_blob(void)
{
	u8 *src, *dst = (u8 *)dsa_key;
	int i, len = 0;

	init_key_info();

	for (i = 0; i < ARRAY_SIZE(dsa_key_info); i++)
		len += dsa_key_info[i].len;

	src = get_buffer(len);

	for (i = 0; i < ARRAY_SIZE(dsa_key_info); i++) {
		memcpy(src, dst, dsa_key_info[i].len);
		dsa_key_info[i].p_data = va_to_pa((va_addr_t)src);
		src += dsa_key_info[i].len;
		dst += dsa_key_info[i].len;
	}
}

int get_dsa_keys_size(void)
{
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(dsa_key_info); i++)
		len += dsa_key_info[i].len;

	return len;
}

extern sec_engine_t *blob_sec;
static void *key_buf_start;

static void keygen_callback(void)
{
	int ret;
#if 1
	ret = encrypt_priv_key_to_blob(blob_sec, DSA_KEY_FILE, key_buf_start,
				get_dsa_keys_size());
	print_debug("%s ret :%d\n", __func__, ret);
#endif
}

int constr_keygen_desc(struct req_info *req_info)
{
	struct abs_req_s *abs_req = req_info->abs_req;
	struct keygen *abs_keygen;
	struct keygen_desc_s *dsa_keygen = req_info->sec_desc->keygen;
	struct keygen_alloc_block *dsa_keygen_pdata = req_info->private_data;
	u32 q_len, r_len;
	int type;

	u32 desc_size, start_idx;

	abs_keygen = &abs_req->req.keygen;
	dsa_keygen->q_dma = abs_keygen->q;
	dsa_keygen->r_dma = abs_keygen->r;
	dsa_keygen->g_dma = abs_keygen->g;
	dsa_keygen->w_dma = abs_keygen->w;
	q_len = abs_keygen->q_len;
	r_len = abs_keygen->r_len;

	switch (q_len) {
	case 64:
		type = SKMM_DSA_SIGN_512;
		break;
	case 128:
		type = SKMM_DSA_SIGN_1K;
		break;
	case 256:
		type = SKMM_DSA_SIGN_2K;
		break;
	case 512:
		type = SKMM_DSA_SIGN_4K;
		break;
	default:
		type = SKMM_DSA_SIGN_1K;
	}

	dsa_keygen->s_dma = get_dsa_s_addr(type);

	desc_size = sizeof(*dsa_keygen) / sizeof(u32);
	start_idx = desc_size - 1;

	start_idx &= HDR_START_IDX_MASK;
	init_job_desc(&dsa_keygen->desc_hdr,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) |
			HDR_ONE);

	dsa_keygen->sgf_ln = q_len << 7 | r_len;
	dsa_keygen->op = CMD_OPERATION | OP_TYPE_UNI_PROTOCOL |
			OP_PCLID_PUBLICKEYPAIR;

	dsa_keygen_pdata->callback = keygen_callback;
	__sync_synchronize();

	return 1;
}

int constr_dsa_sec_desc(struct req_info *req_info)
{
	union desc_u *sec_desc = req_info->sec_desc;
	struct abs_req_s *abs_req = req_info->abs_req;
	u32 desc_size, start_idx;

	struct dsa_verify_desc_s *dsa_verify;
	struct dsa_sign_desc_s *dsa_sign;

	struct dsa_verify *abs_dsa_verify;
	struct dsa_sign *abs_dsa_sign;
	struct dsa_verify_alloc_block *verify_pdata;
	struct dsa_sign_alloc_block *sign_pdata;

	int type = req_info->req_type;
	int q_len, r_len;

	switch (type) {
	case SKMM_DSA_SIGN_512:
	case SKMM_DSA_SIGN_1K:
	case SKMM_DSA_SIGN_2K:
	case SKMM_DSA_SIGN_4K:
		print_debug("type: %d\n", type);

		dsa_sign = sec_desc->dsa_sign;
		abs_dsa_sign = &abs_req->req.dsa_sign;
		sign_pdata = req_info->private_data;

		q_len = abs_dsa_sign->q_len;
		r_len = abs_dsa_sign->r_len;

		sign_pdata->tmp = get_buffer(2 * r_len);
		if (sign_pdata->tmp == NULL)
			return 0;

		dsa_sign->f_dma = abs_dsa_sign->m;

		dsa_sign->q_dma = abs_dsa_sign->q;
		dsa_sign->r_dma = abs_dsa_sign->r;
		dsa_sign->g_dma = abs_dsa_sign->g;
		dsa_sign->s_dma = get_dsa_s_addr(type);
		desc_size = sizeof(*(sec_desc->dsa_sign)) /
				sizeof(u32);

		dsa_sign->c_dma = abs_dsa_sign->c;
		dsa_sign->d_dma = abs_dsa_sign->d;

		start_idx = desc_size - 1;
		start_idx &= HDR_START_IDX_MASK;

		init_job_desc(sec_desc->dsa_sign,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		sec_desc->dsa_sign->sgf_ln =
				(q_len << DSA_L_SHIFT) | r_len;

		dsa_sign->op[0] = (CMD_OPERATION |
				OP_TYPE_UNI_PROTOCOL |
				OP_PCLID_DSASIGN);

		__sync_synchronize();
		break;

	case SKMM_DSA_VERIFY:
		print_debug("type: %d\n", type);
		dsa_verify = sec_desc->dsa_verify;
		abs_dsa_verify = &abs_req->req.dsa_verify;
		verify_pdata = req_info->private_data;

		dsa_verify->q_dma = abs_dsa_verify->q;
		dsa_verify->r_dma = abs_dsa_verify->r;
		dsa_verify->g_dma = abs_dsa_verify->g;
		dsa_verify->w_dma = abs_dsa_verify->pub_key;
		dsa_verify->f_dma = abs_dsa_verify->m;
		dsa_verify->c_dma = abs_dsa_verify->c;
		dsa_verify->d_dma = abs_dsa_verify->d;

		verify_pdata->tmp = get_buffer(abs_dsa_verify->q_len);
		if (verify_pdata->tmp == NULL)
			return 0;
		dsa_verify->tmp_dma = va_to_pa((va_addr_t)verify_pdata->tmp);

		desc_size = sizeof(*(sec_desc->dsa_verify)) /
				sizeof(u32);

		start_idx = desc_size - 1;
		start_idx &= HDR_START_IDX_MASK;

		init_job_desc(sec_desc->dsa_verify,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		sec_desc->dsa_verify->sgf_ln =
				(abs_dsa_verify->q_len << DSA_L_SHIFT) |
				abs_dsa_verify->r_len;
		sec_desc->dsa_verify->op = CMD_OPERATION |
					OP_TYPE_UNI_PROTOCOL |
					OP_PCLID_DSAVERIFY;

		__sync_synchronize();
		break;
	}

	return 1;
}

void assign_dsa_key(void *buf)
{
	phys_addr_t tmp;
	int i;
	static int first = 0;

	if (buf == NULL) {
		buf = get_buffer(get_dsa_keys_size());
		if (buf == NULL) {
			printf("error: no Memory\n");
			exit(0);
		}
		first = 1;
	}

	key_buf_start = buf;
	tmp  = va_to_pa((va_addr_t)buf);

	for (i = 0; i < ARRAY_SIZE(dsa_key_info); i++) {
		dsa_key_info[i].p_data = tmp;
		if (first) {
			print_debug("set default key\n");
			memcpy((u8 *)pa_to_va(tmp), dsa_key_info[i].data,
					dsa_key_info[i].len);
		}
		tmp += dsa_key_info[i].len;
	}
}
