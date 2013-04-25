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

#ifdef PRINT_DEBUG

#define DEBUG_DATA(p, len)	{ \
				int ii = 0; \
				for (; ii < (len); ii++) \
					print_debug("0x%x, ", ((u8 *)p)[ii]); \
				print_debug("\n"); \
				}
#else
#define DEBUG_DATA(p, len)
#endif

enum {
	priv3_p_1024,
	priv3_q_1024,
	priv3_dp_1024,
	priv3_dq_1024,
	priv3_c_1024,
	priv3_tmp1_1024,
	priv3_tmp2_1024,
};

struct key_info key_info[] = {
	{
		.name = priv3_p_1024,
		.data = (u8 *)PRI3_P_1024,
		.len = sizeof(PRI3_P_1024),
	},

	{
		.name = priv3_q_1024,
		.data = (u8 *)PRI3_Q_1024,
		.len = sizeof(PRI3_Q_1024),
	},

	{
		.name = priv3_dp_1024,
		.data = (u8 *)PRI3_DP_1024,
		.len = sizeof(PRI3_DP_1024),
	},

	{
		.name = priv3_dq_1024,
		.data = (u8 *)PRI3_DQ_1024,
		.len = sizeof(PRI3_DQ_1024),
	},

	{
		.name = priv3_c_1024,
		.data = (u8 *)PRI3_C_1024,
		.len = sizeof(PRI3_C_1024),
	},

	{
		.name = priv3_tmp1_1024,
		.len = sizeof(PRI3_P_1024),
	},

	{
		.name = priv3_tmp2_1024,
		.len = sizeof(PRI3_Q_1024),
	}
};

static int get_key(struct key_info *key)
{
	int sum_key, i;

	sum_key = sizeof(key_info) / sizeof(struct key_info);
	i = 0;

	while (i < sum_key) {
		if (key_info[i].name == key->name) {
			key->len = key_info[i].len;
			key->p_data = key_info[i].p_data;
			break;
		}
		i++;
	}

	if (i == sum_key) {
		perror("invalid name\n");
		return -EINVAL;
	}

	return 0;
}

static inline phys_addr_t get_key_addr(int name)
{
	struct key_info key;

	key.name = name;
	get_key(&key);

	return key.p_data;
}

static inline int get_key_len(int name)
{
	struct key_info key;

	key.name = name;
	get_key(&key);

	return key.len;
}

static int constr_sec_desc(struct req_info *req_info)
{
	union desc_u *sec_desc = req_info->sec_desc;
	struct abs_req_s *abs_req = req_info->abs_req;
	u32 desc_size, start_idx, p_len, q_len;

	struct rsa_pub_desc_s *rsa_pub_desc;
	struct rsa_pub *abs_rsa_pub;

	struct rsa_priv_frm3_desc_s *rsa_priv_f3;
	struct rsa_priv3 *abs_rsa_priv3;

	switch (req_info->req_type) {
	case RSA_PUB:
		rsa_pub_desc = sec_desc->rsa_pub_desc;
		abs_rsa_pub = &abs_req->req.rsa_pub;

		rsa_pub_desc->n_dma = abs_rsa_pub->n;
		DEBUG_DATA(pa_to_va(abs_rsa_pub->n), abs_rsa_pub->n_len);

		rsa_pub_desc->e_dma = abs_rsa_pub->e;
		DEBUG_DATA(pa_to_va(abs_rsa_pub->e), abs_rsa_pub->e_len);

		rsa_pub_desc->f_dma = abs_rsa_pub->f;
		DEBUG_DATA(pa_to_va(abs_rsa_pub->f), abs_rsa_pub->f_len);

		desc_size = sizeof(*rsa_pub_desc) /
				sizeof(u32);

		start_idx = desc_size - 1;
		start_idx &= HDR_START_IDX_MASK;

		init_job_desc(rsa_pub_desc,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		rsa_pub_desc->g_dma = abs_rsa_pub->g;
		rsa_pub_desc->sgf_flg =
			(abs_rsa_pub->e_len << 12) |
			abs_rsa_pub->n_len;
		rsa_pub_desc->msg_len = abs_rsa_pub->f_len;
		rsa_pub_desc->op = (CMD_OPERATION |
			OP_TYPE_UNI_PROTOCOL | OP_PCLID_RSAENC_PUBKEY);
		__sync_synchronize();

		break;
	case RSA_PRIV_FORM3:
		rsa_priv_f3 = sec_desc->rsa_priv_f3;
		abs_rsa_priv3 = &abs_req->req.rsa_priv3;

		rsa_priv_f3->p_dma = get_key_addr(priv3_p_1024);
		rsa_priv_f3->q_dma = get_key_addr(priv3_q_1024);
		rsa_priv_f3->dp_dma = get_key_addr(priv3_dp_1024);
		rsa_priv_f3->dq_dma = get_key_addr(priv3_dq_1024);
		rsa_priv_f3->c_dma = get_key_addr(priv3_c_1024);

		p_len = get_key_len(priv3_p_1024);
		q_len = get_key_len(priv3_q_1024);

		rsa_priv_f3->tmp1_dma = get_key_addr(priv3_tmp1_1024);
		rsa_priv_f3->tmp2_dma = get_key_addr(priv3_tmp2_1024);

		rsa_priv_f3->g_dma = abs_rsa_priv3->g;

		desc_size = sizeof(*rsa_priv_f3) /
				sizeof(u32);
		start_idx = desc_size - 1;
		start_idx &= HDR_START_IDX_MASK;
		init_job_desc(rsa_priv_f3,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		rsa_priv_f3->f_dma = abs_rsa_priv3->f;

		rsa_priv_f3->sgf_flg = abs_rsa_priv3->n_len;

		rsa_priv_f3->p_q_len = (q_len << 12) | p_len;
		rsa_priv_f3->op = (CMD_OPERATION |
			OP_TYPE_UNI_PROTOCOL | OP_PCLID_RSADEC_PRVKEY |
			RSA_PRIV_KEY_FRM_3);
		__sync_synchronize();

		break;
	}

	return 0;
}

phys_addr_t parse_abs_to_desc(phys_addr_t req)
{
	struct abs_req_s *abs_req = (void *)pa_to_va(req);
	union desc_u sec_desc;
	struct req_info req_info;
	va_addr_t buf;

	req_info.req_type = abs_req->abs_req_id;
	req_info.sec_desc = &sec_desc;
	req_info.abs_req = abs_req;

	switch (abs_req->abs_req_id) {
	case RSA_PUB:
		print_debug("RSA_PUB op\n");
		buf = (va_addr_t)get_buffer(sizeof(abs_req) +
				sizeof(struct rsa_pub_desc_s));
		if (!buf) {
			perror("Malloc descriptor for sec error\n");
			return 0;
		}

		/*
		 * we set abstract descrpitor address before sec descriptor
		 * so we can find the abstract descriptor by sec descriptor
		 */
		*(struct abs_req_s **)buf = abs_req;
		sec_desc.rsa_pub_desc = (void *)(buf + sizeof(abs_req));

		__sync_synchronize();

		constr_sec_desc(&req_info);
		print_debug("RSA_PUB parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.rsa_pub_desc);

	case RSA_PRIV_FORM3:
		print_debug("RSA PRIV 3 op\n");
		buf = (va_addr_t)get_buffer(sizeof(abs_req) +
				sizeof(struct rsa_priv_frm3_desc_s));
		if (!buf) {
			perror("Malloc descriptor for sec error\n");
			return 0;
		}

		/*
		 * we set abstract descrpitor address before sec descriptor
		 * so we can find the abstract descriptor by sec descriptor
		 */
		*(struct abs_req_s **)buf = abs_req;
		sec_desc.rsa_priv_f3 = (void *)(buf + sizeof(abs_req));

		__sync_synchronize();

		if (constr_sec_desc(&req_info) < 0)
			return 0;
		print_debug("RSA PRIV 3 op complete\n");
		return va_to_pa((va_addr_t)sec_desc.rsa_priv_f3);
	default:
		perror("Invalid request type: ");
		printf("0x%x\n", abs_req->abs_req_id);
		return 0;
	}
}
