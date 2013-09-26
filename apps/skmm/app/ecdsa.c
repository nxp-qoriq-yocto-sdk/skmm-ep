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

enum {
	dsa_sign_q,
	dsa_sign_r,
	dsa_sign_g,
	dsa_sign_s,
};

static u8 Q[] = {
	0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF,
	0XFF, 0XFF, 0XFF, 0XFE,
	0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF
};


static u8 R[] = {
	0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF,
	0X99, 0XDE, 0XF8, 0X36,
	0X14, 0X6B, 0XC9, 0XB1, 0XB4, 0XD2, 0X28, 0X31
};

static u8 G[] = {
	0X18, 0X8D, 0XA8, 0X0E, 0XB0, 0X30, 0X90, 0XF6, 0X7C, 0XBF, 0X20, 0XEB,
	0X43, 0XA1, 0X88, 0X00,
	0XF4, 0XFF, 0X0A, 0XFD, 0X82, 0XFF, 0X10, 0X12, 0X07, 0X19, 0X2B, 0X95,
	0XFF, 0XC8, 0XDA, 0X78,
	0X63, 0X10, 0X11, 0XED, 0X6B, 0X24, 0XCD, 0XD5, 0X73, 0XF9, 0X77, 0XA1,
	0X1E, 0X79, 0X48, 0X11
};

static u8 PRIV_KEY[] = {
	0X13, 0XBD, 0XA6, 0XFE, 0X20, 0XE2, 0X8F, 0X2C, 0X7F, 0X17, 0X7D, 0X27,
	0XBC, 0X1D, 0XDF, 0X69,
	0X73, 0X3C, 0XD3, 0XFC, 0X51, 0X70, 0X4F, 0X34
};

static struct key_info ecdsa_key_info[] = {
	{
		.name = dsa_sign_q,
		.data = (u8 *)Q,
		.len = sizeof(Q),
	},
	{
		.name = dsa_sign_r,
		.data = (u8 *)R,
		.len = sizeof(R),
	},
	{
		.name = dsa_sign_g,
		.data = (u8 *)G,
		.len = sizeof(G),
	},
	{
		.name = dsa_sign_s,
		.data = (u8 *)PRIV_KEY,
		.len = sizeof(PRIV_KEY),
	},
};

static inline phys_addr_t get_dsa_q_addr(int type)
{
	return ecdsa_key_info[dsa_sign_q].p_data;
}

static inline phys_addr_t get_dsa_r_addr(int type)
{
	return ecdsa_key_info[dsa_sign_r].p_data;
}

static inline phys_addr_t get_dsa_g_addr(int type)
{
	return ecdsa_key_info[dsa_sign_g].p_data;
}

static inline phys_addr_t get_dsa_s_addr(int type)
{
	return ecdsa_key_info[dsa_sign_s].p_data;
}

static inline int get_dsa_q_len(int type)
{
	return ecdsa_key_info[dsa_sign_q].len;
}

static inline int get_dsa_r_len(int type)
{
	return ecdsa_key_info[dsa_sign_r].len;
}

void ecdsa_blob(void)
{
	u8 *buf;
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(ecdsa_key_info); i++)
		len += ecdsa_key_info[i].len;

	buf = get_buffer(len);

	for (i = 0; i < ARRAY_SIZE(ecdsa_key_info); i++) {
		memcpy(buf, ecdsa_key_info[i].data, ecdsa_key_info[i].len);
		ecdsa_key_info[i].p_data = va_to_pa((va_addr_t)buf);
		buf += ecdsa_key_info[i].len;
	}
}


static inline void *get_dsa_tmp_addr(int len)
{
	return get_buffer(len * 2);
}

int constr_ecdsa_sec_desc(struct req_info *req_info)
{
	union desc_u *sec_desc = req_info->sec_desc;
	struct abs_req_s *abs_req = req_info->abs_req;
	u32 desc_size, start_idx;

	struct ecdsa_verify_desc_s *ecdsa_verify;
	struct dsa_verify *abs_dsa_verify;
	struct ecdsa_verify_alloc_block *verify_pdata;

	struct ecdsa_sign_desc_s *ecdsa_sign;
	struct dsa_sign *abs_dsa_sign;
	struct ecdsa_sign_alloc_block *sign_pdata;

	int type = req_info->req_type;
	u32 q_len, r_len;

	switch (type) {
	case SKMM_ECDSA_SIGN:
		print_debug("type: %d\n", type);
		ecdsa_sign = sec_desc->ecdsa_sign;
		abs_dsa_sign = &abs_req->req.dsa_sign;
		sign_pdata = req_info->private_data;
		q_len = get_dsa_q_len(type);
		r_len = get_dsa_r_len(type);
		sign_pdata->tmp = get_dsa_tmp_addr(2 * r_len);
		if (sign_pdata->tmp == NULL)
			return 0;

		ecdsa_sign->f_dma = abs_dsa_sign->m;

		ecdsa_sign->q_dma = get_dsa_q_addr(type);
		ecdsa_sign->r_dma = get_dsa_r_addr(type);
		ecdsa_sign->g_dma = get_dsa_g_addr(type);
		ecdsa_sign->s_dma = get_dsa_s_addr(type);

		desc_size = sizeof(*(sec_desc->ecdsa_sign)) /
				sizeof(u32);
#if 0
		start_idx = desc_size - 12;
		start_idx &= HDR_START_IDX_MASK;

		init_job_desc(sec_desc->dsa_sign,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		sec_desc->dsa_sign->sgf_ln =
				(q_len << DSA_L_SHIFT) | r_len;

		dsa_sign->op[0] = (CMD_OPERATION |
				OP_TYPE_UNI_PROTOCOL |
				OP_PCLID_DSASIGN);
		dsa_sign->op[1] = (CMD_MOVE | MOVE_SRC_INFIFO |
				MOVE_DEST_OUTFIFO |
				(2 * r_len));
		dsa_sign->op[2] = (CMD_JUMP | JUMP_COND_NOP | 1);
		dsa_sign->op[3] = (CMD_FIFO_LOAD |
				FIFOLD_CLASS_CLASS1 |
				FIFOLD_TYPE_PK_TYPEMASK |
				(2 * r_len));
		dsa_sign->op[4] = dsa_sign->c_dma;
		dsa_sign->op[6] = (CMD_FIFO_STORE |
				FIFOST_CONT_MASK |
				FIFOST_TYPE_MESSAGE_DATA |
				r_len);
		dsa_sign->op[7] =  abs_dsa_sign->c;
		dsa_sign->op[9] = (CMD_FIFO_STORE |
				FIFOST_TYPE_MESSAGE_DATA |
				r_len);
		dsa_sign->op[10] = abs_dsa_sign->d;
#else
		ecdsa_sign->c_dma = abs_dsa_sign->c;
		ecdsa_sign->d_dma = abs_dsa_sign->d;
		ecdsa_sign->ab_dma = abs_dsa_sign->ab;

		start_idx = desc_size - 1;
		start_idx &= HDR_START_IDX_MASK;

		init_job_desc(sec_desc->ecdsa_sign,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		sec_desc->ecdsa_sign->sgf_ln =
				(q_len << DSA_L_SHIFT) | r_len;

		ecdsa_sign->op[0] = (CMD_OPERATION |
				OP_TYPE_UNI_PROTOCOL |
				OP_PCLID_DSASIGN |
				OP_PCL_PKPROT_ECC);
#endif

		__sync_synchronize();
		break;
	case SKMM_ECDSA_VERIFY:
		print_debug("type: %d\n", type);
		ecdsa_verify = sec_desc->ecdsa_verify;
		abs_dsa_verify = &abs_req->req.dsa_verify;
		verify_pdata = req_info->private_data;

		ecdsa_verify->q_dma = abs_dsa_verify->q;
		ecdsa_verify->r_dma = abs_dsa_verify->r;
		ecdsa_verify->g_dma = abs_dsa_verify->g;
		ecdsa_verify->w_dma = abs_dsa_verify->pub_key;
		ecdsa_verify->f_dma = abs_dsa_verify->m;
		ecdsa_verify->c_dma = abs_dsa_verify->c;
		ecdsa_verify->d_dma = abs_dsa_verify->d;
		ecdsa_verify->ab_dma = abs_dsa_verify->ab;
		verify_pdata->tmp = get_dsa_tmp_addr(abs_dsa_verify->q_len);
		if (verify_pdata->tmp == NULL)
			return 0;
		ecdsa_verify->tmp_dma = va_to_pa((va_addr_t)verify_pdata->tmp);

		desc_size = sizeof(*(sec_desc->ecdsa_verify)) /
				sizeof(u32);

		start_idx = desc_size - 1;
		start_idx &= HDR_START_IDX_MASK;

		init_job_desc(sec_desc->ecdsa_verify,
			(start_idx << HDR_START_IDX_SHIFT) |
			(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		sec_desc->ecdsa_verify->sgf_ln =
				(abs_dsa_verify->q_len << DSA_L_SHIFT) |
				abs_dsa_verify->r_len;
		sec_desc->ecdsa_verify->op = CMD_OPERATION |
					OP_TYPE_UNI_PROTOCOL |
					OP_PCLID_DSAVERIFY |
					OP_PCL_PKPROT_ECC;

		__sync_synchronize();
		break;
	}

	return 1;
}
