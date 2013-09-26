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

static u8 Q[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF
};

static u8 S2[] = {
	0x4d, 0x47, 0x6e, 0x7d, 0xea, 0x49, 0x6e, 0x3c, 0x9a, 0x2e, 0x04, 0xec,
	0xc2, 0x92, 0x6d,
	0x61, 0xdc, 0x12, 0x1b, 0xe3
};

enum {
	ecdh_q,
	ecdh_s,
};

static struct key_info ecdh_key_info[] = {
	{
		.data = Q,
		.len = sizeof(Q),
	},
	{
		.data = S2,
		.len = sizeof(S2),
	},
};

static phys_addr_t get_q_phy_addr(int type)
{
	switch (type) {
	case SKMM_ECDH:
		return ecdh_key_info[ecdh_q].p_data;
	}

	return 0;
}

static phys_addr_t get_s_phy_addr(int type)
{
	switch (type) {
	case SKMM_ECDH:
		return ecdh_key_info[ecdh_s].p_data;
	}

	return 0;
}

static int get_q_len(int type)
{
	switch (type) {
	case SKMM_ECDH:
		return ecdh_key_info[ecdh_q].len;
	}

	return 0;
}

static int get_s_len(int type)
{
	switch (type) {
	case SKMM_ECDH:
		return ecdh_key_info[ecdh_s].len;
	}

	return 0;
}

void ecdh_blob(void)
{
	u8 *buf;
	int i;

	for (i = 0; i < ARRAY_SIZE(ecdh_key_info); i++) {
		buf = get_buffer(ecdh_key_info[i].len);
		print_debug("%d: %d\n", i, ecdh_key_info[i].len);
		memcpy(buf, ecdh_key_info[i].data, ecdh_key_info[i].len);
		ecdh_key_info[i].p_data =
			va_to_pa((va_addr_t)buf);
	}
}

int constr_ecdh_sec_desc(struct req_info *req_info)
{
	union desc_u *sec_desc = req_info->sec_desc;
	struct abs_req_s *abs_req = req_info->abs_req;
	u32 desc_size, start_idx;

	struct dh_key *abs_dh_key;
	struct ecdh_key_desc_s *ecdh_key;

	int type = req_info->req_type;
	int q_len, s_len;

	switch (type) {
	case SKMM_ECDH:
		ecdh_key = sec_desc->ecdh_key;
		abs_dh_key = &abs_req->req.dh_key;

		ecdh_key->w_dma = abs_dh_key->w;
		ecdh_key->z_dma = abs_dh_key->z;
		ecdh_key->ab_dma = abs_dh_key->ab;
		ecdh_key->q_dma = get_q_phy_addr(type);
		ecdh_key->s_dma = get_s_phy_addr(type);

		q_len = get_q_len(type);
		s_len = get_s_len(type);

		desc_size = sizeof(struct ecdh_key_desc_s) / sizeof(u32);
		start_idx = desc_size - 1;

		start_idx &= HDR_START_IDX_MASK;
		init_job_desc(ecdh_key, (start_idx << HDR_START_IDX_SHIFT) |
				(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		ecdh_key->sgf_ln = (q_len << 7) | s_len;
		ecdh_key->op = (CMD_OPERATION | OP_TYPE_UNI_PROTOCOL |
				OP_PCLID_DH | OP_PCL_PKPROT_ECC);

		__sync_synchronize();

	}

	return 1;
}
