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

static u8 S2[] = {
	0x4e, 0x51, 0x30, 0x16, 0xcd, 0xd7, 0xf5, 0x1f, 0xa9, 0xfd, 0x85, 0x95,
	    0xcb, 0xf6, 0x4b,
	0x0e, 0x9b, 0xc3, 0x90, 0xaf, 0x1e
};

static u8 Q[] = {
	0x8b, 0x1f, 0x58, 0xa3, 0x3e, 0xc7, 0x0b, 0x86, 0xff, 0xb7, 0x15, 0xa9,
	    0x33, 0x7c,
	0x39, 0x75, 0xc1, 0x40, 0xc0, 0x42, 0x86, 0x3f, 0x61, 0xbe, 0x10, 0xa9,
	    0x7a, 0x7e, 0xb6,
	0x98, 0xbd, 0x0b, 0x87, 0x9f, 0xb9, 0x23, 0x18, 0x7d, 0xf5, 0xb0, 0xb6,
	    0x94, 0x4a, 0x43,
	0x21, 0xba, 0x4e, 0xa1, 0xb2, 0x7b, 0x83, 0xee, 0x2d, 0xb4, 0x6b, 0xb8,
	    0x4d, 0xee, 0xd1,
	0x55, 0x84, 0xbb, 0xcf, 0x46, 0x24, 0x5a, 0x59, 0x9f, 0x94, 0xc0, 0x87,
	    0xa6, 0x15, 0xad,
	0xb4, 0x9e, 0xb6, 0x97, 0x1d, 0x2c, 0x0e, 0xe6, 0xbb, 0x58, 0x8f, 0x2d,
	    0xeb, 0x1a, 0xe4,
	0x22, 0x24, 0x2c, 0x99, 0x6e, 0x06, 0x44, 0xc9, 0x34, 0x4d, 0x58, 0x63,
	    0x38, 0xbc, 0xfe,
	0x2c, 0x05, 0xaa, 0x0a, 0xf8, 0xd3, 0x4c, 0x27, 0xc1, 0xfd, 0x0b, 0x04,
	    0x03, 0x14, 0x61,
	0x94, 0x8e, 0x91, 0x81, 0xd9, 0xa8, 0xff, 0x6a, 0x9b
};

static u8 S2_2048[] = {
	0x41, 0x92, 0xca, 0x32, 0x13, 0x55, 0x45, 0x17, 0x43, 0xa0, 0x24, 0xa5,
	    0x40, 0xac, 0x98,
	0x20, 0xd1, 0xdd, 0xe8, 0x56, 0x5a, 0x36, 0xf4, 0xcb, 0x36, 0x74, 0x4d,
	    0x8b, 0xbf, 0xec,
	0x69, 0xf5, 0xe3, 0x74, 0xa0, 0x28, 0xf8, 0xbf, 0xf8, 0x0f, 0x55
};

static u8 Q_2048[] = {
	0xbf, 0x9a, 0xc0, 0x94, 0x97, 0xc0, 0x9e, 0x41, 0xbf, 0x40, 0xe4, 0x90,
	    0x0f, 0xfc,
	0xdf, 0x4a, 0x00, 0xb1, 0xfa, 0x71, 0xab, 0x2c, 0x4a, 0x1f, 0x6c, 0x39,
	    0xfa, 0xed, 0x60,
	0xab, 0x30, 0x67, 0x34, 0x84, 0x3b, 0xc3, 0xcc, 0x41, 0xf7, 0x41, 0x16,
	    0x68, 0x69, 0x19,
	0x44, 0xf7, 0x9b, 0xf7, 0xf7, 0xfb, 0xef, 0x3f, 0x1a, 0x62, 0x2e, 0xb6,
	    0x3a, 0x1a, 0xd3,
	0x5a, 0x39, 0xa6, 0x05, 0x24, 0x30, 0x2e, 0xe5, 0xab, 0x58, 0xb8, 0x90,
	    0x0f, 0x0a, 0x6e,
	0xcc, 0xd8, 0xda, 0xc9, 0x80, 0x4f, 0xbe, 0xa9, 0xf6, 0x39, 0x5c, 0x6c,
	    0x10, 0xdf, 0xa8,
	0xfa, 0xc8, 0xf4, 0xc3, 0xfa, 0xdd, 0x5b, 0xb8, 0x4b, 0xb1, 0x01, 0xaf,
	    0x80, 0x4c, 0x17,
	0xbf, 0xf5, 0xf3, 0x30, 0x6e, 0x60, 0xca, 0x70, 0x2b, 0x92, 0xf9, 0x17,
	    0xdc, 0xbd, 0x46,
	0x0f, 0x0f, 0x60, 0xe8, 0x84, 0xca, 0xf5, 0xab, 0x34, 0x25, 0x89, 0xe0,
	    0xc1, 0x36, 0xdb,
	0x27, 0x07, 0x8c, 0xf4, 0xa2, 0x3a, 0xd4, 0x2d, 0x20, 0x68, 0x3f, 0xa8,
	    0xb1, 0x52, 0x62,
	0xb3, 0x24, 0x68, 0x2a, 0xce, 0x40, 0xf7, 0xd2, 0x46, 0x95, 0xd6, 0x99,
	    0x12, 0xf4, 0x85,
	0x6a, 0xd1, 0x65, 0x28, 0x53, 0xaa, 0xa4, 0xb6, 0x27, 0xe7, 0xea, 0x3f,
	    0x3b, 0xda, 0x25,
	0x0d, 0x6b, 0x0a, 0x5a, 0x57, 0x58, 0xda, 0xc4, 0x4a, 0x65, 0xbc, 0xf6,
	    0x40, 0xd2, 0x4c,
	0x2c, 0x16, 0x84, 0xa8, 0x73, 0x13, 0x5d, 0xba, 0x3d, 0xa9, 0xa5, 0x64,
	    0x2c, 0x15, 0xd9,
	0xbb, 0x6c, 0x8b, 0xac, 0xcb, 0x86, 0x8f, 0xdc, 0xb2, 0x38, 0xad, 0xc1,
	    0x26, 0x96, 0x1e,
	0x57, 0xb3, 0x2c, 0xb5, 0x87, 0x76, 0x1b, 0x0f, 0xd5, 0x7a, 0x38, 0xf7,
	    0xd0, 0x7b, 0x93,
	0x4a, 0x96, 0xa4, 0x4e, 0x10, 0x73, 0xc7, 0xb2, 0xa0, 0x86, 0x34, 0x3b,
	    0xc8, 0x13, 0x05,
	0x65, 0xbb
};

static u8 S2_4096[] = {
	0x46, 0x27, 0xe2, 0x07, 0x9d, 0x62, 0xd4, 0x53, 0x75, 0x72, 0xe8, 0xb3,
	    0x35, 0x21, 0x53,
	0x60, 0x05, 0xd1, 0x25, 0x46, 0x10, 0x8e, 0xab, 0x39, 0x44, 0xab, 0x95,
	    0xad, 0x1e, 0x94,
	0xb2, 0x7a, 0xe0, 0x4b, 0x9d, 0xe7, 0xcd, 0x9e, 0x17, 0x14, 0x96, 0xa1,
	    0x42, 0x37, 0x9c,
	0x73, 0xac, 0x66, 0x48, 0x42, 0x0b, 0xc5, 0x2b, 0xee, 0x3d, 0xc7, 0x3d,
	    0xf5, 0x16, 0x43,
	0xc1, 0xa1, 0xda, 0x04, 0xed, 0xa1, 0xc1, 0x58, 0x3c, 0xa8, 0xba, 0xff,
	    0xc9, 0x9c, 0xfa,
	0x18, 0xf1, 0x02, 0x33, 0x10, 0x5d, 0x24, 0x8b, 0x11
};

static u8 Q_4096[] = {
	0xad, 0x06, 0x59, 0xd4, 0xa8, 0x86, 0x4e, 0xc4, 0x45, 0x2f, 0x98, 0xc8,
	    0x22, 0x90,
	0x8c, 0x99, 0xd9, 0xe8, 0x5c, 0x45, 0xe6, 0x34, 0x6d, 0x3f, 0x3d, 0x85,
	    0xab, 0xcd, 0x50,
	0xd7, 0xb8, 0xec, 0x8c, 0x3f, 0x22, 0x57, 0x6c, 0xc2, 0xf8, 0xa4, 0x4e,
	    0x97, 0x11, 0x9a,
	0x76, 0xa3, 0x8a, 0x23, 0x9e, 0x0b, 0x1f, 0x28, 0x02, 0x22, 0xe2, 0xc3,
	    0x91, 0x38, 0xb5,
	0x97, 0xdc, 0xdf, 0xd4, 0x3a, 0xcb, 0x1a, 0x33, 0x87, 0x29, 0x58, 0x96,
	    0x6d, 0x67, 0x37,
	0xa9, 0x20, 0x60, 0x6e, 0xcc, 0xed, 0xd7, 0xaf, 0x5d, 0x5d, 0xe8, 0xfa,
	    0xeb, 0x2c, 0x6d,
	0x81, 0xf8, 0x27, 0x6c, 0x75, 0x74, 0xe1, 0x29, 0x6a, 0x2a, 0xb0, 0xd0,
	    0xe0, 0x1e, 0x3f,
	0xbf, 0x80, 0x60, 0x90, 0x88, 0xd8, 0x0b, 0xd4, 0xd5, 0xae, 0xf7, 0x22,
	    0x9e, 0x06, 0xef,
	0xfa, 0xc6, 0xf4, 0xcc, 0xec, 0x74, 0xb2, 0xc3, 0xb6, 0xcd, 0x3e, 0xc6,
	    0xa0, 0x62, 0x4c,
	0x72, 0xb2, 0x1d, 0x82, 0xd8, 0xd5, 0x99, 0xb8, 0x46, 0x3e, 0x0a, 0xa7,
	    0x3c, 0xa2, 0x4a,
	0x19, 0x98, 0x1c, 0xf6, 0x5e, 0xbf, 0xc7, 0x73, 0x3d, 0xf8, 0x66, 0x65,
	    0x15, 0xf3, 0x5b,
	0xd1, 0x83, 0xea, 0x0d, 0x62, 0xe5, 0x05, 0xd6, 0x78, 0xf0, 0x6a, 0x3b,
	    0xc4, 0x7c, 0x70,
	0xfb, 0xd9, 0x9a, 0x9a, 0x62, 0xe6, 0xa8, 0x4f, 0x76, 0x27, 0x0f, 0xc3,
	    0x9a, 0x5b, 0x8d,
	0xd4, 0x8c, 0xe0, 0x8e, 0x5a, 0xd8, 0xe4, 0xe7, 0x6a, 0xcd, 0xf3, 0x48,
	    0xdc, 0x93, 0x48,
	0x25, 0x09, 0xed, 0xd5, 0x6c, 0xe5, 0x14, 0x54, 0x88, 0x5d, 0x98, 0xe1,
	    0x0d, 0x6a, 0x23,
	0x0c, 0x05, 0x32, 0x02, 0x24, 0xe5, 0x87, 0x1b, 0x50, 0x53, 0xff, 0x4c,
	    0x7d, 0x11, 0xfa,
	0xe0, 0xd5, 0x3d, 0xe6, 0x89, 0x5b, 0xa0, 0x5b, 0x94, 0x39, 0x6a, 0x20,
	    0x61, 0xc7, 0x4e,
	0xd9, 0xf2, 0x16, 0xf2, 0x11, 0x18, 0x0a, 0xe8, 0xeb, 0xe8, 0x55, 0xde,
	    0x58, 0x97, 0x41,
	0xb9, 0x5b, 0x2a, 0x5f, 0xd9, 0x51, 0xb7, 0x83, 0x85, 0xfc, 0x86, 0x97,
	    0x58, 0x5c, 0x0a,
	0xe9, 0x3d, 0x1a, 0x12, 0xd7, 0xb5, 0xe0, 0x8f, 0x93, 0xbb, 0xd9, 0xb9,
	    0xc2, 0xb1, 0x25,
	0x0d, 0xc1, 0x61, 0x56, 0xc7, 0x96, 0x27, 0x88, 0xb0, 0x7c, 0x2f, 0x1a,
	    0xfa, 0xc5, 0xef,
	0x94, 0x66, 0x6d, 0x0c, 0xf2, 0x6b, 0x86, 0xc2, 0xaa, 0x4f, 0x6a, 0xfb,
	    0x6c, 0x55, 0x0e,
	0x1e, 0x56, 0x46, 0xdb, 0x2a, 0x74, 0xab, 0x03, 0x89, 0x7b, 0xe5, 0xab,
	    0x0f, 0x32, 0x54,
	0x73, 0xe3, 0x3f, 0xe3, 0x48, 0x2d, 0xfa, 0x0c, 0xb6, 0xa6, 0x0d, 0x3c,
	    0xcd, 0x2d, 0x4c,
	0xa6, 0x87, 0xd7, 0x37, 0x64, 0x0c, 0x2b, 0xdc, 0x8a, 0x49, 0x09, 0x37,
	    0x2f, 0x9e, 0xa6,
	0xde, 0xe9, 0x37, 0xe5, 0xf9, 0xd5, 0x6f, 0x6f, 0x49, 0x25, 0xbc, 0xc1,
	    0x8f, 0xe8, 0x45,
	0x86, 0x76, 0x1a, 0x7f, 0x21, 0xed, 0xfd, 0x50, 0xb2, 0x07, 0x2c, 0x2c,
	    0xb7, 0xbe, 0xc3,
	0x66, 0x74, 0x30, 0xb0, 0x02, 0x8e, 0xe9, 0x9b, 0x74, 0x1b, 0x8e, 0xa7,
	    0x05, 0xe6, 0x4c,
	0x0b, 0x57, 0xa2, 0x33, 0xe1, 0x64, 0x9e, 0x4b, 0x18, 0xf8, 0x4f, 0x41,
	    0x60, 0x69, 0x39,
	0xda, 0xd5, 0x8d, 0xf1, 0xd2, 0xbb, 0x3d, 0x30, 0x87, 0xee, 0xba, 0x1f,
	    0xb2, 0x37, 0x9a,
	0x20, 0x07, 0xa9, 0xfd, 0xb7, 0x4b, 0xeb, 0xd0, 0xe9, 0x7c, 0xc8, 0x1d,
	    0xff, 0xf7, 0x95,
	0x57, 0xf8, 0x60, 0x98, 0x4a, 0xd5, 0xf6, 0x8c, 0x99, 0x7d, 0xae, 0xab,
	    0xea, 0x0f, 0x07,
	0x74, 0x30, 0x68, 0x8b, 0x57, 0xe6, 0xed, 0xd5, 0x92, 0x94, 0x82, 0x9e,
	    0x45, 0xa4, 0xec,
	0xaa, 0xb2, 0x20, 0xa5, 0xca, 0xc4, 0x4e, 0x50, 0x07, 0x56, 0x7d, 0x2e,
	    0xe9, 0xc6, 0xe5,
	0x97, 0x8e, 0xdb
};

enum {
	dh_1k_q,
	dh_1k_s,
	dh_2k_q,
	dh_2k_s,
	dh_4k_q,
	dh_4k_s,
};

static struct key_info dh_key_info[] = {
	{
		.data = Q,
		.len = sizeof(Q),
	},
	{
		.data = S2,
		.len = sizeof(S2),
	},
	{
		.data = Q_2048,
		.len = sizeof(Q_2048),
	},
	{
		.data = S2_2048,
		.len = sizeof(S2_2048),
	},
	{
		.data = Q_4096,
		.len = sizeof(Q_4096),
	},
	{
		.data = S2_4096,
		.len = sizeof(S2_4096),
	},
};

static phys_addr_t get_q_phy_addr(int type)
{
	switch (type) {
	case SKMM_DH_1K:
		return dh_key_info[dh_1k_q].p_data;
	case SKMM_DH_2K:
		return dh_key_info[dh_2k_q].p_data;
	case SKMM_DH_4K:
		return dh_key_info[dh_4k_q].p_data;
	}

	return 0;
}

static phys_addr_t get_s_phy_addr(int type)
{
	switch (type) {
	case SKMM_DH_1K:
		return dh_key_info[dh_1k_s].p_data;
	case SKMM_DH_2K:
		return dh_key_info[dh_2k_s].p_data;
	case SKMM_DH_4K:
		return dh_key_info[dh_4k_s].p_data;
	}

	return 0;
}

static int get_q_len(int type)
{
	switch (type) {
	case SKMM_DH_1K:
		return dh_key_info[dh_1k_q].len;
	case SKMM_DH_2K:
		return dh_key_info[dh_2k_q].len;
	case SKMM_DH_4K:
		return dh_key_info[dh_4k_q].len;
	}

	return 0;
}

static int get_s_len(int type)
{
	switch (type) {
	case SKMM_DH_1K:
		return dh_key_info[dh_1k_s].len;
	case SKMM_DH_2K:
		return dh_key_info[dh_2k_s].len;
	case SKMM_DH_4K:
		return dh_key_info[dh_4k_s].len;
	}

	return 0;
}

void dh_blob(void)
{
	u8 *buf;
	int i;

	for (i = 0; i < ARRAY_SIZE(dh_key_info); i++) {
		buf = get_buffer(dh_key_info[i].len);
		print_debug("%d: %d\n", i, dh_key_info[i].len);
		memcpy(buf, dh_key_info[i].data, dh_key_info[i].len);
		dh_key_info[i].p_data =
			va_to_pa((va_addr_t)buf);
	}
}

int constr_dh_sec_desc(struct req_info *req_info)
{
	union desc_u *sec_desc = req_info->sec_desc;
	struct abs_req_s *abs_req = req_info->abs_req;
	u32 desc_size, start_idx;

	struct dh_key *abs_dh_key;
	struct dh_key_desc_s *dh_key;

	int type = req_info->req_type;
	int q_len, s_len;

	switch (type) {
	case SKMM_DH_1K:
	case SKMM_DH_2K:
	case SKMM_DH_4K:
		dh_key = sec_desc->dh_key;
		abs_dh_key = &abs_req->req.dh_key;

		dh_key->w_dma = abs_dh_key->w;
		dh_key->z_dma = abs_dh_key->z;
		dh_key->q_dma = get_q_phy_addr(type);
		dh_key->s_dma = get_s_phy_addr(type);

		q_len = get_q_len(type);
		s_len = get_s_len(type);

		desc_size = sizeof(struct dh_key_desc_s) / sizeof(u32);
		start_idx = desc_size - 1;

		start_idx &= HDR_START_IDX_MASK;
		init_job_desc(dh_key, (start_idx << HDR_START_IDX_SHIFT) |
				(desc_size & HDR_DESCLEN_MASK) | HDR_ONE);

		dh_key->sgf_ln = (q_len << 7) | s_len;
		dh_key->op = (CMD_OPERATION | OP_TYPE_UNI_PROTOCOL |
				OP_PCLID_DH);

		__sync_synchronize();

	}

	return 1;
}
