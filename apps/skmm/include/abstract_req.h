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

#ifndef __ABSTRACT_REQ_H__
#define __ABSTRACT_REQ_H__

#include <common.h>

static const char PRI3_P_1024[] = {
	0XD9, 0X06, 0X3D, 0XE5, 0X6C, 0X39, 0XB7, 0XC6, 0XD7, 0X0E, 0X4F, 0X04,
	0X1E, 0X8B, 0XFA, 0X2D, 0XEB, 0XFE, 0X9F, 0XB0, 0X46, 0X4F, 0X1D, 0XE2,
	0XA5, 0X9D, 0XCF, 0X58, 0XA5, 0X05, 0X60, 0X8F, 0XF7, 0XE6, 0X20, 0X54,
	0XAB, 0X8D, 0X4D, 0X32, 0X41, 0XB0, 0XEC, 0X07, 0X27, 0X90, 0X8F, 0XFA,
	0X52, 0X0E, 0X4E, 0XC8, 0X0E, 0XBC, 0X69, 0X1E, 0XBB, 0XCF, 0XBE, 0X0C,
	0XA0, 0X4D, 0XA6, 0X93
};

static const char PRI3_Q_1024[] = {
	0XDE, 0XB8, 0XD9, 0XA6, 0X3D, 0X7B, 0XA3, 0X13, 0XAD, 0XEC, 0X34, 0X0B,
	0X00, 0X44, 0X8D, 0X5B, 0X8C, 0XB9, 0X8C, 0XB7, 0X2D, 0X77, 0X96, 0XFE,
	0XDC, 0XFE, 0XEC, 0X59, 0X1F, 0XFA, 0XC9, 0X94, 0XA7, 0X41, 0XA3, 0XDA,
	0XE8, 0XBC, 0XA4, 0XE0, 0XEA, 0X40, 0X24, 0X25, 0X5F, 0X3E, 0X99, 0XB0,
	0X51, 0X5D, 0X19, 0X15, 0X65, 0X56, 0X5F, 0X92, 0X29, 0X62, 0XFB, 0X1B,
	0X6A, 0X25, 0X52, 0X33
};

static const char PRI3_DP_1024[] = {
	0X18, 0XB6, 0XAC, 0X3F, 0XBE, 0XF6, 0X47, 0XA6, 0X3E, 0X01, 0X24, 0X20,
	0X5C, 0XF6, 0X25, 0XB4, 0X2B, 0X06, 0XA0, 0XDA, 0XAA, 0XF7, 0X01, 0X79,
	0X38, 0XD0, 0XE2, 0XB6, 0XBE, 0X7C, 0X01, 0X46, 0X03, 0XBD, 0XD1, 0XFA,
	0XB2, 0X5A, 0X71, 0XEB, 0X02, 0X2E, 0X0A, 0X82, 0XBA, 0X0F, 0XAD, 0X45,
	0X9D, 0X28, 0X81, 0XF2, 0X28, 0X33, 0X6E, 0X69, 0X10, 0X8C, 0X14, 0XA1,
	0X4D, 0X73, 0XEA, 0X3F
};
static const char PRI3_DQ_1024[] = {
	0X5C, 0X90, 0x0C, 0x7C, 0XA7, 0x8E, 0XCB, 0XBE, 0XB1, 0X08, 0XBA, 0XB4,
	0X5B, 0XBF, 0X21, 0XD4, 0X8B, 0X8E, 0XCD, 0XE9, 0X8F, 0X2E, 0XE5, 0X85,
	0X34, 0X89, 0X89, 0XE6, 0X9C, 0X80, 0XD8, 0XE8, 0X3B, 0XFE, 0XEE, 0XBF,
	0X5A, 0XFC, 0X93, 0XB5, 0X9A, 0X05, 0X60, 0X40, 0X5B, 0XEE, 0X23, 0X67,
	0X80, 0XD9, 0X25, 0X44, 0X9C, 0X22, 0X22, 0X84, 0X17, 0X49, 0X38, 0XBD,
	0X33, 0X07, 0X21, 0XBB
};

static const char PRI3_C_1024[] = {
	0x9E, 0xD9, 0x87, 0x5E, 0x15, 0xF3, 0xFF, 0x36, 0x69, 0xF4, 0xCF, 0xDE,
	0xB5, 0xB4, 0x0F, 0x67, 0xBB, 0x30, 0xCA, 0x75, 0x92, 0x19, 0x95, 0xC9,
	0x5B, 0x37, 0x50, 0x8B, 0x6E, 0xBF, 0xDA, 0xF8, 0xF6, 0x54, 0xAA, 0x6C,
	0x7A, 0xEB, 0x34, 0x5F, 0x2D, 0x49, 0xE9, 0xD3, 0xDA, 0x2E, 0x4D, 0xDB,
	0x3F, 0x46, 0x8F, 0xB3, 0x98, 0x4F, 0x97, 0x33, 0xEF, 0x2A, 0x5A, 0xD4,
	0x55, 0xC3, 0xE4, 0x1D
};

#define RSA_PRIV_KEY_FRM_1     0
#define RSA_PRIV_KEY_FRM_2     1
#define RSA_PRIV_KEY_FRM_3     2

phys_addr_t parse_abs_to_desc(phys_addr_t req);

enum req_type {
	RSA_PUB,
	RSA_PRIV_FORM1,
	RSA_PRIV_FORM2,
	RSA_PRIV_FORM3,
	DSA_SIGN,
	DSA_VERIFY,
	ECDSA_SIGN,
	ECDSA_VERIFY,
	MAX_TYPES
};

struct rsa_pub {
	phys_addr_t n;
	phys_addr_t e;
	phys_addr_t f;
	phys_addr_t g;
	u32 n_len;
	u32 e_len;
	u32 f_len;
};

struct rsa_priv3 {
	phys_addr_t f;
	phys_addr_t g;
	u32 n_len;
};

struct dsa_sign {
	phys_addr_t m;
	phys_addr_t c;
	phys_addr_t d;
};

struct dsa_verify {
	phys_addr_t q;
	phys_addr_t r;
	phys_addr_t g;
	phys_addr_t pub_key;
	phys_addr_t m;
	phys_addr_t temp;
	phys_addr_t c;
	phys_addr_t d;
	u32 q_len;
	u32 r_len;
};

struct abs_req_s {
	u32 abs_req_id;
	union {
		struct rsa_pub rsa_pub;
		struct rsa_priv3 rsa_priv3;
		struct dsa_sign dsa_sign;
		struct dsa_verify dsa_verify;
	} req;
};

struct rsa_pub_desc_s {
	u32 desc_hdr;
	u32 sgf_flg;
	phys_addr_t f_dma;
	phys_addr_t g_dma;
	phys_addr_t n_dma;
	phys_addr_t e_dma;
	u32 msg_len;
	u32 op;
} __packed;

struct rsa_priv_frm1_desc_s {
	u32 desc_hdr;
	u32 sgf_flg;
	phys_addr_t g_dma;
	phys_addr_t f_dma;
	phys_addr_t n_dma;
	phys_addr_t d_dma;
	u32 op;
} __packed;

struct rsa_priv_frm2_desc_s {
	u32 desc_hdr;
	u32 sgf_flg;
	phys_addr_t g_dma;
	phys_addr_t f_dma;
	phys_addr_t d_dma;
	phys_addr_t p_dma;
	phys_addr_t q_dma;
	phys_addr_t tmp1_dma;
	phys_addr_t tmp2_dma;
	u32 p_q_len;
	u32 op;
} __packed;

struct rsa_priv_frm3_desc_s {
	u32 desc_hdr;
	u32 sgf_flg;
	phys_addr_t g_dma;
	phys_addr_t f_dma;
	phys_addr_t c_dma;
	phys_addr_t p_dma;
	phys_addr_t q_dma;
	phys_addr_t dp_dma;
	phys_addr_t dq_dma;
	phys_addr_t tmp1_dma;
	phys_addr_t tmp2_dma;
	u32 p_q_len;
	u32 op;
}; __packed

#define DSA_L_SHIFT	7

struct dsa_sign_desc_s {
	u32 desc_hdr;
	u32 sgf_ln;
	phys_addr_t q_dma;
	phys_addr_t r_dma;
	phys_addr_t g_dma;
	phys_addr_t s_dma;
	phys_addr_t f_dma;
	phys_addr_t c_dma;
	phys_addr_t d_dma;
	u32 op;
}; __packed

struct dsa_verify_desc_s {
	u32 desc_hdr;
	u32 sgf_ln;
	phys_addr_t q_dma;
	phys_addr_t r_dma;
	phys_addr_t g_dma;
	phys_addr_t w_dma;
	phys_addr_t f_dma;
	phys_addr_t c_dma;
	phys_addr_t d_dma;
	phys_addr_t tmp_dma;
	u32 op;
}; __packed

struct ecdsa_sign_desc_s {
	u32 desc_hdr;
	u32 sgf_ln;
	phys_addr_t q_dma;
	phys_addr_t r_dma;
	phys_addr_t g_dma;
	phys_addr_t s_dma;
	phys_addr_t f_dma;
	phys_addr_t c_dma;
	phys_addr_t d_dma;
	phys_addr_t ab_dma;
	u32 op;
};

struct ecdsa_verify_desc_s {
	u32 desc_hdr;
	u32 sgf_ln;
	phys_addr_t q_dma;
	phys_addr_t r_dma;
	phys_addr_t g_dma;
	phys_addr_t w_dma;
	phys_addr_t f_dma;
	phys_addr_t c_dma;
	phys_addr_t d_dma;
	phys_addr_t tmp_dma;
	phys_addr_t ab_dma;
	u32 op;
};

union desc_u {
	struct rsa_pub_desc_s *rsa_pub_desc;
	struct rsa_priv_frm1_desc_s *rsa_priv_f1;
	struct rsa_priv_frm2_desc_s *rsa_priv_f2;
	struct rsa_priv_frm3_desc_s *rsa_priv_f3;
	struct dsa_sign_desc_s *dsa_sign;
	struct dsa_verify_desc_s *dsa_verify;
};

struct req_info {
	int req_type;
	union req_u *req;
	union desc_u *sec_desc;
	struct abs_req_s *abs_req;
};

/* Start Index or SharedDesc Length */
#define HDR_START_IDX_MASK	0x3f
#define HDR_START_IDX_SHIFT	16

/* If non-shared header, 7-bit length */
#define HDR_DESCLEN_MASK	0x7f

/*
 * ONE - should always be set. Combination of ONE (always
 * set) and ZRO (always clear) forms an endianness sanity check
 */
#define HDR_ONE	0x00800000
#define HDR_ZRO	0x00008000



/*
 * Supported descriptor command types as they show up
 * inside a descriptor command word.
 */
#define CMD_SHIFT		27
#define CMD_MASK		0xf8000000

#define CMD_KEY			(0x00 << CMD_SHIFT)
#define CMD_SEQ_KEY		(0x01 << CMD_SHIFT)
#define CMD_LOAD		(0x02 << CMD_SHIFT)
#define CMD_SEQ_LOAD		(0x03 << CMD_SHIFT)
#define CMD_FIFO_LOAD		(0x04 << CMD_SHIFT)
#define CMD_SEQ_FIFO_LOAD	(0x05 << CMD_SHIFT)
#define CMD_STORE		(0x0a << CMD_SHIFT)
#define CMD_SEQ_STORE		(0x0b << CMD_SHIFT)
#define CMD_FIFO_STORE		(0x0c << CMD_SHIFT)
#define CMD_SEQ_FIFO_STORE	(0x0d << CMD_SHIFT)
#define CMD_MOVE_LEN		(0x0e << CMD_SHIFT)
#define CMD_MOVE		(0x0f << CMD_SHIFT)
#define CMD_OPERATION		(0x10 << CMD_SHIFT)
#define CMD_SIGNATURE		(0x12 << CMD_SHIFT)
#define CMD_JUMP		(0x14 << CMD_SHIFT)
#define CMD_MATH		(0x15 << CMD_SHIFT)
#define CMD_DESC_HDR		(0x16 << CMD_SHIFT)
#define CMD_SHARED_DESC_HDR	(0x17 << CMD_SHIFT)
#define CMD_SEQ_IN_PTR		(0x1e << CMD_SHIFT)
#define CMD_SEQ_OUT_PTR		(0x1f << CMD_SHIFT)

/*
 * OPERATION Command Constructs
 */

/* Operation type selectors - OP TYPE */
#define OP_TYPE_SHIFT		24
#define OP_TYPE_MASK		(0x07 << OP_TYPE_SHIFT)

#define OP_TYPE_UNI_PROTOCOL	(0x00 << OP_TYPE_SHIFT)
#define OP_TYPE_PK		(0x01 << OP_TYPE_SHIFT)
#define OP_TYPE_CLASS1_ALG	(0x02 << OP_TYPE_SHIFT)
#define OP_TYPE_CLASS2_ALG	(0x04 << OP_TYPE_SHIFT)
#define OP_TYPE_DECAP_PROTOCOL	(0x06 << OP_TYPE_SHIFT)
#define OP_TYPE_ENCAP_PROTOCOL	(0x07 << OP_TYPE_SHIFT)

/* ProtocolID selectors - PROTID */
#define OP_PCLID_SHIFT		16
#define OP_PCLID_MASK		(0xff << 16)

/* Assuming OP_TYPE = OP_TYPE_UNI_PROTOCOL */
#define OP_PCLID_IKEV1_PRF	(0x01 << OP_PCLID_SHIFT)
#define OP_PCLID_IKEV2_PRF	(0x02 << OP_PCLID_SHIFT)
#define OP_PCLID_SSL30_PRF	(0x08 << OP_PCLID_SHIFT)
#define OP_PCLID_TLS10_PRF	(0x09 << OP_PCLID_SHIFT)
#define OP_PCLID_TLS11_PRF	(0x0a << OP_PCLID_SHIFT)
#define OP_PCLID_DTLS10_PRF	(0x0c << OP_PCLID_SHIFT)
#define OP_PCLID_PRF		(0x06 << OP_PCLID_SHIFT)
#define OP_PCLID_BLOB		(0x0d << OP_PCLID_SHIFT)
#define OP_PCLID_SECRETKEY	(0x11 << OP_PCLID_SHIFT)
#define OP_PCLID_PUBLICKEYPAIR	(0x14 << OP_PCLID_SHIFT)
#define OP_PCLID_DSASIGN	(0x15 << OP_PCLID_SHIFT)
#define OP_PCLID_DSAVERIFY	(0x16 << OP_PCLID_SHIFT)
#define OP_PCLID_DH		(0x17 << OP_PCLID_SHIFT)
#define OP_PCLID_RSAENC_PUBKEY	(0x18 << OP_PCLID_SHIFT)
#define OP_PCLID_RSADEC_PRVKEY	(0x19 << OP_PCLID_SHIFT)

#define OP_PCL_PKPROT_ECC	0x0002
static inline void init_desc(void *desc, u32 options)
{
	out_be32((u32 *)desc, ((options | HDR_ONE)));
}

static inline void init_job_desc(void *desc, u32 options)
{
	init_desc(desc, CMD_DESC_HDR | options);
}

phys_addr_t parse_abs_to_desc(phys_addr_t req);

#endif /* __ABSTRACT_REQ_H__ */
