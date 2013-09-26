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

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/bn.h>
#include <openssl/rsa.h>
#include <openssl/evp.h>
#include <openssl/x509.h>
#include <openssl/pem.h>
#include <openssl/rand.h>

#define PAGESIZE 0x1000
enum {
	priv3_p_1024,
	priv3_q_1024,
	priv3_dp_1024,
	priv3_dq_1024,
	priv3_c_1024,
	priv3_tmp1_1024,
	priv3_tmp2_1024,

	priv3_p_2048,
	priv3_q_2048,
	priv3_dp_2048,
	priv3_dq_2048,
	priv3_c_2048,
	priv3_tmp1_2048,
	priv3_tmp2_2048,

	priv3_p_4096,
	priv3_q_4096,
	priv3_dp_4096,
	priv3_dq_4096,
	priv3_c_4096,
	priv3_tmp1_4096,
	priv3_tmp2_4096,
};

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

/* PRIV FORM3 DECRYPT */
static u8 P_1024[] = {
	0XD9, 0X06, 0X3D, 0XE5, 0X6C, 0X39, 0XB7, 0XC6, 0XD7, 0X0E, 0X4F, 0X04,
	0X1E, 0X8B, 0XFA, 0X2D, 0XEB, 0XFE, 0X9F, 0XB0, 0X46, 0X4F,
	0X1D, 0XE2, 0XA5, 0X9D, 0XCF, 0X58, 0XA5, 0X05, 0X60, 0X8F, 0XF7, 0XE6,
	0X20, 0X54, 0XAB, 0X8D, 0X4D, 0X32, 0X41, 0XB0, 0XEC, 0X07,
	0X27, 0X90, 0X8F, 0XFA, 0X52, 0X0E, 0X4E, 0XC8, 0X0E, 0XBC, 0X69, 0X1E,
	0XBB, 0XCF, 0XBE, 0X0C, 0XA0, 0X4D, 0XA6, 0X93
};

static u8 Q_1024[] = {
	0XDE, 0XB8, 0XD9, 0XA6, 0X3D, 0X7B, 0XA3, 0X13, 0XAD, 0XEC, 0X34, 0X0B,
	0X00, 0X44, 0X8D, 0X5B, 0X8C, 0XB9, 0X8C, 0XB7, 0X2D, 0X77,
	0X96, 0XFE, 0XDC, 0XFE, 0XEC, 0X59, 0X1F, 0XFA, 0XC9, 0X94, 0XA7, 0X41,
	0XA3, 0XDA, 0XE8, 0XBC, 0XA4, 0XE0, 0XEA, 0X40, 0X24, 0X25,
	0X5F, 0X3E, 0X99, 0XB0, 0X51, 0X5D, 0X19, 0X15, 0X65, 0X56, 0X5F, 0X92,
	0X29, 0X62, 0XFB, 0X1B, 0X6A, 0X25, 0X52, 0X33
};

static u8 DP_1024[] = {
	0X18, 0XB6, 0XAC, 0X3F, 0XBE, 0XF6, 0X47, 0XA6, 0X3E, 0X01, 0X24, 0X20,
	0X5C, 0XF6, 0X25, 0XB4, 0X2B, 0X06, 0XA0, 0XDA, 0XAA, 0XF7,
	0X01, 0X79, 0X38, 0XD0, 0XE2, 0XB6, 0XBE, 0X7C, 0X01, 0X46, 0X03, 0XBD,
	0XD1, 0XFA, 0XB2, 0X5A, 0X71, 0XEB, 0X02, 0X2E, 0X0A, 0X82,
	0XBA, 0X0F, 0XAD, 0X45, 0X9D, 0X28, 0X81, 0XF2, 0X28, 0X33, 0X6E, 0X69,
	0X10, 0X8C, 0X14, 0XA1, 0X4D, 0X73, 0XEA, 0X3F
};

static u8 DQ_1024[] = {
	0X5C, 0X90, 0x0C, 0x7C, 0XA7, 0x8E, 0XCB, 0XBE, 0XB1, 0X08, 0XBA, 0XB4,
	0X5B, 0XBF, 0X21, 0XD4, 0X8B, 0X8E,
	0XCD, 0XE9, 0X8F, 0X2E, 0XE5, 0X85, 0X34, 0X89, 0X89, 0XE6, 0X9C, 0X80,
	0XD8, 0XE8, 0X3B, 0XFE, 0XEE, 0XBF,
	0X5A, 0XFC, 0X93, 0XB5, 0X9A, 0X05, 0X60, 0X40, 0X5B, 0XEE, 0X23, 0X67,
	0X80, 0XD9, 0X25, 0X44, 0X9C, 0X22,
	0X22, 0X84, 0X17, 0X49, 0X38, 0XBD, 0X33, 0X07, 0X21, 0XBB
};

static u8 C_1024[] = {
	0x9E, 0xD9, 0x87, 0x5E, 0x15, 0xF3, 0xFF, 0x36, 0x69, 0xF4, 0xCF, 0xDE,
	0xB5, 0xB4, 0x0F, 0x67, 0xBB, 0x30, 0xCA, 0x75, 0x92, 0x19,
	0x95, 0xC9, 0x5B, 0x37, 0x50, 0x8B, 0x6E, 0xBF, 0xDA, 0xF8, 0xF6, 0x54,
	0xAA, 0x6C, 0x7A, 0xEB, 0x34, 0x5F, 0x2D, 0x49, 0xE9, 0xD3,
	0xDA, 0x2E, 0x4D, 0xDB, 0x3F, 0x46, 0x8F, 0xB3, 0x98, 0x4F, 0x97, 0x33,
	0xEF, 0x2A, 0x5A, 0xD4, 0x55, 0xC3, 0xE4, 0x1D
};

static u8 P_2048[] = {
	0xf3, 0x87, 0x7c, 0x11, 0x0d, 0xd4, 0x9d, 0x63, 0xca, 0x11, 0x24, 0x9a,
	0x49, 0xc2,
	0x16, 0xc8, 0xcf, 0x14, 0xb3, 0xb1, 0x78, 0x09, 0xf4, 0x52, 0x11, 0x2e,
	0x8a, 0x36, 0xd0,
	0xc6, 0xbc, 0x03, 0x2d, 0xca, 0x1f, 0xbc, 0x91, 0x4c, 0x21, 0x86, 0x3f,
	0x07, 0x13, 0x7b,
	0xaa, 0x08, 0x05, 0xd0, 0x41, 0x52, 0x6c, 0xeb, 0x4c, 0x8c, 0x4a, 0xd2,
	0x3c, 0xd6, 0x85,
	0x4e, 0xe5, 0xd0, 0xf7, 0x60, 0x5e, 0x25, 0x4b, 0xcc, 0x02, 0x4b, 0x99,
	0x19, 0x5e, 0xd9,
	0x3b, 0xd8, 0xd2, 0x89, 0x96, 0xdc, 0x93, 0xc3, 0x6d, 0x15, 0x19, 0xec,
	0xfc, 0x9b, 0x62,
	0xa9, 0x11, 0xdb, 0xb5, 0x28, 0x50, 0x93, 0xc4, 0x81, 0xe9, 0x95, 0x08,
	0xbe, 0x8c, 0x17,
	0xad, 0xb8, 0x4a, 0x62, 0x4a, 0x6c, 0x1a, 0x6d, 0x40, 0xaf, 0xb4, 0x50,
	0xb2, 0x3f, 0x89,
	0xbc, 0x06, 0x17, 0x66, 0x5d, 0xdf, 0x82, 0xd3, 0x93
};

static u8 Q_2048[] = {
	0xc8, 0xfa, 0x57, 0x33, 0x53, 0x58, 0x53, 0x16, 0x9c, 0x9f, 0x69, 0xd4,
	0xfc, 0x32,
	0x1c, 0x24, 0x9c, 0x28, 0xc1, 0x6c, 0xf4, 0x61, 0x8b, 0x35, 0xd6, 0x5e,
	0xaf, 0x5a, 0x09,
	0xb2, 0x88, 0xae, 0x49, 0x0b, 0x17, 0x74, 0xe2, 0xa5, 0xf1, 0x84, 0xf6,
	0xb1, 0x38, 0xfc,
	0x68, 0x97, 0x54, 0xfc, 0x2d, 0x78, 0xda, 0x40, 0x00, 0xee, 0x3f, 0x5e,
	0x64, 0x49, 0xb1,
	0x7d, 0x97, 0x2d, 0x10, 0x77, 0xb4, 0x66, 0xbe, 0xef, 0xdc, 0x95, 0xa5,
	0x86, 0x90, 0xa0,
	0x09, 0x2f, 0x9b, 0x0e, 0x1a, 0x37, 0xb8, 0x0f, 0x53, 0x27, 0x3e, 0x41,
	0x3f, 0x71, 0x7c,
	0x19, 0x98, 0x58, 0x9f, 0x25, 0x81, 0x30, 0xfb, 0xe7, 0x34, 0xc4, 0x50,
	0xd8, 0x7c, 0x94,
	0xd3, 0x4e, 0x92, 0x13, 0x90, 0xeb, 0x0d, 0x5c, 0xbb, 0xc8, 0xad, 0xc5,
	0x34, 0x5d, 0x5c,
	0x9e, 0x4b, 0x55, 0x9c, 0x06, 0xe2, 0x62, 0x66, 0x8b
};

static u8 DP_2048[] = {
	0xaa, 0xd0, 0x0d, 0x25, 0xa9, 0x1d, 0xc8, 0x87, 0x85, 0xe4, 0x20, 0x58,
	0x58, 0x03,
	0xdb, 0x17, 0x5e, 0xa8, 0x96, 0xa1, 0x94, 0x20, 0xe7, 0x4d, 0xe8, 0xdf,
	0x4b, 0xf2, 0xc6,
	0xdd, 0x9e, 0x62, 0x5c, 0x6b, 0xb9, 0x76, 0xfa, 0x32, 0xfc, 0x10, 0xbd,
	0x84, 0xa9, 0x15,
	0xc4, 0x5d, 0x7c, 0x36, 0x66, 0x9a, 0xf9, 0xea, 0xd0, 0xf6, 0x56, 0x44,
	0xc0, 0x87, 0x77,
	0x59, 0xaf, 0xb8, 0xb2, 0xca, 0xd5, 0x3e, 0x33, 0xbc, 0x14, 0xa0, 0x11,
	0xf2, 0xc9, 0xa4,
	0x75, 0x65, 0xcf, 0xc6, 0xd2, 0x1a, 0x45, 0x4a, 0x7d, 0xe8, 0x9f, 0x16,
	0xaa, 0xd8, 0x12,
	0x8b, 0xa4, 0x0b, 0x07, 0x36, 0x0f, 0xce, 0x95, 0xb6, 0xc9, 0xaf, 0xcf,
	0x42, 0x57, 0xe1,
	0x03, 0xe8, 0x8f, 0xc4, 0x01, 0x8f, 0x71, 0xb9, 0xcf, 0xf5, 0x6d, 0xf4,
	0x6c, 0x12, 0x44,
	0xf9, 0xad, 0x0c, 0x1c, 0xb1, 0x1b, 0x65, 0x18, 0x21
};

static u8 DQ_2048[] = {
	0x2f, 0x59, 0x4b, 0x51, 0xfc, 0x40, 0xc2, 0xd2, 0x80, 0xf3, 0xcd, 0xab,
	0x2f, 0xff, 0x5e,
	0x42, 0x01, 0xc8, 0x58, 0x49, 0xa3, 0x09, 0x98, 0x12, 0x86, 0xfe, 0xa7,
	0xb1, 0x3e, 0xb7,
	0xa1, 0x1b, 0xee, 0x79, 0x7e, 0x7d, 0x3b, 0x28, 0x7d, 0x4e, 0x26, 0x10,
	0x7a, 0xac, 0x7c,
	0xb7, 0xaa, 0x7d, 0xfe, 0xf8, 0x89, 0xf2, 0xaa, 0x45, 0x77, 0x08, 0x75,
	0xf8, 0x26, 0xa8,
	0xc0, 0x05, 0x0d, 0xec, 0x25, 0xc4, 0x4f, 0x17, 0x93, 0x90, 0x6d, 0xfe,
	0xd5, 0xf3, 0xde,
	0x36, 0x29, 0x01, 0x99, 0x8b, 0xa7, 0x01, 0x75, 0x45, 0xb9, 0x95, 0x05,
	0xad, 0x66, 0xdf,
	0x60, 0xd5, 0x00, 0x29, 0x85, 0xd6, 0x54, 0x82, 0x94, 0x1b, 0xec, 0x30,
	0x14, 0xa9, 0xbb,
	0x5e, 0xb7, 0x6b, 0x84, 0xa8, 0xdb, 0xed, 0x0d, 0x10, 0xaa, 0x5f, 0xdb,
	0x86, 0x63, 0x5b,
	0xa7, 0xc3, 0xf3, 0x8e, 0x6b, 0xa7, 0x63, 0x21
};

static u8 C_2048[] = {
	0xf1, 0x12, 0x8d, 0x86, 0x2a, 0xcb, 0x88, 0x60, 0x5c, 0x36, 0xde, 0x9f,
	0x3f, 0x30,
	0x81, 0x82, 0x8d, 0x83, 0x38, 0xf0, 0xef, 0x8a, 0x2d, 0xa1, 0x13, 0x2a,
	0x2d, 0xbe, 0xc1,
	0x99, 0xa0, 0xb1, 0x86, 0xae, 0xff, 0xbc, 0xae, 0xe3, 0x7d, 0x9f, 0x5c,
	0x1c, 0x6b, 0x48,
	0x0b, 0xb9, 0x87, 0xa6, 0x78, 0x55, 0x60, 0xcb, 0xba, 0x67, 0x49, 0xae,
	0x6c, 0x80, 0x33,
	0xa3, 0xe2, 0x5d, 0x8d, 0x45, 0x85, 0xc7, 0xc4, 0x24, 0x03, 0x99, 0xb5,
	0xee, 0x87, 0xfc,
	0xfe, 0x30, 0xa3, 0x1f, 0x4d, 0x4c, 0x64, 0xfa, 0xda, 0x23, 0x83, 0xfa,
	0x65, 0x08, 0xdb,
	0x8f, 0xca, 0xcc, 0xf2, 0x2a, 0xcb, 0xae, 0xf5, 0x47, 0xf8, 0x37, 0xc7,
	0x7d, 0x52, 0x8a,
	0x8b, 0xf9, 0x99, 0xbb, 0x1c, 0xcf, 0x11, 0x73, 0x98, 0x0c, 0x6d, 0x94,
	0xe1, 0x73, 0x8f,
	0x02, 0x5c, 0x33, 0xfa, 0x2f, 0x89, 0xe2, 0x48, 0x83
};

static u8 P_4096[] = {
	0xec, 0xb0, 0x02, 0x03, 0x1c, 0x14, 0xd0, 0xa9, 0x33, 0xd5, 0x13, 0x4e,
	0x7a, 0xd7,
	0xad, 0xdf, 0x8c, 0xe7, 0x16, 0xc1, 0x7a, 0x4b, 0x84, 0xc4, 0x58, 0x96,
	0x53, 0x8b, 0x8f,
	0x15, 0x05, 0xfd, 0xf3, 0xd7, 0x89, 0xfe, 0x6c, 0x49, 0x35, 0x47, 0x60,
	0xbe, 0xff, 0xf7,
	0x07, 0xd8, 0x9e, 0x78, 0x53, 0x52, 0x33, 0x71, 0x28, 0x45, 0x3b, 0xa4,
	0xf5, 0x3a, 0xb4,
	0xfc, 0x4f, 0x63, 0x10, 0x1b, 0xdb, 0x14, 0xf3, 0xf2, 0x40, 0x20, 0xc9,
	0xef, 0xf1, 0x85,
	0x9e, 0xde, 0x31, 0xb0, 0x55, 0x60, 0x8a, 0xa3, 0x24, 0x76, 0x87, 0x16,
	0xc5, 0x60, 0xf8,
	0x2d, 0x99, 0xd3, 0x34, 0x82, 0xc9, 0x4b, 0x1a, 0x3f, 0x1e, 0xca, 0x9f,
	0xfb, 0x5a, 0x0d,
	0xa3, 0x1b, 0x4e, 0xbd, 0x00, 0xa5, 0x89, 0x57, 0x23, 0x1e, 0xcb, 0xd5,
	0x06, 0xe9, 0xd0,
	0x97, 0x52, 0x8f, 0xb3, 0x81, 0xed, 0x48, 0xaa, 0xe0, 0x2d, 0xac, 0xba,
	0x24, 0xf2, 0xc1,
	0xe2, 0xb2, 0x57, 0x0d, 0xd8, 0xa6, 0x48, 0x28, 0xb9, 0xa5, 0x25, 0x6a,
	0x90, 0x6d, 0x5f,
	0x6f, 0xfa, 0x7e, 0xfd, 0xa3, 0x51, 0x84, 0xd0, 0xdb, 0x95, 0xe2, 0xe2,
	0xbe, 0xe5, 0xe3,
	0x40, 0x94, 0x30, 0x8f, 0x25, 0x83, 0x09, 0xa0, 0x47, 0xb9, 0x80, 0x6a,
	0x72, 0x93, 0x69,
	0x16, 0x8f, 0x7e, 0x7e, 0x52, 0x04, 0xc2, 0x9e, 0xe6, 0xae, 0x1e, 0x58,
	0x4f, 0x2e, 0x20,
	0xdb, 0x6d, 0x69, 0x08, 0x77, 0x7d, 0xbc, 0xa4, 0xe0, 0x56, 0x3f, 0x45,
	0x6b, 0x47, 0x23,
	0xc3, 0x4a, 0x3a, 0x5e, 0x96, 0x48, 0x55, 0x60, 0xe0, 0x4b, 0x5e, 0xd8,
	0x35, 0x4d, 0x3f,
	0x57, 0x42, 0x1e, 0x60, 0x54, 0x6a, 0x7e, 0xc6, 0x15, 0x27, 0xe6, 0x8b,
	0xa9, 0xcb, 0x99,
	0xee, 0xc6, 0x28, 0x92, 0xa7, 0x7d, 0x94, 0xa6, 0x4c, 0xcb, 0x61, 0x46,
	0x30, 0x30, 0x8a,
	0x1f, 0xa1
};

static u8 Q_4096[] = {
	0xde, 0xbc, 0xe0, 0xf8, 0x30, 0x44, 0x45, 0xaa, 0x56, 0xf7, 0x0d, 0x8b,
	0xb4, 0x03,
	0xdd, 0x78, 0x16, 0x55, 0xe4, 0xe1, 0xde, 0x78, 0x28, 0x80, 0xbc, 0xc2,
	0xa3, 0x88, 0x57,
	0xf3, 0x2f, 0x0d, 0xf5, 0xa3, 0xf5, 0x34, 0xbe, 0xc9, 0x8b, 0x29, 0x21,
	0x44, 0x62, 0x2b,
	0xa7, 0x9c, 0xb8, 0x47, 0xb4, 0x6a, 0x5a, 0x79, 0x8c, 0x90, 0x17, 0x7b,
	0x24, 0x69, 0x96,
	0xc7, 0x1c, 0x8e, 0x7e, 0x56, 0x25, 0xd0, 0xfb, 0x73, 0x96, 0x08, 0x84,
	0x4f, 0x64, 0xff,
	0xbd, 0xb1, 0xad, 0xb3, 0x75, 0x2c, 0xd5, 0xeb, 0xcb, 0xa1, 0x1e, 0xc7,
	0xdd, 0xc0, 0xae,
	0x4b, 0xdb, 0xd1, 0xf9, 0xf7, 0x8d, 0x35, 0xfa, 0x57, 0xdd, 0xe8, 0x3f,
	0x73, 0x7c, 0x0b,
	0x5a, 0xa8, 0x68, 0x1b, 0xd9, 0x42, 0x2f, 0x36, 0xf3, 0xb5, 0xb7, 0x64,
	0xa1, 0xf7, 0x37,
	0x7a, 0x2b, 0xa7, 0xfa, 0xdf, 0x5f, 0x3e, 0x62, 0xa7, 0x0c, 0x58, 0x8b,
	0xee, 0xf2, 0xed,
	0x6e, 0x1e, 0x06, 0xe8, 0x00, 0xe4, 0x1d, 0x80, 0xe1, 0x9a, 0x40, 0xfe,
	0x28, 0xee, 0x6a,
	0xf4, 0x32, 0x07, 0x2e, 0x22, 0xa9, 0xdd, 0xd6, 0x85, 0x10, 0xc5, 0xc3,
	0x16, 0x5d, 0x6e,
	0x28, 0x82, 0xdb, 0x34, 0x81, 0xbc, 0x53, 0x01, 0xac, 0x45, 0x8b, 0x70,
	0x66, 0x30, 0x57,
	0xf1, 0x83, 0x35, 0xc3, 0xb3, 0x6e, 0x65, 0x55, 0xda, 0x4c, 0x61, 0x93,
	0xfe, 0xfc, 0x90,
	0x8a, 0xc6, 0x30, 0xf1, 0x0b, 0xe6, 0x35, 0x50, 0x0c, 0x87, 0xfe, 0x54,
	0x6e, 0xec, 0xce,
	0xb3, 0x07, 0xd6, 0x8d, 0x61, 0x2b, 0x0e, 0x8d, 0x5f, 0x0d, 0xd8, 0x7b,
	0x11, 0x7e, 0x13,
	0x93, 0xa7, 0x2e, 0xec, 0xd4, 0x0c, 0xf9, 0x7f, 0x6c, 0x3f, 0x70, 0xd1,
	0xf1, 0xa6, 0x2d,
	0x63, 0x13, 0xd2, 0x21, 0xfd, 0x9d, 0x21, 0x53, 0x4d, 0xb0, 0xaa, 0xa5,
	0xe5, 0x67, 0x63,
	0x14, 0xab
};

static u8 DP_4096[] = {
	0x4d, 0x42, 0xd3, 0x15, 0x52, 0xc8, 0x54, 0xa8, 0xb6, 0xfb, 0xb9, 0xf6,
	0xa7, 0x50, 0xda,
	0x38, 0x1c, 0x15, 0x9f, 0x2e, 0xff, 0x0d, 0xc1, 0xc4, 0x1a, 0x2a, 0xd0,
	0x10, 0xc1, 0x5f,
	0x1a, 0x7a, 0xa0, 0x6b, 0x5e, 0x67, 0x47, 0xcf, 0xc9, 0xed, 0x87, 0xde,
	0x31, 0x4d, 0xe1,
	0x28, 0xcb, 0xe9, 0xf9, 0x40, 0xde, 0xfb, 0xf4, 0x2a, 0x4d, 0x62, 0xf6,
	0x8d, 0xf8, 0x60,
	0x58, 0x45, 0xbd, 0x0f, 0x6a, 0xbf, 0x77, 0x36, 0x3f, 0xca, 0xb4, 0x40,
	0x77, 0xf5, 0xa7,
	0x3b, 0x5e, 0xba, 0xf1, 0xd3, 0xb6, 0xcd, 0xb4, 0x6c, 0x60, 0x82, 0x85,
	0x61, 0xb7, 0x01,
	0xa3, 0xb1, 0xf6, 0xf5, 0x2e, 0x62, 0x2e, 0xaa, 0x26, 0x11, 0xfc, 0x91,
	0x1e, 0xff, 0x4e,
	0x9b, 0xee, 0x62, 0xf1, 0xe0, 0x17, 0x69, 0xf3, 0x53, 0xed, 0x33, 0x97,
	0xb3, 0x21, 0x5d,
	0x14, 0x1d, 0x46, 0x7a, 0x86, 0x0b, 0x6f, 0x84, 0x08, 0x44, 0xd6, 0xea,
	0x92, 0xb7, 0xc9,
	0x99, 0x6f, 0xa3, 0x5c, 0x72, 0x43, 0x21, 0xd0, 0x1f, 0xe1, 0x8b, 0xdc,
	0xa3, 0x67, 0x40,
	0xf5, 0x63, 0x18, 0xc6, 0x84, 0x28, 0xb7, 0xc8, 0xc6, 0x2f, 0xae, 0xd6,
	0xa3, 0xd2, 0x3f,
	0xd5, 0x5f, 0xd8, 0x8a, 0x65, 0x30, 0xf2, 0x3a, 0x17, 0x8a, 0x9c, 0xb4,
	0xd3, 0xfb, 0xbc,
	0xda, 0x90, 0x6f, 0x09, 0xb4, 0xf6, 0x16, 0xca, 0xb4, 0x51, 0xc1, 0x7f,
	0xef, 0x4d, 0xd8,
	0x72, 0xe9, 0x93, 0x52, 0xf6, 0x1f, 0x6b, 0x2e, 0xb3, 0xd8, 0xa9, 0x7f,
	0x70, 0x54, 0x73,
	0x3e, 0x3a, 0xb9, 0x7f, 0x93, 0x13, 0x13, 0xb1, 0x83, 0xc8, 0xed, 0x08,
	0xd1, 0x33, 0x78,
	0xf7, 0x63, 0x55, 0x0c, 0x99, 0x4a, 0xd2, 0xbc, 0x3b, 0x2f, 0x1d, 0xae,
	0x0a, 0x81, 0x30,
	0x19, 0x4d, 0x4b, 0x3a, 0x13, 0x5b, 0x3f, 0xc1, 0xd7, 0xad, 0xe4, 0x2c,
	0x7c, 0xdb, 0xc9,
	0xa1
};

static u8 DQ_4096[] = {
	0x30, 0x2d, 0xea, 0xfb, 0xc4, 0x75, 0x00, 0x1c, 0xb8, 0x72, 0xf8, 0x1b,
	0x1f, 0xf5, 0x36,
	0x12, 0xa3, 0xc2, 0x30, 0xa0, 0x2a, 0xdf, 0x12, 0xe3, 0xc5, 0xf0, 0xd4,
	0x2e, 0xc9, 0xd7,
	0x70, 0x76, 0x34, 0x8c, 0x22, 0x9e, 0x26, 0x26, 0xb2, 0x53, 0x53, 0x3e,
	0xd7, 0x7d, 0x59,
	0xe4, 0x2c, 0x78, 0x56, 0x2e, 0x2b, 0x23, 0xdf, 0xa3, 0xcb, 0x70, 0x77,
	0x8f, 0xdf, 0x6d,
	0x72, 0x5f, 0xe0, 0x34, 0x02, 0x3a, 0x12, 0x2a, 0x0e, 0x6a, 0x09, 0x34,
	0xa3, 0x44, 0x2e,
	0x64, 0x20, 0x8e, 0x90, 0xea, 0x01, 0xdb, 0xdf, 0x50, 0x1a, 0xb8, 0x60,
	0xdf, 0x2c, 0xd4,
	0x7b, 0xd8, 0x0f, 0x99, 0xdc, 0x91, 0xb0, 0x75, 0x11, 0x2b, 0x0b, 0x8e,
	0x8f, 0x66, 0x65,
	0x55, 0xc8, 0x2f, 0x3d, 0xef, 0x73, 0x07, 0x0f, 0xf0, 0x5e, 0x9b, 0x05,
	0xed, 0xd3, 0xb2,
	0x39, 0x7f, 0x3c, 0x64, 0xd4, 0xd5, 0x49, 0xc1, 0x98, 0x8a, 0x0e, 0xba,
	0xc1, 0x11, 0x5a,
	0xa0, 0x64, 0x1e, 0x0b, 0x13, 0x4c, 0xce, 0x73, 0xfe, 0x49, 0xd0, 0xb0,
	0x49, 0xe2, 0x18,
	0xb0, 0x39, 0x9d, 0x61, 0x22, 0x1d, 0x3a, 0x29, 0xce, 0x56, 0xf8, 0xbb,
	0xfd, 0xb9, 0xb6,
	0x49, 0x97, 0xe0, 0xe3, 0xaf, 0x38, 0xc5, 0x43, 0xde, 0x47, 0x2b, 0x28,
	0xef, 0x8c, 0x16,
	0x51, 0xab, 0x6f, 0x89, 0x80, 0x77, 0xaa, 0xfa, 0x6d, 0x4b, 0x2e, 0x18,
	0x19, 0xc9, 0xaa,
	0x3a, 0xf4, 0x9b, 0x2f, 0x57, 0xbc, 0x70, 0x74, 0x45, 0x76, 0x5e, 0x0d,
	0x20, 0x6c, 0x15,
	0x8d, 0xbc, 0x7b, 0x18, 0x69, 0x97, 0xdc, 0x3c, 0x93, 0x62, 0x20, 0x3c,
	0xc8, 0x68, 0xb3,
	0xa7, 0x96, 0x1e, 0xe4, 0x3f, 0x6a, 0x38, 0x85, 0xd1, 0xb3, 0x35, 0xb6,
	0x6f, 0x9b, 0xa1,
	0xb2, 0xc7, 0xe1, 0x52, 0x5a, 0x8a, 0xba, 0xda, 0x33, 0x3c, 0xb6, 0x65,
	0xc8, 0xd7, 0x45,
	0xc1
};

static u8 C_4096[] = {
	0xba, 0xb6, 0xa4, 0x6e, 0xe8, 0x53, 0x9e, 0x2c, 0x63, 0xb4, 0xf5, 0x7d,
	0xe2, 0x93,
	0xba, 0xa4, 0xbb, 0x19, 0x7a, 0x0a, 0x58, 0x92, 0xab, 0x85, 0x5c, 0xeb,
	0x47, 0x39, 0xd3,
	0x0e, 0x13, 0x11, 0x30, 0x03, 0x1b, 0xd9, 0x68, 0x52, 0x97, 0x65, 0x73,
	0x9d, 0x89, 0xe4,
	0x5a, 0xfd, 0x4b, 0x7e, 0xa2, 0xd4, 0x79, 0x68, 0xa3, 0xce, 0x7a, 0x3d,
	0xc9, 0x29, 0xbc,
	0xa4, 0xa4, 0x35, 0xd5, 0x50, 0x71, 0x97, 0x8f, 0x4d, 0xa3, 0x95, 0xdf,
	0xde, 0x09, 0xae,
	0x40, 0x74, 0x34, 0xb9, 0x10, 0x74, 0x86, 0x21, 0x9c, 0xb7, 0x22, 0x6a,
	0xb9, 0xd5, 0xbf,
	0xf5, 0xed, 0x99, 0x3e, 0xe7, 0x41, 0x37, 0x7e, 0xb7, 0xff, 0xa8, 0x9e,
	0x21, 0x18, 0x31,
	0x17, 0xa7, 0x06, 0xc2, 0x25, 0xc3, 0x07, 0x31, 0x06, 0x65, 0xbd, 0xe1,
	0xf3, 0xa8, 0xb7,
	0xe0, 0x43, 0x7d, 0x79, 0xe2, 0xd7, 0xf1, 0x53, 0xf2, 0x1b, 0x64, 0x8c,
	0x1d, 0x4c, 0x75,
	0x49, 0x5e, 0x40, 0xf4, 0x4a, 0x73, 0x14, 0x93, 0x76, 0x8d, 0x18, 0xb6,
	0xb7, 0x4f, 0x97,
	0x9d, 0xed, 0x21, 0x22, 0x02, 0x1c, 0xb5, 0xef, 0x5c, 0xb8, 0xd2, 0xba,
	0x37, 0xec, 0x18,
	0x5d, 0x6b, 0x2b, 0x41, 0x31, 0xd2, 0x97, 0xd8, 0x8a, 0x55, 0xa9, 0x47,
	0x6d, 0x61, 0xb3,
	0x5b, 0x9e, 0x68, 0x44, 0x0c, 0x3d, 0xe8, 0x14, 0xe0, 0x39, 0x75, 0x37,
	0xcb, 0xae, 0x46,
	0x96, 0xbf, 0xe3, 0x50, 0x21, 0xaf, 0x9f, 0x84, 0x55, 0xe7, 0x91, 0x3d,
	0xae, 0x9e, 0xd8,
	0x7e, 0xe3, 0x45, 0x01, 0xb0, 0xb6, 0x96, 0xc8, 0xd0, 0xe3, 0x2c, 0x40,
	0x5c, 0x33, 0x2d,
	0x73, 0xec, 0x51, 0x1e, 0x4d, 0xda, 0x5f, 0xd4, 0x23, 0xe8, 0x63, 0x71,
	0x5e, 0xea, 0x22,
	0xc1, 0xc8, 0x75, 0x9c, 0x40, 0xff, 0xbd, 0x7b, 0x23, 0x45, 0x93, 0x75,
	0x82, 0x23, 0x53,
	0xc8, 0xd6
};

struct key_info key_info[] = {
	{
		.name = priv3_p_1024,
		.data = (u8 *)P_1024,
		.len = sizeof(P_1024),
	},

	{
		.name = priv3_q_1024,
		.data = (u8 *)Q_1024,
		.len = sizeof(Q_1024),
	},

	{
		.name = priv3_dp_1024,
		.data = (u8 *)DP_1024,
		.len = sizeof(DP_1024),
	},

	{
		.name = priv3_dq_1024,
		.data = (u8 *)DQ_1024,
		.len = sizeof(DQ_1024),
	},

	{
		.name = priv3_c_1024,
		.data = (u8 *)C_1024,
		.len = sizeof(C_1024),
	},

	{
		.name = priv3_tmp1_1024,
		.len = sizeof(P_1024),
	},

	{
		.name = priv3_tmp2_1024,
		.len = sizeof(Q_1024),
	},

	{
		.name = priv3_p_2048,
		.data = (u8 *)P_2048,
		.len = sizeof(P_2048),
	},

	{
		.name = priv3_q_2048,
		.data = (u8 *)Q_2048,
		.len = sizeof(Q_2048),
	},

	{
		.name = priv3_dp_2048,
		.data = (u8 *)DP_2048,
		.len = sizeof(DP_2048),
	},

	{
		.name = priv3_dq_2048,
		.data = (u8 *)DQ_2048,
		.len = sizeof(DQ_2048),
	},

	{
		.name = priv3_c_2048,
		.data = (u8 *)C_2048,
		.len = sizeof(C_2048),
	},

	{
		.name = priv3_tmp1_2048,
		.len = sizeof(P_2048),
	},

	{
		.name = priv3_tmp2_2048,
		.len = sizeof(Q_2048),
	},

	{
		.name = priv3_p_4096,
		.data = (u8 *)P_4096,
		.len = sizeof(P_4096),
	},

	{
		.name = priv3_q_4096,
		.data = (u8 *)Q_4096,
		.len = sizeof(Q_4096),
	},

	{
		.name = priv3_dp_4096,
		.data = (u8 *)DP_4096,
		.len = sizeof(DP_4096),
	},

	{
		.name = priv3_dq_4096,
		.data = (u8 *)DQ_4096,
		.len = sizeof(DQ_4096),
	},

	{
		.name = priv3_c_4096,
		.data = (u8 *)C_4096,
		.len = sizeof(C_4096),
	},

	{
		.name = priv3_tmp1_4096,
		.data = (u8 *)P_4096,
		.len = sizeof(P_4096),
	},

	{
		.name = priv3_tmp2_4096,
		.data = (u8 *)Q_4096,
		.len = sizeof(Q_4096),
	}
};

void blob_for_test(void)
{
	u32 len = 0, i;
	u8 *key_buf;

	for (i = 0; i < ARRAY_SIZE(key_info); i++)
		len += key_info[i].len;
	print_debug("private key length:%d\n", len);

	key_buf = get_buffer(len);
	if (!key_buf) {
		print_debug("no buffer\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(key_info); i++) {
		if (key_info[i].data)
			memcpy(key_buf, key_info[i].data, key_info[i].len);
		print_debug("key_buf :%p\n", key_buf);
		key_info[i].p_data = va_to_pa((va_addr_t)key_buf);
		key_buf += key_info[i].len;
	}
}

static void get_key(struct key_info *key)
{
	int name = key->name;

	key->len = key_info[name].len;
	key->p_data = key_info[name].p_data;
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

static inline phys_addr_t get_rsa_p_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_addr(priv3_p_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_addr(priv3_p_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_addr(priv3_p_4096);
	}

	return 0;
}
static inline phys_addr_t get_rsa_q_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_addr(priv3_q_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_addr(priv3_q_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_addr(priv3_q_4096);
	}

	return 0;
}

static inline phys_addr_t get_rsa_dp_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_addr(priv3_dp_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_addr(priv3_dp_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_addr(priv3_dp_4096);
	}

	return 0;
}

static inline phys_addr_t get_rsa_dq_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_addr(priv3_dq_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_addr(priv3_dq_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_addr(priv3_dq_4096);
	}

	return 0;
}

static inline phys_addr_t get_rsa_c_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_addr(priv3_c_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_addr(priv3_c_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_addr(priv3_c_4096);
	}

	return 0;
}

static inline void *get_rsa_tmp1_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_buffer(sizeof(P_1024));
	case SKMM_RSA_PRV_OP_2K:
		return get_buffer(sizeof(P_2048));
	case SKMM_RSA_PRV_OP_4K:
		return get_buffer(sizeof(P_4096));
	}

	return 0;
}

static inline void *get_rsa_tmp2_addr(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_buffer(sizeof(Q_1024));
	case SKMM_RSA_PRV_OP_2K:
		return get_buffer(sizeof(Q_2048));
	case SKMM_RSA_PRV_OP_4K:
		return get_buffer(sizeof(Q_4096));
	}

	return 0;
}

static inline int get_rsa_p_len(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_len(priv3_p_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_len(priv3_p_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_len(priv3_p_4096);
	}

	return 0;
}

static inline int get_rsa_q_len(int type)
{
	switch (type) {
	case SKMM_RSA_PRV_OP_1K:
		return get_key_len(priv3_q_1024);
	case SKMM_RSA_PRV_OP_2K:
		return get_key_len(priv3_q_2048);
	case SKMM_RSA_PRV_OP_4K:
		return get_key_len(priv3_q_4096);
	}

	return 0;
}

int constr_rsa_sec_desc(struct req_info *req_info)
{
	union desc_u *sec_desc = req_info->sec_desc;
	struct abs_req_s *abs_req = req_info->abs_req;
	u32 desc_size, start_idx, p_len, q_len;

	struct rsa_pub_desc_s *rsa_pub_desc;
	struct rsa_pub *abs_rsa_pub;

	struct rsa_priv_frm3_desc_s *rsa_priv_f3;
	struct rsa_priv3 *abs_rsa_priv3;
	int type = req_info->req_type;

	struct rsa_priv_alloc_block *pdata;

	switch (type) {
	case SKMM_RSA_PUB_OP:
		print_debug("SKMM_RSA_PUB_OP");
		rsa_pub_desc = sec_desc->rsa_pub_desc;
		abs_rsa_pub = &abs_req->req.rsa_pub;

		rsa_pub_desc->n_dma = abs_rsa_pub->n;

		rsa_pub_desc->e_dma = abs_rsa_pub->e;

		rsa_pub_desc->f_dma = abs_rsa_pub->f;

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
	case SKMM_RSA_PRV_OP_1K:
	case SKMM_RSA_PRV_OP_2K:
	case SKMM_RSA_PRV_OP_4K:
		rsa_priv_f3 = sec_desc->rsa_priv_f3;
		abs_rsa_priv3 = &abs_req->req.rsa_priv3;
		pdata = (struct rsa_priv_alloc_block *)req_info->private_data;

		rsa_priv_f3->p_dma = get_rsa_p_addr(type);
		rsa_priv_f3->q_dma = get_rsa_q_addr(type);
		rsa_priv_f3->dp_dma = get_rsa_dp_addr(type);
		rsa_priv_f3->dq_dma = get_rsa_dq_addr(type);
		rsa_priv_f3->c_dma = get_rsa_c_addr(type);

		pdata->tmp1 = get_rsa_tmp1_addr(type);
		if (pdata->tmp1 == NULL)
			return 0;

		pdata->tmp2 = get_rsa_tmp2_addr(type);
		if (pdata->tmp2 == NULL) {
			put_buffer(pdata->tmp1);
			return 0;
		}

		rsa_priv_f3->tmp1_dma = va_to_pa((va_addr_t)pdata->tmp1);
		rsa_priv_f3->tmp2_dma = va_to_pa((va_addr_t)pdata->tmp2);

		p_len = get_rsa_p_len(type);
		q_len = get_rsa_q_len(type);

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

	return 1;
}

struct keygen_buf {
	u8 *p;
	u8 *q;
	u8 *d;
	u8 *c;
	u8 *dp;
	u8 *dq;
};

void get_keygen_buf(struct keygen_buf *buf, int bits)
{
	int type = SKMM_RSA_PRV_OP_1K;

	print_debug("DEBUG: %s\n", __func__);
	switch (bits) {
	case 1024:
		type = SKMM_RSA_PRV_OP_1K;
		break;
	case 2048:
		type = SKMM_RSA_PRV_OP_2K;
		break;
	case 4096:
		type = SKMM_RSA_PRV_OP_4K;
		break;
	}

	buf->p = (void *)pa_to_va(get_rsa_p_addr(type));
	buf->q = (void *)pa_to_va(get_rsa_q_addr(type));
	buf->c = (void *)pa_to_va(get_rsa_c_addr(type));
	buf->dp = (void *)pa_to_va(get_rsa_dp_addr(type));
	buf->dq = (void *)pa_to_va(get_rsa_dq_addr(type));
}

int ret_pub_key(struct rsa_keygen *rsa_keygen, RSA *rsa)
{
	int fd;
	u8 *n;
	phys_addr_t tmp, offset;
	u32 len;

	print_debug("DEBUG: %s\n", __func__);
	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		error(0, -errno, "fail to open /dev/mem\n");
		return -ENODEV;
	}

	tmp = rsa_keygen->n & ~(PAGESIZE-1);
	offset = rsa_keygen->n - tmp;
	len = PAGESIZE;

	if (tmp + PAGESIZE - rsa_keygen->n < rsa_keygen->n_len)
		len = PAGESIZE * 2;

	n = mmap(NULL, len, PROT_READ | PROT_WRITE,
		    MAP_SHARED, fd, tmp);

	BN_bn2bin(rsa->n, n + offset);

	munmap(n, rsa_keygen->n_len);
	close(fd);

	return 1;
}

int rsa_keygen_sw(struct rsa_keygen *rsa_keygen)
{
	int num;
	unsigned long f4 = RSA_F4;
	struct keygen_buf *buf, buffer;

	BIGNUM *bn = BN_new();
	RSA *rsa = NULL;

	print_debug("DEBUG: %s\n", __func__);
	buf = &buffer;
	num = rsa_keygen->n_len * 8;

	if (!bn)
		goto err;
	rsa = RSA_new();
	if (!rsa)
		goto err;

	if (!BN_set_word(bn, f4) || !RSA_generate_key_ex(rsa, num, bn, NULL))
		goto err;

	get_keygen_buf(buf, rsa_keygen->n_len * 8);

	BN_bn2bin(rsa->p, buf->p);
	BN_bn2bin(rsa->q, buf->q);
	BN_bn2bin(rsa->dmp1, buf->dp);
	BN_bn2bin(rsa->dmq1, buf->dq);
	BN_bn2bin(rsa->iqmp, buf->c);
	ret_pub_key(rsa_keygen, rsa);

	return 1;
err:
	return 0;
}
