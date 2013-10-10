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

int parse_abs_to_sw_req(phys_addr_t req)
{
	struct abs_req_s *abs_req = (void *)pa_to_va(req);
	int type = abs_req->abs_req_id;
	int ret = 0;

	switch (type) {
	case SKMM_RSA_KEYGEN:
		ret = rsa_keygen_sw(&abs_req->req.rsa_keygen);
		break;
	}

	return ret;
}

phys_addr_t parse_abs_to_desc(phys_addr_t req)
{
	struct abs_req_s *abs_req = (void *)pa_to_va(req);
	union desc_u sec_desc;
	struct req_info req_info;

	struct rsa_pub_alloc_block *pub_buf;
	struct rsa_priv_alloc_block *priv_buf;

	struct dsa_sign_alloc_block *sign_buf;
	struct dsa_verify_alloc_block *verify_buf;

	struct ecdsa_verify_alloc_block *ec_verify_buf;
	struct ecdsa_sign_alloc_block *ec_sign_buf;

	struct dh_key_alloc_block *dh_key_buf;
	struct ecdh_key_alloc_block *ecdh_key_buf;

	struct keygen_alloc_block *keygen_buf;

	req_info.req_type = abs_req->abs_req_id;
	req_info.sec_desc = &sec_desc;
	req_info.abs_req = abs_req;

	print_debug(" abs request %p\n", abs_req);
	switch (abs_req->abs_req_id) {
	case SKMM_RSA_KEYGEN:
		return SKMM_SW_REQ;
	case SKMM_RSA_PUB_OP:
		print_debug("RSA_PUB op\n");
		pub_buf = get_buffer(sizeof(struct rsa_pub_alloc_block));
		print_debug("get buffer for public key %p\n", pub_buf);
		if (!pub_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		pub_buf->abs_req = abs_req;
		sec_desc.rsa_pub_desc = &pub_buf->rsa_pub_desc;

		__sync_synchronize();

		constr_rsa_sec_desc(&req_info);
		print_debug("RSA_PUB parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.rsa_pub_desc);

	case SKMM_RSA_PRV_OP_1K:
	case SKMM_RSA_PRV_OP_2K:
	case SKMM_RSA_PRV_OP_4K:
		print_debug("RSA PRIV 3 op\n");
		priv_buf = get_buffer(sizeof(struct rsa_priv_alloc_block));
		print_debug("get buffer for private key %p\n", priv_buf);
		if (!priv_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		priv_buf->abs_req = abs_req;
		sec_desc.rsa_priv_f3 = &priv_buf->rsa_priv;

		__sync_synchronize();
		req_info.private_data = priv_buf;

		if (constr_rsa_sec_desc(&req_info) == 0) {
			put_buffer(priv_buf);
			return 0;
		}
		print_debug("RSA PRIV 3 op complete\n");
		return va_to_pa((va_addr_t)sec_desc.rsa_priv_f3);
	case SKMM_DSA_KEYGEN:
		print_debug("DSA key generate OP\n");
		keygen_buf = get_buffer(sizeof(struct keygen_alloc_block));
		print_debug("get buffer for keygen %p\n", keygen_buf);
		if (!keygen_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		keygen_buf->abs_req = abs_req;
		sec_desc.keygen = &keygen_buf->keygen;
		req_info.private_data = keygen_buf;

		__sync_synchronize();

		if (constr_keygen_desc(&req_info) == 0) {
			put_buffer(keygen_buf);
			return 0;
		}

		print_debug("DSA KEYGEN parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.keygen);

	case SKMM_DSA_SIGN_512:
	case SKMM_DSA_SIGN_1K:
	case SKMM_DSA_SIGN_2K:
	case SKMM_DSA_SIGN_4K:
		print_debug("DSA signature OP\n");
		sign_buf = get_buffer(sizeof(struct dsa_sign_alloc_block));
		print_debug("get buffer for dsa signature %p\n", sign_buf);
		if (!sign_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		sign_buf->abs_req = abs_req;
		sec_desc.dsa_sign = &sign_buf->dsa_sign;
		req_info.private_data = sign_buf;

		__sync_synchronize();

		if (constr_dsa_sec_desc(&req_info) == 0) {
			put_buffer(sign_buf);
			return 0;
		}

		print_debug("DSA SIGNATURE parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.dsa_sign);
	case SKMM_DSA_VERIFY:
		print_debug("DSA OP\n");
		verify_buf = get_buffer(sizeof(struct dsa_verify_alloc_block));
		print_debug("get buffer for private key %p\n", verify_buf);
		if (!verify_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		verify_buf->abs_req = abs_req;
		sec_desc.dsa_verify = &verify_buf->dsa_verify;
		req_info.private_data = verify_buf;

		__sync_synchronize();

		if (constr_dsa_sec_desc(&req_info) == 0) {
			put_buffer(verify_buf);
			return 0;
		}
		print_debug("DSA_VERIFY parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.dsa_verify);
	case SKMM_ECDSA_VERIFY:
		print_debug("ECDSA OP\n");
		ec_verify_buf =
			get_buffer(sizeof(struct ecdsa_verify_alloc_block));
		print_debug("get buffer for private key %p\n", ec_verify_buf);
		if (!ec_verify_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		ec_verify_buf->abs_req = abs_req;
		sec_desc.ecdsa_verify = &ec_verify_buf->ecdsa_verify;
		req_info.private_data = ec_verify_buf;

		__sync_synchronize();

		if (constr_ecdsa_sec_desc(&req_info) == 0) {
			put_buffer(ec_verify_buf);
			return 0;
		}
		print_debug("ECDSA_VERIFY parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.ecdsa_verify);

	case SKMM_ECDSA_SIGN:
		print_debug("ECDSA signature OP\n");
		ec_sign_buf = get_buffer(sizeof(struct ecdsa_sign_alloc_block));
		print_debug("get buffer for ecdsa signature %p\n", ec_sign_buf);
		if (!ec_sign_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		ec_sign_buf->abs_req = abs_req;
		sec_desc.ecdsa_sign = &ec_sign_buf->ecdsa_sign;
		req_info.private_data = ec_sign_buf;

		__sync_synchronize();

		if (constr_ecdsa_sec_desc(&req_info) == 0) {
			put_buffer(ec_sign_buf);
			return 0;
		}

		print_debug("DSA SIGNATURE parse commplete\n");

		return va_to_pa((va_addr_t)sec_desc.ecdsa_sign);
	case SKMM_DH_1K:
	case SKMM_DH_2K:
	case SKMM_DH_4K:
		print_debug("DH OP\n");
		dh_key_buf = get_buffer(sizeof(struct dh_key_alloc_block));
		print_debug("get buffer for dh %p\n", dh_key_buf);
		if (!dh_key_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		dh_key_buf->abs_req = abs_req;
		sec_desc.dh_key = &dh_key_buf->dh_key;
		req_info.private_data = dh_key_buf;

		__sync_synchronize();

		if (constr_dh_sec_desc(&req_info) == 0) {
			put_buffer(dh_key_buf);
			return 0;
		}

		print_debug("DH parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.dh_key);

	case SKMM_ECDH:
		print_debug("ECDH OP\n");
		ecdh_key_buf = get_buffer(sizeof(struct ecdh_key_alloc_block));
		print_debug("get buffer for ecdh %p\n", ecdh_key_buf);
		if (!ecdh_key_buf) {
			print_debug("Malloc descriptor for sec error\n");
			return 0;
		}

		ecdh_key_buf->abs_req = abs_req;
		sec_desc.ecdh_key = &ecdh_key_buf->ecdh_key;
		req_info.private_data = ecdh_key_buf;

		__sync_synchronize();

		if (constr_ecdh_sec_desc(&req_info) == 0) {
			put_buffer(ecdh_key_buf);
			return 0;
		}

		print_debug("ECDH parse complete\n");

		return va_to_pa((va_addr_t)sec_desc.ecdh_key);

	default:
		perror("Invalid request type: ");
		printf("0x%x\n", abs_req->abs_req_id);
		return 0;
	}

	return 0;
}


inline phys_addr_t get_abs_req(phys_addr_t desc)
{
	struct abs_req_s **abs_req;

	abs_req = (struct abs_req_s **)(pa_to_va(desc) - sizeof(abs_req));

	return va_to_pa((va_addr_t)*abs_req);
}

void free_resource(phys_addr_t desc)
{
	struct rsa_priv_alloc_block *priv_buf;
	struct dsa_verify_alloc_block *dsa_verify_buf;
	struct dsa_sign_alloc_block *dsa_sign_buf;
	struct ecdsa_verify_alloc_block *ecdsa_verify_buf;
	struct ecdsa_sign_alloc_block *ecdsa_sign_buf;

	struct abs_req_s **abs_req;

	abs_req = (struct abs_req_s **)(pa_to_va(desc) - sizeof(abs_req));

	switch ((*abs_req)->abs_req_id) {
	case SKMM_RSA_PUB_OP:
		put_buffer(abs_req);
		break;

	case SKMM_RSA_PRV_OP_1K:
	case SKMM_RSA_PRV_OP_2K:
	case SKMM_RSA_PRV_OP_4K:
		priv_buf = (struct rsa_priv_alloc_block *)abs_req;
		put_buffer(priv_buf->tmp1);
		put_buffer(priv_buf->tmp2);
		put_buffer(priv_buf);
		break;
	case SKMM_DSA_KEYGEN:
		put_buffer(abs_req);
		break;
	case SKMM_DSA_VERIFY:
		dsa_verify_buf = (struct dsa_verify_alloc_block *)abs_req;
		put_buffer(dsa_verify_buf->tmp);
		put_buffer(dsa_verify_buf);
		break;
	case SKMM_DSA_SIGN_512:
	case SKMM_DSA_SIGN_1K:
	case SKMM_DSA_SIGN_2K:
	case SKMM_DSA_SIGN_4K:
		dsa_sign_buf = (struct dsa_sign_alloc_block *)abs_req;
		put_buffer(dsa_sign_buf->tmp);
		put_buffer(dsa_sign_buf);
		break;
	case SKMM_ECDSA_VERIFY:
		ecdsa_verify_buf = (struct ecdsa_verify_alloc_block *)abs_req;
		put_buffer(ecdsa_verify_buf->tmp);
		put_buffer(ecdsa_verify_buf);
		break;
	case SKMM_ECDSA_SIGN:
		ecdsa_sign_buf = (struct ecdsa_sign_alloc_block *)abs_req;
		put_buffer(ecdsa_sign_buf->tmp);
		put_buffer(ecdsa_sign_buf);
		break;
	case SKMM_DH_1K:
	case SKMM_DH_2K:
	case SKMM_DH_4K:
	case SKMM_ECDH:
		put_buffer(abs_req);
		break;
	}
}
