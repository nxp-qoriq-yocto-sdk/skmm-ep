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

#include <skmm_sec_blob.h>
#include <abstract_req.h>

/* Blob header and MAC length */
#define BLOB_KEY_HEADER		32
#define BLOB_MAC		16

/* RSA private key length for format 3 */
#define RSA_PRI3_LEN_512	(32 * 5)
#define RSA_PRI3_LEN_1024	(64 * 5)
#define RSA_PRI3_LEN_2048	(128 * 5)
#define RSA_PRI3_LEN_4096	(256 * 5)

/* The reserved fields of KEY file */
#define KEY_FILE_HEADER		16

/* SEC version */
#define SEC_SECVID_MS_IPID_MASK         0xffff0000
#define SEC_SECVID_MS_IPID_SHIFT        16
#define SEC_SECVID_MS_MAJ_REV_MASK      0x0000ff00
#define SEC_SECVID_MS_MAJ_REV_SHIFT     8
#define SEC_CCBVID_ERA_MASK             0xff000000
#define SEC_CCBVID_ERA_SHIFT            24

const char key_identify[] = {
	0XBC, 0xD0, 0X1E, 0XAC, 0XA9, 0X62, 0XBF, 0X5A,
	0XAF, 0XF3, 0X94, 0X32, 0X19, 0X7D, 0X34, 0X40
};

static void *constr_jobdesc_blob_encrypt(struct blob_param *pbp)
{
	struct blob_desc *desc = (struct blob_desc *)pbp->hw_desc;

	memset(desc, 0, sizeof(struct blob_desc));

	out_be32(&desc->head, 0x16 << 27 | 0x00800000 | 11);

	out_be32(&desc->key_cmd, 0x00 << 27 | 0x02 << 25 | pbp->key_idnfr_len);
	out_be64(&desc->key_addr, pbp->key_idnfr);

	out_be32(&desc->in_cmd, 0x1e << 27 | pbp->input_len);
	out_be64(&desc->in_addr, pbp->input);

	out_be32(&desc->out_cmd, 0x1f << 27 | pbp->output_len);
	out_be64(&desc->out_addr, pbp->output);

	out_be32(&desc->opt_cmd, 0x10 << 27 | 0x07 << 24 | 0x0d << 16);

	return desc;
}

static void *constr_jobdesc_blob_decrypt(struct blob_param *pbp)
{
	struct blob_desc *desc = (struct blob_desc *)pbp->hw_desc;

	memset(desc, 0, sizeof(struct blob_desc));

	out_be32(&desc->head, 0x16 << 27 | 0x00800000 | 11);

	out_be32(&desc->key_cmd, 0x00 << 27 | 0x02 << 25 | pbp->key_idnfr_len);
	out_be64(&desc->key_addr, pbp->key_idnfr);

	out_be32(&desc->in_cmd, 0x1e << 27 | pbp->input_len);
	out_be64(&desc->in_addr, pbp->input);

	out_be32(&desc->out_cmd, 0x1f << 27 | pbp->output_len);
	out_be64(&desc->out_addr, pbp->output);

	out_be32(&desc->opt_cmd, 0x10 << 27 | 0x06 << 24 | 0x0d << 16);

	return desc;
}

static void mount_key_dir(void)
{
	int ret;

	ret = system("mount -t jffs2 /dev/mtdblock4 .key");
	if (ret < 0) {
		printf("ret :%d\n", ret);
		exit(0);
	}
}

static void umount_key_dir(void)
{
	int ret;

	ret = system("umount .key");
	if (ret < 0) {
		printf("ret :%d\n", ret);
		exit(0) ;
	}
}

int decrypt_priv_key_from_blob(sec_engine_t *ccsr_sec, int type)
{
	sec_engine_t *sec = ccsr_sec;
	int i;
	void *input_buf;
	void *desc_buf;
	void *key_buf;
	void *output_buf = NULL;
	void *rng_init_desc = NULL;
	int fd;
	char head[KEY_FILE_HEADER];
	char *key_file;
	char rsa_key_file[] = RSA_KEY_FILE;
	char dsa_key_file[] = DSA_KEY_FILE;
	u32 len = 0;
	struct blob_param blob_para;
	u32 rm = 0;

	if (type == BLOB_RSA)
		key_file = rsa_key_file;
	if (type == BLOB_DSA)
		key_file = dsa_key_file;

	mount_key_dir();

	fd = open(key_file, O_RDWR);
	if (fd < 0) {
		print_debug("fail to open %s\n", key_file);
		goto out_blob;
	}
	read(fd, head, KEY_FILE_HEADER);

	if (type == BLOB_RSA)
		len = get_rsa_keys_size();

	blob_para.input_len = len + (BLOB_KEY_HEADER + BLOB_MAC);

	/* add another 128 byte for priv3 tmp1 and tmp2 */
	blob_para.output_len = blob_para.input_len - BLOB_KEY_HEADER - BLOB_MAC;
	blob_para.key_idnfr_len = sizeof(key_identify);

	output_buf = get_buffer(blob_para.output_len);
	input_buf = get_buffer(blob_para.input_len);
	desc_buf = get_buffer(sizeof(struct blob_desc));
	key_buf = get_buffer(sizeof(key_identify));

	blob_para.hw_desc = desc_buf;
	blob_para.key_idnfr = va_to_pa((va_addr_t)key_buf);
	blob_para.input = va_to_pa((va_addr_t)input_buf);
	blob_para.output = va_to_pa((va_addr_t)output_buf);

	memcpy(key_buf, key_identify, sizeof(key_identify));
	read(fd, input_buf, blob_para.input_len);
	close(fd);

	while (sec->jr.head * 8 != in_be32(&sec->jr.regs->irri))
		;

	constr_jobdesc_blob_decrypt(&blob_para);

	sec->jr.i_ring[sec->jr.tail].desc = va_to_pa((va_addr_t)desc_buf);
	sec->jr.tail = MOD_INC(sec->jr.tail, sec->jr.size);
	rm++;

	out_be32(&(sec->jr.regs->irja), rm);

	while (in_be32(&sec->jr.regs->orsf) != rm)
		;

	out_be32(&sec->jr.regs->orjr, rm);

	sec->jr.head = sec->jr.tail;

	print_debug("\n\noriginal data decrypted from blob\n");
	for (i = 0; i < blob_para.output_len; i++) {
		if (i % 8 == 0)
			print_debug("\n");
		print_debug("0x%02x    ", *((unsigned char *)(output_buf + i)));
	}
	print_debug("\n");

	put_buffer(rng_init_desc);
	put_buffer(key_buf);
	put_buffer(desc_buf);
	put_buffer(input_buf);

out_blob:
	if (type == BLOB_RSA)
		assign_rsa_key(output_buf);

	fsl_sec_init(sec);

	umount_key_dir();

	return 0;
}


int encrypt_priv_key_to_blob(sec_engine_t *ccsr_sec, const char *key_file,
				char *key, int len)
{
	int i, wlen;
	sec_engine_t *sec = ccsr_sec;
	void *input_buf;
	void *desc_buf;
	void *key_buf;
	void *output_buf;
	int fd, rm = 0;
	struct blob_param blob_para;
	u8 head[KEY_FILE_HEADER];
	u32 *key_len = (u32 *)head;

	blob_para.input_len = len;

	*key_len = blob_para.input_len;

	blob_para.output_len = BLOB_KEY_HEADER + blob_para.input_len + BLOB_MAC;
	blob_para.key_idnfr_len = sizeof(key_identify);

	input_buf = key;
	desc_buf = get_buffer(sizeof(struct blob_desc));
	key_buf = get_buffer(sizeof(key_identify));
	output_buf = get_buffer(blob_para.output_len);

	blob_para.hw_desc = desc_buf;
	blob_para.key_idnfr = va_to_pa((va_addr_t)key_buf);
	blob_para.input = va_to_pa((va_addr_t)input_buf);
	blob_para.output = va_to_pa((va_addr_t)output_buf);

	memcpy(key_buf, key_identify, sizeof(key_identify));

	print_debug("\nbefore blob encrypt, input is :\n");
	for (i = 0; i < blob_para.input_len; i++) {
		if (i % 8 == 0)
			print_debug("\n");
		print_debug("0x%02x    ", *((unsigned char *)(input_buf + i)));
	}
	print_debug("\n");

	constr_jobdesc_blob_encrypt(&blob_para);

	print_debug("out: %d, in: %d\n", in_be32(&sec->jr.regs->orsf),
			in_be32(&sec->jr.regs->irja));
	while (sec->jr.head * 8 != in_be32(&sec->jr.regs->irri))
		;
	sec->jr.i_ring[sec->jr.tail].desc = va_to_pa((va_addr_t)desc_buf);
	sec->jr.tail = MOD_INC(sec->jr.tail, sec->jr.size);
	rm++;

	out_be32(&(sec->jr.regs->irja), rm);

	while (in_be32(&sec->jr.regs->orsf) != rm)
		;

	out_be32(&sec->jr.regs->orjr, rm);

	print_debug("id: %d, sec->jr.head:%d sec->jr.tail %d\n",
					sec->id, sec->jr.head,
			sec->jr.tail);
	sec->jr.head = sec->jr.tail;

	mount_key_dir();

	fd = open(key_file, O_RDWR);
	if (fd < 0) {
		print_debug("fail to open %s\n", key_file);
		fd = open(key_file, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
		if (fd < 0) {
			printf("fail to creat %s\n", key_file);
			return -ENOENT;
		}
	}

	print_debug("write blob encrypted to %s\n", key_file);
	wlen = write(fd, head, KEY_FILE_HEADER);
	if (wlen != KEY_FILE_HEADER)
		print_debug("write header lengh: %d\n", wlen);

	wlen = write(fd, output_buf, blob_para.output_len);
	if (wlen != blob_para.output_len)
		printf("failed to write encrypted blob to %s %d\n",
				key_file, wlen);
	else
		print_debug("success to write encrypted blob to %s\n",
					key_file);

	close(fd);
	put_buffer(output_buf);
	put_buffer(key_buf);
	put_buffer(desc_buf);

	umount_key_dir();

	return 1;
}
