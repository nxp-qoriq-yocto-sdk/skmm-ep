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
#ifndef	__SKMM_H__
#define __SKMM_H__

#include <common.h>
#include <skmm_sec.h>
#include <skmm_pci.h>
#include <skmm_sram.h>

/*      FIRMWARE VERSION FORMAT
 *--------------------------------------
 *  31  27  23  19  15  11  7   3   0
 *  |   |   |   |   |   |   |   |   |
 *  |  DAY  | M |    YEAR   | MI| MR|
 *--------------------------------------
 */
#define FW_VERSION		0x0f300d11
#define TOTAL_CARD_MEMORY	(512 * 1024) /* 512 K */

/* P4080 specific macros */
#define FIRMWARE_SIZE		(0 * 1024)

/* In P4080 there is only one L3cache configured as 1MB  SRAM.
 * But since final c2x0 EP is going to have two different SRAMs.
 * For code to remain unchanged. Dividing the L3 cache in P4080
 * also in the same way logically.
 */
#define L2_SRAM_SIZE		(256 * 1024)
#define PLATFORM_SRAM_SIZE	(512 * 1024)
#define L2_SRAM_ADDR		0xfffd00000

/* Sec engine related macros */
#define SEC_JR_DEPTH		(128)
#define DEFAULT_POOL_SIZE	(384 * 1024 + 120 * 1024)
#define DEFAULT_EP_POOL_SIZE	(256 * 1024)

/* Setting the same input buffer pool for all the rings */
#define COMMON_IP_BUFFER_POOL

#define NON_ACCESS_MEM 0
#define SEC_ENQ_BUDGET  32
#define MAX_DEQ_BUDGET  12

#define RINGS_JOBS_ADDED(ring) \
	(ring->r_s_c_cntrs->jobs_added - ring->cntrs->jobs_processed)
#define MOD_ADD(x, value, size)	((x + value) & (size - 1))



#define WAIT_FOR_STATE_CHANGE(x)	{ while (DEFAULT == x) SYNC_MEM }
#define MIN(x, y)	(x * (x < y) + y * (x >= y))
#define LINEAR_ROOM(wi, depth, room)	MIN((depth - wi), room)
#define MOD_INC(x, size)	(((x) + 1) & (size - 1))

#define BITS_PER_BYTE	8

#define HIGH_PERF
#define unlikely(x)	__builtin_expect(!!(x), 0)

static inline void out_be32(u32 *p, u32 v)
{
	*(volatile u32 *)(p) = v;
	__sync_synchronize();
}

static inline void out_be64(volatile u64 *p, u64 v)
{
	out_be32((void *)p, (u32)(v>>32));
	out_be32(((void *)p + sizeof(u32)), (u32)v);
	__sync_synchronize();
}

static inline void out_le32(u32 *p, u32 v)
{
	*((volatile u32 *)p) = ((v >> 24) | ((v & 0xff0000) >> 8) |
			((v & 0xff00) << 8) | (v << 24));
	__sync_synchronize();
}

static inline void out_be16(u16 *p, u16 v)
{
	*(volatile u16 *)(p) = v;
	__sync_synchronize();
}

static inline void out_le16(volatile u16 *p, u16 v)
{
	*(volatile u16 *)(p) = (((v & 0xff00) >> 8) | ((v & 0xff) << 8));
	__sync_synchronize();
}

static inline void out_be8(volatile u8 *p, u8 v)
{
	*(volatile u8 *)(p) = v;
	__sync_synchronize();
}

static inline int in_be32(volatile u32 *p)
{
	u32 ret;

	ret =  *(volatile int *)(p);
	__sync_synchronize();
	return ret;
}

static inline int in_be8(volatile u8 *p)
{
	u8 ret;

	ret =  *(volatile u8 *)(p);
	__sync_synchronize();
	return ret;
}

#define ASSIGN8(x, y)  ((x) = (u8)(y))
#define ASSIGN16(x, y) ((x) = (u16)(y))
#define ASSIGN32(x, y) ((x) = (u32)(y))
#define ASSIGN64(x, y) ((x) = (u64)(y))

/* Device cache line related definitions */
#define L1_CACHE_LINE_SIZE_SHIFT	6
#define L1_CACHE_LINE_SIZE	(1 << L1_CACHE_LINE_SIZE_SHIFT)
#define __cache_line_aligned__	__attribute__((__aligned__(L1_CACHE_LINE_SIZE)))


#define ALIGN_TO_L1_CACHE_LINE(x)	(((x) + (L1_CACHE_LINE_SIZE - 1))& \
		~(L1_CACHE_LINE_SIZE-1))
#define ALIGN_TO_L1_CACHE_LINE_REV(x)   (x & ~(L1_CACHE_LINE_SIZE - 1))

/* Application ring properties */
#define APP_RING_PROP_ORDER_MASK	0x01
#define APP_RING_PROP_ORDER_SHIFT	0

#define APP_RING_PROP_AFFINE_MASK	0x0e
#define APP_RING_PROP_AFFINE_SHIFT	1

#define APP_RING_PROP_PRIO_MASK		0xf0
#define APP_RING_PROP_PRIO_SHIFT	4

/* General Macros */
#define true		1
#define RESULT_OK	1
#define INTR_FLAG_SET	1

#define SYNC_MEM	__sync_synchronize();

/*
 *Description : Defines the application ring request entry
 *Fields      : sec_desc  : DMA address of the sec desc
 */
struct req_ring {
	dma_addr_t desc;
} __packed;
typedef struct req_ring req_ring_t;

#define MULTIPLE_RESP_RINGS

#ifdef MULTIPLE_RESP_RINGS
struct dev_ctx {
	volatile u8 r_id;
	volatile u32 wi;
} __packed;

typedef struct dev_ctx dev_ctx_t;
#endif


/*
 * Description : Defines the application ring response entry
 * Fields      : sec_desc  : DMA address of the sec desc
 *               result    : Result of crypto op from sec engine
 */
struct resp_ring {
	dma_addr_t desc;
	u32 result;
} __packed;

typedef struct resp_ring resp_ring_t;

/*
 *Description : Defines the container structure for ring indexes.
 *Fields      : w_index   : Write index of the ring
 *r_index   : Read index of the ring
 */
typedef struct indexes_mem {
	va_addr_t	w_index;
	va_addr_t	r_index;
} indexes_mem_t;

typedef indexes_mem_t shadow_indexes_mem_t;

/*
 * Description : Contains the counters per job ring. There will two
 * copies one forlocal usage and one shadowed for firmware
 * Fields      : Local memory
 * jobs_added          : Count of number of resp jobs added
 * jobs_processed      : Count of number of req jobs processed
 * Shadow copy memory
 * jobs_added          : Count of number of req jobs added by driver
 * jobs_processed      : Count of number of resp jobs processed by driver
 */
typedef struct  ring_counters_mem {
	u32    jobs_added;
	u32    jobs_processed;
} ring_counters_mem_t;

/*
 *Description : Defines the container structure for total counters.
 *Fields      : req_tot_jobs_added :Total no of req jobs added across ring pairs
 *req_tot_jobs_processed: Total no of req jobs processed across ring pairs
 *ring_counters         : Pointer to the array of ring pair counters.
 */
typedef struct counters_mem {
	u32    tot_jobs_added;
	u32    tot_jobs_processed;
} counters_mem_t;
typedef counters_mem_t shadow_counters_mem_t;

/*
 *Description : Defines the container structure for counters of a ring pair.
 *This memory is alloc on host and is shared with firmware.
 *Fields      : resp_jobs_added : Number of jobs added in the resp ring by fw
 *resp_jobs_processed   : Number of jobs processed from resp ring by drv
 */
typedef struct ring_shadow_counters_mem {
	u32	resp_jobs_added;
	u32	req_jobs_processed;
} ring_shadow_counters_mem_t;

/*
 *Description : Defines the container structure for application ring pair
 *Fields      : req_r     : Request ring
 *resp_r    : Response ring
 *ip_pool   : Input pool
 *msi_addr  : Device domain valid msi address. FW uses to gen intr.
 *id        : Id of the ring (1,2,3...)
 *depth     : Depth of the ring
 *response_count    : Count of number of responses added in this ring
 *msi_data  : Data to be written at MSI address
 *intr_ctrl_flag    : Used to control whether intr has to be gen for
 *this ring or not. Driver sets and resets this
 *field to indicate the same.
 *p_req_addr        : Physical address of the req ring
 *p_resp_addr       : Physical address of the resp ring
 *p_ip_pool_addr    : Physical address of the input pool
 *p_msi_addr        : Physical address of the MSI address
 *sec               : Pointer to the affined sec engine
 *next              : Points to the next ring pair in the same prio level.
 */
typedef struct app_ring_pair {
	req_ring_t	*req_r;
	resp_ring_t	*resp_r;
	u8		*resp_j_done_flag;
	void		*ip_pool;
	void		*msi_addr;
	sec_engine_t		*sec;
	indexes_mem_t		*idxs;
	ring_counters_mem_t	*cntrs;
	ring_counters_mem_t	*r_s_c_cntrs;
	ring_shadow_counters_mem_t	*r_s_cntrs;

	u8	id;
	u8	prio;
	u8	c_link;
	u8	max_next_link;
	u8	props;
	u16	msi_data;
	u32	depth;
	u32	intr_ctrl_flag;
	u32	order_j_d_index;

	struct app_ring_pair *next;
#define FSL_CRYPTO_MAX_RING_PAIRS   6
	struct app_ring_pair *rp_links[FSL_CRYPTO_MAX_RING_PAIRS];
} app_ring_pair_t;

/*
 *Description : Defines the priority queue of a particular prio level
 *Fields      : ring      : Pointer to the list of rings in this prio level
 *next      : Pointer to the next prio queue
 */
typedef struct priority_q {
	app_ring_pair_t     *ring;
	struct priority_q   *next;
} priority_q_t;

typedef struct driver_resp_ring {
	u8      id;
	u16	msi_data;
	u16	depth;
	u32	intr_ctrl_flag;
	u32     enqcount;

	resp_ring_t	*resp_r;
	void		*msi_addr;
	indexes_mem_t	*idxs;

	ring_counters_mem_t		*r_cntrs;
	ring_counters_mem_t		*r_s_c_cntrs;
	ring_shadow_counters_mem_t	*r_s_cntrs;
	void *next;
} drv_resp_ring_t;

/*
 *Description : Defines the container structure for resources in the device
 *Fields      : sec_eng_cnt   : Number of sec engines
 *sec           : Array of sec engine rsrc structures
 *ring_count    : Number of rings created in the device (App+cmd)
 *p_q           : Linked list of priority queues created
 */
typedef struct resource {
	u8 sec_eng_cnt;
	u8 ring_count;
	u8 drv_resp_ring_count;

	void	*req_mem;
	void	*ip_pool;
	void	*ep_pool;

	sec_engine_t	*sec;
	priority_q_t	*p_q;

	app_ring_pair_t	*rps;
	app_ring_pair_t *orig_rps;
	app_ring_pair_t	*cmdrp;
	drv_resp_ring_t	*drv_resp_ring;

	u32 *intr_ctrl_flags;

	indexes_mem_t *idxs_mem;

	ring_counters_mem_t *r_cntrs_mem;
	ring_counters_mem_t *r_s_c_cntrs_mem;

	counters_mem_t *cntrs_mem;
	counters_mem_t *s_c_cntrs_mem;

	ring_shadow_counters_mem_t *r_s_cntrs_mem;
	shadow_counters_mem_t	   *s_cntrs_mem;

} resource_t;

/* Handshake related data structures */
typedef struct crypto_c_hs_mem {
	u32 h_ob_mem_l;
	u32 h_ob_mem_h;

	u32 h_msi_mem_l;
	u32 h_msi_mem_h;

	u8  state;
	u8  command;
	u8  data_len;
	u8  pad;

	union cmd_data {
		struct c_config_data {
			u8     num_of_rps;
			u8     max_pri;
			u8     num_of_fwresp_rings;
			u16    req_mem_size;
			u32    drv_resp_ring;
			u32    fw_resp_ring;
			u32    s_cntrs;
			u32    r_s_cntrs;
			u32    fw_resp_ring_depth;
		} config;
		struct c_ring_data {
			u8     rid;
			u8     props;
			u16    msi_data;
			u32    depth;
			u32    resp_ring;
			u32    msi_addr_l;
			u32    msi_addr_h;
			u32    s_r_cntrs;
		} ring;
	} data;
} crypto_c_hs_mem_t;


/*
 *Description : Defines the handshake memory layout on the host
 *Fields      :
 */
typedef struct fsl_h_mem_handshake {
	u8  state;
	u8  result;
	u32 dev_avail_mem;

	union resp_data {
		struct fw_up_data {
			u32    p_ib_mem_base_l;
			u32    p_ib_mem_base_h;
			u32    p_ob_mem_base_l;
			u32    p_ob_mem_base_h;
			u32     no_secs;
		} device;
		struct config_data {
			u32	s_r_cntrs;
			u32	s_cntrs;
			u32	ip_pool;
			u32	resp_intr_ctrl_flag;
		} config;
		struct ring_data {
			u32	req_r;
			u32	intr_ctrl_flag;
		} ring;
	} data;
} fsl_h_mem_handshake_t;

typedef fsl_h_mem_handshake_t   h_hs_mem_t;
typedef crypto_c_hs_mem_t       c_hs_mem_t;

#define JR_SIZE_SHIFT	0
#define JR_SIZE_MASK	0xffff0000
#define JR_NO_SHIFT	16
#define JR_NO_MASK	0xff00ffff
#define SEC_NO_SHIFT	24
#define SEC_NO_MASK	0x00ffffff


/* Identifies different states of the device */
typedef enum handshake_state {
	DEFAULT = 255,
	FIRMWARE_UP = 10,
	FW_INIT_CONFIG_COMPLETE,
	FW_GET_SEC_INFO_COMPLETE,
	FW_INIT_RING_PAIR_COMPLETE,
	FW_INIT_MSI_INFO_COMPLETE,
	FW_INIT_IDX_MEM_COMPLETE,
	FW_INIT_COUNTERS_MEM_COMPLETE,
	FW_INIT_RNG,
	FW_RNG_COMPLETE

} handshake_state_t;

/* Identifies different commands to be sent to the firmware */
typedef enum h_handshake_commands {
	HS_GET_SEC_INFO,
	HS_INIT_CONFIG,
	HS_INIT_RING_PAIR,
	HS_INIT_MSI_INFO,
	HS_INIT_IDX_MEM,
	HS_INIT_COUNTERS_MEM,
	HS_COMPLETE,
	WAIT_FOR_RNG,
	RNG_DONE
} h_handshake_commands_t;

/* Identifies different commands to be sent to the firmware */
typedef enum fw_handshake_commands {
	FW_GET_SEC_INFO,
	FW_INIT_CONFIG,
	FW_INIT_RING_PAIR,
	FW_INIT_MSI_INFO,
	FW_INIT_IDX_MEM,
	FW_INIT_COUNTERS_MEM,
	FW_HS_COMPLETE,
	FW_WAIT_FOR_RNG,
	FW_RNG_DONE

} fw_handshake_commands_t;

/*
 * Description : Defines the application ring job context.
 * This is helpful in data processing to have a mapping
 * between sec desc and the application
 * ring which gave that request.
 * Fields : ring_offset : Offset of the ring pair in which sec desc was
 * posted.
 */
typedef struct job_ctx{
	phys_addr_t  ring_offset;
} job_ctx_t;


/*
 * Description : Defines the memory latout on the device.
 * Since the memory on the device is plain, it is accessed
 * by typecasting it to this structure
 * so that accessing is easy.
 * Fields      :
 */
typedef struct c_mem_layout {

	va_addr_t   v_ib_mem;
	va_addr_t   v_ob_mem;
	va_addr_t   v_msi_mem;

	phys_addr_t p_ib_mem;
	phys_addr_t p_ob_mem;
	phys_addr_t p_msi_mem;

	phys_addr_t p_pci_mem;

	/* Addresses of common pool for all the ring pairs */
	phys_addr_t p_buf_pool_mem;
	va_addr_t   v_buf_pool_mem;

	h_hs_mem_t  *h_hs_mem;

	c_hs_mem_t  *c_hs_mem;

	resource_t  *rsrc_mem;

	u32	free_mem;
	u64	intr_ticks;
	u64	intr_timeout_ticks;

	int dgb_print;
	int err_print;

} c_mem_layout_t;

/* COMMAND RING STRUCTURE */

typedef enum commands {
	DEBUG,
	DEVSTAT,
	REHANDSHAKE,
	PINGDEV,
	RESETDEV,
	RESETSEC,
	RINGSTAT,
	SECSTAT,
	BLOCK_APP_JOBS,
	UNBLOCK_APP_JOBS,
	RNG_INSTANTIATE
} commands_t;

typedef enum debug_commands {
	MD,
	MW,
	PRINT1_DEBUG,
	PRINT1_ERROR,
} dgb_cmd_type_t;


#define MAX_SEC_NO 3
/* SEC STAT */
typedef struct fsl_sec_stat {
	u32 sec_ver;
	u32 cha_ver;
	u32 no_of_sec_engines;
	u32 no_of_sec_jr;
	u32 jr_size;
	struct sec_ctrs_t {
		u32 sec_tot_req_jobs;
		u32 sec_tot_resp_jobs;
	} sec[MAX_SEC_NO];
} fsl_sec_stat_t;

/* DEVICES STAT */
typedef struct fsl_dev_stat_op {
	u32 fwversion;
	u32 totalmem;
	u32 codemem;
	u32 heapmem;
	u32 freemem;
	u32 num_of_sec_engine;     /* ALREADY IN crypto_dev_config_t */
	u32 no_of_app_rings;       /* ALREADY IN crypto_dev_config_t */
	u32 total_jobs_rx;         /* ALREADY IN struct crypto_h_mem_layout */
	u32 total_jobs_pending;    /* ALREADY IN struct crypto_h_mem_layout */
} fsl_dev_stat_op_t;

/* RESOURCE STAT */
struct fsl_ring_stat_op {
	u32 depth;             /* ALREADY IN struct ring_info  */
	u32 tot_size;          /* REALLY NEED THIS ???         */
	u32 priority;          /* PRIORITY OF RING  */
	u32 affinity;          /* AFFINITY OF RING  */
	u32 order;             /* ORDER OF RING    */
	u32 free_count;        /* DEPTH - CURRENT JOBS */
	u32 jobs_processed;    /* ALREADY IN struct  ring_counters_mem */
	u32 jobs_pending;      /* ALREADY IN struct  ring_counters_mem */
	u32 budget;
}__packed;
typedef struct fsl_ring_stat_op fsl_ring_stat_op_t;


/* DEBUG */
typedef struct debug_op {
	u32 total_ticks;
	u32 pcie_data_consume_ticks; /* WRITE OUT HOST TO CARD + CARD TO HOST*/
	u32 job_wait_ticks;          /* WAIT TIME IN JOB QUEUE */
	u32 job_process_ticks;       /* PROCESS TIME */
	u32 sec_job_ticks;           /* TICKS FOR SEC TO COMPLETE JOB */
} debug_op_t;

/* PING */
typedef struct ping_op {
	u32 resp;
} ping_op_t;

typedef struct debug_ip {
	dgb_cmd_type_t cmd_id;
	unsigned int address;
	unsigned int val;
} debug_ip_t;


/*
 *Description : output from the device in repsonse
 *Fields      : ping_op
 *debug_op
 *ring_stat_op
 *dev_stat_op
 */
union op_buffer {
	ping_op_t           ping_op;
	debug_op_t          debug_op;
	fsl_ring_stat_op_t  ring_stat_op;
	fsl_dev_stat_op_t   dev_stat_op;
	fsl_sec_stat_t      sec_op;
}__packed;
typedef union op_buffer op_buffer_t;

/*
 * Description :   command context
 * Fields      :   cmd_type
 * : type of command
 * cmd_completion    : command completion variable
 */
typedef struct cmd_trace_ctx {
	commands_t cmd_type;
	u32 result;
	u32 cmd_completion;
} cmd_trace_ctx_t;


/*
 *Description : output from the device in repsonse of command
 *Fields      : buffer     : output buffer
 */
struct cmd_op {
	cmd_trace_ctx_t *cmd_ctx;
	op_buffer_t buffer;
} __packed;
typedef struct cmd_op cmd_op_t;

/*
 * escription : prepares the command descriptor for command ring
 * Fields     : cmd_type      : type of command
 * ip_info    : input to the command
 * cmd_op     : output buffer
 */
/* COMMAND RING DESC */
struct cmd_ring_req_desc {
	commands_t cmd_type;

	union __ip_info {
		u32    ring_id;    /* RING ID */
		u32    sec_id;     /* SEC ENGINE ID */
		u32    count;      /* COUNT VAR TO CKECK LIVELENESS */
		debug_ip_t dgb;
	} ip_info;

	dma_addr_t cmd_op;
}__packed;
typedef struct cmd_ring_req_desc cmd_ring_req_desc_t;

#endif
