// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mhi.h>
#include <linux/mhi_misc.h>
#include <ipc/gpr-lite.h>
#include <linux/random.h>
#include <dsp/msm_audio_ion.h>
#include <asoc/msm-pcm-pcie.h>

#define DRV_NAME "msm-pcm-pcie-char"

/* header offset declarations */
#define DL_SLOT_INDEX_OFFSET 36
#define UL_SLOT_INDEX_OFFSET 32
#define DL_BUF_OFFSET 28
#define UL_BUF_OFFSET 24
#define UL_UL_TS_OFFSET 8
#define UL_DL_TS_OFFSET 16
#define NUM_UL_SLOTS 3
#define NUM_DL_SLOTS 3
#define UL_SLOT_SIZE 1944
#define DL_SLOT_SIZE 1944
#define NUM_TIME_SYNC_OP 3

/* Payload offset declarations */
#define UL_TS_OFFSET 16
#define DL_TS_OFFSET 8
#define DL_PAYLOAD_OFFSET 24
#define UL_PAYLOAD_OFFSET 24

#define DL_PAYLOAD_SIZE 1920
#define UL_PAYLOAD_SIZE 1920
#define MAX_TIMEOUT (10 * 1000)

/* timesync time calculations */
#define TIMETICKS_TO_US(x) (div_u64((x) * 100ULL, div_u64(arch_timer_get_cntfrq(), 10000ULL)))
#define US_TO_TIMETICKS(kt_ts) div_u64(((kt_ts) * div_u64(arch_timer_get_cntfrq(), 10000ULL)), 100ULL)
#define UL_BUFF_TIME_OFFSET 384000U

#define NUM_OF_DRIFTS (NUM_TIME_SYNC_OP * (NUM_TIME_SYNC_OP - 1))/2
#define MOD(x) (x < 0) ? (-x) : x
#define ONE_SAMPLE_TIME(rate) 1/(rate) * 1000 * 1000 * 1000

#define DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2          0x0400100B
#define DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2          0x0400100A
#define DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE    0x05001000
#define DATA_CMD_RSP_RD_SH_MEM_EP_DATA_BUFFER_DONE    0x05001002

#define MINOR_NUMBER_COUNT 1

#define TIMER_RESTART 0
#define TIMER_NORESTART 1
#define ERROR_TOLERANCE 20
#define WRITE_BUF_SIZE 2048
#define NUM_WRITE_BUFS 2
#define DMA_BUFF_SIZE (sizeof(struct msm_pcm_pcie_header) + \
			(NUM_UL_SLOTS * UL_SLOT_SIZE) + \
			(NUM_DL_SLOTS * DL_SLOT_SIZE))

#define CHANNEL_NAME "adsp_apps"

enum pcie_drv_state {
	PCIE_DRV_INIT,
	PCIE_DRV_PROBED,
	PCIE_DRV_REMOVED,
	PCIE_DRV_DEINIT,
};

struct pcie_pkt_drv {
	char dev_name[20];
	char ch_name[20];
	dev_t audio_pkt_major;
	struct class *audio_pkt_class;
	struct device *dev;
	struct cdev cdev;
};

struct uplink_downlink_timekeeping {
	uint32_t timeout;
	uint64_t ul_ts_saved;
	uint64_t dl_ts_saved;
	uint64_t prev_ul_ts_saved;
	uint64_t prev_device_ul_ts;
	uint64_t prev_device_dl_ts;
	uint64_t rec_kt_diff;
	uint64_t play_kt_diff;
	ktime_t ul_ts_update_marker;
	ktime_t dl_us_schedule_marker;
};

struct uplink_downlink_timers {
	struct uplink_downlink_timekeeping ul_dl_ts;
	struct hrtimer capture_hrt;
	struct hrtimer playback_hrt;
	struct hrtimer capture_done_hrt;
};

static struct uplink_downlink_timers ul_dl_hrt;

struct data_cmd_wr_sh_mem_ep_data_buffer_v2_t {
	uint32_t data_buf_addr_lsw;
	uint32_t data_buf_addr_msw;
	uint32_t data_mem_map_handle;
	uint32_t data_buf_size;
	uint32_t timestamp_lsw;
	uint32_t timestamp_msw;
	uint32_t flags;
	uint32_t md_buf_addr_lsw;
	uint32_t md_buf_addr_msw;
	uint32_t md_mem_map_handle;
	uint32_t md_buf_size;

};

struct data_cmd_rd_sh_mem_ep_data_buffer_v2_t {
	uint32_t buf_addr_lsw;
	uint32_t buf_addr_msw;
	uint32_t mem_map_handle;
	uint32_t buf_size;
	uint32_t md_buf_addr_lsw;
	uint32_t md_buf_addr_msw;
	uint32_t md_mem_map_handle;
	uint32_t md_buf_size;
};

struct apm_cmd_rsp_shared_mem_map_regions_t {
	uint32_t mem_map_handle;
};

struct shmem_buff_info {
	bool is_first_dl_buffer;
	bool err_handled;
	int write_buf_offset;
	int read_buf_offset;
	int first_dl_pkt_rcvd;
	uint32_t write_memhandle;
	uint32_t read_memhandle;
	uint32_t error_counter;
	uint64_t expected_slot_counter;
	void *read_shared_mem_paddr;
	void *write_shared_mem_paddr;
	struct gpr_device *gprdev_local;
};

static struct shmem_buff_info shmem_map_info = { 0 };

struct mem_info_msg {
	uint64_t addr;
	uint64_t size;
	uint32_t seq_num;
} __packed;

struct pcie_dev_resp_msg_t {
	uint32_t ack;
	uint32_t seq_num;
} __packed;

struct mhi_info {
	struct mhi_device *mhidev;
	struct mem_info_msg *mhidev_msg;
	struct pcie_dev_resp_msg_t *mhidev_resp_msg;
};

static struct mhi_info mhidev_info = { 0 };

/*little endian*/
struct msm_pcm_pcie_header {
	uint64_t signature;
	uint32_t sample_freq;
	uint16_t sample_width;
	uint16_t burst_interval;
	uint32_t num_ul_slots;
	uint32_t num_dl_slots;
	uint32_t ul_ring_buf_offset;
	uint32_t dl_ring_buf_offset;
	uint32_t ul_slot_index;
	uint32_t dl_slot_index;
} __packed;

static struct msm_pcm_pcie_header *pcm_pcie_hdr = NULL;

struct time_sync_info {
	uint64_t host_time;
	uint64_t device_time;
	int64_t offset_host_dev;
	bool filled;
};

static struct time_sync_info time_offset[NUM_TIME_SYNC_OP];
static int64_t inst_drift[NUM_OF_DRIFTS];
static int64_t acc_drift, acc_drift_max;
static int last_updated_seq_num = 0;

static struct delayed_work poll_index_work;
static struct delayed_work sync_operation;
static struct delayed_work err_handler_work;
static struct mutex sync_lock;

static struct snd_dma_buffer *voice_pcie_data_buf = NULL;

/* function forward declaration */
static int msm_pcm_pcie_close(struct inode *inode, struct file *file);

/* functions definations */
static int get_header_size(void)
{
	return sizeof(struct msm_pcm_pcie_header);
}

static void init_header(void *mem)
{
	pcm_pcie_hdr = (struct msm_pcm_pcie_header *)mem;
	pcm_pcie_hdr->signature = 0x504349415544494F;
	pcm_pcie_hdr->burst_interval = 960;
	pcm_pcie_hdr->sample_width = 2;
	pcm_pcie_hdr->sample_freq = 48000;
	pcm_pcie_hdr->num_dl_slots = 3;
	pcm_pcie_hdr->num_ul_slots = 3;
	pcm_pcie_hdr->ul_ring_buf_offset = get_header_size();
	pcm_pcie_hdr->dl_ring_buf_offset =
	    get_header_size() + (NUM_UL_SLOTS * UL_SLOT_SIZE);
	pcm_pcie_hdr->ul_slot_index = 0;
	pcm_pcie_hdr->dl_slot_index = 0;
	wmb();
	return;
}

static void time_sync_cb(struct mhi_device *mhi_dev, uint32_t seq_num,
			 uint64_t host_time, uint64_t device_time)
{
	time_offset[seq_num].host_time = host_time;
	time_offset[seq_num].device_time = device_time;
	time_offset[seq_num].filled = true;
	last_updated_seq_num = seq_num;
}

static void mhi_calculate_drift(void)
{
	int i, j, index = 0;
	uint64_t mod_max_drift = 0;
	int64_t max_drift = 0;

	for (i = 0; i < NUM_TIME_SYNC_OP; i++)
		if (time_offset[i].filled == false)
			return;

	for (i = 0; i < NUM_TIME_SYNC_OP; i++)
		time_offset[i].offset_host_dev = time_offset[i].host_time -
		    time_offset[i].device_time;

	for (i = 0; ((i < NUM_TIME_SYNC_OP - 1) && (index < NUM_OF_DRIFTS));
	     i++)
		for (j = i + 1; j < NUM_TIME_SYNC_OP; j++)
			inst_drift[index++] = time_offset[i].offset_host_dev -
			    time_offset[j].offset_host_dev;

	for (i = 0; i < index; i++) {
		if (mod_max_drift < MOD(inst_drift[i])) {
			mod_max_drift = MOD(inst_drift[i]);
			max_drift = inst_drift[i];
		}
	}

	for (i = 0; i < NUM_TIME_SYNC_OP; i++)
		time_offset[i].filled = false;

	acc_drift += max_drift;

	if (MOD(acc_drift) > ONE_SAMPLE_TIME(48000) && acc_drift < 0) {
		acc_drift_max = acc_drift;
		acc_drift = 0;
	} else if (MOD(acc_drift) > ONE_SAMPLE_TIME(48000) && acc_drift > 0) {
		acc_drift_max = acc_drift;
		acc_drift = 0;
	} else {
		acc_drift_max = 0;
	}
	return;
}

static void time_sync_operation(struct mhi_device *dev)
{
	static int seq_no = 0;
	mhi_get_remote_time(dev, seq_no, &time_sync_cb);
	seq_no++;
	if (seq_no == NUM_TIME_SYNC_OP)
		seq_no = 0;
	return;
}

static void time_sync_op(struct work_struct *work)
{
	time_sync_operation(mhidev_info.mhidev);
	schedule_delayed_work(&sync_operation, msecs_to_jiffies(333));
}

static void msm_pcm_pcie_err_handler(struct work_struct *work)
{
	if (false == shmem_map_info.err_handled) {
		shmem_map_info.err_handled = true;
		pr_err("%s: err handler called\n", __func__);
		msm_pcm_pcie_close(NULL, NULL);
	}
}

static int data_cmd_write(void)
{
	struct gpr_pkt *pkt = NULL;
	struct data_cmd_wr_sh_mem_ep_data_buffer_v2_t write_payload;
	int ret = 0;
	uint32_t size;

	if (!shmem_map_info.gprdev_local) {
		ret = -EAGAIN;
		goto handle_err;
	}

	trace_printk("%s: called at %llu\n", __func__, ktime_get());
	trace_printk(" inside data_cmd_write offset %d paddr %pK",
		     shmem_map_info.write_buf_offset,
		     (void *)shmem_map_info.write_shared_mem_paddr);

	size =
	    GPR_HDR_SIZE +
	    sizeof(struct data_cmd_wr_sh_mem_ep_data_buffer_v2_t);

	pkt = kzalloc(size, GFP_ATOMIC);
	if (!pkt)
		return -ENOMEM;

	pkt->hdr.header = GPR_SET_FIELD(GPR_PKT_VERSION, GPR_PKT_VER) |
	    GPR_SET_FIELD(GPR_PKT_HEADER_SIZE, GPR_PKT_HEADER_WORD_SIZE_V) |
	    GPR_SET_FIELD(GPR_PKT_PACKET_SIZE, size);

	pkt->hdr.src_port = 0x2010;
	pkt->hdr.dst_port = 16573;
	pkt->hdr.dst_domain_id = GPR_IDS_DOMAIN_ID_ADSP_V;
	pkt->hdr.src_domain_id = GPR_IDS_DOMAIN_ID_APPS_V;
	pkt->hdr.token = shmem_map_info.write_buf_offset / WRITE_BUF_SIZE;	/* TBD */
	pkt->hdr.opcode = DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2;

	write_payload.data_buf_addr_lsw = shmem_map_info.write_buf_offset;
	shmem_map_info.write_buf_offset += WRITE_BUF_SIZE;
	if (shmem_map_info.write_buf_offset >= NUM_WRITE_BUFS * WRITE_BUF_SIZE)
		shmem_map_info.write_buf_offset = 0;
	trace_printk(" inside data_cmd_write offset %d paddr %pK",
		     shmem_map_info.write_buf_offset,
		     (void *)shmem_map_info.write_shared_mem_paddr);
	write_payload.data_buf_addr_msw = 0;
	write_payload.data_buf_size = 1920;
	write_payload.data_mem_map_handle = shmem_map_info.write_memhandle;
	trace_printk
	    (" inside data_cmd_write offset %d paddr %pK memhandle %d token %d",
	     shmem_map_info.write_buf_offset,
	     (void *)shmem_map_info.write_shared_mem_paddr,
	     shmem_map_info.write_memhandle, pkt->hdr.token);
	write_payload.flags = 0;

	memcpy(&pkt->payload, &write_payload,
	       sizeof(struct data_cmd_wr_sh_mem_ep_data_buffer_v2_t));
	trace_printk(" exit data_cmd_write offset %d paddr %pK",
		     shmem_map_info.write_buf_offset,
		     (void *)shmem_map_info.write_shared_mem_paddr);

	ret = gpr_send_pkt(shmem_map_info.gprdev_local, pkt);

handle_err:
	if (ret < 0) {
		shmem_map_info.error_counter++;
		if (printk_ratelimit())
			pr_err
			    ("%s: gpr_send_pkt failed with ret %d\n, error_counter %u ",
			     __func__, ret, shmem_map_info.error_counter);
	} else {
		/* error debounce logic  */
		if (shmem_map_info.error_counter > 0) {
			shmem_map_info.error_counter--;
		}
		trace_printk("%s: gpr packet sent successfully \n", __func__);
		ret = TIMER_RESTART;
	}

	if (shmem_map_info.error_counter >= ERROR_TOLERANCE) {
		pr_err
		    ("%s, error counter reached threshold closing the timers and driver \n");
		schedule_delayed_work(&err_handler_work, msecs_to_jiffies(5));
		ret = TIMER_NORESTART;
	}

	if (pkt)
		kfree(pkt);
	return ret;
}

int update_pcie_wr_memhandle(uint32_t wr_mem_handle, void *paddr,
			     struct gpr_device *gprdev)
{

	if (gprdev)
		shmem_map_info.gprdev_local = gprdev;

	shmem_map_info.write_memhandle = wr_mem_handle;
	shmem_map_info.write_shared_mem_paddr = paddr;
	trace_printk("%s:  shmem_map_info.write_memhandle %d ", __func__,
		     shmem_map_info.write_memhandle);

	return 0;
}

EXPORT_SYMBOL_GPL(update_pcie_wr_memhandle);

int update_pcie_rd_memhandle(uint32_t rd_mem_handle, void *paddr,
			     struct gpr_device *gprdev)
{
	if (gprdev)
		shmem_map_info.gprdev_local = gprdev;

	shmem_map_info.read_memhandle = rd_mem_handle;
	shmem_map_info.read_shared_mem_paddr = paddr;
	trace_printk("%s: read_memhandle %d ", __func__,
		     shmem_map_info.read_memhandle);

	return 0;
}

EXPORT_SYMBOL_GPL(update_pcie_rd_memhandle);

int update_pcie_memhandle(void *data, void *paddr, struct gpr_device *gprdev)
{
	static struct apm_cmd_rsp_shared_mem_map_regions_t *memhandle = NULL;

	if (gprdev)
		shmem_map_info.gprdev_local = gprdev;

	trace_printk("inside update_pcie_memhandle write_memhandle -%d ",
		     shmem_map_info.write_memhandle);
	if (!shmem_map_info.write_memhandle) {
		memhandle = GPR_PKT_GET_PAYLOAD(struct
						apm_cmd_rsp_shared_mem_map_regions_t,
						data);
		shmem_map_info.write_memhandle = memhandle->mem_map_handle;
		shmem_map_info.write_shared_mem_paddr = paddr;
		trace_printk(" write memhandle updated %d",
			     shmem_map_info.write_memhandle);
	} else if (!shmem_map_info.read_memhandle) {
		memhandle = GPR_PKT_GET_PAYLOAD(struct
						apm_cmd_rsp_shared_mem_map_regions_t,
						data);
		shmem_map_info.read_memhandle = memhandle->mem_map_handle;
		shmem_map_info.read_shared_mem_paddr = paddr;
		trace_printk(" read memhandle updated %pK",
			     shmem_map_info.read_memhandle);
	}
	return 0;
}

EXPORT_SYMBOL_GPL(update_pcie_memhandle);

int data_cmd_write_done(void *data)
{
	return 0;
}

EXPORT_SYMBOL_GPL(data_cmd_write_done);

static int data_cmd_read(void)
{
	struct gpr_pkt *pkt = NULL;
	struct data_cmd_rd_sh_mem_ep_data_buffer_v2_t read_payload;
	int ret = 0;
	uint32_t size;

	if (!shmem_map_info.gprdev_local) {
		ret = -EAGAIN;
		goto handle_err;
	}

	trace_printk("%s: called at %llu\n", __func__, ktime_get());
	trace_printk("%s:  offset %d paddr %pK", __func__,
		     shmem_map_info.read_buf_offset,
		     (void *)shmem_map_info.read_shared_mem_paddr);
	size =
	    GPR_HDR_SIZE +
	    sizeof(struct data_cmd_rd_sh_mem_ep_data_buffer_v2_t);

	pkt = kzalloc(size, GFP_ATOMIC);
	if (!pkt)
		return -ENOMEM;

	pkt->hdr.header = GPR_SET_FIELD(GPR_PKT_VERSION, GPR_PKT_VER) |
	    GPR_SET_FIELD(GPR_PKT_HEADER_SIZE, GPR_PKT_HEADER_WORD_SIZE_V) |
	    GPR_SET_FIELD(GPR_PKT_PACKET_SIZE, size);

	pkt->hdr.src_port = 0x2011;	//GPR_SVC_ADSP_CORE;
	pkt->hdr.dst_port = 17735;	//MODULE_ID_RD_SHARED_MEM_EP;
	pkt->hdr.dst_domain_id = GPR_IDS_DOMAIN_ID_ADSP_V;
	pkt->hdr.src_domain_id = GPR_IDS_DOMAIN_ID_APPS_V;
	pkt->hdr.token = shmem_map_info.read_buf_offset / WRITE_BUF_SIZE;
	pkt->hdr.opcode = DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2;

	read_payload.buf_addr_lsw =
	    shmem_map_info.read_buf_offset + (NUM_WRITE_BUFS * WRITE_BUF_SIZE);
	shmem_map_info.read_buf_offset += WRITE_BUF_SIZE;
	if (shmem_map_info.read_buf_offset >= NUM_WRITE_BUFS * WRITE_BUF_SIZE)
		shmem_map_info.read_buf_offset = 0;

	trace_printk("%s: offset %d paddr %pK", __func__,
		     shmem_map_info.read_buf_offset,
		     (void *)shmem_map_info.read_shared_mem_paddr);

	read_payload.buf_addr_msw = 0;
	read_payload.buf_size = 1920;	//WRITE_BUF_SIZE;
	read_payload.mem_map_handle = shmem_map_info.read_memhandle;
	trace_printk("%s: paddr %pK memhandle %d", __func__,
		     (void *)shmem_map_info.read_shared_mem_paddr,
		     shmem_map_info.read_memhandle);

	memcpy(&pkt->payload, &read_payload,
	       sizeof(struct data_cmd_rd_sh_mem_ep_data_buffer_v2_t));
	trace_printk(" %s: exit  offset %d paddr %pK", __func__,
		     shmem_map_info.read_buf_offset,
		     (void *)shmem_map_info.read_shared_mem_paddr);

	ret = gpr_send_pkt(shmem_map_info.gprdev_local, pkt);

handle_err:
	if (ret < 0) {
		shmem_map_info.error_counter++;
		if (printk_ratelimit())
			pr_err
			    ("%s: gpr_send_pkt failed with ret %d\n, error_counter %u ",
			     __func__, ret, shmem_map_info.error_counter);
	} else {
		/* error debounce logic  */
		if (shmem_map_info.error_counter > 0) {
			shmem_map_info.error_counter--;
		}
		trace_printk("%s: gpr packet sent successfully \n", __func__);
		ret = TIMER_RESTART;
	}

	if (shmem_map_info.error_counter >= ERROR_TOLERANCE) {
		pr_err
		    ("%s, error counter reached threshold closing the timers and driver \n");
		schedule_delayed_work(&err_handler_work, msecs_to_jiffies(5));
		ret = TIMER_NORESTART;
	}

	if (pkt)
		kfree(pkt);

	return ret;
}

static enum hrtimer_restart msm_pcm_pcie_capture_done(struct hrtimer *hrt)
{
	int ret = 0;
	data_cmd_read();
	return ret;
}

int data_cmd_read_done(void *data)
{
	int ret = 0;
	if (shmem_map_info.first_dl_pkt_rcvd)
		return ret;
	hrtimer_start(&(ul_dl_hrt.capture_done_hrt), ms_to_ktime(1),
		      HRTIMER_MODE_REL);
	return ret;
}

EXPORT_SYMBOL_GPL(data_cmd_read_done);

static enum hrtimer_restart msm_pcm_pcie_playback(struct hrtimer *playback)
{
	uint32_t dl_buf_offset, expected_slot_index, expected_slot_index_prev;
	dma_addr_t dl_buf_dma_addr;
	uint8_t *dl_buf_vaddr, *dl_buf_vaddr_prev;
	uint64_t device_ul_ts;
	uint64_t device_dl_ts;
	uint32_t dl_slot_index_org;
	int ret = HRTIMER_NORESTART;
	ktime_t kt;
	ktime_t first_kt;
	bool error_case = false;
	uint64_t host_ul_ts;
	int64_t time_diff = 0;

	if (voice_pcie_data_buf == NULL) {
		return -EINVAL;
	}
	trace_printk("Inside %s \n", __func__);
	if (shmem_map_info.is_first_dl_buffer)
		shmem_map_info.expected_slot_counter = 0;

	dl_slot_index_org =
	    *(uint32_t *) (voice_pcie_data_buf->area + DL_SLOT_INDEX_OFFSET);
	dl_buf_offset =
	    *(uint32_t *) (voice_pcie_data_buf->area + DL_BUF_OFFSET);
	expected_slot_index =
	    shmem_map_info.expected_slot_counter % NUM_DL_SLOTS;
	expected_slot_index_prev =
	    (shmem_map_info.expected_slot_counter - 1) % NUM_DL_SLOTS;
	dl_buf_dma_addr =
	    voice_pcie_data_buf->addr + dl_buf_offset +
	    expected_slot_index * DL_SLOT_SIZE;
	dl_buf_vaddr =
	    voice_pcie_data_buf->area + dl_buf_offset +
	    expected_slot_index * DL_SLOT_SIZE;
	dl_buf_vaddr_prev =
	    voice_pcie_data_buf->area + dl_buf_offset +
	    expected_slot_index_prev * DL_SLOT_SIZE;

	if (shmem_map_info.expected_slot_counter + NUM_DL_SLOTS <=
	    dl_slot_index_org) {
		shmem_map_info.expected_slot_counter = dl_slot_index_org - 1;
		error_case = true;	// handover scenario
		trace_printk
		    ("%s: expected_slot_counter + NUM_DL_SLOTS <= dl_slot_index_org",
		     __func__, shmem_map_info.expected_slot_counter);
	}

	if (shmem_map_info.expected_slot_counter >= dl_slot_index_org) {
		error_case = true;
		dl_buf_vaddr = dl_buf_vaddr_prev;
		trace_printk
		    ("%s: expected slot counter: %llu greater than dl slot index",
		     __func__, shmem_map_info.expected_slot_counter);
	}

	if (shmem_map_info.is_first_dl_buffer) {
		trace_printk("%s: dl_slot_index %lu \n", __func__,
			     dl_slot_index_org);
		mhi_calculate_drift();
		shmem_map_info.first_dl_pkt_rcvd = 1;
		shmem_map_info.read_buf_offset = 0;
		data_cmd_read();
		trace_printk("%s: first buff\n", __func__);
		device_ul_ts = *(uint64_t *) (dl_buf_vaddr + UL_TS_OFFSET);	//TO CHECK
		device_dl_ts = *(uint64_t *) (dl_buf_vaddr + DL_TS_OFFSET);	//TO CHECK
		host_ul_ts =
		    device_ul_ts + time_offset[last_updated_seq_num].host_time -
		    time_offset[last_updated_seq_num].device_time;

		first_kt =
		    ((TIMETICKS_TO_US(device_ul_ts)) -
		     (TIMETICKS_TO_US(__arch_counter_get_cntvct()) -
		      ((TIMETICKS_TO_US
			(time_offset[last_updated_seq_num].host_time) -
			TIMETICKS_TO_US(time_offset
					[last_updated_seq_num].device_time)))));

		first_kt = first_kt * 1000;
		trace_printk("%s: first_kt in us %lu & %ld ms\n", __func__,
			     first_kt, ktime_to_ms(first_kt));

		time_diff =
		    time_offset[last_updated_seq_num].host_time -
		    time_offset[last_updated_seq_num].device_time;
		time_diff = __arch_counter_get_cntvct() - time_diff;
		time_diff = device_ul_ts - time_diff;

		if (time_diff > 0) {
			first_kt =
			    (device_ul_ts -
			     (__arch_counter_get_cntvct() -
			      (time_offset[last_updated_seq_num].host_time -
			       time_offset[last_updated_seq_num].device_time)));

			first_kt = TIMETICKS_TO_US(first_kt);
			first_kt = first_kt * 1000;
			first_kt = ms_to_ktime(20);
			trace_printk("%s: first_kt in us %lu & %ld ms\n",
				     __func__, first_kt, ktime_to_ms(first_kt));

			hrtimer_start(&(ul_dl_hrt.capture_hrt), first_kt,
				      HRTIMER_MODE_REL);
		} else {

			first_kt = ((20000 + (TIMETICKS_TO_US(device_ul_ts)))
				    -
				    (TIMETICKS_TO_US
				     (__arch_counter_get_cntvct())
				     -
				     ((TIMETICKS_TO_US
				       (time_offset
					[last_updated_seq_num].host_time)
				       -
				       TIMETICKS_TO_US(time_offset
						       [last_updated_seq_num].
						       device_time)))));

			first_kt = first_kt * 1000;
			trace_printk
			    ("%s: inside else first_kt in us %lu & %ld ms\n",
			     __func__, first_kt, ktime_to_ms(first_kt));
			hrtimer_start(&(ul_dl_hrt.capture_hrt), first_kt,
				      HRTIMER_MODE_REL);
		}
		shmem_map_info.is_first_dl_buffer = false;
		ul_dl_hrt.ul_dl_ts.prev_device_ul_ts = device_ul_ts;
		ul_dl_hrt.ul_dl_ts.prev_device_dl_ts = device_dl_ts;
	} else {
		device_ul_ts = *(uint64_t *) (dl_buf_vaddr + UL_TS_OFFSET);
		device_dl_ts = *(uint64_t *) (dl_buf_vaddr + DL_TS_OFFSET);	//TO CHECK
		ul_dl_hrt.ul_dl_ts.rec_kt_diff =
		    device_ul_ts - ul_dl_hrt.ul_dl_ts.prev_device_ul_ts;
		ul_dl_hrt.ul_dl_ts.rec_kt_diff =
		    TIMETICKS_TO_US(ul_dl_hrt.ul_dl_ts.rec_kt_diff);

		//just after error case frame  diff between prev and curr timestamp is observed as 0
		if ((ul_dl_hrt.ul_dl_ts.rec_kt_diff == 0)
		    || (ul_dl_hrt.ul_dl_ts.rec_kt_diff > 25000))
			ul_dl_hrt.ul_dl_ts.rec_kt_diff = 20000;

		ul_dl_hrt.ul_dl_ts.play_kt_diff =
		    device_dl_ts - ul_dl_hrt.ul_dl_ts.prev_device_dl_ts;
		ul_dl_hrt.ul_dl_ts.play_kt_diff =
		    TIMETICKS_TO_US(ul_dl_hrt.ul_dl_ts.play_kt_diff);

		if ((ul_dl_hrt.ul_dl_ts.play_kt_diff == 0)
		    || (ul_dl_hrt.ul_dl_ts.play_kt_diff > 25000))
			ul_dl_hrt.ul_dl_ts.play_kt_diff = 20000;

		ul_dl_hrt.ul_dl_ts.prev_device_ul_ts = device_ul_ts;
		ul_dl_hrt.ul_dl_ts.prev_device_dl_ts = device_dl_ts;
		if (error_case) {
			ul_dl_hrt.ul_dl_ts.rec_kt_diff = 20000;
			ul_dl_hrt.ul_dl_ts.play_kt_diff = 20000;
		}
		ul_dl_hrt.ul_dl_ts.rec_kt_diff = 20000;
		ul_dl_hrt.ul_dl_ts.play_kt_diff = 20000;

		trace_printk("%s: dl_slot_index %lu \n", __func__,
			     dl_slot_index_org);
		trace_printk("%s: expected_slot_counter %llu \n", __func__,
			     shmem_map_info.expected_slot_counter);
		trace_printk("%s: ul_dl_hrt.ul_dl_ts.rec_kt_diff %lu \n",
			     __func__, ul_dl_hrt.ul_dl_ts.rec_kt_diff);
		trace_printk("%s: ul_dl_hrt.ul_dl_ts.play_kt_diff %lu \n",
			     __func__, ul_dl_hrt.ul_dl_ts.play_kt_diff);
	}

	trace_printk("copying from pcie to spf paddr = %pK, offset %d ",
		     (void *)shmem_map_info.write_shared_mem_paddr,
		     shmem_map_info.write_buf_offset);
	if (shmem_map_info.write_shared_mem_paddr) {
		if (error_case == true) {
			memset(shmem_map_info.write_shared_mem_paddr +
			       shmem_map_info.write_buf_offset, 0,
			       DL_PAYLOAD_SIZE);
		} else {
			memcpy(shmem_map_info.write_shared_mem_paddr +
			       shmem_map_info.write_buf_offset,
			       dl_buf_vaddr + DL_PAYLOAD_OFFSET,
			       DL_PAYLOAD_SIZE);
		}
	}

	ret = data_cmd_write();
	if (TIMER_NORESTART == ret) {
		return HRTIMER_NORESTART;
	} else {
		ret = HRTIMER_RESTART;
	}

	ul_dl_hrt.ul_dl_ts.dl_us_schedule_marker = ktime_get();
	ul_dl_hrt.ul_dl_ts.ul_ts_saved = device_ul_ts;
	ul_dl_hrt.ul_dl_ts.dl_ts_saved = device_dl_ts;

	trace_printk("%s: dl path ul_ts_saved %lu \n", __func__,
		     ul_dl_hrt.ul_dl_ts.ul_ts_saved);

	kt = ul_dl_hrt.ul_dl_ts.play_kt_diff * 1000;
	kt = ms_to_ktime(20);

	if (error_case == false) {
		shmem_map_info.expected_slot_counter++;
	} else {
		ul_dl_hrt.ul_dl_ts.dl_us_schedule_marker = 0;
	}

	hrtimer_forward(&(ul_dl_hrt.playback_hrt), ktime_get(), kt);
	return ret;
}

static enum hrtimer_restart msm_pcm_pcie_capture(struct hrtimer *capture)
{
	int ret;
	ktime_t kt;
	volatile uint32_t ul_slot_index, ul_buf_offset, ul_slot_index_mod,
	    ul_ts_buf_offset, ul_dl_ts_buf_offset;
	void *ul_buf_vaddr, *ul_ts_buf_vaddr, *ul_dl_ts_buf_vaddr;
	dma_addr_t ul_buf_addr;
	uint64_t je;
	uint64_t update_ul_ts = 0;

	if (voice_pcie_data_buf == NULL) {
		return -EINVAL;
	}
	trace_printk("Inside %s \n", __func__);
	ul_slot_index =
	    *(uint32_t *) (voice_pcie_data_buf->area + UL_SLOT_INDEX_OFFSET);
	ul_slot_index_mod = ul_slot_index % NUM_UL_SLOTS;
	ul_buf_offset =
	    *(uint32_t *) (voice_pcie_data_buf->area + UL_BUF_OFFSET);
	ul_ts_buf_offset =
	    *(uint32_t *) (voice_pcie_data_buf->area + UL_UL_TS_OFFSET);
	ul_dl_ts_buf_offset =
	    *(uint32_t *) (voice_pcie_data_buf->area + UL_DL_TS_OFFSET);
	ul_buf_addr =
	    voice_pcie_data_buf->addr + ul_buf_offset +
	    (ul_slot_index_mod * UL_SLOT_SIZE);
	ul_buf_vaddr =
	    (void *)(voice_pcie_data_buf->area + ul_buf_offset +
		     (ul_slot_index_mod * UL_SLOT_SIZE));
	ul_ts_buf_vaddr =
	    (void *)(voice_pcie_data_buf->area + ul_ts_buf_offset +
		     (ul_slot_index_mod * UL_SLOT_SIZE));
	ul_dl_ts_buf_vaddr =
	    (void *)(voice_pcie_data_buf->area + ul_dl_ts_buf_offset +
		     (ul_slot_index_mod * UL_SLOT_SIZE));

	ret = data_cmd_read();

	if (TIMER_NORESTART == ret)
		return HRTIMER_NORESTART;

	if (ret < 0) {
		if (printk_ratelimit())
			trace_printk("%s: port read fails: %d\n", __func__,
				     ret);
		goto exit;
	}

	trace_printk("%s: ul_ts_saved in us %lu \n", __func__,
		     ul_dl_hrt.ul_dl_ts.ul_ts_saved);
	trace_printk("%s: dl_ts_saved in us %lu \n", __func__,
		     ul_dl_hrt.ul_dl_ts.dl_ts_saved);
	trace_printk("%s: prev_ul_ts_saved in us %lu \n", __func__,
		     ul_dl_hrt.ul_dl_ts.prev_ul_ts_saved);

	/* case we havent recieved a dl packet with schedule or we are ahead in time */
	if (ul_dl_hrt.ul_dl_ts.ul_ts_update_marker >
	    ul_dl_hrt.ul_dl_ts.dl_us_schedule_marker) {
		update_ul_ts =
		    ul_dl_hrt.ul_dl_ts.prev_ul_ts_saved + UL_BUFF_TIME_OFFSET;
	}
	/* case we have recieved an updated uplink schedule from dl */
	else {
		update_ul_ts = ul_dl_hrt.ul_dl_ts.ul_ts_saved;
	}

	ul_dl_hrt.ul_dl_ts.prev_ul_ts_saved = update_ul_ts;
	memcpy(ul_buf_vaddr + UL_UL_TS_OFFSET, &update_ul_ts, 8);
	memcpy(ul_buf_vaddr + UL_DL_TS_OFFSET, &ul_dl_hrt.ul_dl_ts.dl_ts_saved,
	       8);

	//copy samples from prtd->dma_addr to IPA DMA Buffer
	trace_printk("copying from spf paddr = %pK, offset %d ",
		     shmem_map_info.read_shared_mem_paddr,
		     shmem_map_info.read_buf_offset);
	trace_printk("copying to pcie vaddr = %pK ",
		     ul_buf_vaddr + UL_PAYLOAD_OFFSET);

	trace_printk
	    ("%s: ul_ts_update_marker : %llu & dl_us_schedule_marker: %llu \n",
	     __func__, ul_dl_hrt.ul_dl_ts.ul_ts_update_marker,
	     ul_dl_hrt.ul_dl_ts.dl_us_schedule_marker);
	trace_printk("%s: uplink timestamp = %lu \n", __func__, update_ul_ts);
	ul_dl_hrt.ul_dl_ts.ul_ts_update_marker = ktime_get();

	if (shmem_map_info.read_shared_mem_paddr) {
		memset(ul_buf_vaddr + UL_PAYLOAD_OFFSET, 0, UL_PAYLOAD_SIZE);
		wmb();
		memcpy(ul_buf_vaddr + UL_PAYLOAD_OFFSET,
		       (shmem_map_info.read_shared_mem_paddr +
			(shmem_map_info.read_buf_offset +
			 NUM_WRITE_BUFS * WRITE_BUF_SIZE)), UL_PAYLOAD_SIZE);
	}
	ul_slot_index++;
	trace_printk("%s: ul_slot_index: %llu\n", __func__, ul_slot_index);
	pcm_pcie_hdr->ul_slot_index = ul_slot_index;

exit:
	kt = ul_dl_hrt.ul_dl_ts.rec_kt_diff * 1000;
	kt = ms_to_ktime(20);
	je = ktime_get();

	hrtimer_forward(&(ul_dl_hrt.capture_hrt), ktime_get(), kt);
	return HRTIMER_RESTART;
}

static void poll_dl_index(struct work_struct *work)
{
	volatile uint32_t *dl_index_addr;
	dl_index_addr =
	    (uint32_t *) (voice_pcie_data_buf->area + DL_SLOT_INDEX_OFFSET);

	if (*dl_index_addr >= 1) {
		trace_printk
		    ("%s: First downlink packet arrived, dl index value %d \n",
		     __func__, *dl_index_addr);
		ul_dl_hrt.ul_dl_ts.prev_device_ul_ts = 0;
		ul_dl_hrt.ul_dl_ts.prev_device_dl_ts = 0;
		ul_dl_hrt.ul_dl_ts.rec_kt_diff = 20000;
		ul_dl_hrt.ul_dl_ts.play_kt_diff = 20000;
		shmem_map_info.is_first_dl_buffer = true;
		ul_dl_hrt.ul_dl_ts.timeout = 0;
		time_sync_operation(mhidev_info.mhidev);
		hrtimer_start(&(ul_dl_hrt.playback_hrt), ms_to_ktime(0),
			      HRTIMER_MODE_REL);
		return;
	}
	ul_dl_hrt.ul_dl_ts.timeout++;
	if (ul_dl_hrt.ul_dl_ts.timeout >= MAX_TIMEOUT) {
		trace_printk("%s Downlink packets did not arrive, Time out\n",
			     __func__);
		ul_dl_hrt.ul_dl_ts.timeout = 0;
		return;
	}
	schedule_delayed_work(&poll_index_work, msecs_to_jiffies(1));
}

static int mhi_transfer(bool unmap)
{
	int ret = 0;

	struct mem_info_msg *mhidev_msg = mhidev_info.mhidev_msg;
	struct pcie_dev_resp_msg_t *mhidev_resp_msg =
	    mhidev_info.mhidev_resp_msg;

	dev_dbg(&(mhidev_info.mhidev->dev), "%s: Enter with %u \n", __func__,
		unmap);

	if (!mhidev_resp_msg) {
		ret = -ENOMEM;
		return ret;
	}

	ret =
	    mhi_queue_buf(mhidev_info.mhidev, DMA_FROM_DEVICE, mhidev_resp_msg,
			  sizeof(*mhidev_resp_msg), MHI_EOT);
	if (ret) {
		dev_err(&(mhidev_info.mhidev->dev),
			"Failed to queue buffer to MHI chan, ret %d\n", ret);
		return ret;
	}

	if (!mhidev_msg) {
		ret = -ENOMEM;
		return ret;
	}

	if (unmap) {
		mhidev_msg->addr = 0;
		mhidev_msg->size = 0;
	} else {
		mhidev_msg->addr = voice_pcie_data_buf->addr;
		mhidev_msg->size = DMA_BUFF_SIZE;
	}
	get_random_bytes(&(mhidev_msg->seq_num), sizeof(mhidev_msg->seq_num));

	dev_info(&(mhidev_info.mhidev->dev),
		 "voice_pcie_data_buf-area = %pK, size=%x\n",
		 voice_pcie_data_buf->area, DMA_BUFF_SIZE);
	dev_info(&(mhidev_info.mhidev->dev),
		 "voice_pcie_data_buf-addr = %pK, size=%x\n",
		 voice_pcie_data_buf->addr, DMA_BUFF_SIZE);
	dev_info(&(mhidev_info.mhidev->dev),
		 "mhi_transfer: Sequence Number transmitted %u\n",
		 mhidev_msg->seq_num);

	ret = mhi_queue_buf(mhidev_info.mhidev, DMA_TO_DEVICE, mhidev_msg,
			    sizeof(struct mem_info_msg), MHI_EOT);
	if (ret) {
		dev_err(&(mhidev_info.mhidev->dev),
			"Failed to transfer mem and size to MHI chan, ret %d\n",
			ret);
		return ret;
	}

	if (!unmap) {
		time_sync_operation(mhidev_info.mhidev);
	}
	trace_printk("%s: Exit \n", __func__);
	return ret;
}

static int mhi_setup(struct mhi_device *mhi_dev)
{
	int ret = 0;

	dev_info(&(mhi_dev->dev), "%s: Enter \n", __func__);

	ret = mhi_prepare_for_transfer(mhi_dev, 0);
	if (ret) {
		dev_err(&mhi_dev->dev, "Failed to start MHI chan, ret %d\n",
			ret);
		return ret;
	}

	return ret;
}

static int msm_pcm_pcie_open(struct inode *inode, struct file *filp)
{
	trace_printk("%s Enter\n", __func__);


	mutex_init(&sync_lock);

	mutex_lock(&sync_lock);
	if (mhidev_info.mhidev != NULL) {
		memset(&shmem_map_info, 0, sizeof(shmem_map_info));
		memset(&(ul_dl_hrt.ul_dl_ts), 0,
		       sizeof(struct uplink_downlink_timekeeping));
		memset(voice_pcie_data_buf->area, 0, DMA_BUFF_SIZE);
		init_header(voice_pcie_data_buf->area);
		trace_printk("%s: pcm_pcie_hdr->sample_freq:(%lu), pcm_pcie_hdr->dl_slot_index:(%lu), \
			  pcm_pcie_hdr->ul_slot_index:(%lu) \n", __func__,
			     pcm_pcie_hdr->sample_freq, pcm_pcie_hdr->dl_slot_index, pcm_pcie_hdr->ul_slot_index);
		mhi_transfer(false);
	}

	mutex_unlock(sync_lock);

	if (mhidev_info.mhidev != NULL) {
		mhi_device_get_sync(mhidev_info.mhidev);
	}

	schedule_delayed_work(&poll_index_work, msecs_to_jiffies(1));
	schedule_delayed_work(&sync_operation, msecs_to_jiffies(100));

	return 0;
}

static void msm_pcm_pcie_cancel_timers()
{
	trace_printk("%s\n", __func__);

	cancel_delayed_work_sync(&sync_operation);
	cancel_delayed_work_sync(&poll_index_work);

	hrtimer_cancel(&(ul_dl_hrt.playback_hrt));
	hrtimer_cancel(&(ul_dl_hrt.capture_hrt));
}

static int msm_pcm_pcie_close(struct inode *inode, struct file *filp)
{
	trace_printk("%s\n", __func__);

	msm_pcm_pcie_cancel_timers();

	mhi_transfer(true);
	if (mhidev_info.mhidev != NULL) {
		mhi_device_put(mhidev_info.mhidev);
	}

	if (pcm_pcie_hdr) {
		/* reset slot index at end of call. */
		pcm_pcie_hdr->ul_slot_index = 0;
		pcm_pcie_hdr->dl_slot_index = 0;
	}

	memset(&(shmem_map_info), 0, sizeof(shmem_map_info));
	memset(&(ul_dl_hrt.ul_dl_ts), 0,
	       sizeof(struct uplink_downlink_timekeeping));

	return 0;
}

void msm_pcm_pcie_crash_handler(void)
{
	msm_pcm_pcie_close(NULL, NULL);
}

EXPORT_SYMBOL_GPL(msm_pcm_pcie_crash_handler);

static const struct file_operations msm_pcie_fops = {
	.owner = THIS_MODULE,
	.open = msm_pcm_pcie_open,
	.release = msm_pcm_pcie_close,
};

static int mhi_audio_probe(struct mhi_device *mhi_dev,
			   const struct mhi_device_id *mhi_id)
{
	int ret = 0;

	dev_info(&mhi_dev->dev, "%s: Enter, mhi_dev->dev.parent:(%pK)\n",
		 __func__, mhi_dev->dev.parent->parent);

	voice_pcie_data_buf =
	    devm_kzalloc(&mhi_dev->dev, sizeof(struct snd_dma_buffer),
			 GFP_KERNEL);

	if (!voice_pcie_data_buf) {
		dev_err(&mhi_dev->dev,
			"%s:error allocating pcie data buffer \n", __func__);
		return -ENOMEM;
	}

	voice_pcie_data_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	voice_pcie_data_buf->dev.dev = &mhi_dev->dev;
	voice_pcie_data_buf->private_data = NULL;

	voice_pcie_data_buf->area =
	    dma_alloc_coherent(mhi_dev->dev.parent->parent, DMA_BUFF_SIZE,
			       &voice_pcie_data_buf->addr, GFP_KERNEL);
	if (!voice_pcie_data_buf->area) {
		dev_err(&mhi_dev->dev, "%s: DMA buffer allocation failed\n",
			__func__);
		ret = -ENOMEM;
		goto release_pcie_buf;
	}

	dev_info(&mhi_dev->dev, "%s: 1 dma_alloc_attrs success PA :0x%lx\n",
		 __func__, (unsigned long)voice_pcie_data_buf->addr);
	dev_info(&mhi_dev->dev, "%s: 1 dma_alloc_attrs success VA :0x%lx\n",
		 __func__, (unsigned long)voice_pcie_data_buf->area);

	INIT_DELAYED_WORK(&poll_index_work, poll_dl_index);
	INIT_DELAYED_WORK(&sync_operation, time_sync_op);
	INIT_DELAYED_WORK(&err_handler_work, msm_pcm_pcie_err_handler);

	memset(voice_pcie_data_buf->area, 0, DMA_BUFF_SIZE);
	init_header(voice_pcie_data_buf->area);

	mhidev_info.mhidev = mhi_dev;

	ret = mhi_setup(mhi_dev);

	if (ret)
		goto release_dma_buf;

	mhidev_info.mhidev_msg =
	    devm_kzalloc(&mhi_dev->dev, sizeof(*(mhidev_info.mhidev_msg)),
			 GFP_KERNEL);

	if (!mhidev_info.mhidev_msg) {
		ret = -ENOMEM;
		goto release_dma_buf;
	}

	mhidev_info.mhidev_resp_msg =
	    devm_kzalloc(&mhi_dev->dev, sizeof(*(mhidev_info.mhidev_resp_msg)),
			 GFP_KERNEL);

	if (!mhidev_info.mhidev_resp_msg) {
		ret = -ENOMEM;
		goto release_mhi_msg_buf;
	}

	return ret;

release_mhi_msg_buf:
	devm_kfree(&mhi_dev->dev, mhidev_info.mhidev_msg);

release_dma_buf:
	dma_free_coherent(mhi_dev->dev.parent->parent, DMA_BUFF_SIZE,
			  voice_pcie_data_buf->area, voice_pcie_data_buf->addr);

release_pcie_buf:
	devm_kfree(&mhi_dev->dev, voice_pcie_data_buf);

	return ret;
}

static void mhi_audio_remove(struct mhi_device *mhi_dev)
{
	dev_dbg(&mhi_dev->dev, "%s\n", __func__);

	msm_pcm_pcie_cancel_timers();

	if (voice_pcie_data_buf) {
		dma_free_coherent(mhi_dev->dev.parent->parent, DMA_BUFF_SIZE,
				  voice_pcie_data_buf->area,
				  voice_pcie_data_buf->addr);
		pcm_pcie_hdr = NULL;
	}

	cancel_delayed_work_sync(&err_handler_work);
	return;
}

static void mhi_audio_ul_xfer_cb(struct mhi_device *mhi_dev,
				 struct mhi_result *mhi_result)
{
	trace_printk("%s\n", __func__);
	return;
}

static void mhi_audio_dl_xfer_cb(struct mhi_device *mhi_dev,
				 struct mhi_result *mhi_result)
{

	struct pcie_dev_resp_msg_t *mhidev_resp_msg =
	    mhidev_info.mhidev_resp_msg;

	dev_info(&mhi_dev->dev, "%s: status:%d receive_len:%zu\n", __func__,
		 mhi_result->transaction_status, mhi_result->bytes_xferd);

	memcpy((void *)mhidev_resp_msg, mhi_result->buf_addr,
	       mhi_result->bytes_xferd);

	if (mhi_result->bytes_xferd == sizeof(*mhidev_resp_msg)) {
		dev_info(&mhi_dev->dev, "%s and ack  %u\n", __func__,
			 mhidev_resp_msg->ack);
		dev_info(&mhi_dev->dev, "%s and seq num %u\n", __func__,
			 mhidev_resp_msg->seq_num);
	} else {
		uint32_t ack = mhidev_resp_msg->ack;
		dev_info(&mhi_dev->dev, "%s and ack %u\n", __func__, ack);
	}

	return;
}

static const struct mhi_device_id mhi_audio_match_table[] = {
	{.chan = "AUDIO_VOICE_0"},
	{}
};

static struct mhi_driver mhi_audio_driver = {
	.id_table = mhi_audio_match_table,
	.probe = mhi_audio_probe,
	.remove = mhi_audio_remove,
	.ul_xfer_cb = mhi_audio_ul_xfer_cb,
	.dl_xfer_cb = mhi_audio_dl_xfer_cb,
	.driver = {
		   .name = "msm-pcm-pcie",
		   .owner = THIS_MODULE,
		   }
};

static int msm_pcm_pcie_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pcie_pkt_drv *pcie_pkt = NULL;

	dev_info(&(pdev->dev), "%s: dev name %s\n", __func__,
		 dev_name(&pdev->dev));
	ret = mhi_driver_register(&mhi_audio_driver);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: mhi_driver_register failed with ret = %d",
			__func__, ret);
		return -EPROBE_DEFER;
	}

	/* register as character driver */
	pcie_pkt = devm_kzalloc(&pdev->dev, sizeof(*pcie_pkt), GFP_KERNEL);
	if (!pcie_pkt) {
		dev_err(&pdev->dev, "%s: error allocating pcie_pkt \n",
			__func__);
		goto err_pcie_pkt_alloc;
		ret = -ENOMEM;
	}
	platform_set_drvdata(pdev, pcie_pkt);

	ret = alloc_chrdev_region(&pcie_pkt->audio_pkt_major, 0,
				  MINOR_NUMBER_COUNT, DRV_NAME);
	if (ret < 0) {
		dev_err(&pdev->dev, "alloc_chrdev_region failed ret: %d\n",
			ret);
		goto err_chrdev_region_alloc;
	}

	pcie_pkt->audio_pkt_class = class_create(THIS_MODULE, DRV_NAME);

	if (IS_ERR(pcie_pkt->audio_pkt_class)) {
		ret = PTR_ERR(pcie_pkt->audio_pkt_class);
		dev_err(&pdev->dev, "%s:class create failed", __func__);
		goto err_class_create;
	}
	pcie_pkt->dev = device_create(pcie_pkt->audio_pkt_class, NULL,
				      pcie_pkt->audio_pkt_major, NULL,
				      DRV_NAME);

	if (IS_ERR(pcie_pkt->dev)) {
		ret = PTR_ERR(pcie_pkt->dev);
		dev_err(&pdev->dev, "%s: device create failed", __func__);
		goto err_device_create;
	}

	strlcpy(pcie_pkt->dev_name, DRV_NAME, 20);
	strlcpy(pcie_pkt->ch_name, CHANNEL_NAME, 20);
	dev_set_name(pcie_pkt->dev, pcie_pkt->dev_name);

	cdev_init(&pcie_pkt->cdev, &msm_pcie_fops);
	pcie_pkt->cdev.owner = THIS_MODULE;

	ret = cdev_add(&pcie_pkt->cdev, pcie_pkt->audio_pkt_major,
		       MINOR_NUMBER_COUNT);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed for %s ret:%d\n",
			pcie_pkt->dev_name, ret);
		goto err_cdev_add;
	}

	hrtimer_init(&(ul_dl_hrt.playback_hrt), CLOCK_BOOTTIME,
		     HRTIMER_MODE_REL);
	ul_dl_hrt.playback_hrt.function = msm_pcm_pcie_playback;

	hrtimer_init(&(ul_dl_hrt.capture_hrt), CLOCK_BOOTTIME,
		     HRTIMER_MODE_REL);
	ul_dl_hrt.capture_hrt.function = msm_pcm_pcie_capture;

	hrtimer_init(&(ul_dl_hrt.capture_done_hrt), CLOCK_BOOTTIME,
		     HRTIMER_MODE_REL);
	ul_dl_hrt.capture_done_hrt.function = msm_pcm_pcie_capture_done;

	return ret;

err_cdev_add:
	device_destroy(pcie_pkt->audio_pkt_class, pcie_pkt->audio_pkt_major);
err_device_create:
	class_destroy(pcie_pkt->audio_pkt_class);
err_class_create:
	unregister_chrdev_region(MAJOR(pcie_pkt->audio_pkt_major),
				 MINOR_NUMBER_COUNT);
err_chrdev_region_alloc:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, pcie_pkt);

err_pcie_pkt_alloc:
	mhi_driver_unregister(&mhi_audio_driver);

	return ret;
}

static int msm_pcm_pcie_remove(struct platform_device *pdev)
{

	struct pcie_pkt_drv *pcie_pkt = platform_get_drvdata(pdev);

	cdev_del(&pcie_pkt->cdev);
	device_destroy(pcie_pkt->audio_pkt_class, pcie_pkt->audio_pkt_major);
	class_destroy(pcie_pkt->audio_pkt_class);
	unregister_chrdev_region(MAJOR(pcie_pkt->audio_pkt_major),
				 MINOR_NUMBER_COUNT);

	hrtimer_cancel(&ul_dl_hrt.capture_done_hrt);
	mhi_driver_unregister(&mhi_audio_driver);
	dev_info(&pdev->dev, "%s: done \n", __func__);
	return 0;
}

static const struct of_device_id msm_pcm_pcie_dt_match[] = {
	{.compatible = "qcom,msm-pcm-pcie"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_pcm_pcie_dt_match);

static struct platform_driver msm_pcm_pcie_driver = {
	.driver = {
		   .name = "msm-pcm-pcie",
		   .owner = THIS_MODULE,
		   .of_match_table = msm_pcm_pcie_dt_match,
		   },
	.probe = msm_pcm_pcie_probe,
	.remove = msm_pcm_pcie_remove,
};

static int __init msm_soc_pcie_init(void)
{
	return platform_driver_register(&msm_pcm_pcie_driver);
}

module_init(msm_soc_pcie_init);

static void __exit msm_soc_pcie_exit(void)
{
	platform_driver_unregister(&msm_pcm_pcie_driver);
}

module_exit(msm_soc_pcie_exit);

MODULE_DESCRIPTION("PCIe PCM module platform driver");
MODULE_LICENSE("GPL v2");
