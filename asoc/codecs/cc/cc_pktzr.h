/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __CC_PKTZR_H_
#define __CC_PKTZR_H_

#include <linux/types.h>
#include <linux/platform_device.h>
#include <ipc/audio-cc-ipc.h>

struct cc_pktzr_pkt_t {
	struct audio_cc_msg_pkt pkt_hdr;
	uint8_t payload[0];
} __packed;

struct cc_pktzr_pkt_priv {
	spinlock_t cc_pktzr_lock;
	uint32_t token;
	struct list_head cc_list;
	bool pktzr_init_complete;
	void *handle;
	int srvc_id;
	char * channel_name;
	uint8_t dst_domain_id;
	uint8_t src_domain_id;
	uint32_t src_port;
	uint32_t dst_port;
} __packed;

struct cc_pktzr_pkt_node {
	struct completion thread_complete;
	struct list_head list;
	uint32_t token;
	uint32_t opcode;
	void *resp_payload;
	size_t resp_size;
} __packed;

int cc_pktzr_send_packet(uint32_t opcode, void *req_payload, size_t req_size,
	void **resp_payload, size_t *resp_size);
int cc_pktzr_init(struct device *dev);
void cc_pktzr_deinit(void);
void cc_pktzr_register_device(void);
void cc_pktzr_deregister_device(void);
#endif /* __CC_PKTZR_H_  */
