// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include "cc_pktzr.h"

#define CC_THREAD_TIMEOUT_MS 5000

static struct cc_pktzr_pkt_priv *ppriv;

static void cc_pktzr_recv_cb(void *data, size_t size)
{
	struct cc_pktzr_pkt_t *msg_pkt = NULL;
	struct cc_pktzr_pkt_node *pnode = NULL, *tmp = NULL;

	if (!data)
		return;
	msg_pkt = (struct cc_pktzr_pkt_t *)data;

	spin_lock(&ppriv->cc_pktzr_lock);

	if (ppriv->cc_list.prev && ppriv->cc_list.next) {
		list_for_each_entry_safe(pnode, tmp, &ppriv->cc_list, list) {
			if (pnode && (pnode->token == msg_pkt->pkt_hdr.token)) {
				pnode->resp_payload = kzalloc(size,
								GFP_ATOMIC);
				if (!pnode->resp_payload)
				{
					spin_unlock(&ppriv->cc_pktzr_lock);
					return;
				}
				memcpy(pnode->resp_payload, msg_pkt->payload,
							size);
				pnode->resp_size = size;
				complete(&pnode->thread_complete);
				spin_unlock(&ppriv->cc_pktzr_lock);
				return;
			}
		}
	}
	spin_unlock(&ppriv->cc_pktzr_lock);
	return;
}

/**
 * cc_pktzr_send_packet - Function to send packet to IPC
 * @opcode:       Unique int number for every packet
 * @req_payload:  Payload to be send
 * @req_size:     size of requested payload
 * @resp_payload: Pointer to copy response payload from IPC
 *		  Calling function should do memory free
 * @resp_size:    size of response payload
 *
 * Returns 0 on success or error on failure
 */
int cc_pktzr_send_packet(uint32_t opcode, void *req_payload, size_t req_size,
	void **resp_payload, size_t *resp_size)
{
	struct cc_pktzr_pkt_t *msg_pkt = NULL;
	struct cc_pktzr_pkt_node *pnode = NULL, *tmp = NULL;
	ssize_t pkt_size = 0;
	int found = 0;
	int ret = -EINVAL;

	if (!ppriv || !ppriv->pktzr_init_complete) {
		pr_err("%s: packetizer not initialized\n",
							__func__);
		return -EINVAL;
	}
	spin_lock(&ppriv->cc_pktzr_lock);
	if (++ppriv->token == 0)
		ppriv->token = 1;

	pkt_size = sizeof(struct cc_pktzr_pkt_t) + req_size;
	msg_pkt = kzalloc(pkt_size, GFP_ATOMIC);
	if (!msg_pkt) {
		spin_unlock(&ppriv->cc_pktzr_lock);
		return -ENOMEM;
	}

	pnode = kzalloc(sizeof(struct cc_pktzr_pkt_node), GFP_ATOMIC);
	if (!pnode) {
		kfree(msg_pkt);
		spin_unlock(&ppriv->cc_pktzr_lock);
		return -ENOMEM;
	}

	pnode->token = ppriv->token;
	pnode->opcode = opcode;
	msg_pkt->pkt_hdr.opcode = opcode;
	msg_pkt->pkt_hdr.token = ppriv->token;
	msg_pkt->pkt_hdr.dst_port = ppriv->dst_port;
	msg_pkt->pkt_hdr.dst_domain_id = ppriv->dst_domain_id;
	msg_pkt->pkt_hdr.src_port = ppriv->src_port;
	msg_pkt->pkt_hdr.src_domain_id = ppriv->src_domain_id;

	if (req_payload)
		memcpy(msg_pkt->payload, (char *)req_payload, req_size);

	INIT_LIST_HEAD(&pnode->list);
	list_add_tail(&pnode->list, &ppriv->cc_list);
	spin_unlock(&ppriv->cc_pktzr_lock);

	init_completion(&pnode->thread_complete);

	ret = audio_cc_ipc_send_pkt(ppriv->handle, msg_pkt, pkt_size);
	if (ret < 0) {
		pr_err("%s Failed to send pkt\n", __func__);
		goto err;
	}

	ret = wait_for_completion_timeout(&pnode->thread_complete,
					msecs_to_jiffies(CC_THREAD_TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Wait for thread timedout\n", __func__);
		ret = -ETIMEDOUT;
		goto err;
	}

	spin_lock(&ppriv->cc_pktzr_lock);
	found = 0;

	if (ppriv->cc_list.prev && ppriv->cc_list.next) {
		list_for_each_entry_safe(pnode, tmp, &ppriv->cc_list, list) {
			if (pnode && (pnode->token == msg_pkt->pkt_hdr.token)) {
				found = 1;
				*resp_payload = pnode->resp_payload;
				*resp_size = pnode->resp_size;
				list_del(&pnode->list);
				kfree(pnode);
				pnode = NULL;
				kfree(msg_pkt);
				msg_pkt = NULL;
				break;
			}
		}
	}

	spin_unlock(&ppriv->cc_pktzr_lock);
	if (!found || (*resp_payload == NULL)) {
		pr_err("%s: packet response not received\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	return 0;
err:
	kfree(msg_pkt);
	msg_pkt = NULL;
	spin_lock(&ppriv->cc_pktzr_lock);
	if (pnode) {
		list_del(&pnode->list);
		kfree(pnode);
		pnode = NULL;
	}
	spin_unlock(&ppriv->cc_pktzr_lock);
	return ret;
}
EXPORT_SYMBOL(cc_pktzr_send_packet);

/**
 * cc_pktzr_deregister_device - Function to deregister to CC IPC driver
 */
void cc_pktzr_deregister_device(void)
{
	int ret = 0;

	if (!ppriv)
		return;

	ret = audio_cc_ipc_deregister_device(ppriv->handle, ppriv->srvc_id);
	if (ret < 0)
		pr_err("%s: Failed to deregister device\n", __func__);
}
EXPORT_SYMBOL(cc_pktzr_deregister_device);

/**
 * cc_pktzr_register_device - Function to register to CC IPC driver
 */
void cc_pktzr_register_device(void)
{
	int ret = 0;
	if (!ppriv)
		return;

	ret = audio_cc_ipc_register_device(ppriv->srvc_id,
			ppriv->channel_name, cc_pktzr_recv_cb, &(ppriv->handle));
	if (ret < 0)
		pr_err("%s: Failed to register device\n", __func__);
}
EXPORT_SYMBOL(cc_pktzr_register_device);

/**
 * cc_pktzr_init - Function to register device
 * @dev: Device Node
 *
 * Returns 0 on success or error on failure
 */
int cc_pktzr_init(struct device *dev)
{
	int ret = 0;
	const char *channel_name = NULL;
	size_t channel_name_len = 0;
	int srvc_id = 0, dst_domain_id = 0, src_domain_id = 0;
	int  src_port = 0, dst_port = 0;

	if (!ppriv) {
		ppriv = kzalloc((sizeof(struct cc_pktzr_pkt_priv)),
							GFP_ATOMIC);
		if (!ppriv)
			return -ENOMEM;

	} else {
		pr_debug("%s: Already initialized\n", __func__);
		goto done;
	}

	ret = of_property_read_u32(dev->of_node, "qcom,service-id", &srvc_id);
	if (ret) {
		dev_dbg(dev, "%s: could not find service-id entry in dt\n",
				__func__);
		goto err;

	}
	ppriv->srvc_id = srvc_id;

	ret = of_property_read_string(dev->of_node, "qcom,channel-name",
					&channel_name);
	if (ret) {
		dev_dbg(dev, "%s: could not find channel_name entry in dt\n",
				__func__);
		goto err;

	}
	channel_name_len = strlen(channel_name) + 1;
	ppriv->channel_name = kzalloc(channel_name_len, GFP_ATOMIC);
	if (!ppriv->channel_name) {
		ret = -ENOMEM;
		goto err;
	}
	strscpy(ppriv->channel_name, channel_name, channel_name_len);

	ret = of_property_read_u32(dev->of_node, "qcom,dst-domain-id",
					&dst_domain_id);
	if (ret) {
		dev_dbg(dev, "%s: could not find dst-domain-id entry in dt\n",
				__func__);
		goto err;

	}
	ppriv->dst_domain_id = dst_domain_id;

	ret = of_property_read_u32(dev->of_node, "qcom,src-domain-id",
					&src_domain_id);
	if (ret) {
		dev_dbg(dev, "%s: could not find src-domain-id entry in dt\n",
				__func__);
		goto err;

	}
	ppriv->src_domain_id = src_domain_id;

	ret = of_property_read_u32(dev->of_node, "qcom,src-port",
					&src_port);
	if (ret) {
		dev_dbg(dev, "%s: could not find src-port entry in dt\n",
				__func__);
		goto err;

	}
	ppriv->src_port = src_port;

	ret = of_property_read_u32(dev->of_node, "qcom,dst-port",
					&dst_port);
	if (ret) {
		dev_dbg(dev, "%s: could not find dst_port entry in dt\n",
				__func__);
		goto err;

	}
	ppriv->dst_port = dst_port;

	cc_pktzr_register_device();
	spin_lock_init(&ppriv->cc_pktzr_lock);
	INIT_LIST_HEAD(&ppriv->cc_list);
	ppriv->pktzr_init_complete = true;
done:
	return 0;
err:
	if (ppriv) {
		if (ppriv->channel_name)
			kfree(ppriv->channel_name);
		kfree(ppriv);
	}
	ppriv = NULL;
	return ret;
}
EXPORT_SYMBOL(cc_pktzr_init);

/**
 * cc_pktzr_deinit - Function to de-register device
 */
void cc_pktzr_deinit(void)
{
	if (!ppriv)
		return;

	cc_pktzr_deregister_device();
	if (ppriv->channel_name)
		kfree(ppriv->channel_name);
	kfree(ppriv);
	ppriv = NULL;
}
EXPORT_SYMBOL(cc_pktzr_deinit);
