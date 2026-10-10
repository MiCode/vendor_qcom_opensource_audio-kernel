/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _AUDIO_CC_IPC_H
#define _AUDIO_CC_IPC_H

#include <linux/errno.h>
#include <linux/types.h>

#define AUDIO_CC_IPC_READ_SIZE_MAX (4 * 1024)
#define AUDIO_CC_IPC_WRITE_SIZE_MAX (256 * 1024)

enum audio_cc_subsys_state {
	AUDIO_CC_SUBSYS_DOWN,
	AUDIO_CC_SUBSYS_UP,
};

/* Function signature of read buffer callback funtion to client */
typedef void (*audio_cc_ipc_cb_t)(void *data, size_t len);

/*
 * struct audio_cc_ipc_client_info - contains client data
 * @srvc_id		service id which the client registers with
 * @cbf	        read buffer call back function to the client
 *
 * The client needs to register with ipc with the service id,
 * they are expecting a response and ipc will call the function
 * provided in client and pass the data
 */
struct audio_cc_ipc_client_info {
	int srvc_id;
	audio_cc_ipc_cb_t cbf;
};

/*
 * struct audio_cc_msg_pkt - structure similiar to gpr packet
 * @header 			Not used
 * @dst_domain_id	Destination domain id where the packet is to be delivered
 * @src_domain_id	Source domain id from where the packet came
 * @client_data		Not used
 * @src_port		Source port(address) from where the packet came
 * @dst_port		Destination port(address) where the packet is to be	delivered
 * @token			Client transaction ID provided by the sender
 * @opcode			Defines both the action and the payload structure
 */
struct audio_cc_msg_pkt {
	u32 header;
	u8 dst_domain_id;
	u8 src_domain_id;
	u8 client_data;
	u8 reserved;
	u32 src_port;
	u32 dst_port;
	u32 token;
	u32 opcode;
} __packed;

#if IS_ENABLED(CONFIG_AUDIO_CC_IPC)

/* Description	: clients register for the ipc service using this api
 * srvc_id		: service id of the client
 * channel_name	: channel name to be used for the communication
 * cbf	        : callback function for client
 * handle       : private handle of the device registered, this will be filled
 *        by ipc driver and client needs to use it for further communications.
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_register_device(int srvc_id, char *channel_name,
		audio_cc_ipc_cb_t cbf, void **handle);

/* Description	: clients deregister for the ipc service using this api
 * handle       : private handle of the device returned during register
 * srvc_id		: service id of the client
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_deregister_device(void *handle, int srvc_id);

/* Description 	: clients can use this function to send their packets
 * handle       : private handle of the device returned during register
 * pkt			: packet to be send
 * pkt_size		: size of the packet
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_send_pkt(void *handle, void *pkt, u32 pkt_size);

#else

static inline int audio_cc_ipc_register_device(int srvc_id, char *channel_name,
		audio_cc_ipc_cb_t cbf, void **handle)
{
	return -ENOSYS;
}
static inline int audio_cc_ipc_deregister_device(void *handle, int srvc_id)
{
	return -ENOSYS;
}
static inline int audio_cc_ipc_send_pkt(void *handle, void *pkt, u32 pkt_size)
{
	return -ENOSYS;
}
#endif

#endif /* _AUDIO_CC_IPC_H */
