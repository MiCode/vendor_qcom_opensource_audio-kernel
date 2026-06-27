/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/rpmsg.h>
#include <ipc/audio-cc-ipc.h>
#include <dsp/audio_notifier.h>
#include <soc/snd_event.h>

#define CC_IPC_DRIVER_NAME "audio-cc-ipc"
#define CC_IPC_NAME_MAX_LEN 32
#define CC_IPC_MAX_DEV		2
#define CC_IPC_MAX_CLIENTS	2

#define MINOR_NUMBER_COUNT 1
#define TIMEOUT_MS 2000

/*
 * struct cc_ipc_cdev - holds char device related data
 * @cls			class of the char device
 * @dev			device region allocated
 * @cdev		char dev created
 * @dev_num		count for the device
 */
struct cc_ipc_cdev {
	struct class *cls;
	struct device *dev;
	struct cdev cdev;
	dev_t dev_num;
};

/*
 * struct cc_ipc_priv - for device private data
 * @rsp_queue		    sk_buff queue having response buffers available
 * @rsp_qwait		    wait queue to wait on response buffer queue
 * @slock_rsp			lock for accessing response queue
 * @clients			    clients registering kernel calbacks with the device
 * @slock_client		spin lock to synchronize client operations
 * @ch			        reference to rpmsg endpoint
 * @ch_name				channel associated with device and rpmsg
 * @cdev_name			char device name
 * @cdev				char dev info
 * @mlock			    mutex for file ops and othre general synchronization
 * @dev					info of the device created
 * @add_child_dev_work	work queue to add child device if any
 *
 * This struture holds the private data
 * for each of the devices parsed from the DT
 */
struct cc_ipc_priv {
	/* Respone buffer related */
	struct sk_buff_head rsp_queue;
	wait_queue_head_t rsp_qwait;
	spinlock_t slock_rsp;

	/* Registered Clients for the device */
	struct audio_cc_ipc_client_info clients[CC_IPC_MAX_CLIENTS];
	spinlock_t slock_client;

	/* Channel related */
	struct rpmsg_endpoint *ch;
	char ch_name[CC_IPC_NAME_MAX_LEN];
	char cdev_name[CC_IPC_NAME_MAX_LEN];
	struct cc_ipc_cdev cdev;

	struct mutex mlock;
	/* Each cc_ipc_priv is shared between platform & rpmsg */
	struct device *pdev;
	struct device *rpdev;
};

struct cc_ipc_plat_private {
	struct device *dev;
	bool is_initial_boot;
	struct cc_ipc_priv *g_ipriv[CC_IPC_MAX_DEV];
	struct work_struct add_child_dev_work;
	struct mutex g_ipriv_lock;
	struct work_struct ssr_snd_event_work;
	atomic_t audio_cc_state;
	int cc_ipc_num_cdev;
	int cc_ipc_dev_cnt;
};

struct cc_ipc_plat_private *cc_ipc_plat_priv;

static void cc_ipc_add_child_dev_func(struct work_struct *work)
{
	int ret = 0;
	struct cc_ipc_plat_private *ippriv =
	    container_of(work, struct cc_ipc_plat_private, add_child_dev_work);

	ret =
	    of_platform_populate(ippriv->dev->of_node, NULL, NULL, ippriv->dev);
	if (ret) {
		dev_err(ippriv->dev, "%s: failed to add child nodes ret %d\n",
			__func__, ret);
		return;
	}

	ippriv->is_initial_boot = false;

}

static enum audio_cc_subsys_state audio_cc_get_state(void)
{
	return atomic_read(&cc_ipc_plat_priv->audio_cc_state);
}

static int audio_cc_set_state(enum audio_cc_subsys_state state)
{
	dev_dbg(cc_ipc_plat_priv->dev,"%s: setting audio cc state %d\n", __func__, state);
	if (state < AUDIO_CC_SUBSYS_DOWN || state > AUDIO_CC_SUBSYS_UP)
		return -EINVAL;
	atomic_set(&cc_ipc_plat_priv->audio_cc_state, state);
	return 0;
}

static void cc_ipc_snd_event_func(struct work_struct *work)
{
	pr_debug("%s:\n", __func__);
	audio_cc_set_state(AUDIO_CC_SUBSYS_UP);
	snd_event_notify_v2(cc_ipc_plat_priv->dev, SND_EVENT_UP, AUDIO_NOTIFIER_CC_DOMAIN);
}

static ssize_t cc_ipc_fread(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;
	struct sk_buff *skb = NULL;
	int ret = 0, copy = 0;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}

	/*
	 * Wait is signalled from cc_ipc_rpmsg_callback(), or from flush API.
	 * Also use interruptible wait_for_completion API to allow the system
	 * to go in suspend.
	 */
	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	if (skb_queue_empty(&ipriv->rsp_queue)) {
		spin_unlock_irqrestore(&ipriv->slock_rsp, flags);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		/* Wait until we get data or the endpoint goes away */
		if (wait_event_interruptible(ipriv->rsp_qwait,
					!skb_queue_empty(&ipriv->rsp_queue)))
			return -ERESTARTSYS;

		spin_lock_irqsave(&ipriv->slock_rsp, flags);
	}
	skb = skb_dequeue(&ipriv->rsp_queue);
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);
	if (!skb)
		return -EFAULT;

	copy = min_t(size_t, count, skb->len);
	if (copy_to_user(buf, skb->data, copy))
		ret = -EFAULT;
	else
		ret = copy;

	dev_dbg(ipriv->pdev, "%s: ch %s, cp %d, ret %d\n",
		__func__, ipriv->ch_name, copy, ret);

	kfree_skb(skb);
	return ret;
}

static unsigned int cc_ipc_fpoll(struct file *file, poll_table *wait)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return POLLERR;
	}
	dev_dbg(ipriv->pdev, "%s: ch %s\n", __func__, ipriv->ch_name);

	poll_wait(file, &ipriv->rsp_qwait, wait);
	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	if (!skb_queue_empty(&ipriv->rsp_queue))
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);

	return mask;
}

static int cc_ipc_send_pkt(struct cc_ipc_priv *ipriv, void *pkt, uint32_t pkt_size)
{
	int ret;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}
	dev_dbg(ipriv->pdev, "%s: ch %s, size %d\n",
		__func__, ipriv->ch_name, pkt_size);

	if ((audio_cc_get_state() != AUDIO_CC_SUBSYS_UP)) {
		dev_err(ipriv->pdev,"%s: Still cc  is not Up\n", __func__);
		return -ENETRESET;
	}

	mutex_lock(&cc_ipc_plat_priv->g_ipriv_lock);
	ret = rpmsg_send(ipriv->ch, pkt, pkt_size);
	mutex_unlock(&cc_ipc_plat_priv->g_ipriv_lock);
	if (ret < 0)
		dev_err_ratelimited(ipriv->pdev, "%s: failed, ch %s, ret %d\n",
				__func__, ipriv->ch_name, ret);
	return ret;
}


static ssize_t cc_ipc_fwrite(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	void *kbuf;
	int ret = 0;
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}
	dev_dbg(ipriv->pdev, "%s: ch %s: count %zd\n", __func__, ipriv->ch_name,
		count);

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	if (mutex_lock_interruptible(&ipriv->mlock)) {
		ret = -ERESTARTSYS;
		goto free_buf;
	}
	ret = cc_ipc_send_pkt(ipriv, kbuf, count);
	if (ret < 0) {
		dev_err_ratelimited(ipriv->pdev, "%s: Send Failed, ch %s, ret %d\n",
				__func__, ipriv->ch_name, ret);
		mutex_unlock(&ipriv->mlock);
		goto free_buf;
	}
	mutex_unlock(&ipriv->mlock);

free_buf:
	kfree(kbuf);
	return ret < 0 ? ret : count;
}

static int cc_ipc_fflush(struct file *file, fl_owner_t id)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}
	dev_dbg(ipriv->pdev, "%s: ch %s\n", __func__, ipriv->ch_name);

	wake_up_interruptible(&ipriv->rsp_qwait);
	return 0;
}

static int cc_ipc_fopen(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct cc_ipc_cdev *cdev =  container_of(inode->i_cdev, struct cc_ipc_cdev, cdev);
	struct cc_ipc_priv *ipriv =  container_of(cdev, struct cc_ipc_priv, cdev);

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}
	dev_dbg(ipriv->pdev, "%s: ch %s\n", __func__, ipriv->ch_name);

	file->private_data = ipriv;

	return ret;
}

static int cc_ipc_frelease(struct inode *inode, struct file *file)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;
	struct sk_buff *skb = NULL;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}
	dev_dbg(ipriv->pdev, "%s: ch %s\n", __func__, ipriv->ch_name);

	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	/* Discard all SKBs */
	while (!skb_queue_empty(&ipriv->rsp_queue)) {
		skb = skb_dequeue(&ipriv->rsp_queue);
		kfree_skb(skb);
	}
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);
	wake_up_interruptible(&ipriv->rsp_qwait);
	file->private_data = NULL;
	return 0;
}

static int cc_ipc_internal_release(struct rpmsg_device *rpdev)
{
	struct cc_ipc_priv *ipriv = dev_get_drvdata(&rpdev->dev);
	struct sk_buff *skb = NULL;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}
	dev_dbg(ipriv->rpdev, "%s: ch %s\n", __func__, ipriv->ch_name);

	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	/* Discard all SKBs */
	while (!skb_queue_empty(&ipriv->rsp_queue)) {
		skb = skb_dequeue(&ipriv->rsp_queue);
		kfree_skb(skb);
	}
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);
	wake_up_interruptible(&ipriv->rsp_qwait);

	return 0;
}

static const struct file_operations cc_ipc_fops = {
	.owner =                THIS_MODULE,
	.open =                 cc_ipc_fopen,
	.read =                 cc_ipc_fread,
	.write =                cc_ipc_fwrite,
	.flush =                cc_ipc_fflush,
	.poll =                 cc_ipc_fpoll,
	.release =              cc_ipc_frelease,
};

/* Description	: clients register for the ipc service using this api
 * srvc_id		: service id of the client
 * channel_name	: channel name to be used for the communication
 * cbf	        : callback function for client
 * handle       : private handle of the device registered, this will be filled
 *        by ipc driver and client needs to use it for further communications.
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_register_device(int srvc_id, char *channel_name,
	audio_cc_ipc_cb_t cbf, void **handle)
{
	struct cc_ipc_priv *ipriv = NULL;
	unsigned long flags;
	int i = 0, ret = 0;

	// TODO: This should be called only once
	pr_debug("%s: srvc_id %d: ch %s:", __func__, srvc_id, channel_name);

	for (i = 0; i < CC_IPC_MAX_DEV; i++) {
		if ((cc_ipc_plat_priv->g_ipriv[i] != NULL) &&
			!strncmp(channel_name, cc_ipc_plat_priv->g_ipriv[i]->ch_name, CC_IPC_NAME_MAX_LEN)) {
			ipriv = cc_ipc_plat_priv->g_ipriv[i];
			break;
		}
	}

	if (ipriv == NULL) {
		pr_err("%s: Invalid private data\n", __func__);
		ret = -ENODEV;
		goto done;
	}

	ret = -ENOSPC;
	spin_lock_irqsave(&ipriv->slock_client, flags);
	for (i = 0; i < CC_IPC_MAX_CLIENTS; i++) {
		if (ipriv->clients[i].cbf == NULL) {
			ipriv->clients[i].cbf = cbf;
			ipriv->clients[i].srvc_id = srvc_id;
			*handle = (void *)ipriv;
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&ipriv->slock_client, flags);

done:
	pr_debug("%s: ch %s, ret %d\n", __func__, channel_name, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(audio_cc_ipc_register_device);

/* Description	: clients deregister for the ipc service using this api
 * handle       : private handle of the device returned during register
 * srvc_id		: service id of the client
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_deregister_device(void *handle, int srvc_id)
{
	struct cc_ipc_priv *ipriv = (struct cc_ipc_priv *)handle;
	unsigned long flags;
	int ret = -ENOENT;
	int i = 0;

	pr_debug("%s: srvc_id %d", __func__, srvc_id);

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return ret;
	}

	spin_lock_irqsave(&ipriv->slock_client, flags);
	for (i = 0; i < CC_IPC_MAX_CLIENTS; i++) {
		if (ipriv->clients[i].srvc_id == srvc_id) {
			ipriv->clients[i].cbf = NULL;
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&ipriv->slock_client, flags);
	pr_debug("%s: ch %s, ret %d\n", __func__, ipriv->ch_name, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(audio_cc_ipc_deregister_device);

/* Description 	: clients can use this function to send their packets
 * handle       : private handle of the device returned during register
 * pkt			: packet to be send
 * pkt_size		: size of the packet
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_send_pkt(void *handle, void *pkt, uint32_t pkt_size)
{
	return cc_ipc_send_pkt(handle, pkt, pkt_size);
}
EXPORT_SYMBOL_GPL(audio_cc_ipc_send_pkt);

static int cc_ipc_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				int len, void *priv, u32 addr__unused)
{
	struct cc_ipc_priv *ipriv = dev_get_drvdata(&rpdev->dev);
	struct audio_cc_msg_pkt *msg_pkt = NULL;
	unsigned long flags;
	int ret = 0, i = 0;
	struct sk_buff *skb = NULL;

	if (!ipriv || !data) {
		pr_err("%s: Invalid ipriv or data\n", __func__);
		return -EINVAL;
	}

	dev_dbg_ratelimited(ipriv->rpdev, "%s: received buff: ch %s\n",
			__func__, ipriv->ch_name);
	msg_pkt = (struct audio_cc_msg_pkt *) data;

	/* call client callback if matches with clients srv_id registered with
	 * device, else call cache read buff and call complete().
	 */
	spin_lock_irqsave(&ipriv->slock_client, flags);
	for (i = 0; i < CC_IPC_MAX_CLIENTS; i++) {
		if (ipriv->clients[i].cbf != NULL &&
			ipriv->clients[i].srvc_id == msg_pkt->dst_port) {
			spin_unlock_irqrestore(&ipriv->slock_client, flags);
			ipriv->clients[i].cbf(data, len);
			return ret;
		}
	}
	spin_unlock_irqrestore(&ipriv->slock_client, flags);

	if (len <= 0 || len > AUDIO_CC_IPC_READ_SIZE_MAX) {
		dev_info_ratelimited(ipriv->rpdev,
				"%s: ignoring read with len(%d) ch %s\n",
				__func__, len, ipriv->ch_name);
		return -EINVAL;
	}

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb) {
		dev_err_ratelimited(ipriv->rpdev,
				"%s: failed to alloc skb ch %s\n", __func__, ipriv->ch_name);
		return -ENOMEM;
	}

	skb_put_data(skb, data, len);
	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	skb_queue_tail(&ipriv->rsp_queue, skb);
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);

	/* wake up any blocking processes, waiting for new data */
	wake_up_interruptible(&ipriv->rsp_qwait);
	return ret;
}

static int cc_ipc_create_char_device(struct cc_ipc_priv *ipriv)
{
	int ret = 0;
	struct device *dev = NULL;
	char *ch_name = NULL;
	char *cdev_name = NULL;
	struct cc_ipc_cdev *cdev = NULL;

	if (ipriv == NULL) {
		pr_err("%s: Invalid ipriv param\n", __func__);
		return -ENODEV;
	}
	dev = ipriv->pdev;

	ch_name = &ipriv->ch_name[0];
	cdev_name = &ipriv->cdev_name[0];
	cdev = &ipriv->cdev;
	ret = alloc_chrdev_region(&cdev->dev_num, 0, MINOR_NUMBER_COUNT,
							cdev_name);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to alloc char dev %s, ch %s, err %d\n",
			__func__, cdev_name, ch_name, ret);
		goto done;
	}

	cdev->cls = class_create(THIS_MODULE, cdev_name);
	if (IS_ERR(cdev->cls)) {
		ret = PTR_ERR(cdev->cls);
		dev_err(dev, "%s: Failed to create class, dev %s, ch %s, err %d\n",
			__func__, cdev_name, ch_name, ret);
		goto err_class;
	}

	cdev->dev =
	   device_create(cdev->cls, NULL, cdev->dev_num, NULL, cdev_name);
	if (IS_ERR(cdev->dev)) {
		ret = PTR_ERR(cdev->dev);
		dev_err(dev, "%s: Failed to create device, dev %s, ch %s, err %d\n",
			__func__, cdev_name, ch_name, ret);
		goto err_dev_create;
	}

	cdev_init(&cdev->cdev, &cc_ipc_fops);
	ret = cdev_add(&cdev->cdev, cdev->dev_num, MINOR_NUMBER_COUNT);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to register char dev %s, ch %s, err %d\n",
			__func__, cdev_name, ch_name, ret);
		goto err_cdev_add;
	}
	dev_dbg(dev, "%s: registered ch %s, cdev_name %s\n", __func__, ch_name, cdev_name);

	return 0;

err_cdev_add:
	device_destroy(cdev->cls, cdev->dev_num);

err_dev_create:
	class_destroy(cdev->cls);

err_class:
	unregister_chrdev_region(cdev->dev_num, MINOR_NUMBER_COUNT);

done:
	return ret;
}

static int cc_ipc_notifier_service_cb(struct notifier_block *this,
				      unsigned long opcode, void *ptr)
{
	struct audio_notifier_cb_data *cb_data = ptr;

	pr_debug("%s: opcode %d\n", __func__, opcode);

	switch (opcode) {
	case AUDIO_NOTIFIER_SERVICE_DOWN:
		/**
		 * In case of Slate OTA update when CC goes down, avoid CC-state update
		 * to 0. This will intact state of CC to 1, and child dev of audio_cc_ipc_plat
		 * driver can be probed as part of GLINK client probe call. There is no impact
		 * of Slate SSR down handling.
		 */
		if (!cc_ipc_plat_priv->is_initial_boot) {
			audio_cc_set_state(AUDIO_CC_SUBSYS_DOWN);
			snd_event_notify_v2(cc_ipc_plat_priv->dev, SND_EVENT_DOWN, cb_data->domain);
		}
		break;
	case AUDIO_NOTIFIER_SERVICE_UP:
		/*
		 * In case of SSR, domain state is not updated as part of audio notifier register
		 * and service call back can't be triggered from audio notifier.
		 * Trigger SND_EVENT_UP notification as part of rpmsg probe.
		 */
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block notifier_cc_nb = {
	.notifier_call  = cc_ipc_notifier_service_cb,
	.priority = 0,
};

static void cc_ipc_ssr_disable(struct device *dev, void *data)
{
	pr_debug("%s:\n", __func__);
}

static const struct snd_event_ops cc_ipc_ops = {
	.disable = cc_ipc_ssr_disable,
};

static void cc_ipc_destroy_char_device(struct cc_ipc_priv *ipriv)
{
	struct device *dev = NULL;
	if (ipriv == NULL) {
		pr_err("%s: Invalid ipriv param\n", __func__);
		return;
	}
	dev = ipriv->pdev;

	cdev_del(&ipriv->cdev.cdev);
	device_destroy(ipriv->cdev.cls, ipriv->cdev.dev_num);
	class_destroy(ipriv->cdev.cls);
	unregister_chrdev_region(ipriv->cdev.dev_num, MINOR_NUMBER_COUNT);
	dev_dbg(dev, "%s: ch %s, cdev_name %s\n", __func__, ipriv->ch_name,
		ipriv->cdev_name);
}

static int cc_ipc_plat_init(struct cc_ipc_plat_private *iplat_priv)
{
	int itr = 0, err, ret;
	struct device *dev = iplat_priv->dev;
	struct cc_ipc_priv *ipriv = NULL;
	struct of_phandle_iterator of_itr;
	struct device_node *node = NULL;
	const char *of_data = NULL;

	/* Iterate over all the audio ipc nodes */
	of_for_each_phandle(&of_itr, err, dev->of_node, "qcom,glink-cc-ipc",
			    NULL, 0) {
		if (itr >= CC_IPC_MAX_DEV) {
			dev_err(dev, "%s: cc_ipc out of range\n", __func__,
				itr);
			ret = -ERANGE;
			goto err_range;
		}
		node = of_itr.node;

		ipriv = devm_kzalloc(dev, sizeof(*ipriv), GFP_KERNEL);
		if (!ipriv) {
			ret = -ENOMEM;
			goto err_alloc;
		}
		dev_dbg(dev, "%s: Added g_ipriv[%d]=%p\n", __func__, itr,
			ipriv);
		ipriv->pdev = dev;

		/* Get channels & cdev_name */
		ret =
		    of_property_read_string(node, "qcom,glink-channels",
					    &of_data);
		if (ret) {
			dev_err(dev,
				"%s: cannot obtain channel name from dt node\n",
				__func__);
			goto err_of_node;
		}
		dev_dbg(dev, "%s: itr=%d, ch %s\n", __func__, itr, of_data);
		strlcpy(ipriv->ch_name, of_data, CC_IPC_NAME_MAX_LEN);

		ret = of_property_read_string(node, "cdev_name", &of_data);
		if (ret) {
			dev_err(dev,
				"%s: cannot obtain cdev name from dt node\n",
				__func__);
			goto err_of_node;
		}
		dev_dbg(dev, "%s: itr=%d, cdev_name %s\n", __func__, itr,
			of_data);
		strlcpy(ipriv->cdev_name, of_data, CC_IPC_NAME_MAX_LEN);

		mutex_init(&ipriv->mlock);
		spin_lock_init(&ipriv->slock_rsp);
		skb_queue_head_init(&ipriv->rsp_queue);
		init_waitqueue_head(&ipriv->rsp_qwait);
		spin_lock_init(&ipriv->slock_client);

		/* Create ipc cdev */
		ret = cc_ipc_create_char_device(ipriv);
		if (ret < 0) {
			dev_err(dev, "%s: Failed to create char device\n",
				__func__);
			goto err_ipc_char;
		}
		/* TODO: Makeit list */
		iplat_priv->g_ipriv[itr] = ipriv;
		itr++;
	}
	iplat_priv->cc_ipc_num_cdev = itr;

	return 0;

err_ipc_char:
	mutex_destroy(&ipriv->mlock);
err_of_node:
	devm_kfree(dev, ipriv);
err_alloc:
err_range:
	dev_err(dev, "%s: Failed: itr=%d, ret=%d\n", __func__, itr, ret);
	for (itr = 0; itr < CC_IPC_MAX_DEV; itr++) {
		ipriv = iplat_priv->g_ipriv[itr];
		if (ipriv == NULL)
			continue;

		cc_ipc_destroy_char_device(ipriv);
		mutex_destroy(&ipriv->mlock);
		devm_kfree(dev, ipriv);
		iplat_priv->g_ipriv[itr] = NULL;
	}

	return ret;
}

static void cc_ipc_plat_deinit(struct cc_ipc_plat_private *iplat_priv)
{
	int itr;
	struct device *dev = iplat_priv->dev;
	struct cc_ipc_priv *ipriv = NULL;

	for (itr = 0; itr < CC_IPC_MAX_DEV; itr++) {
		ipriv = iplat_priv->g_ipriv[itr];
		if (ipriv == NULL)
			continue;

		cc_ipc_destroy_char_device(ipriv);
		mutex_destroy(&ipriv->mlock);

		devm_kfree(dev, ipriv);
		iplat_priv->g_ipriv[itr] = NULL;
		dev_dbg(dev, "%s: Removed g_ipriv[%d]=%p\n", __func__, itr,
			ipriv);
	}
}

static int cc_ipc_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;
	struct device *dev = &rpdev->dev;
	const char *ch_name = NULL;
	struct cc_ipc_priv *ipriv = NULL;
	int itr = 0;

	ret = of_property_read_string(dev->of_node,
				"qcom,glink-channels", &ch_name);
	if (ret) {
		dev_err(dev, "%s: cannot obtain channel name from dt node\n",
			__func__);
		goto cleanup;
	}
	for (itr = 0; itr < CC_IPC_MAX_DEV; itr++) {
		if ((cc_ipc_plat_priv->g_ipriv[itr] != NULL) &&
		    !strncmp(ch_name, cc_ipc_plat_priv->g_ipriv[itr]->ch_name,
			     CC_IPC_NAME_MAX_LEN)) {
			ipriv = cc_ipc_plat_priv->g_ipriv[itr];
			break;
		}
	}
	if (ipriv == NULL) {
		dev_err(dev, "%s: no ipc g_ipriv\n", __func__);
		ret = -ENODEV;
		goto cleanup;
	}

	dev_dbg(dev, "%s: ch %s\n", __func__, ipriv->ch_name);
	mutex_lock(&cc_ipc_plat_priv->g_ipriv_lock);
	ipriv->ch = rpdev->ept;
	ipriv->rpdev = dev;
	mutex_unlock(&cc_ipc_plat_priv->g_ipriv_lock);
	dev_set_drvdata(dev, ipriv);

	if (cc_ipc_plat_priv->is_initial_boot) {
		schedule_work(&cc_ipc_plat_priv->add_child_dev_work);
	}

	cc_ipc_plat_priv->cc_ipc_dev_cnt++;
	if (cc_ipc_plat_priv->cc_ipc_dev_cnt == CC_IPC_MAX_DEV)
		schedule_work(&cc_ipc_plat_priv->ssr_snd_event_work);
	else if (cc_ipc_plat_priv->cc_ipc_dev_cnt > CC_IPC_MAX_DEV)
		dev_err(dev, "%s: dev cnt %d excedded max cnt %d\n", __func__,
			    cc_ipc_plat_priv->cc_ipc_dev_cnt, CC_IPC_MAX_DEV);

	return 0;

cleanup:
	dev_err(dev, "%s: failed: ret=%d\n", __func__, ret);
	return ret;
}

static void cc_ipc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct cc_ipc_priv *ipriv = dev_get_drvdata(dev);

	if (ipriv) {
		dev_dbg(dev, "%s: ch %s\n", __func__, ipriv->ch_name);
		cc_ipc_internal_release(rpdev);
		mutex_lock(&cc_ipc_plat_priv->g_ipriv_lock);
		ipriv->ch = NULL;
		ipriv->rpdev = NULL;
		mutex_unlock(&cc_ipc_plat_priv->g_ipriv_lock);
		dev_set_drvdata(dev, NULL);
		cc_ipc_plat_priv->cc_ipc_dev_cnt--;
	} else {
		dev_err(dev, "%s: no ipc g_ipriv\n", __func__);
	}

	dev_dbg(dev, "%s: dev cnt %d\n", __func__, cc_ipc_plat_priv->cc_ipc_dev_cnt);
}

static const struct of_device_id cc_ipc_of_match[] = {
	{.compatible = "qcom,audio_cc_ipc"},
	{ }
};
MODULE_DEVICE_TABLE(of, cc_ipc_of_match);

static struct rpmsg_driver cc_ipc_rpmsg_driver = {
	.probe = cc_ipc_rpmsg_probe,
	.remove = cc_ipc_rpmsg_remove,
	.callback = cc_ipc_rpmsg_callback,
	.drv = {
		.name = "audio_cc_ipc",
		.of_match_table = cc_ipc_of_match,
	},
};

static int audio_cc_ipc_platform_driver_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_debug("%s", __func__);

	cc_ipc_plat_priv = devm_kzalloc(&pdev->dev, sizeof(struct cc_ipc_plat_private), GFP_KERNEL);
	if (!cc_ipc_plat_priv)
		return -ENOMEM;

	cc_ipc_plat_priv->dev = &pdev->dev;

	mutex_init(&cc_ipc_plat_priv->g_ipriv_lock);

	INIT_WORK(&cc_ipc_plat_priv->add_child_dev_work, cc_ipc_add_child_dev_func);
	INIT_WORK(&cc_ipc_plat_priv->ssr_snd_event_work, cc_ipc_snd_event_func);

	ret = cc_ipc_plat_init(cc_ipc_plat_priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: failed to cc_ipc_plat_init\n",
			__func__);
		goto cc_ipc_err;
	}

	ret = register_rpmsg_driver(&cc_ipc_rpmsg_driver);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: failed to register rpmsg driver\n",
			__func__);
		goto rpmsg_err;
	}

	/*
	 * TODO: Move audio notififer register/deregister to rpmsg probe/remove to
	 * get notified about the SSR up and down events and synchronize the SUBSYTEM UP/DOWN events
	 * with rpmsg probe/remove calls
	 */
	audio_cc_set_state(AUDIO_CC_SUBSYS_UP);
	ret = audio_notifier_register("audio_cc_ipc", AUDIO_NOTIFIER_CC_DOMAIN,
						&notifier_cc_nb);
	if (ret < 0)
		pr_err("%s: Audio notifier register failed ret = %d\n", __func__, ret);

	ret = snd_event_client_register_v2(&pdev->dev, &cc_ipc_ops, NULL, AUDIO_NOTIFIER_CC_DOMAIN);
	if (!ret)
		snd_event_notify_v2(&pdev->dev, SND_EVENT_UP, AUDIO_NOTIFIER_CC_DOMAIN);
	else
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",	__func__, ret);

	cc_ipc_plat_priv->is_initial_boot = true;
	cc_ipc_plat_priv->cc_ipc_dev_cnt = 0;

	return 0;

rpmsg_err:
	cc_ipc_plat_deinit(cc_ipc_plat_priv);

cc_ipc_err:
	cancel_work_sync(&cc_ipc_plat_priv->add_child_dev_work);
	mutex_destroy(&cc_ipc_plat_priv->g_ipriv_lock);
	devm_kfree(&pdev->dev, cc_ipc_plat_priv);
	cc_ipc_plat_priv = NULL;

	return ret;
}

static int audio_cc_ipc_platform_driver_remove(struct platform_device *pdev)
{
	pr_debug("%s", __func__);

	audio_notifier_deregister("audio_cc_ipc");
	snd_event_client_deregister(&pdev->dev);
	unregister_rpmsg_driver(&cc_ipc_rpmsg_driver);
	cc_ipc_plat_deinit(cc_ipc_plat_priv);
	cancel_work_sync(&cc_ipc_plat_priv->add_child_dev_work);
	mutex_destroy(&cc_ipc_plat_priv->g_ipriv_lock);
	devm_kfree(&pdev->dev, cc_ipc_plat_priv);
	cc_ipc_plat_priv = NULL;

	return 0;
}

static const struct of_device_id audio_cc_ipc_of_match[]  = {
	{ .compatible = "qcom,audio-cc-ipc-platform", },
	{},
};

static struct platform_driver audio_cc_ipc_driver = {
	.probe = audio_cc_ipc_platform_driver_probe,
	.remove = audio_cc_ipc_platform_driver_remove,
	.driver = {
		.name = "audio-cc-ipc-platform",
		.owner = THIS_MODULE,
		.of_match_table = audio_cc_ipc_of_match,
	}
};

static int __init audio_cc_ipc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&audio_cc_ipc_driver);
	if (ret < 0)
		pr_err("audio_cc_ipc: failed to register plat driver\n");

	return ret;
}

static void __exit audio_cc_ipc_exit(void)
{
	platform_driver_unregister(&audio_cc_ipc_driver);
}

module_init(audio_cc_ipc_init);
module_exit(audio_cc_ipc_exit);
MODULE_DESCRIPTION("SoC Audio CC_IPC Driver");
MODULE_LICENSE("GPL v2");
