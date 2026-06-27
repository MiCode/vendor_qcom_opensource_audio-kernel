// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <soc/snd_event.h>
#include <dsp/audio_notifier.h>
#include "cc_defs.h"
#include "cc_pktzr.h"

#define CC_MAX_SYSFS_BUF_SZ PAGE_SIZE
#define CC_MAX_NUM_ELEM_IN_LIST 64
#define CC_MAX_ELEM_LIST_SIZE (2 * CC_MAX_NUM_ELEM_IN_LIST)

#define CC_CODEC_STR "cc_codec"

typedef uint8_t reg_val_sz_t;

struct cc_id_name_map {
	uint32_t id;
	char name[CC_MAX_STRLEN];
};

struct register_region {
	struct cc_id_name_map *region_id;
	uint32_t min_addr;
	uint32_t max_addr;
	uint32_t addr;
	uint32_t count;
	struct device *dev;
};

struct cc_usecase_info {
	uint32_t uc_id;
	uint32_t sample_rate;
	uint32_t bit_width;
	uint32_t channels;
} __packed;

struct cc_interface {
	struct cc_usecase_info uc_info;
	size_t action_size;
	void *payload;
	size_t payload_size;
	struct list_head path_list;
	struct list_head action_value_list;
};

struct cc_path_list {
	struct list_head list;
	uint32_t src_id;
	uint32_t sink_id;
};

struct cc_iface_list {
	struct list_head iface_list;
	struct cc_interface *iface;
};

struct cc_action_value_list {
	uint32_t action_id;
	uint32_t action_type;
	uint32_t num_params;
	union {
		uint32_t enumerated;
		uint32_t *uint_array;
		int32_t *int_array;
		uint8_t *char_array;
	} param;
	struct list_head list;
	size_t size;
} __packed;

struct cc_action {
	uint32_t action_type;
	struct cc_id_name_map *act_id;
	struct snd_kcontrol_new kcontrol;
	char (*enum_str)[CC_MAX_STRLEN];
	char **texts;
	struct soc_enum action_enum;
	struct soc_multi_mixer_control mixer;
	uint32_t num_params;
	uint32_t def;
	uint32_t param_min;
	uint32_t param_max;
	struct list_head list;
};

struct cc_element {
	uint32_t elem_type;
	struct snd_kcontrol_new *kcontrol;
	char **texts;
	char s_name[CC_MAX_STREAM_STRLEN];
	struct soc_enum elem_enum;
	struct soc_mixer_control *mixer;
	struct snd_soc_dapm_widget widget;
	struct snd_soc_dapm_widget widget_ip_op;
	struct cc_id_name_map *elem_id;
	uint32_t num_src;
	struct cc_id_name_map *src_id;
	struct list_head list;
};

struct cc_codec_priv {
	uint32_t num_id;
	struct cc_id_name_map *id_name;
	uint32_t num_reg_region;
	struct register_region *reg_region;
	uint32_t num_elem;
	struct cc_element *elems;
	uint32_t num_action;
	struct cc_action *actions;
	uint32_t num_routes;
	struct snd_soc_dapm_route *audio_map;
	uint32_t num_iface;
	struct cc_interface *iface;
	struct snd_soc_dai_driver *dai;
	struct snd_soc_component_driver cc_comp;
	struct class *class;	/* Debugging only */
	struct notifier_block cc_adsp_nb;
};

typedef int (cc_add_elem_func)(struct cc_element *, int);

static int cc_parse_name(char *dst, uint8_t *ptr, uint32_t *parsed_len)
{
	uint32_t i = 0;

	for (i = 0; i < CC_MAX_STRLEN; i++) {
		dst[i] = ptr[i];
		if (ptr[i] == 0)
			break;
	}

	if (ptr[i] != 0)
		return -EINVAL;

	*parsed_len = ((i + 1) * sizeof(char));

	return 0;
}

static int cc_parse_id_name(struct cc_codec_priv *cc_priv,
				uint8_t *ptr, uint32_t *parsed_len)
{
	uint32_t tmp_len = 0;
	int ret = 0;

	pr_debug("%s: Start: id: 0x%x, num_id=%u\n", __func__,
		 *(uint32_t *) ptr, cc_priv->num_id);
	cc_priv->id_name[cc_priv->num_id].id = *(uint32_t *) ptr;
	ptr += sizeof(uint32_t);

	ret = cc_parse_name(cc_priv->id_name[cc_priv->num_id].name,
				ptr, &tmp_len);
	if (ret) {
		pr_err("%s: Invalid name of id: 0x%x\n",
			__func__, cc_priv->id_name[cc_priv->num_id].id);
		return ret;
	}

	*parsed_len = (sizeof(uint32_t) + tmp_len);
	pr_debug("%s: End: id: 0x%x, name=%s, num_id=%u, parsed_len=%u\n",
		 __func__, cc_priv->id_name[cc_priv->num_id].id,
		 cc_priv->id_name[cc_priv->num_id].name, cc_priv->num_id,
		 *parsed_len);
	cc_priv->num_id++;

	return 0;
}

static int cc_del_path_for_elem(struct cc_element *elem,
					uint32_t src_id, uint32_t sink_id)
{
	struct list_head *node_ext = NULL, *next_ext = NULL;
	struct list_head *node = NULL, *next = NULL;
	struct cc_path_list *path_list = NULL;
	struct cc_iface_list *elem_ifaces = NULL;
	int found = 0;

	list_for_each_safe(node_ext, next_ext, &elem->list) {
		elem_ifaces = list_entry(node_ext, struct cc_iface_list, iface_list);
		list_for_each_safe(node, next, &elem_ifaces->iface->path_list) {
			path_list = list_entry(node, struct cc_path_list, list);
			if ((path_list->sink_id == sink_id) &&
				(path_list->src_id == src_id)) {
				found = 1;
				break;
			}
		}

		if (found) {
			list_del(node);
			kfree(path_list);
			path_list = NULL;
		}
	}

	return 0;
}

static int cc_add_path_for_elem(struct cc_element *elem,
					uint32_t src_id, uint32_t sink_id)
{
	struct list_head *node = NULL, *next = NULL;
	struct cc_path_list *path_list = NULL;
	struct cc_iface_list *elem_ifaces = NULL;

	list_for_each_safe(node, next, &elem->list) {
		elem_ifaces = list_entry(node,
				struct cc_iface_list, iface_list);
		path_list = kzalloc(sizeof(struct cc_path_list), GFP_KERNEL);
		if (!path_list)
			return -ENOMEM;
		path_list->src_id = src_id;
		path_list->sink_id = sink_id;
		list_add_tail(&path_list->list, &elem_ifaces->iface->path_list);
	}

	return 0;
}

static void cc_modify_elem_mux_val(struct cc_element *elem, uint32_t src_id)
{
	struct list_head *node = NULL, *next = NULL;
	struct list_head *node_ext = NULL, *next_ext = NULL;
	struct cc_path_list *path_list = NULL;
	struct cc_iface_list *elem_ifaces = NULL;

	list_for_each_safe(node_ext, next_ext, &elem->list) {
		elem_ifaces = list_entry(node_ext,
					struct cc_iface_list, iface_list);
		list_for_each_safe(node, next, &elem_ifaces->iface->path_list) {
			path_list = list_entry(node, struct cc_path_list, list);
			if (path_list->sink_id == elem->elem_id->id) {
				path_list->src_id = src_id;
				break;
			}
		}
	}
}

static int cc_add_array_action_val(struct cc_action *act,
	void *params, size_t size_params)
{
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	struct cc_action_value_list *act_val = NULL;

	if (!params || !size_params)
		return -EINVAL;

	list_for_each_safe(node, next, &act->list) {
		act_ifaces = list_entry(node, struct cc_iface_list, iface_list);

		act_val = kzalloc(sizeof(struct cc_action_value_list),
					GFP_KERNEL);
		if (!act_val)
			return -ENOMEM;

		act_val->action_id = act->act_id->id;
		act_val->action_type = act->action_type;
		act_val->num_params = act->num_params;
		act_val->param.char_array = kzalloc(size_params, GFP_KERNEL);
		if (!act_val->param.char_array)
			return -ENOMEM;

		memcpy(act_val->param.char_array, params, size_params);

		list_add_tail(&act_val->list,
			&act_ifaces->iface->action_value_list);

		act_val->size = (size_params +
				sizeof(act_val->action_id));
		act_ifaces->iface->action_size += act_val->size;
	}

	return 0;
}

static void cc_modify_array_action_val(struct cc_action *act,
	void *params, size_t size_params)
{
	struct list_head *node_ext = NULL, *next_ext = NULL;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	struct cc_action_value_list *act_val = NULL;

	if (!params || !size_params)
		return;

	list_for_each_safe(node_ext, next_ext, &act->list) {
		act_ifaces = list_entry(node_ext,
					struct cc_iface_list, iface_list);
		list_for_each_safe(node, next,
			&act_ifaces->iface->action_value_list) {
			act_val = list_entry(node,
					struct cc_action_value_list, list);
			if (act_val->action_id == act->act_id->id) {
				memcpy(act_val->param.char_array, params,
						size_params);
				break;
			}
		}
	}
}

static int cc_add_enum_action_val(struct cc_action *act, uint32_t val)
{
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	struct cc_action_value_list *act_val = NULL;

	list_for_each_safe(node, next, &act->list) {
		act_ifaces = list_entry(node, struct cc_iface_list, iface_list);

		act_val = kzalloc(sizeof(struct cc_action_value_list),
					GFP_KERNEL);
		if (!act_val)
			return -ENOMEM;

		act_val->action_id = act->act_id->id;
		act_val->action_type = act->action_type;
		act_val->num_params = 1;
		act_val->param.enumerated = val;
		list_add_tail(&act_val->list,
			&act_ifaces->iface->action_value_list);

		act_val->size = (sizeof(uint32_t) +
				sizeof(act_val->action_id));
		act_ifaces->iface->action_size += act_val->size;
	}

	return 0;
}

static void cc_modify_enum_action_val(struct cc_action *act, uint32_t val)
{
	struct list_head *node_ext = NULL, *next_ext = NULL;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	struct cc_action_value_list *act_val = NULL;

	list_for_each_safe(node_ext, next_ext, &act->list) {
		act_ifaces = list_entry(node_ext,
					struct cc_iface_list, iface_list);
		list_for_each_safe(node, next,
			&act_ifaces->iface->action_value_list) {
			act_val = list_entry(node,
					struct cc_action_value_list, list);
			if (act_val->action_id == act->act_id->id) {
				act_val->param.enumerated = val;
				break;
			}
		}
	}
}

static ssize_t cc_reg_region_info_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct register_region *reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	return snprintf(buf, CC_MAX_SYSFS_BUF_SZ, "%d:%s\n",
			reg_region->region_id->id, reg_region->region_id->name);
}

static DEVICE_ATTR(info, S_IRUGO, cc_reg_region_info_show, NULL);

static ssize_t cc_reg_range_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct register_region *reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	return snprintf(buf, CC_MAX_SYSFS_BUF_SZ, "0x%04x-0x%04x\n",
			reg_region->min_addr, reg_region->max_addr);
}

static DEVICE_ATTR(range, S_IRUGO, cc_reg_range_show, NULL);

static ssize_t cc_reg_addr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct register_region *reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	return snprintf(buf, CC_MAX_SYSFS_BUF_SZ, "0x%04x\n", reg_region->addr);
}

static ssize_t cc_reg_addr_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	uint32_t addr = 0;
	int ret = 0;
	struct register_region *reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	ret = kstrtouint(buf, 16, &addr);
	if (ret < 0) {
		dev_err(dev, "%s: error ret=%d, use hex format\n", __func__,
			ret);
		return ret;
	}

	if (addr < reg_region->min_addr || addr > reg_region->max_addr) {
		dev_err(dev, "Invalid addr %x, range: 0x%04x-0x%04x\n",
			addr, reg_region->min_addr, reg_region->max_addr);
		return -EINVAL;
	}

	reg_region->addr = addr;

	return count;
}

static DEVICE_ATTR(addr, S_IWUSR | S_IRUGO, cc_reg_addr_show,
		   cc_reg_addr_store);

static ssize_t cc_reg_count_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct register_region *reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	return snprintf(buf, CC_MAX_SYSFS_BUF_SZ, "%u\n", reg_region->count);
}

static ssize_t cc_reg_count_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t cnt = 0, addr_range = 0;
	int ret = 0;
	struct register_region *reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	ret = kstrtouint(buf, 10, &cnt);
	if (ret < 0) {
		pr_err("%s: error ret=%d, use dec format\n", __func__, ret);
		return ret;
	}

	addr_range = reg_region->max_addr - reg_region->min_addr + 1;
	if (cnt > addr_range) {
		dev_err(dev, "Count should be <= %u\n", addr_range);
		return -EINVAL;
	}

	reg_region->count = cnt;

	return count;
}

static DEVICE_ATTR(count, S_IWUSR | S_IRUGO, cc_reg_count_show,
		   cc_reg_count_store);

static int cc_send_pkt_with_response(uint32_t opcode, void *payload,
				     size_t size)
{
	int rc = 0;
	struct cc_resp_generic *resp = NULL;
	size_t resp_size = 0;

	rc = cc_pktzr_send_packet(opcode, payload, size, (void **)&resp,
				  &resp_size);
	if (rc) {
		pr_err("%s: opcode: %d: send_packet error: rc=%d\n", __func__,
		       opcode, rc);
		return -EINVAL;
	}

	if (!resp || resp_size < CC_MIN_IPC_PAYLOAD_SIZE
	    || resp->opcode != opcode) {
		pr_err("%s: opcode: %d: Invalid response\n", __func__, opcode);
		rc = -EINVAL;
		goto end;
	}

	if (resp->response != RESPONSE_SUCESS) {
		rc = -EINVAL;
		pr_err("%s: opcode: %d failure resp: %d\n",
		       __func__, opcode, resp->response);
	}

end:
	if (!IS_ERR_OR_NULL(resp)) {
		kfree(resp);
		resp = NULL;
	}

	return rc;
}

static ssize_t cc_reg_value_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret = 0, i, buf_len;
	struct register_region *reg_region = NULL;
	cc_reg_request_t read_req;
	cc_reg_response_t *read_resp = NULL;
	size_t req_size = 0, resp_size =
	    0 /* register output value size */;
	reg_val_sz_t *reg_value = NULL;
	uint32_t valid_range = 0, resp_num_reg;

	reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	read_req.dev_id = reg_region->region_id->id;
	read_req.start_addr = reg_region->addr;
	valid_range = reg_region->max_addr - reg_region->addr + 1;
	if (reg_region->count > valid_range) {
		dev_err(dev,
			 "%s: warning: requested address count(%u) exceeds possible range(%u)\n",
			 __func__, reg_region->count, valid_range);
		return -ERANGE;
	}
	read_req.num_reg = reg_region->count;
	req_size = sizeof(cc_reg_request_t);

	/* Send read register command */
	dev_dbg(dev, "%s: input: id=%u, addr=0x%x, count=%u: req-size=%lu\n",
		__func__, read_req.dev_id, read_req.start_addr,
		read_req.num_reg, req_size);
	ret =
	    cc_pktzr_send_packet(CC_CODEC_OPCODE_READ_REGISTER, &read_req,
				 req_size, (void **)&read_resp, &resp_size);
	if (ret) {
		dev_err(dev, "%s: error send_pckt=%d\n", __func__, ret);
		return -EINVAL;
	}
	if (!read_resp || resp_size < CC_MIN_IPC_PAYLOAD_SIZE) {
		dev_err(dev, "%s: Invalid response\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	if (read_resp->success != 0) {
		dev_err(dev, "%s: error responce=%d\n", __func__,
			read_resp->success);
		ret = -EINVAL;
		goto end;
	}
	dev_dbg(dev, "%s: responce=%d & resp_size=%lu\n", __func__,
		read_resp->success, resp_size);

	reg_value = (reg_val_sz_t *) & read_resp->payload[0];
	resp_num_reg =
	    (resp_size - CC_MIN_IPC_PAYLOAD_SIZE -
	     sizeof(cc_reg_response_t)) / sizeof(reg_val_sz_t);
	/* Loading output values */
	ret =
	    scnprintf(buf, CC_MAX_SYSFS_BUF_SZ,
		     "Reg-base is 0x%x:\nFormat(hex): Offset-Value\n",
		     reg_region->min_addr);
	if (ret < 0) {
		goto end;
	}
	for (i = 0, buf_len = ret;
	     i < read_req.num_reg && i < resp_num_reg; i++) {
		/* NOTE: With this format, max 450 registers can be read */
		ret =
		    scnprintf(buf + buf_len, CC_MAX_SYSFS_BUF_SZ - buf_len,
			     "%04x: %02x\n", (read_req.start_addr + i) & 0xffff,
			     reg_value[i]);
		if (ret < 0) {
			break;
		}

		/* Check if it exceeds the limit */
		if ((CC_MAX_SYSFS_BUF_SZ - buf_len - 1) < ret)
			break;

		buf_len += ret;
	}

end:
	if (!IS_ERR_OR_NULL(read_resp)) {
		kfree(read_resp);
		read_resp = NULL;
	}

	return ret < 0 ? -EINVAL : buf_len;
}

static ssize_t cc_reg_value_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret = 0, i = 0;
	struct register_region *reg_region = NULL;
	cc_reg_request_t *write_req = NULL;
	size_t req_size = 0 /* register input val size */ ;
	uint32_t valid_range = 0;
	reg_val_sz_t *reg_value = NULL;
	char *val_key = NULL;

	reg_region = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(reg_region)) {
		dev_err(dev, "%s: error reg_region=%d\n", __func__,
			PTR_ERR(reg_region));
		return -EINVAL;
	}

	valid_range = reg_region->max_addr - reg_region->addr + 1;
	if (reg_region->count > valid_range) {
		dev_err(dev,
			 "%s: warning: requested address count(%u) exceeds possible range(%u)\n",
			 __func__, reg_region->count, valid_range);
		return -ERANGE;
	}

	req_size =
	    ((sizeof(reg_val_sz_t) * reg_region->count) +
	     sizeof(cc_reg_request_t));
	write_req = (cc_reg_request_t *) kzalloc(req_size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(write_req)) {
		dev_err(dev, "%s: error write_req mem=%d\n", __func__,
			PTR_ERR(write_req));
		return -ENOMEM;
	}

	write_req->dev_id = reg_region->region_id->id;
	write_req->start_addr = reg_region->addr;
	write_req->num_reg = reg_region->count;
	reg_value = (reg_val_sz_t *) & write_req->payload[0];

	/* Loading input values */
	for (i = 0; i < reg_region->count; i++) {
		val_key = strsep((char **)&buf, " ");
		if (val_key == NULL) {
			dev_err(dev, "%s: val_key is null: i=%d\n", __func__,
				i);
			break;
		}

		ret = sscanf(val_key, "%hhx", reg_value + i);
		if (ret < 0) {
			dev_err(dev, "%s: error ret=%d, use hex format\n",
				__func__, ret);
			break;
		}
		dev_dbg(dev, "%s: Value=%hhu, ret=%d\n", __func__,
			*(uint8_t *) (reg_value + i), ret);
	}

	if (ret < 0) {
		goto end;
	}

	/* Check if inputs are proper */
	if (i != reg_region->count || strsep((char **)&buf, " ") != NULL) {
		dev_err(dev,
			"%s: Number of input values doesn't match with count value\n",
			__func__);
		ret = -ERANGE;
		goto end;
	}

	/* Send write register command */
	dev_dbg(dev, "%s: input: id=%u, addr=0x%x, count=%u\n", __func__,
		write_req->dev_id, write_req->start_addr, write_req->num_reg);
	ret =
	    cc_send_pkt_with_response(CC_CODEC_OPCODE_WRITE_REGISTER, write_req,
				      req_size);
	if (ret) {
		dev_err(dev, "%s: error send_pckt=%d\n", __func__, ret);
		ret = -EINVAL;
	}

end:
	if (!IS_ERR_OR_NULL(write_req)) {
		kfree(write_req);
		write_req = NULL;
	}

	return ret < 0 ? -EINVAL : count;
}

static DEVICE_ATTR(value, S_IWUSR | S_IRUGO, cc_reg_value_show,
		   cc_reg_value_store);

static struct attribute *cc_reg_attrs[] = {
	&dev_attr_info.attr,
	&dev_attr_range.attr,
	&dev_attr_addr.attr,
	&dev_attr_count.attr,
	&dev_attr_value.attr,
	NULL,
};

static struct attribute_group cc_reg_attr_grp = {
	.name = "reg_debug",
	.attrs = cc_reg_attrs,
};

static int cc_enable_reg_debugging(struct cc_codec_priv *cc_priv,
				      struct device *dev)
{
	int ret = 0, i, num_dev;
	struct register_region *reg_region = NULL;

	if (!cc_priv || !dev) {
		pr_err("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	num_dev = cc_priv->num_reg_region;

	cc_priv->class = class_create(THIS_MODULE, CC_CODEC_STR);
	if (IS_ERR_OR_NULL(cc_priv->class)) {
		ret = PTR_ERR(cc_priv->class);
		dev_err(dev, "%s: class_create failed: ret %d\n", __func__,
			ret);
		return ret;
	}

	for (i = 0; i < num_dev; i++) {
		reg_region = &cc_priv->reg_region[i];
		reg_region->dev =
		    device_create(cc_priv->class, dev, i, reg_region,
				  "cc_dev%d", reg_region->region_id->id);
		if (IS_ERR_OR_NULL(reg_region->dev)) {
			ret = PTR_ERR(reg_region->dev);
			dev_err(dev, "%s: device_create(%d) failed: ret %d\n",
				__func__, i, ret);
			break;
		}

		ret =
		    sysfs_create_group(&reg_region->dev->kobj,
				       &cc_reg_attr_grp);
		if (ret) {
			device_destroy(cc_priv->class, i);
			dev_err(dev, "%s: sysfs_create(%d) failed: ret %d\n",
				__func__, i, ret);
			break;
		}
	}

	if (!ret)
		return 0;

	/* Cleanup resources */
	for (i = i - 1; i >= 0; i--) {
		reg_region = &cc_priv->reg_region[i];
		sysfs_remove_group(&reg_region->dev->kobj, &cc_reg_attr_grp);
		device_destroy(cc_priv->class, i);
	}
	class_destroy(cc_priv->class);

	return ret;
}

static void cc_disable_reg_debugging(struct cc_codec_priv *cc_priv)
{
	int i, num_dev;
	struct register_region *reg_region = NULL;

	if (!cc_priv) {
		pr_err("%s: invalid params\n", __func__);
		return;
	}
	num_dev = cc_priv->num_reg_region;

	if (!IS_ERR_OR_NULL(cc_priv->class)) {
		for (i = 0; i < num_dev; i++) {
			reg_region = &cc_priv->reg_region[i];
			if (!IS_ERR_OR_NULL(reg_region->dev)) {
				sysfs_remove_group(&reg_region->dev->kobj,
						   &cc_reg_attr_grp);
				device_destroy(cc_priv->class, i);
			}
		}
		class_destroy(cc_priv->class);
	}
}

static int cc_parse_reg_region(struct cc_codec_priv *cc_priv,
		struct device *dev, uint8_t *payload,
		uint32_t num_dev, uint32_t *parsed_len)
{
	int ret = 0;
	int i = 0;
	uint32_t tmp_len = 0;

	if (!cc_priv || !dev || !payload || !parsed_len)
		return -EINVAL;

	if (num_dev > CC_MAX_REG_REGION) {
		dev_err(dev, "num register regions > %d\n",
			CC_MAX_REG_REGION);
		return -EINVAL;
	}

	if (num_dev == 0)
		return 0;

	cc_priv->reg_region = kzalloc(
			num_dev * sizeof(struct register_region), GFP_KERNEL);
	if (!cc_priv->reg_region)
		return -ENOMEM;
	cc_priv->num_reg_region = num_dev;

	*parsed_len = 0;
	for (i = 0; i < num_dev; i++) {
		ret = cc_parse_id_name(cc_priv, payload, &tmp_len);
		if (ret) {
			dev_err(dev, "%s: parse_id_name failed for device %d\n", __func__, i);
			break;
		}
		payload += tmp_len;
		*parsed_len += tmp_len;

		cc_priv->reg_region[i].region_id =
			&cc_priv->id_name[cc_priv->num_id - 1];

		cc_priv->reg_region[i].min_addr = *((uint32_t *)payload);
		tmp_len = sizeof(cc_priv->reg_region[i].min_addr);
		payload += tmp_len;
		*parsed_len += tmp_len;

		cc_priv->reg_region[i].max_addr = *((uint32_t *)payload);
		tmp_len = sizeof(cc_priv->reg_region[i].max_addr);
		payload += tmp_len;
		*parsed_len += tmp_len;
	}

	return ret;
}

static int cc_parse_elem_src(struct cc_codec_priv *cc_priv,
	struct cc_element *elem, uint8_t *payload, uint32_t *parsed_len)
{
	int ret = 0;
	int i = 0;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;

	if (!cc_priv || !elem || !payload || !parsed_len)
		return -EINVAL;

	if (elem->num_src == 0 || elem->num_src > CC_MAX_NUM_SRC_PER_ELEM) {
		pr_err("%s: invalid number of source for element: %d\n",
			__func__, elem->num_src);
		return -EINVAL;
	}

	*parsed_len = 0;
	elem->src_id = &cc_priv->id_name[cc_priv->num_id];
	for (i = 0; i < elem->num_src; i++) {
		ret = cc_parse_id_name(cc_priv, tmp, &tmp_len);
		if (ret)
			break;
		*parsed_len += tmp_len;
		tmp += tmp_len;
	}

	return ret;
}

static int cc_elem_ctl_mux_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct cc_element *elem = &cc_priv->elems[w->shift];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint32_t val = ucontrol->value.enumerated.item[0];
	struct cc_path_list *path_list = NULL;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *elem_ifaces = NULL;
	int found = 0;
	int rc = 0;

	if (val > elem->num_src)
		return -EINVAL;

	list_for_each_safe(node, next, &elem->list) {
		elem_ifaces = list_entry(node,
			struct cc_iface_list, iface_list);
		break;
	}

	if (!elem_ifaces)
		return 0;

	list_for_each_safe(node, next, &elem_ifaces->iface->path_list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (path_list->sink_id == elem->elem_id->id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (val)
			cc_modify_elem_mux_val(elem, elem->src_id[val - 1].id);
		else
			cc_del_path_for_elem(elem, path_list->src_id,
						elem->elem_id->id);
	} else {
		if (val)
			rc = cc_add_path_for_elem(elem,
				elem->src_id[val - 1].id, elem->elem_id->id);
	}

	snd_soc_dapm_mux_update_power(w->dapm, kcontrol, val, e, NULL);

	return rc;
}

static int cc_elem_ctl_mux_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct cc_element *elem = NULL;
	struct cc_path_list *path_list = NULL;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *elem_ifaces = NULL;
	int found = 0;
	int i = 0;

	elem = &cc_priv->elems[w->shift];

	list_for_each_safe(node, next, &elem->list) {
		elem_ifaces = list_entry(node, struct cc_iface_list, iface_list);
		break;
	}

	if (!elem_ifaces)
		return 0;

	list_for_each_safe(node, next, &elem_ifaces->iface->path_list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (path_list->sink_id == elem->elem_id->id) {
			found = 1;
			break;
		}
	}

	if (!found) {
		ucontrol->value.enumerated.item[0] = 0;
		return 0;
	}

	for (i = 0; i < elem->num_src; i++) {
		if (path_list->src_id == elem->src_id[i].id)
			break;
	}

	if (i == elem->num_src)
		return -EINVAL;

	ucontrol->value.enumerated.item[0] = i + 1;

	return 0;
}


static int cc_elem_ctl_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct cc_path_list *path_list = NULL;
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_element *elem = &cc_priv->elems[w->shift];
	uint32_t sink_id = elem->elem_id->id;
	uint32_t src_id = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *elem_ifaces = NULL;
	int found = 0;
	uint32_t val = 0;
	int rc = 0;

	val = ucontrol->value.integer.value[0];

	if (elem->elem_type == ELEM_TYPE_MIX) {
		src_id = elem->src_id[mixer->shift].id;
	} else if (elem->elem_type == ELEM_TYPE_SWITCH ||
		elem->elem_type == ELEM_TYPE_INPUT ||
		elem->elem_type == ELEM_TYPE_OUTPUT) {
		src_id = 1;
	} else {
		dev_err(comp->dev, "%s: invalid element: %d\n",
			__func__, elem->elem_id->id);
		return -EINVAL;
	}

	list_for_each_safe(node, next, &elem->list) {
		elem_ifaces = list_entry(node, struct cc_iface_list, iface_list);
		break;
	}

	if (!elem_ifaces)
		return 0;

	if (!val)
		snd_soc_dapm_mixer_update_power(w->dapm, kcontrol, val, NULL);

	list_for_each_safe(node, next, &elem_ifaces->iface->path_list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (path_list->sink_id == elem->elem_id->id &&
			path_list->src_id == src_id) {
			found = 1;
			break;
		}
	}

	if (val && found)
		return 0;

	if (val) {
		rc = cc_add_path_for_elem(elem, src_id, sink_id);
	} else if (found) {
		rc = cc_del_path_for_elem(elem, src_id, sink_id);
	}

	if (val && !rc)
		snd_soc_dapm_mixer_update_power(w->dapm, kcontrol, val, NULL);

	return rc;
}

static int cc_elem_ctl_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_element *elem = &cc_priv->elems[w->shift];
	struct cc_path_list *path_list = NULL;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *elem_ifaces = NULL;
	int found = 0;

	list_for_each_safe(node, next, &elem->list) {
		elem_ifaces = list_entry(node,
				struct cc_iface_list, iface_list);
		break;
	}

	if (!elem_ifaces)
		return 0;

	list_for_each_safe(node, next, &elem_ifaces->iface->path_list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if ((elem->elem_type == ELEM_TYPE_MIX) &&
		    (elem->elem_id->id == path_list->sink_id) &&
		    (path_list->src_id == elem->src_id[mixer->shift].id)) {
			found = 1;
			break;
		}

		if ((elem->elem_type == ELEM_TYPE_SWITCH ||
		     elem->elem_type == ELEM_TYPE_INPUT ||
		     elem->elem_type == ELEM_TYPE_OUTPUT) &&
		    (elem->elem_id->id == path_list->sink_id)) {
			found = 1;
			break;
		}
	}

	ucontrol->value.integer.value[0] = found;

	return 0;
}

static int cc_trigger_uc(struct cc_element *elem, int on)
{
	struct list_head *node = NULL, *next = NULL;
	struct cc_interface *iface = NULL;
	struct cc_iface_list *elem_ifaces = NULL;
	uint32_t buf[CC_MAX_ELEM_LIST_SIZE] = {0};
	struct cc_uc_start_stop_t *uc = NULL;
	struct cc_path_list *path_list = NULL;
	struct cc_action_value_list *act_val = NULL;
	int i = 0;
	size_t size = 0;
	int rc = 0;
	uint32_t count = 0;
	uint8_t *tmp = NULL;

	list_for_each_safe(node, next, &elem->list) {
		elem_ifaces = list_entry(node,
			struct cc_iface_list, iface_list);
		break;
	}

	if (!elem_ifaces)
		return -EINVAL;

	iface = elem_ifaces->iface;

	if (!on) {
		if (iface->payload) {
			rc = cc_send_pkt_with_response(
				CC_CODEC_OPCODE_STOP_USECASE,
				iface->payload, iface->payload_size);
			kfree(iface->payload);
			iface->payload = NULL;
			iface->payload_size = 0;
			return rc;
		}

		return -EINVAL;
	}

	count = 0;
	list_for_each_safe(node, next, &iface->path_list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		buf[i++] = path_list->src_id;
		buf[i++] = path_list->sink_id;
		count++;

		if (count >= CC_MAX_NUM_ELEM_IN_LIST) {
			pr_err("%s: too many connections for uc: %d\n",
				__func__, iface->uc_info.uc_id);
			return -EINVAL;
		}
	}

	size = sizeof(struct cc_uc_start_stop_t) - sizeof(uc->payload) +
		i * sizeof(uint32_t);

	size += iface->action_size;
	uc = (struct cc_uc_start_stop_t *)kzalloc(size, GFP_KERNEL);
	if (!uc)
		return -ENOMEM;

	memcpy(uc, &iface->uc_info, sizeof(iface->uc_info));
	uc->direction = iface->uc_info.uc_id;
	uc->num_paths = count;
	memcpy(uc->payload, buf, i * sizeof(uint32_t));
	tmp = uc->payload + i * sizeof(uint32_t);


	count = 0;
	list_for_each_safe(node, next, &iface->action_value_list) {
		act_val = list_entry(node, struct cc_action_value_list, list);
		memcpy(tmp, &act_val->action_id, sizeof(act_val->action_id));
		tmp += sizeof(act_val->action_id);
		if (act_val->action_type == ACTION_TYPE_ENUM) {
			memcpy(tmp, &act_val->param.enumerated,
				sizeof(act_val->param.enumerated));
			tmp += sizeof(act_val->param.enumerated);
		} else {
			memcpy(tmp, act_val->param.char_array,
				act_val->size - sizeof(uint32_t));
			tmp += (act_val->size - sizeof(uint32_t));
		}
		count++;
	}
	uc->num_action = count;

	rc = cc_send_pkt_with_response(CC_CODEC_OPCODE_START_USECASE,
		uc, size);

	if (!rc) {
		iface->payload = uc;
		iface->payload_size = size;
	}

	return rc;
}

static void cc_stop_all_pb_stream(struct cc_codec_priv *cc_priv)
{
	int i = 0;
	struct cc_element *elem = NULL;

	if (!cc_priv || !cc_priv->elems) {
		pr_err("%s: driver data is null\n", __func__);
		return;
	}

	for (i = 0; i < cc_priv->num_elem; i++) {
		elem = &cc_priv->elems[i];
		if (elem->elem_type == ELEM_TYPE_STREAM_OUT) {
			pr_info("%s: powering down stream: %s\n",
					__func__, elem->s_name);
			cc_trigger_uc(elem, 0);
		}
	}
}

static int cc_widget_ev_func(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct cc_element *elem = &cc_priv->elems[w->shift];

	switch (event) {
		case SND_SOC_DAPM_PRE_PMU:
			break;
		case SND_SOC_DAPM_POST_PMU:
			if (w->id == snd_soc_dapm_aif_out ||
				w->id == snd_soc_dapm_aif_in)
				return cc_trigger_uc(elem, 1);
			break;
		case SND_SOC_DAPM_PRE_PMD:
			if (w->id == snd_soc_dapm_aif_out ||
				w->id == snd_soc_dapm_aif_in)
				return cc_trigger_uc(elem, 0);
			break;
		case SND_SOC_DAPM_POST_PMD:
			break;
	}
	return 0;
}

static void cc_widget_populate(struct cc_element *elem,
	enum snd_soc_dapm_type id, uint32_t num_kcontrol, int index)
{
	elem->widget.id = id;
	elem->widget.name = elem->elem_id->name;
	elem->widget.reg = SND_SOC_NOPM;
	elem->widget.mask = 1;
	elem->widget.shift = index;
	elem->widget.on_val = 1;
	elem->widget.off_val = 0;
	elem->widget.kcontrol_news = elem->kcontrol;
	elem->widget.num_kcontrols = num_kcontrol;
	elem->widget.event = cc_widget_ev_func;
	elem->widget.event_flags = SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD |
				SND_SOC_DAPM_POST_PMD;
}

static void cc_widget_populate_ip_op(struct cc_element *elem,
	enum snd_soc_dapm_type id, uint32_t num_kcontrol, int index)
{
	elem->widget_ip_op.id = id;
	elem->widget_ip_op.name = elem->s_name;
	elem->widget_ip_op.reg = SND_SOC_NOPM;
	elem->widget_ip_op.mask = 1;
	elem->widget_ip_op.shift = index;
	elem->widget_ip_op.on_val = 1;
	elem->widget_ip_op.off_val = 0;
	elem->widget_ip_op.kcontrol_news = elem->kcontrol;
	elem->widget_ip_op.num_kcontrols = num_kcontrol;
	elem->widget_ip_op.event = cc_widget_ev_func;
	elem->widget_ip_op.event_flags = SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD |
				SND_SOC_DAPM_POST_PMD;
}

static int cc_add_mux_element(struct cc_element *elem, int index)
{
	int i = 0;
	uint32_t items = 0;

	elem->kcontrol = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!elem->kcontrol)
		return -ENOMEM;

	items = elem->num_src + 1;
	elem->texts = kzalloc(items * sizeof(char *), GFP_KERNEL);
	if (!elem->texts)
		return -ENOMEM;

	pr_debug("%s:adding element %s: %d\n",
		__func__, elem->elem_id->name, elem->num_src);
	elem->texts[0] = "ZERO";
	for (i = 1; i < items; i++) {
		pr_debug("adding: %s\n", elem->src_id[i - 1].name);
		elem->texts[i] = elem->src_id[i - 1].name;
	}

	elem->elem_enum.reg = SND_SOC_NOPM;
	elem->elem_enum.items = items;
	elem->elem_enum.texts = (const char *const *)elem->texts;
	elem->elem_enum.mask = items ? roundup_pow_of_two(items) - 1 : 0;

	elem->kcontrol->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	elem->kcontrol->name = elem->elem_id->name;
	elem->kcontrol->info = snd_soc_info_enum_double;
	elem->kcontrol->get = cc_elem_ctl_mux_get;
	elem->kcontrol->put = cc_elem_ctl_mux_put;
	elem->kcontrol->private_value = (unsigned long)&elem->elem_enum;

	cc_widget_populate(elem, snd_soc_dapm_mux, 1, index);

	return 0;
}

static int cc_add_mixer_element(struct cc_element *elem, int index)
{
	int i = 0;
	uint32_t num_kcontrol = 0;

	num_kcontrol = elem->num_src;
	elem->mixer = kzalloc(num_kcontrol * sizeof(struct soc_mixer_control),
					GFP_KERNEL);
	if (!elem->mixer)
		return -ENOMEM;

	elem->kcontrol = kzalloc(num_kcontrol * sizeof(struct snd_kcontrol_new),
					GFP_KERNEL);
	if (!elem->kcontrol)
		return -ENOMEM;

	for (i = 0; i < num_kcontrol; i++) {
		elem->mixer[i].reg = SND_SOC_NOPM;
		elem->mixer[i].rreg = SND_SOC_NOPM;
		elem->mixer[i].shift = i;
		elem->mixer[i].rshift = i;
		elem->mixer[i].max = 1;
		elem->mixer[i].platform_max = 1;
		elem->mixer[i].invert = 0;
		elem->mixer[i].autodisable = 0;

		elem->kcontrol[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		elem->kcontrol[i].name = elem->src_id[i].name;
		elem->kcontrol[i].info = snd_soc_info_volsw;
		elem->kcontrol[i].get = cc_elem_ctl_get;
		elem->kcontrol[i].put = cc_elem_ctl_put;
		elem->kcontrol[i].private_value = (unsigned long)&elem->mixer[i];
	}

	cc_widget_populate(elem, snd_soc_dapm_mixer, num_kcontrol, index);

	return 0;
}

static int cc_add_switch_element(struct cc_element *elem, int index)
{
	if (!elem)
		return -EINVAL;

	elem->mixer = kzalloc(sizeof(struct soc_mixer_control), GFP_KERNEL);
	if (!elem->mixer)
		return -ENOMEM;

	elem->kcontrol = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!elem->kcontrol)
		return -ENOMEM;

	elem->mixer->reg = SND_SOC_NOPM;
	elem->mixer->rreg = SND_SOC_NOPM;
	elem->mixer->shift = 0;
	elem->mixer->rshift = 0;
	elem->mixer->max = 1;
	elem->mixer->platform_max = 1;
	elem->mixer->invert = 0;
	elem->mixer->autodisable = 0;

	elem->kcontrol->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	elem->kcontrol->name = "Switch";
	elem->kcontrol->info = snd_soc_info_volsw;
	elem->kcontrol->get = cc_elem_ctl_get;
	elem->kcontrol->put = cc_elem_ctl_put;
	elem->kcontrol->private_value = (unsigned long)elem->mixer;

	cc_widget_populate(elem, snd_soc_dapm_switch, 1, index);

	return 0;
}

static int cc_add_input_element(struct cc_element *elem, int index)
{
	int rc = 0;

	if (!elem)
		return -EINVAL;

	strlcpy(elem->s_name, elem->elem_id->name, CC_MAX_STREAM_STRLEN);
	strlcat(elem->elem_id->name, "_S", CC_MAX_STRLEN);
	rc = cc_add_switch_element(elem, index);
	if (rc)
		return rc;

	cc_widget_populate_ip_op(elem, snd_soc_dapm_input, 0, index);

	return 0;
}

static int cc_add_output_element(struct cc_element *elem, int index)
{
	int rc = 0;

	if (!elem)
		return -EINVAL;

	strlcpy(elem->s_name, elem->elem_id->name, CC_MAX_STREAM_STRLEN);
	strlcat(elem->elem_id->name, "_S", CC_MAX_STRLEN);
	rc = cc_add_switch_element(elem, index);
	if (rc)
		return rc;

	cc_widget_populate_ip_op(elem, snd_soc_dapm_output, 0, index);

	return 0;
}

static int cc_add_stream_element(struct cc_element *elem, int index)
{
	if (!elem)
		return -EINVAL;

	strlcpy(elem->s_name, elem->elem_id->name,
		CC_MAX_STREAM_STRLEN);
	if (elem->elem_type == ELEM_TYPE_STREAM_IN) {
		cc_widget_populate(elem, snd_soc_dapm_aif_out, 0, index);
		strlcat(elem->s_name, " Capture", sizeof(elem->s_name));
	} else {
		cc_widget_populate(elem, snd_soc_dapm_aif_in, 0, index);
		strlcat(elem->s_name, " Playback", sizeof(elem->s_name));
	}

	elem->widget.sname = elem->s_name;

	return 0;
}

static int cc_parse_element(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_elem, uint32_t *parsed_len)
{
	int i = 0;
	uint32_t elem_type = 0;
	uint32_t num_src = 0;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;
	struct cc_element *elem = NULL;
	cc_add_elem_func *elem_func = NULL;
	int ret = 0;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	if (num_elem == 0 || num_elem > CC_NUM_ELEM_MAX) {
		pr_err("%s: requested elements %d not in range of 0 and %d\n",
				__func__, num_elem, CC_NUM_ELEM_MAX);
		return -EINVAL;
	}

	cc_priv->elems = kzalloc(num_elem * sizeof(struct cc_element),
					GFP_KERNEL);
	if (!cc_priv->elems)
		return -ENOMEM;

	cc_priv->num_elem = num_elem;
	*parsed_len = 0;

	for (i = 0; i < num_elem; i++) {
		elem = &cc_priv->elems[i];
		elem_type = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_parse_id_name(cc_priv, tmp, &tmp_len);
		if (ret)
			return ret;
		tmp += tmp_len;
		*parsed_len += tmp_len;
		pr_debug("parsing element: %s\n",
			cc_priv->id_name[cc_priv->num_id - 1].name);

		switch (elem_type) {
		case ELEM_TYPE_MUX:
			num_src =  *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);
			elem_func = cc_add_mux_element;
			break;
		case ELEM_TYPE_MIX:
			num_src =  *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);
			elem_func = cc_add_mixer_element;
			break;
		case ELEM_TYPE_SWITCH:
			num_src = 0;
			elem_func = cc_add_switch_element;
			break;
		case ELEM_TYPE_INPUT:
			num_src = 0;
			cc_priv->num_routes++;
			elem_func = cc_add_input_element;
			break;
		case ELEM_TYPE_OUTPUT:
			num_src = 0;
			cc_priv->num_routes++;
			elem_func = cc_add_output_element;
			break;
		case ELEM_TYPE_STREAM_IN:
		case ELEM_TYPE_STREAM_OUT:
			num_src = 0;
			elem_func = cc_add_stream_element;
			break;
		case ELEM_TYPE_MAX:
		default:
			pr_err("%s: unsupported element type: %d\n",
					__func__, elem_type);
			return -EINVAL;
			break;
		}

		elem->elem_type = elem_type;
		elem->elem_id = &cc_priv->id_name[cc_priv->num_id - 1];
		elem->num_src = num_src;

		if (num_src) {
			ret = cc_parse_elem_src(cc_priv, elem,
							tmp, &tmp_len);
			if (ret)
				return ret;

			tmp += tmp_len;
			*parsed_len += tmp_len;
		}

		if (!elem_func)
			return -EINVAL;

		ret = elem_func(elem, i);
		if (ret) {
			pr_err("%s: could not add element: %d\n");
			return ret;
		}

		INIT_LIST_HEAD(&elem->list);
	}

	return 0;
}

static inline int cc_act_is_volatile(uint32_t action_type)
{
	return (action_type % 2);
}

static int cc_action_set(uint32_t id, void *ptr, size_t size)
{
	struct cc_set_get_param_t *set_param = NULL;
	size_t param_sz = 0;
	int rc = 0;

	param_sz = sizeof(*set_param) - sizeof(set_param->payload) + size;

	set_param = (struct cc_set_get_param_t *)kzalloc(param_sz, GFP_KERNEL);

	if(!set_param)
		return -ENOMEM;

	set_param->action_id = id;
	memcpy(set_param->payload, ptr, size);

	rc = cc_send_pkt_with_response(CC_CODEC_OPCODE_SET_PARAM,
					set_param, param_sz);

	kfree(set_param);
	set_param = NULL;
	return rc;
}

static int cc_action_get(uint32_t id, void *ptr, size_t size)
{
	struct cc_set_get_param_t get_param;
	struct cc_set_get_param_t *resp = 0;
	size_t resp_size = 0;
	int rc = 0;

	memset(&get_param, 0, sizeof(get_param));
	get_param.action_id = id;
	rc = cc_pktzr_send_packet(CC_CODEC_OPCODE_GET_PARAM,
		&get_param, sizeof(get_param), (void **)&resp, &resp_size);

	if (rc)
		return rc;

	memcpy(ptr, resp, size);

	return 0;
}

static int cc_action_ctl_enum_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_enum *tmp_enum = (struct soc_enum *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[tmp_enum->shift_l];
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_action_value_list *act_val = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	uint32_t value = 0;
	int rc = 0;

	if (cc_act_is_volatile(act->action_type)) {
		rc = cc_action_get(act->act_id->id, &value, sizeof(uint32_t));
		if (!rc)
			ucontrol->value.enumerated.item[0] = value;
		return rc;
	}

	list_for_each_safe(node, next, &act->list) {
		act_ifaces = list_entry(node, struct cc_iface_list, iface_list);
		break;
	}

	if (!act_ifaces)
		return 0;

	list_for_each_safe(node, next, &act_ifaces->iface->action_value_list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			break;
		}
	}

	if (found)
		ucontrol->value.enumerated.item[0] = act_val->param.enumerated;
	else
		ucontrol->value.enumerated.item[0] = 0;

	return 0;
}

static int cc_action_ctl_enum_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_enum *tmp_enum = (struct soc_enum *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[tmp_enum->shift_l];
	struct cc_action_value_list *act_val = NULL;
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_interface *iface = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	uint32_t value = 0;

	list_for_each_safe(node, next, &act->list) {
		act_ifaces = list_entry(node, struct cc_iface_list, iface_list);
		break;
	}

	if (!act_ifaces)
		return 0;

	iface = act_ifaces->iface;

	if (cc_act_is_volatile(act->action_type)) {
		value = ucontrol->value.enumerated.item[0];
		return cc_action_set(act->act_id->id, &value, sizeof(uint32_t));
	}

	list_for_each_safe(node, next, &act_ifaces->iface->action_value_list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			break;
		}
	}

	if (!found)
		cc_add_enum_action_val(act, ucontrol->value.enumerated.item[0]);
	else
		cc_modify_enum_action_val(act, ucontrol->value.enumerated.item[0]);

	return 0;
}

static int cc_action_ctl_array_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[mixer->shift];
	struct cc_action_value_list *act_val = NULL;
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	size_t size = 0;
	void *ptr = NULL;
	int i = 0;

	list_for_each_safe(node, next, &act->list) {
		act_ifaces = list_entry(node, struct cc_iface_list, iface_list);
		break;
	}

	if (!act_ifaces)
		return 0;

	if (cc_act_is_volatile(act->action_type)) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY_VOLATILE:
			size = act->num_params * sizeof(uint8_t);
			ptr = (void *)&ucontrol->value.bytes.data[0];
			break;
		case ACTION_TYPE_INT_ARRAY_VOLATILE:
		case ACTION_TYPE_UINT_ARRAY_VOLATILE:
			size = act->num_params * sizeof(int32_t);
			ptr = (void *)&ucontrol->value.integer.value[0];
			break;
		default:
			return -EINVAL;
			break;
		}
		return cc_action_get(act->act_id->id, ptr, size);
	}

	list_for_each_safe(node, next, &act_ifaces->iface->action_value_list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			break;
		}
	}

	if (!found)
		return 0;

	for (i = 0; i < act->num_params; i++) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY:
			ucontrol->value.bytes.data[i] =
				act_val->param.char_array[i];
			break;
		case ACTION_TYPE_INT_ARRAY:
			ucontrol->value.integer.value[i] =
				act_val->param.int_array[i];
			break;
		case ACTION_TYPE_UINT_ARRAY:
			ucontrol->value.integer.value[i] =
				act_val->param.uint_array[i];
			break;
		default:
			break;
		}
	}

	return 0;
}

static int cc_action_ctl_array_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[mixer->shift];
	struct cc_action_value_list *act_val = NULL;
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_interface *iface = NULL;
	struct cc_iface_list *act_ifaces = NULL;
	size_t size = 0;
	void *ptr = NULL;

	list_for_each_safe(node, next, &act->list) {
		act_ifaces = list_entry(node, struct cc_iface_list, iface_list);
		break;
	}

	if (!act_ifaces)
		return 0;

	iface = act_ifaces->iface;

	if (cc_act_is_volatile(act->action_type)) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY_VOLATILE:
			size = act->num_params * sizeof(uint8_t);
			ptr = (void *)&ucontrol->value.bytes.data[0];
			break;
		case ACTION_TYPE_INT_ARRAY_VOLATILE:
		case ACTION_TYPE_UINT_ARRAY_VOLATILE:
			size = act->num_params * sizeof(int32_t);
			ptr = (void *)&ucontrol->value.integer.value[0];
			break;
		default:
			return -EINVAL;
			break;
		}
		return cc_action_set(act->act_id->id, ptr, size);
	}

	if (act_ifaces->iface->action_value_list.prev &&
		act_ifaces->iface->action_value_list.next) {
		list_for_each_safe(node, next, &act_ifaces->iface->action_value_list) {
			act_val = list_entry(node,
					struct cc_action_value_list, list);
			if (act_val && act_val->action_id == act->act_id->id) {
				found = 1;
				break;
			}
		}
	}

	if (!act_val)
		return 0;

	switch (act->action_type) {
	case ACTION_TYPE_CHAR_ARRAY:
		size = act_val->num_params * sizeof(uint8_t);
		ptr = &ucontrol->value.bytes.data[0];
		break;
	case ACTION_TYPE_INT_ARRAY:
		size = act_val->num_params * sizeof(int32_t);
		ptr = &ucontrol->value.integer.value[0];
		break;
	case ACTION_TYPE_UINT_ARRAY:
		size = act_val->num_params * sizeof(uint32_t);
		ptr = &ucontrol->value.integer.value[0];
		break;
	default:
		return -EINVAL;
		break;
	}


	if (!found)
		cc_add_array_action_val(act, ptr, size);
	else
		cc_modify_array_action_val(act, ptr, size);

	return 0;
}

static int cc_create_enum_action(struct cc_codec_priv *cc_priv,
	struct cc_action *action, uint8_t *payload,
	uint32_t *parsed_len, int index)
{
	int i = 0;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;
	int ret = 0;

	*parsed_len = 0;
	action->enum_str = kzalloc(
		action->num_params * sizeof(*(action->enum_str)), GFP_KERNEL);
	if (!action->enum_str)
		return -ENOMEM;

	action->texts = kzalloc(
		action->num_params * sizeof(char *), GFP_KERNEL);
	if (!action->texts)
		return -ENOMEM;;

	for (i = 0; i < action->num_params; i++) {
		ret = cc_parse_name(action->enum_str[i], tmp, &tmp_len);
		if (ret)
			return ret;
		tmp += tmp_len;
		*parsed_len += tmp_len;
		action->texts[i] = action->enum_str[i];
	}

	action->action_enum.reg = SND_SOC_NOPM;
	action->action_enum.shift_l = index;
	action->action_enum.shift_r = index;
	action->action_enum.items = action->num_params;
	action->action_enum.texts = (const char *const *)action->texts;
	action->action_enum.mask = action->num_params ?
			roundup_pow_of_two(action->num_params) - 1 : 0;

	action->kcontrol.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	action->kcontrol.name = action->act_id->name;
	action->kcontrol.info = snd_soc_info_enum_double;
	action->kcontrol.get = cc_action_ctl_enum_get;
	action->kcontrol.put = cc_action_ctl_enum_put;
	action->kcontrol.private_value = (unsigned long)&action->action_enum;

	return 0;
}

static int cc_create_array_action(struct cc_codec_priv *cc_priv,
					struct cc_action *action, int index)
{
	action->mixer.reg = SND_SOC_NOPM;
	action->mixer.shift = index;
	action->mixer.rshift = index;
	action->mixer.max = action->param_max;
	action->mixer.count = action->num_params;
	action->mixer.platform_max = action->param_max;
	action->mixer.invert = 0;

	action->kcontrol.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	action->kcontrol.name = action->act_id->name;
	action->kcontrol.info = snd_soc_info_multi_ext;
	action->kcontrol.get = cc_action_ctl_array_get;
	action->kcontrol.put = cc_action_ctl_array_put;
	action->kcontrol.private_value = (unsigned long)&action->mixer;

	return 0;
}

static int cc_parse_action(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_action, uint32_t *parsed_len)
{
	int ret = 0;
	struct cc_action *action = NULL;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;
	int i = 0;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	*parsed_len = 0;
	if (num_action == 0 || num_action > CC_NUM_ACTIONS_MAX)
		return -EINVAL;

	cc_priv->num_action = num_action;
	cc_priv->actions = kzalloc(num_action * sizeof(struct cc_action),
					GFP_KERNEL);
	if (!cc_priv->actions)
		return -ENOMEM;

	for (i = 0; i < num_action; i++) {
		action = &cc_priv->actions[i];
		action->action_type = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_parse_id_name(cc_priv, tmp, &tmp_len);
		if (ret)
			return ret;
		tmp += tmp_len;
		*parsed_len += tmp_len;

		action->num_params = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		action->act_id = &cc_priv->id_name[cc_priv->num_id - 1];

		switch (action->action_type) {
		case ACTION_TYPE_ENUM:
		case ACTION_TYPE_ENUM_VOLATILE:
			ret = cc_create_enum_action(cc_priv, action,
							tmp, &tmp_len, i);
			if (ret)
				return ret;
			tmp += tmp_len;
			*parsed_len += tmp_len;
			break;
		case ACTION_TYPE_CHAR_ARRAY:
		case ACTION_TYPE_CHAR_ARRAY_VOLATILE:
		case ACTION_TYPE_INT_ARRAY:
		case ACTION_TYPE_INT_ARRAY_VOLATILE:
		case ACTION_TYPE_UINT_ARRAY:
		case ACTION_TYPE_UINT_ARRAY_VOLATILE:
			action->param_min = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			action->param_max = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			action->def = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_create_array_action(cc_priv, action, i);
			if (ret)
				return ret;
			break;
		case ACTION_TYPE_MAX:
		default:
			pr_err("%s: unsupported action type: %d\n",
					__func__, action->action_type);
			return -EINVAL;
			break;
		}

		INIT_LIST_HEAD(&action->list);
	}

	return 0;
}

static int cc_find_name_from_id(struct cc_codec_priv *cc_priv,
		uint32_t id, char **str)
{
	int i = 0;

	if (!cc_priv || !str)
		return -EINVAL;

	for (i = 0; i < cc_priv->num_id; i++) {
		if (cc_priv->id_name[i].id == id)
			break;
	}

	if (i == cc_priv->num_id) {
		pr_err("%s: invalid id %d\n", __func__, id);
		return -EINVAL;
	}

	*str = cc_priv->id_name[i].name;

	return 0;
}

static int cc_find_elem_from_id(struct cc_codec_priv *cc_priv,
		uint32_t id, struct cc_element **elem)
{
	int i = 0;

	if (!cc_priv || !elem)
		return -EINVAL;

	for (i = 0; i < cc_priv->num_elem; i++) {
		if (cc_priv->elems[i].elem_id->id == id)
			break;
	}

	if (i == cc_priv->num_elem) {
		pr_err("%s: invalid element id %d\n", __func__, id);
		return -EINVAL;
	}

	*elem = &cc_priv->elems[i];

	return 0;
}

static int cc_find_action_from_id(struct cc_codec_priv *cc_priv,
		uint32_t id, struct cc_action **act)
{
	int i = 0;

	if (!cc_priv || !act)
		return -EINVAL;

	for (i = 0; i < cc_priv->num_action; i++) {
		if (cc_priv->actions[i].act_id->id == id)
			break;
	}

	if (i == cc_priv->num_action) {
		pr_err("%s: invalid action id %d\n", __func__, id);
		return -EINVAL;
	}

	*act = &cc_priv->actions[i];

	return 0;
}

static int cc_parse_route(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_route, uint32_t *parsed_len)
{
	int ret = 0;
	uint8_t *tmp = payload;
	struct cc_element *elem = NULL;
	uint32_t route_type = 0;
	uint32_t id = 0;
	char *str = NULL;
	int i = 0;
	int j = 0;
	uint32_t total_routes = 0;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	if (num_route == 0 || num_route > CC_NUM_ROUTES_MAX)
		return -EINVAL;

	total_routes = num_route + cc_priv->num_routes;
	cc_priv->audio_map = kzalloc(
		total_routes * sizeof(struct snd_soc_dapm_route), GFP_KERNEL);
	if (!cc_priv->audio_map)
		return -ENOMEM;

	*parsed_len = 0;

	for (i = 0; i < num_route; i++) {
		route_type = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		id = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_find_elem_from_id(cc_priv, id, &elem);
		if (ret)
			return ret;

		if (elem->elem_type == ELEM_TYPE_OUTPUT)
			route_type = ROUTE_TYPE_SWITCH;

		cc_priv->audio_map[i].sink = elem->elem_id->name;

		switch (route_type) {
		case ROUTE_TYPE_CONTROL:
			id = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_find_name_from_id(cc_priv, id, &str);
			if (ret)
				return ret;
			break;
		case ROUTE_TYPE_SWITCH:
			str = "Switch";
			break;
		case ROUTE_TYPE_DUMMY:
			str = NULL;
			break;
		case ROUTE_TYPE_MAX:
		default:
			pr_err("%s: unsupported route type: %d\n",
					__func__, route_type);
			return -EINVAL;
			break;
		}

		cc_priv->audio_map[i].control = str;

		id = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_find_elem_from_id(cc_priv, id, &elem);
		if (ret)
			return ret;

		cc_priv->audio_map[i].source = elem->elem_id->name;
	}

	for (j = 0; j < cc_priv->num_elem; j++) {
		elem = &cc_priv->elems[j];
		if (elem->elem_type == ELEM_TYPE_OUTPUT) {
			cc_priv->audio_map[i].sink = elem->s_name;
			cc_priv->audio_map[i].control = NULL;
			cc_priv->audio_map[i].source = elem->elem_id->name;
			i++;
		}

		if (elem->elem_type == ELEM_TYPE_INPUT) {
			cc_priv->audio_map[i].sink = elem->elem_id->name;
			cc_priv->audio_map[i].control = "Switch";
			cc_priv->audio_map[i].source = elem->s_name;
			i++;
		}

	}

	cc_priv->num_routes = total_routes;

	return 0;
}

static int cc_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params, struct snd_soc_dai *dai)
{
	struct cc_codec_priv *cc_priv = (struct cc_codec_priv *)
					dev_get_drvdata(dai->component->dev);
	int id = dai->driver->id - 1;
	struct cc_interface *iface = &cc_priv->iface[id];

	iface->uc_info.sample_rate = params_rate(hw_params);
	iface->uc_info.channels = params_channels(hw_params);
	iface->uc_info.bit_width = params_width(hw_params);

	return 0;
}

static struct snd_soc_dai_ops cc_dai_ops = {
	.hw_params = cc_dai_hw_params,
};

static int cc_parse_interface(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_intf, uint32_t *parsed_len)
{
	int ret = 0;
	uint8_t *tmp = payload;
	uint32_t id = 0;
	int i = 0;
	int j = 0;
	struct cc_interface *iface = NULL;
	uint32_t num_elements = 0;
	uint32_t num_actions = 0;
	struct cc_element *elem = NULL;
	struct cc_action *act = NULL;
	struct snd_soc_pcm_stream *stream = NULL;
	struct cc_iface_list *ifaces = NULL;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	if (num_intf == 0 || num_intf > CC_NUM_IFACE_MAX)
		return -EINVAL;

	cc_priv->iface = kzalloc(
		num_intf * sizeof(struct cc_interface), GFP_KERNEL);
	if (!cc_priv->iface)
		return -ENOMEM;

	cc_priv->dai = kzalloc(
		num_intf * sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!cc_priv->dai)
		return -ENOMEM;

	*parsed_len = 0;
	cc_priv->num_iface = num_intf;

	for (i = 0; i < num_intf; i++) {
		iface = &cc_priv->iface[i];
		id = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		iface->uc_info.uc_id = i;

		ret = cc_find_elem_from_id(cc_priv, id, &elem);
		if (ret) {
			pr_err("%s: Invalid stream id %d in interface\n",
				__func__, id);
			return ret;
		}

		if (elem->elem_type == ELEM_TYPE_STREAM_OUT) {
			stream = &cc_priv->dai[i].playback;
		} else if (elem->elem_type == ELEM_TYPE_STREAM_IN) {
			stream = &cc_priv->dai[i].capture;
		} else {
			pr_err("%s: %d Element is not stream\n", __func__, id);
			return -EILSEQ;
		}
		stream->stream_name = elem->s_name;

		stream->formats = *(uint64_t *)tmp;
		tmp += sizeof(uint64_t);
		*parsed_len += sizeof(uint64_t);

		stream->rates = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->rate_min = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->rate_max = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);
		stream->channels_min = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->channels_max = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		num_elements = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		num_actions = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		cc_priv->dai[i].name = elem->elem_id->name;
		cc_priv->dai[i].id = i + 1;
		cc_priv->dai[i].ops = &cc_dai_ops;

		INIT_LIST_HEAD(&cc_priv->iface[i].path_list);
		INIT_LIST_HEAD(&cc_priv->iface[i].action_value_list);

		for (j = 0; j < num_elements; j++) {
			id = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_find_elem_from_id(cc_priv, id, &elem);
			if (ret) {
				pr_err("%s: iface:%d, invalid elem:0x%x\n",
					__func__, i, id);
				return ret;
			}

			ifaces = kzalloc(sizeof(struct cc_iface_list),
							GFP_KERNEL);
			if (!ifaces)
				return -ENOMEM;
			ifaces->iface = &cc_priv->iface[i];

			list_add_tail(&ifaces->iface_list, &elem->list);
		}

		for (j = 0; j < num_actions; j++) {
			id = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_find_action_from_id(cc_priv, id, &act);
			if (ret) {
				pr_err("%s: iface:%d, invalid action:0x%x\n",
					__func__, i, id);
				return ret;
			}

			ifaces = kzalloc(sizeof(struct cc_iface_list),
							GFP_KERNEL);
			if (!ifaces)
				return -ENOMEM;
			ifaces->iface = &cc_priv->iface[i];

			list_add_tail(&ifaces->iface_list, &act->list);
		}
	}

	return 0;
}

static int cc_parse_plat_info(struct cc_codec_priv *cc_priv,
		struct device *dev, uint8_t *plat_info, uint32_t size)
{
	struct cc_plat_info_t *pinfo = NULL;
	uint8_t *tmp = NULL;
	uint32_t tmp_len = 0;
	int ret = 0;

	if (size < CC_MIN_IPC_PAYLOAD_SIZE) {
		pr_err("%s: Invalid payload\n", __func__);
		return -EINVAL;
	}

	pinfo = (struct cc_plat_info_t *)plat_info;

	dev_dbg(dev, "%s: id=%d, dev=%d, elem=%d, act=%d, route=%d, iface=%d\n",
		__func__, pinfo->num_id, pinfo->num_dev, pinfo->num_elem,
		pinfo->num_action, pinfo->num_route, pinfo->num_intf);

	cc_priv->id_name = kzalloc(pinfo->num_id *
				sizeof(struct cc_id_name_map), GFP_KERNEL);
	if (!cc_priv->id_name)
		return -ENOMEM;

	tmp = pinfo->payload;

	ret = cc_parse_reg_region(cc_priv, dev, tmp,
				pinfo->num_dev, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;
	dev_dbg(dev, "%s: reg_region %u\n", __func__, tmp_len);

	ret = cc_parse_element(cc_priv, tmp, pinfo->num_elem, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;
	dev_dbg(dev, "%s: element %u\n", __func__, tmp_len);

	ret = cc_parse_action(cc_priv, tmp, pinfo->num_action, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;
	dev_dbg(dev, "%s: action %u\n", __func__, tmp_len);

	ret = cc_parse_route(cc_priv, tmp, pinfo->num_route, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;
	dev_dbg(dev, "%s: route %u\n", __func__, tmp_len);

	ret = cc_parse_interface(cc_priv, tmp, pinfo->num_intf, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;
	dev_dbg(dev, "%s: interface %u\n", __func__, tmp_len);

	return 0;
}

static int cc_soc_codec_probe(struct snd_soc_component *component)
{
	int ret = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(component->dev);
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);
	struct device *dev = component->dev;
	struct cc_element *elem = NULL;
	int i = 0;

	for (i = 0; i < cc_priv->num_elem; i++) {
		ret = snd_soc_dapm_new_controls(dapm,
				&cc_priv->elems[i].widget, 1);
		if (ret < 0) {
			dev_err(dev, "%s: failed to add dapm control, ret:%d\n",
				__func__, ret);
			return ret;
		}

		if (cc_priv->elems[i].elem_type == ELEM_TYPE_INPUT ||
			cc_priv->elems[i].elem_type == ELEM_TYPE_OUTPUT) {
			ret = snd_soc_dapm_new_controls(dapm,
				&cc_priv->elems[i].widget_ip_op, 1);
			if (ret < 0) {
				dev_err(dev, "%s: fail to add ip/op dapm: %d\n",
					__func__, ret);
				return ret;
			}
		}
	}

	ret = snd_soc_dapm_add_routes(dapm,
			cc_priv->audio_map, cc_priv->num_routes);
	if (ret < 0) {
		dev_err(dev, "%s: failed to add routes, ret:%d\n",
				__func__, ret);
		return ret;
	}

	ret = snd_soc_dapm_new_widgets(dapm->card);
	if (ret < 0) {
		dev_err(dev, "%s: failed to add widgets, ret:%d\n",
				__func__, ret);
		return ret;
	}

	for (i = 0; i < cc_priv->num_action; i++) {
		ret = snd_soc_add_component_controls(component,
			&cc_priv->actions[i].kcontrol, 1);
		if (ret < 0) {
			dev_err(dev, "%s: failed to add snd ctrls, ret:%d\n",
					__func__, ret);
			return ret;
		}
	}

	for (i = 0; i < cc_priv->num_elem; i++) {
		elem = &cc_priv->elems[i];
		if (elem->elem_type == ELEM_TYPE_INPUT ||
		    elem->elem_type == ELEM_TYPE_OUTPUT ||
		    elem->elem_type == ELEM_TYPE_STREAM_IN ||
		    elem->elem_type == ELEM_TYPE_STREAM_OUT) {
			snd_soc_dapm_ignore_suspend(dapm, elem->s_name);
		}
	}

	snd_soc_dapm_sync(dapm);

	return 0;
}

static void cc_soc_codec_remove(struct snd_soc_component *component)
{
	return;
}

static int cc_get_plat_info(uint8_t **ptr, size_t *size)
{
	int rc = 0;
	if (!ptr || !size)
		return -EINVAL;

	rc = cc_pktzr_send_packet(CC_CODEC_OPCODE_GET_PLAT_INFO, NULL, 0,
		(void **)ptr, size);

	return rc;
}

static void cc_cleanup_iface(struct cc_interface *iface)
{
	struct cc_path_list *path = NULL;
	struct cc_action_value_list *act_val = NULL;
	struct list_head *next = NULL, *node = NULL;
	int type = 0;

	if (iface->path_list.prev && iface->path_list.next) {
		list_for_each_safe(node, next, &iface->path_list) {
			path = list_entry(node,
					struct cc_path_list, list);
			list_del(node);
			if (path)
				kfree(path);
		}
	}

	if (iface->action_value_list.prev && iface->action_value_list.next) {
		list_for_each_safe(node, next, &iface->action_value_list) {
			act_val = list_entry(node,
					struct cc_action_value_list, list);
			type = act_val ? act_val->action_type : -1;

			if (type == ACTION_TYPE_CHAR_ARRAY) {
				kfree(act_val->param.char_array);
				act_val->param.char_array = NULL;
			} else if (type == ACTION_TYPE_INT_ARRAY) {
				kfree(act_val->param.int_array);
				act_val->param.int_array = NULL;
			} else if (type == ACTION_TYPE_UINT_ARRAY) {
				kfree(act_val->param.uint_array);
				act_val->param.char_array = NULL;
			}
			list_del(node);
			if (act_val)
				kfree(act_val);
		}
	}

	memset(&iface->path_list, 0, sizeof(struct list_head));
	memset(&iface->action_value_list, 0, sizeof(struct list_head));
}

static void cc_cleanup_action(struct cc_action *action)
{
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *act_ifaces = NULL;

	if (action->act_id)
		action->act_id = NULL;

	if (action->enum_str) {
		kfree(action->enum_str);
		action->enum_str = NULL;
	}

	if (action->texts) {
		kfree(action->texts);
		action->texts = NULL;
	}

	if (action->list.prev && action->list.next) {
		list_for_each_safe(node, next, &action->list) {
			act_ifaces = list_entry(node,
						struct cc_iface_list, iface_list);
			list_del(node);
			if (act_ifaces)
				kfree(act_ifaces);
		}
	}

}

static void cc_cleanup_element(struct cc_element *element)
{
	struct list_head *node = NULL, *next = NULL;
	struct cc_iface_list *elem_ifaces = NULL;

	if (element->kcontrol) {
		kfree(element->kcontrol);
		element->kcontrol = NULL;
	}

	if (element->mixer) {
		kfree(element->mixer);
		element->mixer = NULL;
	}

	if (element->texts) {
		kfree(element->texts);
		element->texts = NULL;
	}

	if (element->list.prev && element->list.next) {
		list_for_each_safe(node, next, &element->list) {
			elem_ifaces = list_entry(node,
						struct cc_iface_list, iface_list);
			list_del(node);
			if (elem_ifaces)
				kfree(elem_ifaces);
		}
	}

	element->elem_id = NULL;

	element->src_id = NULL;
}

static void cc_cleanup(struct cc_codec_priv *cc_priv)
{
	int i = 0;

	if (!cc_priv)
		return;

	if (cc_priv->iface) {
		for (i = 0; i < cc_priv->num_iface; i++)
			cc_cleanup_iface(&cc_priv->iface[i]);
		kfree(cc_priv->iface);
		cc_priv->iface = NULL;
		kfree(cc_priv->dai);
		cc_priv->dai = NULL;
		cc_priv->num_iface = 0;
	}

	if (cc_priv->audio_map) {
		kfree(cc_priv->audio_map);
		cc_priv->audio_map = NULL;
		cc_priv->num_routes = 0;
	}

	if (cc_priv->actions) {
		for (i = 0; i < cc_priv->num_action; i++)
			cc_cleanup_action(&cc_priv->actions[i]);
		kfree(cc_priv->actions);
		cc_priv->actions = NULL;
		cc_priv->num_action = 0;
	}

	if (cc_priv->elems) {
		for (i = 0; i < cc_priv->num_elem; i++)
			cc_cleanup_element(&cc_priv->elems[i]);
		kfree(cc_priv->elems);
		cc_priv->elems = NULL;
		cc_priv->num_elem = 0;
	}

	if (cc_priv->reg_region) {
		cc_disable_reg_debugging(cc_priv);
		kfree(cc_priv->reg_region);
		cc_priv->reg_region = NULL;
		cc_priv->num_reg_region = 0;
	}

	if (cc_priv->id_name) {
		kfree(cc_priv->id_name);
		cc_priv->id_name = NULL;
		cc_priv->num_id = 0;
	}
}

static int cc_cdc_get_status(struct device *dev)
{

	struct cc_get_cdc_status_t *cdc_status = NULL;
	size_t resp_size = 0;
	int rc = 0;

	rc =  cc_pktzr_send_packet(CC_CODEC_OPCODE_GET_CODEC_STATUS, NULL, 0,
				(void **)&cdc_status, &resp_size);
	if (rc || (cdc_status->status == CC_CDC_STATUS_DOWN)) {
		pr_err("%s: cc codec is not up\n", __func__);
		return 0;
	}

	pr_info("%s: cc codec status %d\n", __func__, cdc_status->status);

	return rc;
}

static int cc_cdc_ssr_enable(struct device *dev, void *data)
{
	cc_pktzr_register_device();
	return cc_cdc_get_status(dev);
}

static void cc_cdc_ssr_disable(struct device *dev, void *data)
{
	cc_pktzr_deregister_device();
}

static const struct snd_event_ops cc_cdc_ssr_ops = {
	.enable = cc_cdc_ssr_enable,
	.disable = cc_cdc_ssr_disable,
};

static int cc_cdc_notifier_service_cb(struct notifier_block *this,
				      unsigned long opcode, void *ptr)
{
	struct cc_codec_priv *cc_priv = NULL;

	cc_priv = container_of(this, struct cc_codec_priv, cc_adsp_nb);
	if (!cc_priv)
		return NOTIFY_OK;

	if (opcode == AUDIO_NOTIFIER_SERVICE_DOWN)
		cc_stop_all_pb_stream(cc_priv);

	return NOTIFY_OK;
}

static int cc_cdc_probe(struct platform_device *pdev)
{
	struct cc_codec_priv *cc_priv = NULL;
	size_t plat_info_size = 0;
	uint8_t *plat_info = NULL;
	int ret = 0;

	cc_priv = devm_kzalloc(&pdev->dev, sizeof(struct cc_codec_priv),
					GFP_KERNEL);
	if (!cc_priv)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, cc_priv);

	ret = cc_pktzr_init(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: packet init failed: %d\n",
				__func__, ret);
		return ret;
	}

	ret = cc_get_plat_info(&plat_info, &plat_info_size);
	if (!plat_info || ret) {
		dev_err(&pdev->dev, "%s: cc payload not found\n", __func__);
		goto free_priv;
	}

	ret = cc_parse_plat_info(cc_priv, &pdev->dev,
				plat_info, plat_info_size);
	if (ret) {
		dev_err(&pdev->dev, "%s: parse plat info, failed: ret:%d\n",
				__func__, ret);
		goto free_priv;
	}

	ret = cc_enable_reg_debugging(cc_priv, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: enable register debug, failed: ret:%d\n", __func__,
			ret);
		goto free_priv;
	}

	cc_priv->cc_comp.probe = cc_soc_codec_probe;
	cc_priv->cc_comp.remove = cc_soc_codec_remove;
	cc_priv->cc_comp.name = CC_CODEC_STR;

	ret = snd_soc_register_component(&pdev->dev, &cc_priv->cc_comp,
				cc_priv->dai, cc_priv->num_iface);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: register codec failed, ret:%d\n",
				__func__, ret);
		goto free_priv;
	}

	ret = snd_event_client_register_v2(&pdev->dev, &cc_cdc_ssr_ops, NULL,
					AUDIO_NOTIFIER_CC_DOMAIN);
	if (!ret)
		snd_event_notify_v2(&pdev->dev, SND_EVENT_UP, AUDIO_NOTIFIER_CC_DOMAIN);
	else
		dev_err(&pdev->dev, "%s: Registration with SND event fwk failed ret = %d\n",
			__func__, ret);

	cc_priv->cc_adsp_nb.notifier_call = cc_cdc_notifier_service_cb;
	cc_priv->cc_adsp_nb.priority = 0;
	ret = audio_notifier_register(CC_CODEC_STR, AUDIO_NOTIFIER_ADSP_DOMAIN,
						&cc_priv->cc_adsp_nb);
	if (ret)
		dev_err(&pdev->dev, "%s: ADSP ssr register fail ret:%d\n",
			__func__, ret);

	return 0;

free_priv:
	cc_cleanup(cc_priv);
	cc_pktzr_deinit();
	devm_kfree(&pdev->dev, cc_priv);
	cc_priv = NULL;
	return ret;
}

static int cc_cdc_remove(struct platform_device *pdev)
{
	struct cc_codec_priv *cc_priv = dev_get_drvdata(&pdev->dev);

	audio_notifier_deregister(CC_CODEC_STR);
	cc_pktzr_deinit();
	snd_event_client_deregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	cc_cleanup(cc_priv);
	devm_kfree(&pdev->dev, cc_priv);
	cc_priv = NULL;

	return 0;
}

static const struct of_device_id cc_cdc_dt_match[] = {
	{.compatible = "qcom,cc-codec"},
	{}
};

static struct platform_driver cc_cdc_driver = {
	.driver = {
		.name = "cc-codec",
		.owner = THIS_MODULE,
		.of_match_table = cc_cdc_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = cc_cdc_probe,
	.remove = cc_cdc_remove,
};

static int __init cc_cdc_init(void)
{
	return platform_driver_register(&cc_cdc_driver);
}
module_init(cc_cdc_init);

static void __exit cc_cdc_exit(void)
{
	platform_driver_unregister(&cc_cdc_driver);
}
module_exit(cc_cdc_exit);

MODULE_DESCRIPTION("cc codec driver");
MODULE_LICENSE("GPL v2");
