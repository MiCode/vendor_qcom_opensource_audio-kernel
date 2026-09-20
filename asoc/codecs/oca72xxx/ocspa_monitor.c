/*
 * oca72xxx_monitor.c
 *
 * Copyright (c) 2025 OCS Technology CO., LTD
 *
 * Author: Wall <Wall@orient-chip.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include "ocspa.h"
#include "ocspa_log.h"
#include "ocspa_monitor.h"
#include "ocspa_fw_process.h"
#include "ocspa_device.h"

#define OCA_MONITOT_BIN_PARSE_VERSION	"V5.3.0"

#define OCA_GET_32_DATA(w, x, y, z) \
	((uint32_t)((((uint8_t)w) << 24) | (((uint8_t)x) << 16) | \
	(((uint8_t)y) << 8) | ((uint8_t)z)))

/****************************************************************************
 *
 * oca72xxx monitor bin check
 *
 ****************************************************************************/
static int oca_monitor_check_header_v_1_0_0(struct device *dev,
				char *data, uint32_t data_len)
{
	int i = 0;
	struct oca_bin_header *header = (struct oca_bin_header *)data;

	if (header->bin_data_type != DATA_TYPE_MONITOR_ANALOG) {
		OCA_DEV_LOGE(dev, "monitor data_type check error!");
		return -EINVAL;
	}

	if (header->bin_data_size != OCA_MONITOR_HDR_DATA_SIZE) {
		OCA_DEV_LOGE(dev, "monitor data_size error!");
		return -EINVAL;
	}

	if (header->data_byte_len != OCA_MONITOR_HDR_DATA_BYTE_LEN) {
		OCA_DEV_LOGE(dev, "monitor data_byte_len error!");
		return -EINVAL;
	}

	for (i = 0; i < OCA_MONITOR_DATA_VER_MAX; i++) {
		if (header->bin_data_ver == i) {
			OCA_LOGD("monitor bin_data_ver[0x%x]", i);
			break;
		}
	}
	if (i == OCA_MONITOR_DATA_VER_MAX)
		return -EINVAL;

	return 0;
}

static int oca_monitor_check_data_v1_size(struct device *dev,
				char *data, int32_t data_len)
{
	int32_t bin_header_len  = sizeof(struct oca_bin_header);
	int32_t monitor_header_len = sizeof(struct oca_monitor_header);
	int32_t monitor_data_len = sizeof(struct vmax_step_config);
	int32_t len = 0;
	struct oca_monitor_header *monitor_header = NULL;

	OCA_DEV_LOGD(dev, "enter");

	if (data_len < bin_header_len + monitor_header_len) {
		OCA_DEV_LOGE(dev, "bin len is less than oca_bin_header and monitoor_header,check failed");
		return -EINVAL;
	}

	monitor_header = (struct oca_monitor_header *)(data + bin_header_len);
	len = data_len - bin_header_len - monitor_header_len;
	if (len < monitor_header->step_count * monitor_data_len) {
		OCA_DEV_LOGE(dev, "bin data len is not enough,check failed");
		return -EINVAL;
	}

	OCA_DEV_LOGD(dev, "succeed");

	return 0;
}

static int oca_monitor_check_data_size(struct device *dev,
			char *data, int32_t data_len)
{
	int ret = -1;
	struct oca_bin_header *header = (struct oca_bin_header *)data;

	switch (header->bin_data_ver) {
	case OCA_MONITOR_DATA_VER:
		ret = oca_monitor_check_data_v1_size(dev, data, data_len);
		if (ret < 0)
			return ret;
		break;
	default:
		OCA_DEV_LOGE(dev, "bin data_ver[0x%x] non support",
			header->bin_data_ver);
		return -EINVAL;
	}

	return 0;
}


static int oca_monitor_check_bin_header(struct device *dev,
				char *data, int32_t data_len)
{
	int ret = -1;
	struct oca_bin_header *header = NULL;

	if (data_len < sizeof(struct oca_bin_header)) {
		OCA_DEV_LOGE(dev, "bin len is less than oca_bin_header,check failed");
		return -EINVAL;
	}
	header = (struct oca_bin_header *)data;

	switch (header->header_ver) {
	case HEADER_VERSION_1_0_0:
		ret = oca_monitor_check_header_v_1_0_0(dev, data, data_len);
		if (ret < 0) {
			OCA_DEV_LOGE(dev, "monitor bin haeder info check error!");
			return ret;
		}
		break;
	default:
		OCA_DEV_LOGE(dev, "bin version[0x%x] non support",
			header->header_ver);
		return -EINVAL;
	}

	return 0;
}

static int oca_monitor_bin_check_sum(struct device *dev,
			char *data, int32_t data_len)
{
	int i, data_sum = 0;
	uint32_t *check_sum = (uint32_t *)data;

	for (i = 4; i < data_len; i++)
		data_sum += data[i];

	if (*check_sum != data_sum) {
		OCA_DEV_LOGE(dev, "check_sum[%d] is not equal to data_sum[%d]",
				*check_sum, data_sum);
		return -ENOMEM;
	}

	OCA_DEV_LOGD(dev, "succeed");

	return 0;
}

static int oca_monitor_bin_check(struct device *dev,
				char *monitor_data, uint32_t data_len)
{
	int ret = -1;

	if (monitor_data == NULL || data_len == 0) {
		OCA_DEV_LOGE(dev, "none data to parse");
		return -EINVAL;
	}

	ret = oca_monitor_bin_check_sum(dev, monitor_data, data_len);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "bin data check sum failed");
		return ret;
	}

	ret = oca_monitor_check_bin_header(dev, monitor_data, data_len);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "bin data len check failed");
		return ret;
	}

	ret = oca_monitor_check_data_size(dev, monitor_data, data_len);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "bin header info check failed");
		return ret;
	}

	return 0;
}

/*****************************************************************************
 *
 * oca72xxx monitor header bin parse
 *
 *****************************************************************************/
static void oca_monitor_write_to_table_v1(struct device *dev,
			struct vmax_step_config *vmax_step,
			char *vmax_data, uint32_t step_count)
{
	int i = 0;
	int index = 0;
	int vmax_step_size = (int)sizeof(struct vmax_step_config);

	for (i = 0; i < step_count; i++) {
		index = vmax_step_size * i;
		vmax_step[i].vbat_min =
			OCA_GET_32_DATA(vmax_data[index + 3],
					vmax_data[index + 2],
					vmax_data[index + 1],
					vmax_data[index + 0]);
		vmax_step[i].vbat_max =
			OCA_GET_32_DATA(vmax_data[index + 7],
					vmax_data[index + 6],
					vmax_data[index + 5],
					vmax_data[index + 4]);
		vmax_step[i].vmax_vol =
			OCA_GET_32_DATA(vmax_data[index + 11],
					vmax_data[index + 10],
					vmax_data[index + 9],
					vmax_data[index + 8]);
	}

	for (i = 0; i < step_count; i++)
		OCA_DEV_LOGI(dev, "vbat_min:%d, vbat_max%d, vmax_vol:0x%x",
			vmax_step[i].vbat_min,
			vmax_step[i].vbat_max,
			vmax_step[i].vmax_vol);
}

static int oca_monitor_parse_vol_data_v1(struct device *dev,
			struct oca_monitor *monitor, char *monitor_data)
{
	uint32_t step_count = 0;
	char *vmax_data = NULL;
	struct vmax_step_config *vmax_step = NULL;

	OCA_DEV_LOGD(dev, "enter");

	step_count = monitor->monitor_hdr.step_count;
	if (step_count) {
		vmax_step = devm_kzalloc(dev, sizeof(struct vmax_step_config) * step_count,
					GFP_KERNEL);
		if (vmax_step == NULL) {
			OCA_DEV_LOGE(dev, "vmax_cfg vmalloc failed");
			return -ENOMEM;
		}
		memset(vmax_step, 0,
			sizeof(struct vmax_step_config) * step_count);
	}

	vmax_data = monitor_data + sizeof(struct oca_bin_header) +
		sizeof(struct oca_monitor_header);
	oca_monitor_write_to_table_v1(dev, vmax_step, vmax_data, step_count);
	monitor->vmax_cfg = vmax_step;

	OCA_DEV_LOGI(dev, "vmax_data parse succeed");

	return 0;
}

static int oca_monitor_parse_data_v1(struct device *dev,
			struct oca_monitor *monitor, char *monitor_data)
{
	int ret = -1;
	int header_len = 0;
	struct oca_monitor_header *monitor_hdr = &monitor->monitor_hdr;

	header_len = sizeof(struct oca_bin_header);
	memcpy(monitor_hdr, monitor_data + header_len,
		sizeof(struct oca_monitor_header));

	OCA_DEV_LOGI(dev, "monitor_switch:%d, monitor_time:%d (ms), monitor_count:%d, step_count:%d",
		monitor_hdr->monitor_switch, monitor_hdr->monitor_time,
		monitor_hdr->monitor_count, monitor_hdr->step_count);

	ret = oca_monitor_parse_vol_data_v1(dev, monitor, monitor_data);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "vmax_data parse failed");
		return ret;
	}

	monitor->bin_status = OCA_MONITOR_CFG_OK;

	return 0;
}


static int oca_monitor_parse_v_1_0_0(struct device *dev,
			struct oca_monitor *monitor, char *monitor_data)
{
	int ret = -1;
	struct oca_bin_header *header = (struct oca_bin_header *)monitor_data;

	switch (header->bin_data_ver) {
	case OCA_MONITOR_DATA_VER:
		ret = oca_monitor_parse_data_v1(dev, monitor, monitor_data);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void oca72xxx_monitor_cfg_free(struct oca_monitor *monitor)
{
	struct oca72xxx *oca72xxx =
		container_of(monitor, struct oca72xxx, monitor);

	monitor->bin_status = OCA_MONITOR_CFG_WAIT;
	memset(&monitor->monitor_hdr, 0,
		sizeof(struct oca_monitor_header));
	if (monitor->vmax_cfg) {
		devm_kfree(oca72xxx->dev, monitor->vmax_cfg);
		monitor->vmax_cfg = NULL;
	}
}

int oca72xxx_monitor_bin_parse(struct device *dev,
				char *monitor_data, uint32_t data_len)
{
	int ret = -1;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_monitor *monitor = NULL;
	struct oca_bin_header *bin_header = NULL;

	if (oca72xxx == NULL) {
		OCA_DEV_LOGE(dev, "get struct oca72xxx failed");
		return -EINVAL;
	}

	monitor = &oca72xxx->monitor;
	monitor->bin_status = OCA_MONITOR_CFG_WAIT;

	OCA_DEV_LOGI(dev, "monitor bin parse version: %s",
			OCA_MONITOT_BIN_PARSE_VERSION);

	ret = oca_monitor_bin_check(dev, monitor_data, data_len);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "monitor bin check failed");
		return ret;
	}

	bin_header = (struct oca_bin_header *)monitor_data;
	switch (bin_header->bin_data_ver) {
	case DATA_VERSION_V1:
		ret = oca_monitor_parse_v_1_0_0(dev, monitor,
				monitor_data);
		if (ret < 0) {
			oca72xxx_monitor_cfg_free(monitor);
			return ret;
		}
		break;
	default:
		OCA_DEV_LOGE(dev, "Unrecognized this bin data version[0x%x]",
			bin_header->bin_data_ver);
	}

	return 0;
}

/***************************************************************************
 *
 * oca72xxx monitor get adjustment vmax of power
 *
 ***************************************************************************/
static int oca_monitor_get_battery_capacity(struct device *dev,
				struct oca_monitor *monitor, int *vbat_capacity)
{
	char name[] = "battery";
	int ret = -1;
	union power_supply_propval prop = { 0 };
	struct power_supply *psy = NULL;

	psy = power_supply_get_by_name(name);
	if (psy == NULL) {
		OCA_DEV_LOGE(dev, "no struct power supply name:%s", name);
		return -EINVAL;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "get vbat capacity failed");
		return -EINVAL;
	}
	*vbat_capacity = prop.intval;
	OCA_DEV_LOGI(dev, "The percentage is %d",
		*vbat_capacity);

	return 0;
}

static int oca_search_vmax_from_table(struct device *dev,
				struct oca_monitor *monitor,
				const int vbat_vol, int *vmax_vol)
{
	int i = 0;
	int vmax_set = 0;
	uint32_t vmax_flag = 0;
	struct oca_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	struct vmax_step_config *vmax_cfg = monitor->vmax_cfg;

	if (monitor->bin_status == OCA_MONITOR_CFG_WAIT) {
		OCA_DEV_LOGE(dev, "vmax_cfg not loaded or parse failed");
		return -ENODATA;
	}

	for (i = 0; i < monitor_hdr->step_count; i++) {
		if (vbat_vol == OCA_VBAT_MAX) {
			vmax_set = OCA_VMAX_MAX;
			vmax_flag = 1;
			OCA_DEV_LOGD(dev, "vbat=%d, setting vmax=0x%x",
				vbat_vol, vmax_set);
			break;
		}

		if (vbat_vol >= vmax_cfg[i].vbat_min &&
			vbat_vol < vmax_cfg[i].vbat_max) {
			vmax_set = vmax_cfg[i].vmax_vol;
			vmax_flag = 1;
			OCA_DEV_LOGD(dev, "read setting vmax=0x%x, step[%d]: vbat_min=%d,vbat_max=%d",
				vmax_set, i,
				vmax_cfg[i].vbat_min,
				vmax_cfg[i].vbat_max);
			break;
		}
	}

	if (!vmax_flag) {
		OCA_DEV_LOGE(dev, "vmax_cfg not found");
		return -ENODATA;
	}

	*vmax_vol = vmax_set;
	return 0;
}

/***************************************************************************
 *
 * oca72xxx no dsp monitor func
 *
 ***************************************************************************/
int oca72xxx_monitor_no_dsp_get_vmax(struct oca_monitor *monitor, int32_t *vmax)
{
	int vbat_capacity = 0;
	int ret = -1;
	int vmax_vol = 0;
	struct oca72xxx *oca72xxx =
		container_of(monitor, struct oca72xxx, monitor);
	struct device *dev = oca72xxx->dev;

	ret = oca_monitor_get_battery_capacity(dev, monitor, &vbat_capacity);
	if (ret < 0)
		return ret;

	if (monitor->custom_capacity)
		vbat_capacity = monitor->custom_capacity;
	OCA_DEV_LOGI(dev, "get_battery_capacity is[%d]", vbat_capacity);

	ret = oca_search_vmax_from_table(dev, monitor,
				vbat_capacity, &vmax_vol);
	if (ret < 0) {
		OCA_DEV_LOGE(dev, "not find vmax_vol");
		return ret;
	}

	*vmax = vmax_vol;
	return 0;
}


/***************************************************************************
 *
 * oca72xxx monitor sysfs nodes
 *
 ***************************************************************************/
static ssize_t oca_attr_get_vbat(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	int vbat_capacity = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_monitor *monitor = &oca72xxx->monitor;

	if (monitor->custom_capacity == 0) {
		ret = oca_monitor_get_battery_capacity(dev, monitor,
					&vbat_capacity);
		if (ret < 0) {
			OCA_DEV_LOGE(oca72xxx->dev, "get battery_capacity failed");
			return ret;
		}
		len += snprintf(buf + len, PAGE_SIZE - len,
			"vbat capacity=%d\n", vbat_capacity);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"vbat capacity=%d\n",
				monitor->custom_capacity);
	}

	return len;
}

static ssize_t oca_attr_set_vbat(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int ret = -1;
	int capacity = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_monitor *monitor = &oca72xxx->monitor;

	ret = kstrtouint(buf, 0, &capacity);
	if (ret < 0)
		return ret;
	OCA_DEV_LOGI(oca72xxx->dev, "set capacity = %d", capacity);
	if (capacity >= OCA_VBAT_CAPACITY_MIN &&
			capacity <= OCA_VBAT_CAPACITY_MAX){
		monitor->custom_capacity = capacity;
	} else {
		OCA_DEV_LOGE(oca72xxx->dev, "vbat_set=invalid,please input value [%d-%d]",
			OCA_VBAT_CAPACITY_MIN, OCA_VBAT_CAPACITY_MAX);
		return -EINVAL;
	}

	return len;
}

static ssize_t oca_attr_get_vmax(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = -1;
	int vbat_capacity = 0;
	int vmax_get = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_monitor *monitor = &oca72xxx->monitor;

	ret = oca_monitor_get_battery_capacity(dev, monitor,
					&vbat_capacity);
	if (ret < 0)
		return ret;
	OCA_DEV_LOGI(oca72xxx->dev, "get_battery_capacity is [%d]",
		vbat_capacity);

	if (monitor->custom_capacity) {
		vbat_capacity = monitor->custom_capacity;
		OCA_DEV_LOGI(oca72xxx->dev, "get custom_capacity is [%d]",
			vbat_capacity);
	}

	ret = oca_search_vmax_from_table(oca72xxx->dev, monitor,
				vbat_capacity, &vmax_get);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "not find vmax_vol");
		len += snprintf(buf + len, PAGE_SIZE - len,
			"not_find_vmax_vol\n");
		return len;
	}
	len += snprintf(buf + len, PAGE_SIZE - len,
		"0x%x\n", vmax_get);
	OCA_DEV_LOGI(oca72xxx->dev, "0x%x", vmax_get);

	return len;
}

static ssize_t oca_attr_set_vmax(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t vmax_set = 0;
	int ret = -1;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &vmax_set);
	if (ret < 0)
		return ret;

	OCA_DEV_LOGI(oca72xxx->dev, "vmax_set=0x%x", vmax_set);

	OCA_DEV_LOGE(oca72xxx->dev, "no_dsp system,vmax_set invalid");
	return -EINVAL;

	return count;
}

int oca72xxx_dev_monitor_switch_set(struct oca_monitor *monitor, uint32_t enable)
{
	struct oca72xxx *oca72xxx =
			container_of(monitor, struct oca72xxx, monitor);
	struct oca_monitor_header *monitor_hdr = &monitor->monitor_hdr;

	OCA_DEV_LOGI(oca72xxx->dev, "monitor switch set =%d", enable);

	if (!monitor->bin_status) {
		OCA_DEV_LOGE(oca72xxx->dev, "bin parse faile or not loaded,set invalid");
		return -EINVAL;
	}

	if (monitor_hdr->monitor_switch == enable)
		return 0;

	if (enable > 0) {
		monitor_hdr->monitor_switch = 1;
	} else {
		monitor_hdr->monitor_switch = 0;
	}

	return 0;
}
static DEVICE_ATTR(vbat, S_IWUSR | S_IRUGO,
	oca_attr_get_vbat, oca_attr_set_vbat);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO,
	oca_attr_get_vmax, oca_attr_set_vmax);

static struct attribute *oca_monitor_vol_adjust[] = {
	&dev_attr_vbat.attr,
	&dev_attr_vmax.attr,
	NULL
};

static struct attribute_group oca_monitor_vol_adjust_group = {
	.attrs = oca_monitor_vol_adjust,
};

/***************************************************************************
 *
 * oca72xxx monitor init
 *
 ***************************************************************************/
void oca72xxx_monitor_init(struct device *dev, struct oca_monitor *monitor,
				struct device_node *dev_node)
{
	int ret = -1;
	struct oca72xxx *oca72xxx =
		container_of(monitor, struct oca72xxx, monitor);

	monitor->dev_index = oca72xxx->dev_index;
	monitor->monitor_hdr.monitor_time = OCA_DEFAULT_MONITOR_TIME;

	ret = sysfs_create_group(&dev->kobj, &oca_monitor_vol_adjust_group);
	if (ret < 0)
		OCA_DEV_LOGE(dev, "failed to create monitor vol_adjust sysfs nodes");

	if (!ret)
		OCA_DEV_LOGI(dev, "monitor init succeed");
}

void oca72xxx_monitor_exit(struct oca_monitor *monitor)
{
	struct oca72xxx *oca72xxx =
		container_of(monitor, struct oca72xxx, monitor);
	/*rm attr node*/
	sysfs_remove_group(&oca72xxx->dev->kobj,
			&oca_monitor_vol_adjust_group);
}

