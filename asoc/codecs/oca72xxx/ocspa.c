/*
 * oca72xxx.c  oca72xxx pa module
 *
 * Copyright (c) 2025 OCS Technology CO., LTD
 *
 * Author: Wall <Wall@orient-chip.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#define DEBUG 1

#include <linux/i2c.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <uapi/sound/asound.h>
#include <sound/control.h>
#include <sound/soc.h>
#include "ocspa.h"
#include "ocspa_device.h"
#include "ocspa_log.h"
#include "ocspa_monitor.h"
#include "ocspa_fw.h"
#include "ocspa_fw_process.h"
#if IS_ENABLED(CONFIG_MIEV)
#include <miev/mievent.h>
#endif

/*****************************************************************
* oca72xxx marco
******************************************************************/
#define OCA72XXX_I2C_NAME	"oca72xxx_pa"
#define OCA72XXX_DRIVER_VERSION	"V5.3.0"
#define OCA72XXX_FW_BIN_NAME	"ocs72xxx_cf.bin"

/*************************************************************************
 * oca72xxx variable
 ************************************************************************/
static LIST_HEAD(g_oca72xxx_list);
static DEFINE_MUTEX(g_oca72xxx_mutex_lock);
unsigned int g_oca72xxx_dev_cnt ;

static const char *const oca72xxx_pa_switch[] = {"Disable", "Enable"};
static const char *const oca72xxx_monitor_switch[] = {"Disable", "Enable"};

#ifdef OCA_KERNEL_VER_OVER_4_19_1
static struct oca_componet_codec_ops oca_componet_codec_ops = {
	.add_codec_controls = snd_soc_add_component_controls,
	.unregister_codec = snd_soc_unregister_component,
};
#else
static struct oca_componet_codec_ops oca_componet_codec_ops = {
	.add_codec_controls = snd_soc_add_codec_controls,
	.unregister_codec = snd_soc_unregister_codec,
};
#endif


/************************************************************************
 *
 * oca72xxx device update profile
 *
 ************************************************************************/
static void oca72xxx_update_voltage_max(struct oca72xxx *oca72xxx,
				struct oca_data_container *data_container)
{
	int i = 0;
	struct oca_voltage_desc *vol_desc = &oca72xxx->oca_dev.vol_desc;

	if (data_container == NULL || data_container->len <= 2
		|| vol_desc->addr == OCA_REG_NONE)
		return;

	for (i = 0; i < data_container->len; i = i + 2) {
		if (data_container->data[i] == vol_desc->addr) {
			vol_desc->vol_max = data_container->data[i + 1];
			OCA_DEV_LOGD(oca72xxx->dev, "get voltage max = 0x%02x",
			data_container->data[i + 1]);
			return;
		}
	}
}

static int oca72xxx_power_down(struct oca72xxx *oca72xxx, char *profile)
{
	int ret = 0;
	struct oca_prof_desc *prof_desc = NULL;
	struct oca_prof_info *prof_info = &oca72xxx->acf_info.prof_info;
	struct oca_data_container *data_container = NULL;
	struct oca_device *oca_dev = &oca72xxx->oca_dev;

	OCA_DEV_LOGD(oca72xxx->dev, "enter");

	if (!prof_info->status) {
		OCA_DEV_LOGE(oca72xxx->dev, "profile_cfg not load");
		return -EINVAL;
	}

	prof_desc = oca72xxx_acf_get_prof_desc_form_name(oca72xxx->dev, &oca72xxx->acf_info, profile);
	if (prof_desc == NULL)
		goto no_bin_pwr_off;

	if (!prof_desc->prof_st)
		goto no_bin_pwr_off;


	data_container = &prof_desc->data_container;
	OCA_DEV_LOGD(oca72xxx->dev, "get profile[%s] data len [%d]",
			profile, data_container->len);

	if (oca_dev->hwen_status == OCA_DEV_HWEN_OFF) {
		OCA_DEV_LOGI(oca72xxx->dev, "profile[%s] has already load ", profile);
	} else {
		if (oca_dev->ops.pwr_off_func) {
			ret = oca_dev->ops.pwr_off_func(oca_dev, data_container);
			if (ret < 0) {
				OCA_DEV_LOGE(oca72xxx->dev, "load profile[%s] failed ", profile);
				goto pwr_off_failed;
			}
		} else {
			ret = oca72xxx_dev_default_pwr_off(oca_dev, data_container);
			if (ret < 0) {
				OCA_DEV_LOGE(oca72xxx->dev, "load profile[%s] failed ", profile);
				goto pwr_off_failed;
			}
		}
	}

	oca72xxx->current_profile = prof_desc->prof_name;
	return 0;

pwr_off_failed:
no_bin_pwr_off:
	oca72xxx_dev_hw_pwr_ctrl(&oca72xxx->oca_dev, false);
	oca72xxx->current_profile = oca72xxx->prof_off_name;
	return ret;
}

static int oca72xxx_power_on(struct oca72xxx *oca72xxx, char *profile)
{
	int ret = -EINVAL;
	struct oca_prof_desc *prof_desc = NULL;
	struct oca_prof_info *prof_info = &oca72xxx->acf_info.prof_info;
	struct oca_data_container *data_container = NULL;
	struct oca_device *oca_dev = &oca72xxx->oca_dev;

	OCA_DEV_LOGD(oca72xxx->dev, "enter");

	if (!prof_info->status) {
		OCA_DEV_LOGE(oca72xxx->dev, "profile_cfg not load");
		return -EINVAL;
	}

	if (0 == strncmp(profile, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX))
		return oca72xxx_power_down(oca72xxx, profile);

	prof_desc = oca72xxx_acf_get_prof_desc_form_name(oca72xxx->dev, &oca72xxx->acf_info, profile);
	if (prof_desc == NULL) {
		OCA_DEV_LOGE(oca72xxx->dev, "not found [%s] parameter", profile);
		return -EINVAL;
	}

	if (!prof_desc->prof_st) {
		OCA_DEV_LOGE(oca72xxx->dev, "not found data container");
		return -EINVAL;
	}

	data_container = &prof_desc->data_container;
	OCA_DEV_LOGD(oca72xxx->dev, "get profile[%s] data len [%d]",
			profile, data_container->len);
	oca72xxx_update_voltage_max(oca72xxx, data_container);
	if (oca_dev->ops.pwr_on_func) {
		ret = oca_dev->ops.pwr_on_func(oca_dev, data_container);
		if (ret < 0) {
			OCA_DEV_LOGE(oca72xxx->dev, "load profile[%s] failed ",
				profile);
			return oca72xxx_power_down(oca72xxx, oca72xxx->prof_off_name);
		}
	} else {
		ret = oca72xxx_dev_default_pwr_on(oca_dev, data_container);
		if (ret < 0) {
			OCA_DEV_LOGE(oca72xxx->dev, "load profile[%s] failed ",
				profile);
			return oca72xxx_power_down(oca72xxx, oca72xxx->prof_off_name);
		}
	}

	oca72xxx->current_profile = prof_desc->prof_name;
	OCA_DEV_LOGD(oca72xxx->dev, "load profile[%s] succeed", profile);

	return 0;
}



int oca72xxx_update_profile(struct oca72xxx *oca72xxx, char *profile)
{
	int ret = -1;

	OCA_DEV_LOGD(oca72xxx->dev, "load profile[%s] enter", profile);

	if (!strncmp(profile, oca72xxx->current_profile, OCA_PROFILE_STR_MAX)) {
		OCA_DEV_LOGI(oca72xxx->dev, "has already load profile[%s]", profile);
		return 0;
	}
	
	mutex_lock(&oca72xxx->reg_lock);
	if (0 == strncmp(profile, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX)) {
		ret = oca72xxx_power_down(oca72xxx, profile);
		oca72xxx->pa_status = 0;
	} else {
		ret = oca72xxx_power_on(oca72xxx, profile);
	}
	if (oca72xxx->oca_dev.chipid == 0x09) 
		oca72xxx->pa_status = 0; //rst_gpio reg01 default close
	mutex_unlock(&oca72xxx->reg_lock);

	return ret;
}

char *oca72xxx_show_current_profile(int dev_index)
{
	struct list_head *pos = NULL;
	struct oca72xxx *oca72xxx = NULL;

	list_for_each(pos, &g_oca72xxx_list) {
		oca72xxx = list_entry(pos, struct oca72xxx, list);
		if (oca72xxx->dev_index == dev_index) {
			OCA_DEV_LOGI(oca72xxx->dev, "current profile is [%s]",
				oca72xxx->current_profile);
			return oca72xxx->current_profile;
		}
	}

	OCA_LOGE("not found struct oca72xxx, dev_index = [%d]", dev_index);
	return NULL;
}
EXPORT_SYMBOL(oca72xxx_show_current_profile);

int oca72xxx_set_pa(int dev_index, uint32_t on_off)
{
	int ret = 0;
	struct list_head *pos = NULL;
	struct oca72xxx *oca72xxx = NULL;

	list_for_each(pos, &g_oca72xxx_list) {
		oca72xxx = list_entry(pos, struct oca72xxx, list);
		if (oca72xxx->dev_index == dev_index) {
			OCA_DEV_LOGI(oca72xxx->dev, "set dev_index = %d, on_off = %d",
				dev_index, on_off);
		 	if (oca72xxx->oca_dev.chipid == 0x09) { //oca72559
				if (on_off) {//on
					if ((strncmp(oca72xxx->current_profile, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX) == 0) &&
						(strncmp(oca72xxx->last_profile, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX) != 0)) {
						oca72xxx_update_profile(oca72xxx, oca72xxx->last_profile);
					}
					mutex_lock(&oca72xxx->reg_lock);
					if (oca72xxx->pa_status == 0) {
						OCA_DEV_LOGI(oca72xxx->dev,"oca72559 0x01 write 0x78");
						ret = oca72xxx_dev_i2c_write_byte(&oca72xxx->oca_dev, 0x01, 0x78);
						if (ret < 0) 
							OCA_DEV_LOGE(oca72xxx->dev,"oca72559 0x01 write failed, on_off = %d", on_off);
						else
							oca72xxx->pa_status = on_off;
					}
					mutex_unlock(&oca72xxx->reg_lock);
				} else {//off
					oca72xxx_update_profile(oca72xxx, oca72xxx->prof_off_name);
				}
			} else { //oca72390
				if (on_off) {
					if ((strncmp(oca72xxx->current_profile, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX) == 0) &&
						(strncmp(oca72xxx->last_profile, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX) != 0)) {
						oca72xxx_update_profile(oca72xxx, oca72xxx->last_profile);
					}
					mutex_lock(&oca72xxx->reg_lock);
					if (oca72xxx->pa_status == 0) {
					    OCA_DEV_LOGI(oca72xxx->dev,"oca72390 0x01 write 0x0d");
						ret = oca72xxx_dev_i2c_write_byte(&oca72xxx->oca_dev, 0x01, 0x0d);
						if (ret < 0) 
							OCA_DEV_LOGE(oca72xxx->dev,"oca72390 0x01 write failed, on_off = %d", on_off);
						else
							oca72xxx->pa_status = on_off;
					}
					mutex_unlock(&oca72xxx->reg_lock);
				} else {
					oca72xxx_update_profile(oca72xxx, oca72xxx->prof_off_name);
				}
			}
		}
	}
	return ret;
}
EXPORT_SYMBOL(oca72xxx_set_pa);

int oca72xxx_set_profile(int dev_index, char *profile)
{
	struct list_head *pos = NULL;
	struct oca72xxx *oca72xxx = NULL;

	if (!profile) {
		OCA_LOGE("profile is NULL");
		return -EINVAL;
	}

	list_for_each(pos, &g_oca72xxx_list) {
		oca72xxx = list_entry(pos, struct oca72xxx, list);
		if (oca72xxx->dev_index == dev_index) {
			OCA_DEV_LOGD(oca72xxx->dev, "set dev_index = %d, profile = %s",
				dev_index, profile);
			return oca72xxx_update_profile(oca72xxx, profile);
		}
	}

	OCA_LOGE("not found struct oca72xxx, dev_index = [%d]", dev_index);
	return -EINVAL;
}
EXPORT_SYMBOL(oca72xxx_set_profile);

int oca72xxx_set_profile_by_id(int dev_index, int profile_id)
{
	char *profile = NULL;

	profile = oca72xxx_ctos_get_prof_name(profile_id);
	if (profile == NULL) {
		OCA_LOGE("oca72xxx, dev_index[%d] profile[%d] not support!",
					dev_index, profile_id);
		return -EINVAL;
	}

	OCA_LOGI("oca72xxx, dev_index[%d] set profile[%s] by id[%d]",
					dev_index, profile, profile_id);
	return oca72xxx_set_profile(dev_index, profile);
}
EXPORT_SYMBOL(oca72xxx_set_profile_by_id);

int oca72xxx_set_boost_voltage(int dev_index, int status)
{
	int ret = 0;
	struct list_head *pos = NULL;
	struct oca72xxx *oca72xxx = NULL;
	struct oca_voltage_desc *vol_desc = NULL;

	list_for_each(pos, &g_oca72xxx_list) {
		oca72xxx = list_entry(pos, struct oca72xxx, list);
		if (oca72xxx == NULL) {
			OCA_LOGE("struct oca72xxx not ready");
			return -EINVAL;
		}

		if (oca72xxx->dev_index == dev_index) {
			OCA_DEV_LOGD(oca72xxx->dev, "enter, set dev_index = %d, status = %d", dev_index, status);
			if (0 == strncmp(oca72xxx->current_profile, oca72xxx->prof_off_name,
				OCA_PROFILE_STR_MAX)) {
				OCA_DEV_LOGE(oca72xxx->dev, "PA is power down, can not set voltage!");
				return -EINVAL;
			}

			vol_desc = &oca72xxx->oca_dev.vol_desc;
			if (vol_desc->addr == OCA_REG_NONE) {
				OCA_DEV_LOGI(oca72xxx->dev, "PA unsupport set boost voltage!");
				return -EINVAL;
			}
			if (status == OCA_VOLTAGE_HIGH) {
				OCA_DEV_LOGI(oca72xxx->dev, "set reg voltage max: 0x%02x = 0x%02x",
					vol_desc->addr, vol_desc->vol_max);
				ret = oca72xxx_dev_i2c_write_byte(&oca72xxx->oca_dev,
					vol_desc->addr, vol_desc->vol_max);
				if (ret < 0)
					return ret;
			} else if (status == OCA_VOLTAGE_LOW) {
				OCA_DEV_LOGI(oca72xxx->dev, "set reg voltage min: 0x%02x = 0x%02x",
					vol_desc->addr, vol_desc->vol_min);
				ret = oca72xxx_dev_i2c_write_byte(&oca72xxx->oca_dev,
					vol_desc->addr, vol_desc->vol_min);
				if (ret < 0)
					return ret;
			} else {
				OCA_DEV_LOGE(oca72xxx->dev, "unsupport status: %d", status);
				return -EINVAL;
			}

			return 0;
		}
	}

	OCA_LOGE("not found struct oca72xxx, dev_index = [%d]", dev_index);
	return -EINVAL;
}
EXPORT_SYMBOL(oca72xxx_set_boost_voltage);

int oca72xxx_get_boost_voltage(int dev_index, int *status)
{
	int ret = 0;
	struct list_head *pos = NULL;
	struct oca72xxx *oca72xxx = NULL;
	uint8_t reg_voltage_val = 0;
	struct oca_voltage_desc *vol_desc = NULL;

	list_for_each(pos, &g_oca72xxx_list) {
		oca72xxx = list_entry(pos, struct oca72xxx, list);
		if (oca72xxx == NULL) {
			OCA_LOGE("struct oca72xxx not ready");
			return -EINVAL;
		}

		if (oca72xxx->dev_index == dev_index) {
			if (0 == strncmp(oca72xxx->current_profile, oca72xxx->prof_off_name,
				OCA_PROFILE_STR_MAX)) {
				OCA_DEV_LOGE(oca72xxx->dev, "PA is power down, can not get voltage!");
				return -EINVAL;
			}

			vol_desc = &oca72xxx->oca_dev.vol_desc;
			if (vol_desc->addr == OCA_REG_NONE) {
				OCA_DEV_LOGI(oca72xxx->dev, "PA unsupport get boost voltage!");
				return -EINVAL;
			}
			ret = oca72xxx_dev_i2c_read_byte(&oca72xxx->oca_dev,
					vol_desc->addr, &reg_voltage_val);
				if (ret < 0)
					return ret;

			if (reg_voltage_val ==  vol_desc->vol_max) {
				*status = OCA_VOLTAGE_HIGH;
			} else if (reg_voltage_val ==  vol_desc->vol_min) {
				*status = OCA_VOLTAGE_LOW;
			} else {
				OCA_DEV_LOGE(oca72xxx->dev, "undefined reg val 0x%02x = 0x%02x",
					vol_desc->addr, reg_voltage_val);
				return -EINVAL;
			}

			return 0;
		}
	}

	OCA_LOGE("not found struct oca72xxx, dev_index = [%d]", dev_index);
	return -EINVAL;
}
EXPORT_SYMBOL(oca72xxx_get_boost_voltage);

/****************************************************************************
 *
 * oca72xxx Kcontrols
 *
 ****************************************************************************/
static int oca72xxx_profile_switch_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	int count = 0;
	char *name = NULL;
	char *profile_name = NULL;
	int len = 0;
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;

	if (oca72xxx == NULL) {
		OCA_LOGE("get struct oca72xxx failed");
		return -EINVAL;
	}

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	/*make sure have prof */
	count = oca72xxx_acf_get_profile_count(oca72xxx->dev, &oca72xxx->acf_info);
	if (count <= 0) {
		uinfo->value.enumerated.items = 0;
		OCA_DEV_LOGE(oca72xxx->dev, "get count[%d] failed", count);
		return 0;
	}

	uinfo->value.enumerated.items = count;
	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	name = uinfo->value.enumerated.name;
	count = uinfo->value.enumerated.item;
	profile_name = oca72xxx_acf_get_prof_name_form_index(oca72xxx->dev,
		&oca72xxx->acf_info, count);
	if (profile_name == NULL) {
		len = strscpy(uinfo->value.enumerated.name, "NULL",
			strlen("NULL") + 1);
		if (len < 0)
			OCA_DEV_LOGE(oca72xxx->dev, "copy errno %ld\n", len);
		return 0;
	}

	len = strscpy(name, profile_name, sizeof(uinfo->value.enumerated.name));
	if (len < 0)
		OCA_DEV_LOGE(oca72xxx->dev, "copy errno %ld\n", len);

	return 0;
}

static int oca72xxx_profile_switch_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int ret = -1;
	char *profile_name = NULL;
	int index = ucontrol->value.integer.value[0];
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;
	struct acf_bin_info *acf_info = NULL;

	if (oca72xxx == NULL) {
		OCA_LOGE("get struct oca72xxx failed");
		return -EINVAL;
	}

	acf_info = &oca72xxx->acf_info;

	profile_name = oca72xxx_acf_get_prof_name_form_index(oca72xxx->dev, acf_info, index);
	if (!profile_name) {
		OCA_DEV_LOGE(oca72xxx->dev, "not found profile name,index=[%d]",
				index);
		return -EINVAL;
	}

	oca72xxx->kctl_profile = profile_name;
	if ((strncmp(profile_name, oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX) != 0) &&
		(strncmp(oca72xxx->last_profile, profile_name, OCA_PROFILE_STR_MAX) != 0)) {
			oca72xxx->last_profile = profile_name;
		OCA_DEV_LOGI(oca72xxx->dev, " last_profile: [%s]", profile_name);
	}

	OCA_DEV_LOGI(oca72xxx->dev, "set profile [%s]", profile_name);

	ret = oca72xxx_update_profile(oca72xxx, profile_name);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "set dev_index[%d] profile failed, profile = %s",
			oca72xxx->dev_index, profile_name);
		return ret;
	}

	return 0;
}

static int oca72xxx_profile_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int index = 0;

	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;

	if (oca72xxx == NULL) {
		OCA_LOGE("get struct oca72xxx failed");
		return -EINVAL;
	}

	if (!oca72xxx->kctl_profile) {
		OCA_DEV_LOGE(oca72xxx->dev, "profile not init");
		return -EINVAL;
	}

	OCA_DEV_LOGI(oca72xxx->dev, "kctl_profile:[%s]",
		oca72xxx->kctl_profile);


	index = oca72xxx_acf_get_prof_index_form_name(oca72xxx->dev,
		&oca72xxx->acf_info, oca72xxx->kctl_profile);
	if (index < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "get profile index failed");
		return index;
	}

	ucontrol->value.integer.value[0] = index;

	return 0;
}

static int oca72xxx_vmax_get_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = INT_MIN;
	uinfo->value.integer.max = OCA_VMAX_MAX;

	return 0;
}

static int oca72xxx_vmax_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int ret = -1;
	int vmax_val = 0;
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;

	if (oca72xxx == NULL) {
		OCA_LOGE("get struct oca72xxx failed");
		return -EINVAL;
	}

	ret = oca72xxx_monitor_no_dsp_get_vmax(&oca72xxx->monitor, &vmax_val);
	if (ret < 0)
		return ret;

	ucontrol->value.integer.value[0] = vmax_val;
	OCA_DEV_LOGI(oca72xxx->dev, "get vmax = [0x%x]", vmax_val);

	return 0;
}

static int oca72xxx_monitor_switch_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	int count;
	int len = 0;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = ARRAY_SIZE(oca72xxx_monitor_switch);

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	len = strscpy(uinfo->value.enumerated.name,
		oca72xxx_monitor_switch[uinfo->value.enumerated.item],
		strlen(oca72xxx_monitor_switch[uinfo->value.enumerated.item]) + 1);
	if (len < 0)
		OCA_LOGE("copy errno %ld\n", len);

	return 0;
}

static int oca72xxx_monitor_switch_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	uint32_t ctrl_value = ucontrol->value.integer.value[0];
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;
	struct oca_monitor *oca_monitor = &oca72xxx->monitor;
	int ret = -1;

	ret = oca72xxx_dev_monitor_switch_set(oca_monitor, ctrl_value);
	if (ret)
		return ret;

	return 0;
}

static int oca72xxx_monitor_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;
	struct oca_monitor *oca_monitor = &oca72xxx->monitor;

	ucontrol->value.integer.value[0] = oca_monitor->monitor_hdr.monitor_switch;

	OCA_DEV_LOGD(oca72xxx->dev, "monitor switch is %ld", ucontrol->value.integer.value[0]);
	return 0;
}

static int oca72xxx_pa_switch_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	int count = 0;
	int len = 0;
	
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = ARRAY_SIZE(oca72xxx_pa_switch);

	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	len = strscpy(uinfo->value.enumerated.name,
		oca72xxx_pa_switch[uinfo->value.enumerated.item],
		strlen(oca72xxx_pa_switch[uinfo->value.enumerated.item]) + 1);
	if (len < 0)
		OCA_LOGE("copy errno %ld\n", len);
	return 0;
}

static int oca72xxx_pa_switch_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	uint32_t ctrl_value = ucontrol->value.integer.value[0];
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;
	int ret = -1;

	ret = oca72xxx_set_pa(oca72xxx->dev_index, ctrl_value);
	if (ret)
		return ret;

	return 0;
}

static int oca72xxx_pa_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;
	
	ucontrol->value.integer.value[0] = oca72xxx->pa_status;

	OCA_DEV_LOGD(oca72xxx->dev, "pa switch is %ld", ucontrol->value.integer.value[0]);
	
	return 0;
}

static int oca72xxx_hal_monitor_time_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = OCA_MONITOR_TIME_MIN;
	uinfo->value.integer.max = OCA_MONITOR_TIME_MAX;

	return 0;
}

static int oca72xxx_hal_monitor_time_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct oca72xxx *oca72xxx = (struct oca72xxx *)kcontrol->private_value;

	if (oca72xxx == NULL) {
		OCA_LOGE("get struct oca72xxx failed");
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] =
			oca72xxx->monitor.monitor_hdr.monitor_time;

	OCA_LOGI("get monitor time %ld", ucontrol->value.integer.value[0]);

	return 0;
}

static int oca72xxx_kcontrol_dynamic_create(struct oca72xxx *oca72xxx,
						void *codec)
{
	struct snd_kcontrol_new *oca72xxx_kcontrol = NULL;
	oca_snd_soc_codec_t *soc_codec = (oca_snd_soc_codec_t *)codec;
	char *kctl_name[OCA72XXX_PRIVATE_KCONTROL_NUM];
	int kcontrol_num = OCA72XXX_PRIVATE_KCONTROL_NUM;
	int ret = -1;

	OCA_DEV_LOGD(oca72xxx->dev, "enter");
	oca72xxx->codec = soc_codec;

	oca72xxx_kcontrol = devm_kzalloc(oca72xxx->dev,
			sizeof(struct snd_kcontrol_new) * kcontrol_num,
			GFP_KERNEL);
	if (oca72xxx_kcontrol == NULL) {
		OCA_DEV_LOGE(oca72xxx->dev, "oca72xxx_kcontrol devm_kzalloc failed");
		return -ENOMEM;
	}

	kctl_name[0] = devm_kzalloc(oca72xxx->dev, OCA_NAME_BUF_MAX,
			GFP_KERNEL);
	if (kctl_name[0] == NULL)
		return -ENOMEM;

	snprintf(kctl_name[0], OCA_NAME_BUF_MAX, "oca72xxx_profile_switch_%d",
			oca72xxx->dev_index);

	oca72xxx_kcontrol[0].name = kctl_name[0];
	oca72xxx_kcontrol[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	oca72xxx_kcontrol[0].info = oca72xxx_profile_switch_info;
	oca72xxx_kcontrol[0].get = oca72xxx_profile_switch_get;
	oca72xxx_kcontrol[0].put = oca72xxx_profile_switch_put;
	oca72xxx_kcontrol[0].private_value = (unsigned long)oca72xxx;

	kctl_name[1] = devm_kzalloc(oca72xxx->codec->dev, OCA_NAME_BUF_MAX,
			GFP_KERNEL);
	if (kctl_name[1] == NULL)
		return -ENOMEM;

	snprintf(kctl_name[1], OCA_NAME_BUF_MAX, "oca72xxx_vmax_get_%d",
			oca72xxx->dev_index);

	oca72xxx_kcontrol[1].name = kctl_name[1];
	oca72xxx_kcontrol[1].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	oca72xxx_kcontrol[1].access = SNDRV_CTL_ELEM_ACCESS_READ;
	oca72xxx_kcontrol[1].info = oca72xxx_vmax_get_info;
	oca72xxx_kcontrol[1].get = oca72xxx_vmax_get;
	oca72xxx_kcontrol[1].private_value = (unsigned long)oca72xxx;

	kctl_name[2] = devm_kzalloc(oca72xxx->codec->dev, OCA_NAME_BUF_MAX,
			GFP_KERNEL);
	if (kctl_name[2] == NULL)
		return -ENOMEM;

	snprintf(kctl_name[2], OCA_NAME_BUF_MAX, "oca72xxx_monitor_switch_%d",
			oca72xxx->dev_index);

	oca72xxx_kcontrol[2].name = kctl_name[2];
	oca72xxx_kcontrol[2].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	oca72xxx_kcontrol[2].info = oca72xxx_monitor_switch_info;
	oca72xxx_kcontrol[2].get = oca72xxx_monitor_switch_get;
	oca72xxx_kcontrol[2].put = oca72xxx_monitor_switch_put;
	oca72xxx_kcontrol[2].private_value = (unsigned long)oca72xxx;

	kctl_name[3] = devm_kzalloc(oca72xxx->codec->dev, OCA_NAME_BUF_MAX,
			GFP_KERNEL);
	if (kctl_name[3] == NULL)
		return -ENOMEM;

	snprintf(kctl_name[3], OCA_NAME_BUF_MAX, "oca72xxx_pa_switch_%d",
			oca72xxx->dev_index);

	oca72xxx_kcontrol[3].name = kctl_name[3];
	oca72xxx_kcontrol[3].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	oca72xxx_kcontrol[3].info = oca72xxx_pa_switch_info;
	oca72xxx_kcontrol[3].get = oca72xxx_pa_switch_get;
	oca72xxx_kcontrol[3].put = oca72xxx_pa_switch_put;
	oca72xxx_kcontrol[3].private_value = (unsigned long)oca72xxx;

	ret = oca_componet_codec_ops.add_codec_controls(oca72xxx->codec,
				oca72xxx_kcontrol, kcontrol_num);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "add codec controls failed, ret = %d",
			ret);
		return ret;
	}

	OCA_DEV_LOGI(oca72xxx->dev, "add codec controls[%s,%s,%s,%s]",
		oca72xxx_kcontrol[0].name,
		oca72xxx_kcontrol[1].name,
		oca72xxx_kcontrol[2].name,
		oca72xxx_kcontrol[3].name);

	return 0;
}

static int oca72xxx_public_kcontrol_create(struct oca72xxx *oca72xxx,
						void *codec)
{
	struct snd_kcontrol_new *oca72xxx_kcontrol = NULL;
	oca_snd_soc_codec_t *soc_codec = (oca_snd_soc_codec_t *)codec;
	char *kctl_name[OCA72XXX_PUBLIC_KCONTROL_NUM];
	int kcontrol_num = OCA72XXX_PUBLIC_KCONTROL_NUM;
	int ret = -1;

	OCA_DEV_LOGD(oca72xxx->dev, "enter");
	oca72xxx->codec = soc_codec;

	oca72xxx_kcontrol = devm_kzalloc(oca72xxx->dev,
			sizeof(struct snd_kcontrol_new) * kcontrol_num,
			GFP_KERNEL);
	if (oca72xxx_kcontrol == NULL) {
		OCA_DEV_LOGE(oca72xxx->dev, "oca72xxx_kcontrol devm_kzalloc failed");
		return -ENOMEM;
	}

	kctl_name[0] = devm_kzalloc(oca72xxx->dev, OCA_NAME_BUF_MAX,
			GFP_KERNEL);
	if (kctl_name[0] == NULL)
		return -ENOMEM;

	snprintf(kctl_name[0], OCA_NAME_BUF_MAX, "oca72xxx_hal_monitor_time");

	oca72xxx_kcontrol[0].name = kctl_name[0];
	oca72xxx_kcontrol[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	oca72xxx_kcontrol[0].access = SNDRV_CTL_ELEM_ACCESS_READ;
	oca72xxx_kcontrol[0].info = oca72xxx_hal_monitor_time_info;
	oca72xxx_kcontrol[0].get = oca72xxx_hal_monitor_time_get;
	oca72xxx_kcontrol[0].private_value = (unsigned long)oca72xxx;


	ret = oca_componet_codec_ops.add_codec_controls(oca72xxx->codec,
				oca72xxx_kcontrol, kcontrol_num);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "add codec controls failed, ret = %d",
			ret);
		return ret;
	}

	OCA_DEV_LOGI(oca72xxx->dev, "add public codec controls[%s]",
		oca72xxx_kcontrol[0].name);

	return 0;
}

/****************************************************************************
 *
 *oca72xxx kcontrol create
 *
 ****************************************************************************/
int oca72xxx_add_codec_controls(void *codec)
{
	struct list_head *pos = NULL;
	struct oca72xxx *oca72xxx = NULL;
	int ret = -1;

	list_for_each(pos, &g_oca72xxx_list) {
		oca72xxx = list_entry(pos, struct oca72xxx, list);
		ret = oca72xxx_kcontrol_dynamic_create(oca72xxx, codec);
		if (ret < 0)
			return ret;

		if (oca72xxx->dev_index == 0) {
			ret = oca72xxx_public_kcontrol_create(oca72xxx, codec);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL(oca72xxx_add_codec_controls);


/****************************************************************************
 *
 * oca72xxx firmware cfg load
 *
 ***************************************************************************/
static void oca72xxx_fw_cfg_free(struct oca72xxx *oca72xxx)
{
	OCA_DEV_LOGD(oca72xxx->dev, "enter");
	oca72xxx_acf_profile_free(oca72xxx->dev, &oca72xxx->acf_info);
	oca72xxx_monitor_cfg_free(&oca72xxx->monitor);
}

static int oca72xxx_init_default_prof(struct oca72xxx *oca72xxx)
{
	char *profile = NULL;

	profile = oca72xxx_acf_get_prof_off_name(oca72xxx->dev, &oca72xxx->acf_info);
	if (profile == NULL) {
		OCA_DEV_LOGE(oca72xxx->dev, "get profile off name failed");
		return -EINVAL;
	}

	snprintf(oca72xxx->prof_off_name, OCA_PROFILE_STR_MAX, "%s", profile);
	oca72xxx->current_profile = profile;
	OCA_DEV_LOGI(oca72xxx->dev, "init profile name [%s]",
		oca72xxx->current_profile);

	return 0;
}

static void oca72xxx_fw_load_work(struct work_struct *work)
{
	int ret = -1;
	struct oca72xxx *oca72xxx = container_of(work,
			struct oca72xxx, fw_load_work.work);
	struct acf_bin_info *acf_info = &oca72xxx->acf_info;
	const struct firmware *cont = NULL;

	OCA_DEV_LOGD(oca72xxx->dev, "enter");

	ret = request_firmware(&cont, oca72xxx->fw_name, oca72xxx->dev);
	if ((ret) || (!cont)) {
		OCA_DEV_LOGD(oca72xxx->dev, "load [%s] failed!", oca72xxx->fw_name);
		if (acf_info->load_count == OCA_READ_CHIPID_RETRIES) {
			acf_info->load_count = 0;
		} else {
			acf_info->load_count++;
			/* sleep 1s */
			msleep(1000);
			OCA_DEV_LOGD(oca72xxx->dev, "load [%s] try [%d]!",
						oca72xxx->fw_name, acf_info->load_count);
			oca72xxx_fw_load_work(work);
		}
		return;
	}

	OCA_DEV_LOGD(oca72xxx->dev, "loaded %s - size: %zu",
		oca72xxx->fw_name, (cont ? cont->size : 0));

	mutex_lock(&oca72xxx->reg_lock);
	acf_info->fw_data = vmalloc(cont->size);
	if (!acf_info->fw_data) {
		OCA_DEV_LOGE(oca72xxx->dev, "fw_data kzalloc memory failed");
		goto exit_vmalloc_failed;
	}
	memset(acf_info->fw_data, 0, cont->size);
	memcpy(acf_info->fw_data, cont->data, cont->size);
	acf_info->fw_size = cont->size;

	ret = oca72xxx_acf_parse(oca72xxx->dev, &oca72xxx->acf_info);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "fw_data parse failed");
		goto exit_acf_parse_failed;
	}

	ret = oca72xxx_init_default_prof(oca72xxx);
	if (ret < 0) {
		oca72xxx_fw_cfg_free(oca72xxx);
		goto exit_acf_parse_failed;
	}
	OCA_DEV_LOGI(oca72xxx->dev, "acf parse succeed");

exit_acf_parse_failed:
exit_vmalloc_failed:
	mutex_unlock(&oca72xxx->reg_lock);
	release_firmware(cont);
}

static void oca72xxx_fw_load_init(struct oca72xxx *oca72xxx)
{
#ifdef OCA_CFG_UPDATE_DELAY
	int cfg_timer_val = OCA_CFG_UPDATE_DELAY_TIMER;
#else
	int cfg_timer_val = 0;
#endif
	OCA_DEV_LOGI(oca72xxx->dev, "enter");
	snprintf(oca72xxx->fw_name, OCA72XXX_FW_NAME_MAX, "%s", OCA72XXX_FW_BIN_NAME);
	oca72xxx_acf_init(&oca72xxx->oca_dev, &oca72xxx->acf_info, oca72xxx->dev_index);

	INIT_DELAYED_WORK(&oca72xxx->fw_load_work, oca72xxx_fw_load_work);
	schedule_delayed_work(&oca72xxx->fw_load_work,
			msecs_to_jiffies(cfg_timer_val));
}

/****************************************************************************
 *
 *oca72xxx attribute node
 *
 ****************************************************************************/
static ssize_t oca72xxx_attr_get_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int ret = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_device *oca_dev = &oca72xxx->oca_dev;

	mutex_lock(&oca72xxx->reg_lock);
	for (i = 0; i < oca_dev->reg_max_addr; i++) {
		if (!(oca_dev->reg_access[i] & OCA_DEV_REG_RD_ACCESS))
			continue;
		ret = oca72xxx_dev_i2c_read_byte(&oca72xxx->oca_dev, i, &reg_val);
		if (ret < 0) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"read reg [0x%x] failed\n", i);
			OCA_DEV_LOGE(oca72xxx->dev, "read reg [0x%x] failed", i);
		} else {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_val);
			OCA_DEV_LOGD(oca72xxx->dev, "reg:0x%02X=0x%02X",
					i, reg_val);
		}
	}
	mutex_unlock(&oca72xxx->reg_lock);

	return len;
}

static ssize_t oca72xxx_attr_set_reg(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t len)
{
	unsigned int databuf[2] = { 0 };
	int ret = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);

	mutex_lock(&oca72xxx->reg_lock);
	if (sscanf(buf, "0x%x 0x%x", &databuf[0], &databuf[1]) == 2) {
		if (databuf[0] >= oca72xxx->oca_dev.reg_max_addr) {
			OCA_DEV_LOGE(oca72xxx->dev, "set reg[0x%x] error,is out of reg_addr_max[0x%x]",
				databuf[0], oca72xxx->oca_dev.reg_max_addr);
			mutex_unlock(&oca72xxx->reg_lock);
			return -EINVAL;
		}

		ret = oca72xxx_dev_i2c_write_byte(&oca72xxx->oca_dev,
					databuf[0], databuf[1]);
		if (ret < 0)
			OCA_DEV_LOGE(oca72xxx->dev, "set [0x%x]=0x%x failed",
				databuf[0], databuf[1]);
		else
			OCA_DEV_LOGD(oca72xxx->dev, "set [0x%x]=0x%x succeed",
				databuf[0], databuf[1]);
	} else {
		OCA_DEV_LOGE(oca72xxx->dev, "i2c write cmd input error");
	}
	mutex_unlock(&oca72xxx->reg_lock);

	return len;
}

static ssize_t oca72xxx_attr_get_profile(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_prof_info *prof_info = &oca72xxx->acf_info.prof_info;

	if (!prof_info->status) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"profile_cfg not load\n");
		return len;
	}

	OCA_DEV_LOGI(oca72xxx->dev, "current profile:[%s]", oca72xxx->current_profile);

	for (i = 0; i < prof_info->count; i++) {
		if (!strncmp(oca72xxx->current_profile, prof_info->prof_name_list[i],
				OCA_PROFILE_STR_MAX))
			len += snprintf(buf + len, PAGE_SIZE - len,
				">%s\n", prof_info->prof_name_list[i]);
		else
			len += snprintf(buf + len, PAGE_SIZE - len,
				" %s\n", prof_info->prof_name_list[i]);
	}

	return len;
}

static ssize_t oca72xxx_attr_set_profile(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t len)
{
	char profile[OCA_PROFILE_STR_MAX] = {0};
	int ret = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);

	if (strlen(buf) > OCA_PROFILE_STR_MAX) {
		OCA_DEV_LOGE(oca72xxx->dev, "input profile_str_len is out of max[%d]",
				OCA_PROFILE_STR_MAX);
		return -EINVAL;
	}

	if (sscanf(buf, "%s", profile) == 1) {
		OCA_DEV_LOGD(oca72xxx->dev, "set profile [%s]", profile);
		ret = oca72xxx_update_profile(oca72xxx, profile);
		if (ret < 0) {
			OCA_DEV_LOGE(oca72xxx->dev, "set profile[%s] failed",
				profile);
			return ret;
		}
	}

	return len;
}

static ssize_t oca72xxx_attr_get_hwen(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	int hwen = oca72xxx->oca_dev.hwen_status;

	if (hwen >= OCA_DEV_HWEN_INVALID)
		len += snprintf(buf + len, PAGE_SIZE - len, "hwen_status: invalid\n");
	else if (hwen == OCA_DEV_HWEN_ON)
		len += snprintf(buf + len, PAGE_SIZE - len, "hwen_status: on\n");
	else if (hwen == OCA_DEV_HWEN_OFF)
		len += snprintf(buf + len, PAGE_SIZE - len, "hwen_status: off\n");

	return len;
}

static ssize_t oca72xxx_attr_set_hwen(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t len)
{
	int ret = -1;
	unsigned int state;
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 0, &state);
	if (ret) {
		OCA_DEV_LOGE(oca72xxx->dev, "fail to channelge str to int");
		return ret;
	}

	mutex_lock(&oca72xxx->reg_lock);
	if (state == OCA_DEV_HWEN_OFF)
		oca72xxx_dev_hw_pwr_ctrl(&oca72xxx->oca_dev, false); /*OFF*/
	else if (state == OCA_DEV_HWEN_ON)
		oca72xxx_dev_hw_pwr_ctrl(&oca72xxx->oca_dev, true); /*ON*/
	else
		OCA_DEV_LOGE(oca72xxx->dev, "input [%d] error, hwen_on=[%d],hwen_off=[%d]",
			state, OCA_DEV_HWEN_ON, OCA_DEV_HWEN_OFF);
	mutex_unlock(&oca72xxx->reg_lock);
	return len;
}

int oca72xxx_ocarw_write(struct oca72xxx *oca72xxx,
			const char *buf, size_t count)
{
	int i = 0, ret = -1;
	char *data_buf = NULL;
	int buf_len = 0;
	int temp_data = 0;
	int data_str_size = 0;
	char *reg_data;
	struct oca_i2c_packet *packet = &oca72xxx->i2c_packet;

	OCA_DEV_LOGD(oca72xxx->dev, "enter");
	/* one addr or one data string Composition of Contains two bytes of symbol(0X)*/
	/* and two byte of hexadecimal data*/
	data_str_size = 2 + 2 * OCARW_DATA_BYTES;

	/* The buf includes the first address of the register to be written and all data */
	buf_len = OCARW_ADDR_BYTES + packet->reg_num * OCARW_DATA_BYTES;
	OCA_DEV_LOGI(oca72xxx->dev, "buf_len = %d,reg_num = %d", buf_len, packet->reg_num);
	data_buf = vmalloc(buf_len);
	if (data_buf == NULL) {
		OCA_DEV_LOGE(oca72xxx->dev, "alloc memory failed");
		return -ENOMEM;
	}
	memset(data_buf, 0, buf_len);

	data_buf[0] = packet->reg_addr;
	reg_data = data_buf + 1;

	OCA_DEV_LOGD(oca72xxx->dev, "reg_addr: 0x%02x", data_buf[0]);

	/*ag:0x00 0x01 0x01 0x01 0x01 0x00\x0a*/
	for (i = 0; i < packet->reg_num; i++) {
		ret = sscanf(buf + OCARW_HDR_LEN + 1 + i * (data_str_size + 1),
			"0x%x", &temp_data);
		if (ret != 1) {
			OCA_DEV_LOGE(oca72xxx->dev, "sscanf failed,ret=%d", ret);
			vfree(data_buf);
			data_buf = NULL;
			return ret;
		}
		reg_data[i] = temp_data;
		OCA_DEV_LOGD(oca72xxx->dev, "[%d] : 0x%02x", i, reg_data[i]);
	}

	mutex_lock(&oca72xxx->reg_lock);
	ret = i2c_master_send(oca72xxx->oca_dev.i2c, data_buf, buf_len);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "write failed");
		vfree(data_buf);
		data_buf = NULL;
		return -EFAULT;
	}
	mutex_unlock(&oca72xxx->reg_lock);

	vfree(data_buf);
	data_buf = NULL;

	OCA_DEV_LOGD(oca72xxx->dev, "down");
	return 0;
}

static int oca72xxx_ocarw_data_check(struct oca72xxx *oca72xxx,
			int *data, size_t count)
{
	struct oca_i2c_packet *packet = &oca72xxx->i2c_packet;
	int req_data_len = 0;
	int act_data_len = 0;
	int data_str_size = 0;

	if ((data[OCARW_HDR_ADDR_BYTES] != OCARW_ADDR_BYTES) ||
		(data[OCARW_HDR_DATA_BYTES] != OCARW_DATA_BYTES)) {
		OCA_DEV_LOGE(oca72xxx->dev, "addr_bytes [%d] or data_bytes [%d] unsupport",
			data[OCARW_HDR_ADDR_BYTES], data[OCARW_HDR_DATA_BYTES]);
		return -EINVAL;
	}

	/* one data string Composition of Contains two bytes of symbol(0x)*/
	/* and two byte of hexadecimal data*/
	data_str_size = 2 + 2 * OCARW_DATA_BYTES;
	act_data_len = count - OCARW_HDR_LEN - 1;

	/* There is a comma(,) or space between each piece of data */
	if (data[OCARW_HDR_WR_FLAG] == OCARW_FLAG_WRITE) {
		/*ag:0x00 0x01 0x01 0x01 0x01 0x00\x0a*/
		req_data_len = (data_str_size + 1) * packet->reg_num;
		if (req_data_len > act_data_len) {
			OCA_DEV_LOGE(oca72xxx->dev, "data_len checkfailed,requeset data_len [%d],actaul data_len [%d]",
				req_data_len, act_data_len);
			return -EINVAL;
		}
	}

	return 0;
}

/* flag addr_bytes data_bytes reg_num reg_addr*/
static int oca72xxx_ocarw_parse_buf(struct oca72xxx *oca72xxx,
			const char *buf, size_t count, int *wr_status)
{
	int data[OCARW_HDR_MAX] = {0};
	struct oca_i2c_packet *packet = &oca72xxx->i2c_packet;
	int ret = -1;

	if (sscanf(buf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
		&data[OCARW_HDR_WR_FLAG], &data[OCARW_HDR_ADDR_BYTES],
		&data[OCARW_HDR_DATA_BYTES], &data[OCARW_HDR_REG_NUM],
		&data[OCARW_HDR_REG_ADDR]) == 5) {

		packet->reg_addr = data[OCARW_HDR_REG_ADDR];
		packet->reg_num = data[OCARW_HDR_REG_NUM];
		*wr_status = data[OCARW_HDR_WR_FLAG];
		ret = oca72xxx_ocarw_data_check(oca72xxx, data, count);
		if (ret < 0)
			return ret;

		return 0;
	}

	return -EINVAL;
}

static ssize_t oca72xxx_attr_ocarw_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_i2c_packet *packet = &oca72xxx->i2c_packet;
	int wr_status = 0;
	int ret = -1;

	if (count < OCARW_HDR_LEN) {
		OCA_DEV_LOGE(oca72xxx->dev, "data count too smaller, please check write format");
		OCA_DEV_LOGE(oca72xxx->dev, "string %s,count=%ld",
			buf, (u_long)count);
		return -EINVAL;
	}

	OCA_DEV_LOGI(oca72xxx->dev, "string:[%s],count=%ld", buf, (u_long)count);
	ret = oca72xxx_ocarw_parse_buf(oca72xxx, buf, count, &wr_status);
	if (ret < 0) {
		OCA_DEV_LOGE(oca72xxx->dev, "can not parse string");
		return ret;
	}

	if (wr_status == OCARW_FLAG_WRITE) {
		ret = oca72xxx_ocarw_write(oca72xxx, buf, count);
		if (ret < 0)
			return ret;
	} else if (wr_status == OCARW_FLAG_READ) {
		packet->status = OCARW_I2C_ST_READ;
		OCA_DEV_LOGI(oca72xxx->dev, "read_cmd:reg_addr[0x%02x], reg_num[%d]",
			packet->reg_addr, packet->reg_num);
	} else {
		OCA_DEV_LOGE(oca72xxx->dev, "please check str format, unsupport read_write_status: %d",
			wr_status);
		return -EINVAL;
	}

	return count;
}

static ssize_t oca72xxx_attr_ocarw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct oca72xxx *oca72xxx = dev_get_drvdata(dev);
	struct oca_i2c_packet *packet = &oca72xxx->i2c_packet;
	int data_len = 0;
	size_t len = 0;
	int ret = -1, i = 0;
	char *reg_data = NULL;

	if (packet->status != OCARW_I2C_ST_READ) {
		OCA_DEV_LOGE(oca72xxx->dev, "please write read cmd first");
		return -EINVAL;
	}

	data_len = OCARW_DATA_BYTES * packet->reg_num;
	reg_data = (char *)vmalloc(data_len);
	if (reg_data == NULL) {
		OCA_DEV_LOGE(oca72xxx->dev, "memory alloc failed");
		ret = -EINVAL;
		goto exit;
	}

	mutex_lock(&oca72xxx->reg_lock);
	ret = oca72xxx_dev_i2c_read_msg(&oca72xxx->oca_dev, packet->reg_addr,
				(char *)reg_data, data_len);
	if (ret < 0) {
		ret = -EFAULT;
		mutex_unlock(&oca72xxx->reg_lock);
		goto exit;
	}
	mutex_unlock(&oca72xxx->reg_lock);

	OCA_DEV_LOGI(oca72xxx->dev, "reg_addr 0x%02x, reg_num %d",
		packet->reg_addr, packet->reg_num);

	for (i = 0; i < data_len; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"0x%02x,", reg_data[i]);
		OCA_DEV_LOGI(oca72xxx->dev, "0x%02x", reg_data[i]);
	}

	ret = len;

exit:
	if (reg_data) {
		vfree(reg_data);
		reg_data = NULL;
	}
	packet->status = OCARW_I2C_ST_NONE;
	return ret;
}

static ssize_t oca72xxx_drv_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
		"driver_ver: %s \n", OCA72XXX_DRIVER_VERSION);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
		oca72xxx_attr_get_reg, oca72xxx_attr_set_reg);
static DEVICE_ATTR(profile, S_IWUSR | S_IRUGO,
		oca72xxx_attr_get_profile, oca72xxx_attr_set_profile);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO,
		oca72xxx_attr_get_hwen, oca72xxx_attr_set_hwen);
static DEVICE_ATTR(ocarw, S_IWUSR | S_IRUGO,
	oca72xxx_attr_ocarw_show, oca72xxx_attr_ocarw_store);
static DEVICE_ATTR(drv_ver, S_IRUGO, oca72xxx_drv_ver_show, NULL);

static struct attribute *oca72xxx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_profile.attr,
	&dev_attr_hwen.attr,
	&dev_attr_ocarw.attr,
	&dev_attr_drv_ver.attr,
	NULL
};

static struct attribute_group oca72xxx_attribute_group = {
	.attrs = oca72xxx_attributes
};

/****************************************************************************
 *
 *oca72xxx device probe
 *
 ****************************************************************************/

int oca72xxx_dtsi_dev_index_check(struct oca72xxx *cur_oca72xxx)
{
	struct list_head *pos = NULL;
	struct oca72xxx *list_oca72xxx = NULL;

	list_for_each(pos, &g_oca72xxx_list) {
		list_oca72xxx = list_entry(pos, struct oca72xxx, list);
		if (list_oca72xxx->dev_index == cur_oca72xxx->dev_index) {
			OCA_DEV_LOGE(cur_oca72xxx->dev, "dev_index has already existing,check failed");
			return -EINVAL;
		}
	}

	return 0;
}

static int oca72xxx_dtsi_parse(struct oca72xxx *oca72xxx,
				struct device_node *dev_node)
{
	int ret = -1;
	int32_t dev_index = -EINVAL;
	int32_t voltage_min = -EINVAL;
	struct oca_voltage_desc *vol_desc = &oca72xxx->oca_dev.vol_desc;

	ret = of_property_read_u32(dev_node, "dev_index", &dev_index);
	if (ret < 0) {
		OCA_DEV_LOGI(oca72xxx->dev, "dev_index parse failed, user default[%d], ret=%d",
				g_oca72xxx_dev_cnt, ret);
		oca72xxx->dev_index = g_oca72xxx_dev_cnt;
	} else {
		oca72xxx->dev_index = dev_index;
		OCA_DEV_LOGI(oca72xxx->dev, "parse dev_index=[%d]",
				oca72xxx->dev_index);
	}

	ret = oca72xxx_dtsi_dev_index_check(oca72xxx);
	if (ret < 0)
		return ret;

	ret = of_get_named_gpio(dev_node, "reset-gpio", 0);
	if (ret < 0) {
		OCA_DEV_LOGI(oca72xxx->dev, "no reset gpio provided, hardware reset unavailable");
		oca72xxx->oca_dev.rst_gpio = OCA_NO_RESET_GPIO;
		oca72xxx->oca_dev.hwen_status = OCA_DEV_HWEN_INVALID;
	} else {
		oca72xxx->oca_dev.rst_gpio = ret;
		oca72xxx->oca_dev.hwen_status = OCA_DEV_HWEN_OFF;
		OCA_DEV_LOGI(oca72xxx->dev, "reset gpio[%d] parse succeed", ret);
		if (gpio_is_valid(oca72xxx->oca_dev.rst_gpio)) {
			ret = devm_gpio_request_one(oca72xxx->dev,
					oca72xxx->oca_dev.rst_gpio,
					GPIOF_OUT_INIT_LOW, "oca72xxx_reset");
			if (ret < 0) {
				OCA_DEV_LOGE(oca72xxx->dev, "reset request failed");
				return ret;
			}
		}
	}

	ret = of_property_read_u32(dev_node, "oca-voltage-min", &voltage_min);
	if (ret < 0) {
		OCA_DEV_LOGI(oca72xxx->dev, "voltage_min parse failed, user default[0x%02x]",
				OCA_BOOST_VOLTAGE_MIN);
		vol_desc->vol_min = OCA_BOOST_VOLTAGE_MIN;
	} else {
		vol_desc->vol_min = voltage_min;
		OCA_DEV_LOGI(oca72xxx->dev, "parse oca_voltage_min=[0x%02x]",
				vol_desc->vol_min);
	}
	
	return 0;
}

static struct oca72xxx *oca72xxx_malloc_init(struct i2c_client *client)
{
	struct oca72xxx *oca72xxx = NULL;

	oca72xxx = devm_kzalloc(&client->dev, sizeof(struct oca72xxx),
			GFP_KERNEL);
	if (oca72xxx == NULL) {
		OCA_DEV_LOGE(&client->dev, "failed to devm_kzalloc oca72xxx");
		return NULL;
	}
	memset(oca72xxx, 0, sizeof(struct oca72xxx));

	oca72xxx->dev = &client->dev;
	oca72xxx->oca_dev.dev = &client->dev;
	oca72xxx->oca_dev.i2c_bus = client->adapter->nr;
	oca72xxx->oca_dev.i2c_addr = client->addr;
	oca72xxx->oca_dev.i2c = client;
	oca72xxx->oca_dev.reg_access = NULL;
	oca72xxx->oca_dev.hwen_status = OCA_DEV_HWEN_INVALID;
	oca72xxx->off_bin_status = OCA72XXX_NO_OFF_BIN;
	oca72xxx->codec = NULL;
	oca72xxx->current_profile = oca72xxx->prof_off_name;
	oca72xxx->pa_status = 0;
	oca72xxx->last_profile = oca72xxx->prof_off_name;
	oca72xxx->kctl_profile = oca72xxx->prof_off_name;

	mutex_init(&oca72xxx->reg_lock);

	OCA_DEV_LOGI(&client->dev, "struct oca72xxx devm_kzalloc and init down");
	return oca72xxx;
}

#ifdef OCA_KERNEL_VER_OVER_6_6_0
static int oca72xxx_i2c_probe(struct i2c_client *client)
#else
static int oca72xxx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct device_node *dev_node = client->dev.of_node;
	struct oca72xxx *oca72xxx = NULL;
	int ret = -1;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		OCA_DEV_LOGE(&client->dev, "check_functionality failed");
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	/* oca72xxx i2c_dev struct init */
	oca72xxx = oca72xxx_malloc_init(client);
	if (oca72xxx == NULL)
		goto exit_malloc_init_failed;

	i2c_set_clientdata(client, oca72xxx);

	/* oca72xxx dev_node parse */
	ret = oca72xxx_dtsi_parse(oca72xxx, dev_node);
	if (ret < 0)
		goto exit_dtsi_parse_failed;

	/*hw power on PA*/
	oca72xxx_dev_hw_pwr_ctrl(&oca72xxx->oca_dev, true);

	/* oca72xxx devices private attributes init */
	ret = oca72xxx_dev_init(&oca72xxx->oca_dev);
	if (ret < 0)
		goto exit_device_init_failed;

	/*product register reset */
	oca72xxx_dev_soft_reset(&oca72xxx->oca_dev);

	/*hw power off */
	oca72xxx_dev_hw_pwr_ctrl(&oca72xxx->oca_dev, false);

	/* create debug attrbute nodes */
	ret = sysfs_create_group(&oca72xxx->dev->kobj, &oca72xxx_attribute_group);
	if (ret < 0)
		OCA_DEV_LOGE(oca72xxx->dev, "failed to create sysfs nodes, will not allowed to use");

	/* cfg_load init */
	oca72xxx_fw_load_init(oca72xxx);

	/*monitor init*/
	oca72xxx_monitor_init(oca72xxx->dev, &oca72xxx->monitor, dev_node);

	/*add device to total list */
	mutex_lock(&g_oca72xxx_mutex_lock);
	g_oca72xxx_dev_cnt++;
	list_add(&oca72xxx->list, &g_oca72xxx_list);
	mutex_unlock(&g_oca72xxx_mutex_lock);

	OCA_DEV_LOGI(oca72xxx->dev, "succeed, dev_index=[%d], g_oca72xxx_dev_cnt= [%d]",
			oca72xxx->dev_index, g_oca72xxx_dev_cnt);

	return 0;

exit_device_init_failed:
	oca72xxx_dev_hw_pwr_ctrl(&oca72xxx->oca_dev, false);
#if IS_ENABLED(CONFIG_MIEV)
	mievent_report(906001351, "PA internal exception", oca72xxx->dev);
#endif

exit_dtsi_parse_failed:
	OCA_DEV_LOGE(oca72xxx->dev, "pa init failed");
#if IS_ENABLED(CONFIG_MIEV)
	mievent_report(906001353,"PA i2c exception", oca72xxx->dev);
#endif

	devm_kfree(&client->dev, oca72xxx);
	oca72xxx = NULL;
exit_malloc_init_failed:
exit_check_functionality_failed:
	return ret;
}

#ifdef OCA_KERNEL_VER_OVER_6_1_0
static void oca72xxx_i2c_remove(struct i2c_client *client)
#else
static int oca72xxx_i2c_remove(struct i2c_client *client)
#endif
{
	struct oca72xxx *oca72xxx = i2c_get_clientdata(client);

	oca72xxx_monitor_exit(&oca72xxx->monitor);

	/*rm attr node*/
	sysfs_remove_group(&oca72xxx->dev->kobj, &oca72xxx_attribute_group);

	oca72xxx_fw_cfg_free(oca72xxx);

	mutex_lock(&g_oca72xxx_mutex_lock);
	g_oca72xxx_dev_cnt--;
	list_del(&oca72xxx->list);
	mutex_unlock(&g_oca72xxx_mutex_lock);

	devm_kfree(&client->dev, oca72xxx);
	oca72xxx = NULL;

#ifdef OCA_KERNEL_VER_OVER_6_1_0
#else
	return 0;
#endif
}

static void oca72xxx_i2c_shutdown(struct i2c_client *client)
{
	struct oca72xxx *oca72xxx = i2c_get_clientdata(client);

	OCA_DEV_LOGI(&client->dev, "enter");

	/*soft and hw power off*/
	oca72xxx_update_profile(oca72xxx, oca72xxx->prof_off_name);
}


static const struct i2c_device_id oca72xxx_i2c_id[] = {
	{OCA72XXX_I2C_NAME, 0},
	{},
};

static const struct of_device_id extpa_of_match[] = {
	{.compatible = "ocs,oca72xxx_pa"},
	{},
};

static struct i2c_driver oca72xxx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OCA72XXX_I2C_NAME,
		.of_match_table = extpa_of_match,
		},
	.probe = oca72xxx_i2c_probe,
	.remove = oca72xxx_i2c_remove,
	.shutdown = oca72xxx_i2c_shutdown,
	.id_table = oca72xxx_i2c_id,
};

static int __init ocspa_init(void)
{
	int ret;

	OCA_LOGI("driver version: %s", OCA72XXX_DRIVER_VERSION);

	ret = i2c_add_driver(&oca72xxx_i2c_driver);
	if (ret < 0) {
		OCA_LOGE("Unable to register driver, ret= %d", ret);
		return ret;
	}
	return 0;
}

static void __exit ocspa_exit(void)
{
	OCA_LOGI("enter");
	i2c_del_driver(&oca72xxx_i2c_driver);
}

module_init(ocspa_init);
module_exit(ocspa_exit);

int mievent_report(unsigned int eventid,const char *value,struct device *dev)
{
#if IS_ENABLED(CONFIG_MIEV)
	struct misight_mievent *mievent;
	char i2c_info[20];
	sprintf(i2c_info,"%s",dev->kobj.name);
	dev_info(dev, "%s: reg = %s, KeyWord = %s DFS report\n", __func__, i2c_info,value);
	mievent  = cdev_tevent_alloc(eventid);
	cdev_tevent_add_str(mievent, "I2cAddress",i2c_info);
	cdev_tevent_add_str(mievent, "Keyword", value);
	cdev_tevent_write(mievent);
	cdev_tevent_destroy(mievent);
#endif
	return 0;
}

MODULE_AUTHOR("<Wall@orient-chip.com>");
MODULE_DESCRIPTION("ocs oca72xxx amp driver");
MODULE_LICENSE("GPL v2");
