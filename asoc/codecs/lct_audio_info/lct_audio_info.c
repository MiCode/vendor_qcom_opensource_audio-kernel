// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 XiaoMi Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <asm/setup.h>
#include <linux/bootconfig.h>

#define AUDIO_INFO_NAME_MAX     (64)
#define CN_HW_NAME     "androidboot.hwname=flame"
#define GL_HW_NAME     "androidboot.hwname=blaze"

struct lct_audio_info {
    char codec_name[AUDIO_INFO_NAME_MAX];
    char pa_name[AUDIO_INFO_NAME_MAX];
};

static DEFINE_MUTEX(g_audio_info_mutex_lock);
static struct lct_audio_info g_audio_info;

static char __platform_cmdline[2048];
static char *g_cmd_line = __platform_cmdline;

/* get cmdline from bootargs */
const char *get_cmdline_from_bootargs(void)
{
	struct device_node *of_chosen = NULL;
	char *bootargs = NULL;

	if (__platform_cmdline[0] != 0) {
		return g_cmd_line;
	}
	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen) {
		bootargs = (char *)of_get_property(of_chosen, "bootargs", NULL);
		if (!bootargs) {
			pr_err("%s: failed to get bootargs\n", __func__);
        } else {
			strcpy(__platform_cmdline, bootargs);
		}
	} else {
		pr_err("%s: failed to get /chosen\n", __func__);
	}
	return g_cmd_line;
}

/* 1:CN  0:GL */
int distinguish_audio_pa_by_cmdline(void)
{
    get_cmdline_from_bootargs();
    if(strstr(g_cmd_line, CN_HW_NAME)){
        return 1;
    } else if (strstr(g_cmd_line, GL_HW_NAME)){
        return 0;
    }else {
        return -EINVAL;
    }
}
EXPORT_SYMBOL(distinguish_audio_pa_by_cmdline);

static ssize_t audio_codec_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
    ssize_t ret = -EINVAL;

    if(0 == strlen(g_audio_info.codec_name))
       ret = sprintf(buf, "%s\n", "WCD9370");
    else
       ret = sprintf(buf, "%s\n", g_audio_info.codec_name);
    return ret;
}

static ssize_t audio_pa_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
    ssize_t ret = -EINVAL;

    if(0 == strlen(g_audio_info.pa_name))
       ret = sprintf(buf, "%s\n", "unkonw_pa");
    else
       ret = sprintf(buf, "%s\n", g_audio_info.pa_name);
    return ret;
}

static struct kobj_attribute dev_attr_audio_codec =
		__ATTR(audio_codec, 0644, audio_codec_show, NULL);
static struct kobj_attribute dev_attr_audio_pa =
		__ATTR(audio_pa, 0644, audio_pa_show, NULL);

static struct kobject *android_audio;

int lct_audio_info_create_sysfs(void)
{
    int ret = -EINVAL;

    if (android_audio != NULL) {
        pr_err("android_audio already exists\n");
        return ret;
    }

    android_audio = kobject_create_and_add("android_audio", NULL);
    if(android_audio == NULL) {
        pr_err(" android_audio_create_sysfs_ failed\n");
        ret = -ENOMEM;
        return ret;
    }

    ret = sysfs_create_file(android_audio, &dev_attr_audio_codec.attr);
    if(ret) {
        pr_err("%s audio_codec failed \n", __func__);
        kobject_del(android_audio);
        return ret;
    }

    ret = sysfs_create_file(android_audio, &dev_attr_audio_pa.attr);
    if(ret) {
        pr_err("%s audio_pa failed \n", __func__);
        kobject_del(android_audio);
        return ret;
    }

    g_audio_info.codec_name[AUDIO_INFO_NAME_MAX - 1] = '\0';
    g_audio_info.pa_name[AUDIO_INFO_NAME_MAX - 1] = '\0';

    return 0;
}
EXPORT_SYMBOL(lct_audio_info_create_sysfs);

int lct_audio_info_set_codec_name(const char *codec_name, int count)
{
    int max_count = count>AUDIO_INFO_NAME_MAX?AUDIO_INFO_NAME_MAX:count;
    mutex_lock(&g_audio_info_mutex_lock);
    strncpy(g_audio_info.codec_name, codec_name, max_count);
    mutex_unlock(&g_audio_info_mutex_lock);
    return strlen(g_audio_info.codec_name);
}
EXPORT_SYMBOL(lct_audio_info_set_codec_name);

int lct_audio_info_set_pa_name(const char *pa_name, int count)
{
    int max_count = count>AUDIO_INFO_NAME_MAX?AUDIO_INFO_NAME_MAX:count;
    mutex_lock(&g_audio_info_mutex_lock);
    strncat(g_audio_info.pa_name, pa_name, max_count);
    g_audio_info.pa_name[AUDIO_INFO_NAME_MAX - 1] = '\0';
    mutex_unlock(&g_audio_info_mutex_lock);
    return strlen(g_audio_info.pa_name);
}
EXPORT_SYMBOL(lct_audio_info_set_pa_name);

MODULE_DESCRIPTION("audio pa info driver");
MODULE_LICENSE("GPL v2");


