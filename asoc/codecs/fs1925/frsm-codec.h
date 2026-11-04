/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-07-20 File created.
 */

#ifndef __FRSM_CODEC_H__
#define __FRSM_CODEC_H__

#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include "internal.h"

/* Supported rates and data formats */
#define FRSM_CODEC_RATES (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|\
		SNDRV_PCM_RATE_32000|SNDRV_PCM_RATE_44100|\
		SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_88200|\
		SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_176400|\
		SNDRV_PCM_RATE_192000)
#define FRSM_CODEC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE|\
		SNDRV_PCM_FMTBIT_S24_LE|\
		SNDRV_PCM_FMTBIT_S24_3LE|\
		SNDRV_PCM_FMTBIT_S32_LE)
#define FRSM_SOC_ENUM_EXT(xname, xhandler_info, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = xhandler_info, \
	.get = xhandler_get, .put = xhandler_put }

static const unsigned int frsm_codec_rates[] = {
	8000, 16000, 32000, 44100, 48000, 88200, 96000, 176400, 192000
};
static const struct snd_pcm_hw_constraint_list frsm_constraints = {
	.list = frsm_codec_rates,
	.count = ARRAY_SIZE(frsm_codec_rates),
};

static struct frsm_dev *frsm_get_drvdata_from_dai(struct snd_soc_dai *dai)
{
	if (dai == NULL || dai->component == NULL) {
		pr_err("frsm_dev dai is null");
		return NULL;
	}

	return snd_soc_component_get_drvdata(dai->component);
}

static struct frsm_dev *frsm_get_drvdata_from_kctrl(struct snd_kcontrol *kctrl)
{
	struct snd_soc_component *cmpnt;

	if (kctrl == NULL) {
		pr_err("frsm_dev kcontrol is null");
		return NULL;
	}

	cmpnt = snd_soc_kcontrol_component(kctrl);
	if (cmpnt == NULL) {
		pr_err("frsm_dev cmpnt is null");
		return NULL;
	}

	return snd_soc_component_get_drvdata(cmpnt);
}

static struct frsm_dev *frsm_get_drvdata_from_widget(
		struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_component *cmpnt;

	if (widget == NULL) {
		pr_err("frsm_dev widget is null");
		return NULL;
	}

	cmpnt = snd_soc_dapm_to_component(widget->dapm);
	if (cmpnt == NULL) {
		pr_err("frsm_dev cmpnt is null");
		return NULL;
	}

	return snd_soc_component_get_drvdata(cmpnt);
}

static struct snd_soc_dapm_context *frsm_get_dapm_from_cmpnt(
		struct snd_soc_component *cmpnt)
{
	return snd_soc_component_get_dapm(cmpnt);
}

static int frsm_append_name_prefix(struct frsm_dev *frsm_dev, const char **name)
{
	char str[FRSM_STRING_MAX];
	const char *str_prefix;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	str_prefix = frsm_dev->pdata->name_prefix;
	if (str_prefix == NULL || name == NULL || *name == NULL)
		return 0;

	snprintf(str, sizeof(str), "%s %s", str_prefix, *name);

	*name = devm_kstrdup(frsm_dev->dev, str, GFP_KERNEL);
	if (*name == NULL)
		return -ENOMEM;

	dev_dbg(frsm_dev->dev, "append name:%s\n", *name);

	return 0;
}

static int frsm_dai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct frsm_dev *frsm_dev;
	int ret;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL) {
		pr_err("dai_startup: frsm_dev is null\n");
		return -EINVAL;
	}

	if (substream->runtime == NULL) {
		dev_dbg(frsm_dev->dev, "substream runtime is null\n");
		goto func_exit;
	}

	ret = snd_pcm_hw_constraint_mask64(substream->runtime,
			SNDRV_PCM_HW_PARAM_FORMAT, FRSM_CODEC_FORMATS);
	if (ret < 0) {
		dev_err(frsm_dev->dev,
				"Failed to set param format: %d\n", ret);
		return ret;
	}

	ret = snd_pcm_hw_constraint_list(substream->runtime,
			0, SNDRV_PCM_HW_PARAM_RATE, &frsm_constraints);
	if (ret < 0) {
		dev_err(frsm_dev->dev,
				"Failed to set param rate: %d\n", ret);
		return ret;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

func_exit:
	dev_dbg(frsm_dev->dev, "dai: start up\n");
	ret = frsm_send_event(frsm_dev, EVENT_DEV_INIT);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return 0;
}

static int frsm_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct frsm_dev *frsm_dev;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("dai_sysclk: frsm_dev is null\n");
		return -EINVAL;
	}

	if (freq == frsm_dev->hw_params.bclk)
		return 0;

	dev_dbg(frsm_dev->dev, "dai: sysclk: %d\n", freq);
	frsm_dev->hw_params.bclk = freq;

	return 0;
}

static int frsm_dai_format(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct frsm_hw_params *hw_params;
	struct frsm_dev *frsm_dev;
	unsigned int format;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL) {
		pr_err("dai_format: frsm_dev is null\n");
		return -EINVAL;
	}

	dev_info(frsm_dev->dev, "dai: format: 0x%X\n", fmt);

	format = fmt & SND_SOC_DAIFMT_MASTER_MASK;
	switch (format) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* Only supports slave mode */
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
	default:
		dev_err(frsm_dev->dev, "Invalid DAI master/slave mode\n");
		return -EINVAL;
	}

	hw_params = &frsm_dev->hw_params;
	format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	switch (format) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		hw_params->format = format;
		hw_params->offset = (format == SND_SOC_DAIFMT_DSP_A) ? 1 : 0;
		break;
	default:
		dev_err(frsm_dev->dev, "Invalid DAI format: %d\n", format);
		return -EINVAL;
	}

	return 0;
}

static int frsm_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct frsm_hw_params *hw_params;
	struct frsm_dev *frsm_dev;
	unsigned int format;
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("dai_hw_params: frsm_dev is null\n");
		return -EINVAL;
	}

	hw_params = &frsm_dev->hw_params;
	hw_params->rate = params_rate(params);
	hw_params->channels = params_channels(params);
	format = params_format(params);
	hw_params->bit_width = snd_pcm_format_physical_width(format);

	dev_dbg(frsm_dev->dev,
			"dai: stream.%d RATE:%d CHN:%d WIDTH:%d\n",
			substream->stream, hw_params->rate,
			hw_params->channels, hw_params->bit_width);

	if (hw_params->channels < 2)
		hw_params->channels = 2;
	else if (hw_params->channels == 3)
		hw_params->channels = 4;

	if (hw_params->bclk == 0)
		hw_params->bclk = hw_params->rate
				* hw_params->channels
				* hw_params->bit_width;

	ret = frsm_send_event(frsm_dev, EVENT_HW_PARAMS);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_dai_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct frsm_dev *frsm_dev;
	enum frsm_event event;
	int ret;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("dai_mute: frsm_dev is null\n");
		return -EINVAL;
	}

	if (stream != SNDRV_PCM_STREAM_PLAYBACK) {
		dev_dbg(frsm_dev->dev, "dai: capture %s\n",
				mute ? "mute" : "unmute");
		return 0;
	}

	if (mute) {
		event = EVENT_STREAM_OFF;
		dev_info(frsm_dev->dev, "dai: playback mute\n");
	} else {
		event = EVENT_STREAM_ON;
		dev_info(frsm_dev->dev, "dai: playback unmute\n");
	}

	ret = frsm_send_amp_event(event, NULL, 0);
	ret = frsm_send_event(frsm_dev, event);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return 0;
}

static int frsm_dai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct frsm_dev *frsm_dev;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("dai_trigger: frsm_dev is null\n");
		return -EINVAL;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(frsm_dev->dev, "trigger start:%d\n", cmd);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_dbg(frsm_dev->dev, "trigger stop:%d\n", cmd);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void frsm_dai_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct frsm_dev *frsm_dev;
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	frsm_dev = frsm_get_drvdata_from_dai(dai);
	if (frsm_dev == NULL) {
		pr_err("dai_shutdown: frsm_dev is null\n");
		return;
	}

	dev_dbg(frsm_dev->dev, "dai: shut down\n");
	ret = frsm_send_event(frsm_dev, EVENT_SET_IDLE);
	frsm_dev->hw_params.bclk = 0;

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
}

static const struct snd_soc_dai_ops frsm_codec_dai_ops = {
	.startup = frsm_dai_startup,
	.set_sysclk = frsm_dai_sysclk,
	.set_fmt = frsm_dai_format,
	.hw_params = frsm_dai_hw_params,
	.mute_stream = frsm_dai_mute,
	.trigger = frsm_dai_trigger,
	.shutdown = frsm_dai_shutdown,
	//.no_capture_mute = 1,
};

static struct snd_soc_dai_driver frsm_codec_dai = {
	.name = "frsm-codec-aif",
	.id = 1,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = FRSM_CODEC_RATES,
		.formats = FRSM_CODEC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = FRSM_CODEC_RATES,
		.formats = FRSM_CODEC_FORMATS,
	},
	.ops = &frsm_codec_dai_ops,
#if KERNEL_VERSION_HIGHER(5, 12, 0)
	.symmetric_rate = 1,
#else
	.symmetric_rates = 1,
#endif
};

static int frsm_codec_switch_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	bool state_on;

	if (frsm_dev == NULL) {
		pr_err("switch_get: frsm_dev is null\n");
		return -EINVAL;
	}

	state_on = test_bit(EVENT_STREAM_ON, &frsm_dev->state);
	ucontrol->value.integer.value[0] = state_on;

	return 0;
}

static int frsm_codec_switch_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	bool state_on, turn_on;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL) {
		pr_err("switch_put: frsm_dev is null\n");
		return -EINVAL;
	}

	state_on = test_bit(EVENT_STREAM_ON, &frsm_dev->state);
	turn_on = !!ucontrol->value.integer.value[0];
	if ((state_on  ^ turn_on) == 0)
		return 0;

	dev_info(frsm_dev->dev, "Switch put: %s\n", turn_on ? "On" : "Off");
	if (turn_on) {
		ret  = frsm_send_event(frsm_dev, EVENT_HW_PARAMS);
		ret |= frsm_send_event(frsm_dev, EVENT_STREAM_ON);
	} else {
		ret = frsm_send_event(frsm_dev, EVENT_SET_IDLE);
	}

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_codec_scene_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	struct frsm_dev *frsm_dev;
	struct scene_table *scene;
	const char *str = NULL;
	int offset;
	int count;

	frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	if (frsm_dev->tbl_scene) {
		scene = (struct scene_table *)frsm_dev->tbl_scene->buf;
		count = frsm_dev->tbl_scene->size / sizeof(*scene) - 1;
		uinfo->value.enumerated.items = count;
		if (uinfo->value.enumerated.item >= count)
			uinfo->value.enumerated.item = count - 1;
		offset = (scene + uinfo->value.enumerated.item + 1)->name;
		frsm_get_fwm_string(frsm_dev, offset, &str);
	} else {
		dev_err(frsm_dev->dev, "Failed to find scene table\n");
		count = 0;
	}

	if (count == 0 || str == NULL) {
		uinfo->value.enumerated.items = 1;
		uinfo->value.enumerated.item = 0;
		str = "None";
	}

	strscpy(uinfo->value.enumerated.name, str,
			sizeof(uinfo->value.enumerated.name));
	dev_dbg(frsm_dev->dev, "Scene info.%d: %s\n", count, str);

	return 0;
}

static int frsm_codec_scene_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev;
	int scene_id;

	frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	if (frsm_dev == NULL || frsm_dev->dev == NULL) {
		pr_err("scene_get: frsm_dev is null\n");
		return -EINVAL;
	}

	if (frsm_dev->next_scene <= 0)
		scene_id = 0;
	else
		scene_id = frsm_dev->next_scene;

	dev_dbg(frsm_dev->dev, "Scene get: %d\n", scene_id);
	ucontrol->value.integer.value[0] = scene_id;

	return 0;
}

static int frsm_codec_scene_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev;
	struct scene_table *scene;
	struct fwm_table *table;
	int scene_id;
	int ret;

	frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	if (frsm_dev == NULL || frsm_dev->dev == NULL) {
		pr_err("scene_put: frsm_dev is null\n");
		return -EINVAL;
	}

	table = frsm_get_fwm_table(frsm_dev, INDEX_SCENE);
	if (table == NULL)
		return -EINVAL;

	scene = (struct scene_table *)table->buf;
	scene_id = (char)ucontrol->value.integer.value[0];
	if (scene_id < 0 || scene_id + 1 >= table->size / sizeof(*scene)) {
		dev_err(frsm_dev->dev, "Invalid scene index:%d\n", scene_id);
		return -EINVAL;
	}

	if (scene_id == frsm_dev->next_scene)
		return 0;

	dev_info(frsm_dev->dev, "Switch scene: %d->%d\n",
			frsm_dev->next_scene, scene_id);
	frsm_dev->next_scene = scene_id;
	ret = frsm_send_event(frsm_dev, EVENT_SET_SCENE);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_codec_monitor_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	int state;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("monitor_get: frsm_dev is null\n");
		return -EINVAL;
	}

	state = frsm_dev->pdata->mntr_enable;
	ucontrol->value.integer.value[0] = state;

	return 0;
}

static int frsm_codec_monitor_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_platform_data *pdata;
	struct frsm_dev *frsm_dev;
	bool enable, mntr_on;
	int ret;

	frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("monitor_put: frsm_dev is null\n");
		return -EINVAL;
	}

	pdata = frsm_dev->pdata;
	enable = !!ucontrol->value.integer.value[0];
	if (enable) {
		dev_info(frsm_dev->dev, "Monitor put: On\n");
		mntr_on = test_bit(EVENT_STAT_MNTR, &frsm_dev->state);
		if (mntr_on && !pdata->mntr_enable) {
			ret = frsm_stub_mntr_switch(frsm_dev, false);
			FRSM_DELAY_MS(20); // 20ms
		}
		pdata->mntr_enable = true;
		ret = frsm_stub_mntr_switch(frsm_dev, true);
	} else {
		dev_info(frsm_dev->dev, "Monitor put: Off\n");
		ret = frsm_stub_mntr_switch(frsm_dev, false);
		pdata->mntr_enable = false;
	}

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return 0;
}

static int frsm_codec_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct frsm_dev *frsm_dev;
	int ret;

	frsm_dev = frsm_get_drvdata_from_widget(w);
	if (frsm_dev == NULL)
		return -EINVAL;

	dev_dbg(frsm_dev->dev, "DAC Event: %d\n", event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(frsm_dev->dev, "DAC Event: POST_PMU\n");
		ret = frsm_send_event(frsm_dev, EVENT_START_UP);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(frsm_dev->dev, "DAC Event: PRE_PMD\n");
		ret = frsm_send_event(frsm_dev, EVENT_SHUT_DOWN);
		break;
	default:
		ret = 0;
	}

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

int frsm_codec_volume_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);

	if (frsm_dev == NULL) {
		pr_err("volume_get: frsm_dev is null\n");
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = frsm_dev->volume;
	dev_dbg(frsm_dev->dev, "Volume get: %d", frsm_dev->volume);

	return 0;
}

int frsm_codec_volume_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;

	if (frsm_dev == NULL) {
		pr_err("volume_put: frsm_dev is null\n");
		return -EINVAL;
	}

	frsm_dev->volume = ucontrol->value.integer.value[0];
	if (frsm_dev->volume < mc->min || frsm_dev->volume > mc->max)
		return -EINVAL;

	dev_info(frsm_dev->dev, "Volume set: %d\n", frsm_dev->volume);
	ret = frsm_send_event(frsm_dev, EVENT_SET_VOL);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

int frsm_codec_vol_step_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);

	if (frsm_dev == NULL) {
		pr_err("volume_get: frsm_dev is null\n");
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = frsm_dev->vol_step_down;
	dev_dbg(frsm_dev->dev, "Volume step down get: %d", frsm_dev->vol_step_down);

	return 0;
}

int frsm_codec_vol_step_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;

	if (frsm_dev == NULL) {
		pr_err("volume_put: frsm_dev is null\n");
		return -EINVAL;
	}

	frsm_dev->vol_step_down = ucontrol->value.integer.value[0];
	if (frsm_dev->vol_step_down < mc->min || frsm_dev->vol_step_down > mc->max)
		return -EINVAL;

	dev_info(frsm_dev->dev, "Volume step down set: %d\n", frsm_dev->vol_step_down);
	ret = frsm_send_event(frsm_dev, EVENT_SET_VOL);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

int frsm_codec_safe_vol_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);

	if (frsm_dev == NULL) {
		pr_err("safe_vol_get: frsm_dev is null\n");
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = frsm_dev->safe_vol;
	dev_dbg(frsm_dev->dev, "Safe vol get: %d", frsm_dev->safe_vol);

	return 0;
}

int frsm_codec_safe_vol_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;

	if (frsm_dev == NULL) {
		pr_err("safe_vol_put: frsm_dev is null\n");
		return -EINVAL;
	}

	frsm_dev->safe_vol = ucontrol->value.integer.value[0];
	if (frsm_dev->safe_vol < mc->min || frsm_dev->safe_vol > mc->max)
		return -EINVAL;

	dev_info(frsm_dev->dev, "Safe vol set: %d\n", frsm_dev->safe_vol);
	ret = frsm_send_event(frsm_dev, EVENT_SET_VOL);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_codec_swap_channel_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);

	if (frsm_dev == NULL) {
		pr_err("swap_channel_get: frsm_dev is null\n");
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = frsm_dev->swap_channel;

	return 0;
}

static int frsm_codec_swap_channel_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	bool swap_on;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("swap_channel_put: frsm_dev is null\n");
		return -EINVAL;
	}

	swap_on = !!ucontrol->value.integer.value[0];
	dev_info(frsm_dev->dev, "Swap channel put: %s\n",
			swap_on ? "On" : "Off");
	frsm_dev->swap_channel = swap_on;
	ret = frsm_send_event(frsm_dev, EVENT_SET_CHANN);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_codec_spksw_gpio_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	struct frsm_gpio *gpio;
	int voltage_level = 0;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("frsm_dev or pdate is null\n");
		return -EINVAL;
	}

	gpio = frsm_dev->pdata->gpio + FRSM_PIN_SWITCH;
	if (!IS_ERR_OR_NULL(gpio->gpiod)) {
		voltage_level = gpiod_get_value(gpio->gpiod);
		if (voltage_level < 0) {
			dev_err(frsm_dev->dev, "Failed to get GPIO value: %d\n", voltage_level);
			voltage_level = 0;
		}
	}

	ucontrol->value.integer.value[0] = voltage_level;

	return 0;
}

static int frsm_codec_spksw_gpio_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	struct frsm_gpio *gpio;
	int voltage_level;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("frsm_dev or pdate is null\n");
		return -EINVAL;
	}

	voltage_level = ucontrol->value.integer.value[0];
	dev_info(frsm_dev->dev, "set spksw gpio: %s\n",
			voltage_level ? "High" : "Low");
	gpio = frsm_dev->pdata->gpio + FRSM_PIN_SWITCH;
	if (!IS_ERR_OR_NULL(gpio->gpiod)) {
		gpiod_set_value(gpio->gpiod, !!voltage_level);
	}

	return 0;
}

#ifdef CONFIG_FRSM_MIXER_DTYPE
static int frsm_dtype_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(uint16_t) * (FRSM_DEV_MAX + 1);

	return 0;
}

int frsm_dtype_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct frsm_dev *frsm_dev = frsm_get_drvdata_from_kctrl(kcontrol);
	uint16_t params[FRSM_DEV_MAX+1] = { 0 };
	int ret;

	ret = frsm_send_amp_event(EVENT_GET_DTYPE,
			params, sizeof(params));
	if (ret && ret != -ENOTSUPP) {
		dev_err(frsm_dev->dev, "Failed to get dtype:%d\n", ret);
		return ret;
	}

	if (params[0] < 1 || params[0] > FRSM_DEV_MAX) {
		dev_err(frsm_dev->dev, "Invalid ndev of dtype:%d\n", params[0]);
		return -EINVAL;
	}

	/* ndev,dtype1,...,dtypeN */
	memcpy(ucontrol->value.bytes.data, params, sizeof(params));

	return 0;
}

static bool dtype_ctrl_inited;
static const struct snd_kcontrol_new frsm_dtype_control[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "FSM_Dev",
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.info = frsm_dtype_info,
		.get = frsm_dtype_get,
	},
};

static int frsm_add_dtype_kcontrol(struct snd_soc_component *cmpnt)
{
	if (dtype_ctrl_inited)
		return 0;

	dev_info(cmpnt->dev, "Add dtype control\n");
	dtype_ctrl_inited = true;
	return snd_soc_add_component_controls(cmpnt,
			frsm_dtype_control, ARRAY_SIZE(frsm_dtype_control));
}
#endif

static const char *const frsm_codec_switch[] = { "Off", "On" };
static const struct soc_enum frsm_codec_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(frsm_codec_switch), frsm_codec_switch),
};

static const struct snd_kcontrol_new frsm_codec_controls[] = {
	SOC_ENUM_EXT("FSAMP Switch", frsm_codec_enum[0],
			frsm_codec_switch_get, frsm_codec_switch_put),
	FRSM_SOC_ENUM_EXT("FSAMP Scene", frsm_codec_scene_info,
			frsm_codec_scene_get, frsm_codec_scene_put),
	SOC_ENUM_EXT("FSAMP Monitor", frsm_codec_enum[0],
			frsm_codec_monitor_get, frsm_codec_monitor_put),
	SOC_SINGLE_EXT("FSAMP Volume", SND_SOC_NOPM, 0, FRSM_VOLUME_MAX, 0,
			frsm_codec_volume_get, frsm_codec_volume_put),
	SOC_SINGLE_EXT("FSAMP Volume Step", SND_SOC_NOPM, 0, FRSM_VOLUME_MAX, 0,
			frsm_codec_vol_step_get, frsm_codec_vol_step_put),
	SOC_SINGLE_EXT("FSAMP Ramp Volume", SND_SOC_NOPM, 0, FRSM_VOLUME_MAX, 0,
			frsm_codec_safe_vol_get, frsm_codec_safe_vol_put),
	SOC_ENUM_EXT("FSAMP Swap Channel", frsm_codec_enum[0],
			frsm_codec_swap_channel_get,
			frsm_codec_swap_channel_put),
	SOC_SINGLE_EXT("FSAMP SW GPIO",  SND_SOC_NOPM, 0, 1, 0,
			frsm_codec_spksw_gpio_get, frsm_codec_spksw_gpio_put),
};

static const struct snd_kcontrol_new frsm_dac_port[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static struct snd_soc_dapm_widget frsm_codec_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MIXER_E("FSAMP DAC_Port", SND_SOC_NOPM, 0, 0,
		frsm_dac_port, ARRAY_SIZE(frsm_dac_port),
		frsm_codec_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_INPUT("SDO"),
};

static const struct snd_soc_dapm_route frsm_codec_routes[] = {
	{ "FSAMP DAC_Port", "Switch", "AIF IN" },
	{ "OUT", NULL, "FSAMP DAC_Port" },
	{ "AIF OUT", NULL, "SDO" },
};

static int frsm_init_cmpnt_controls(struct frsm_dev *frsm_dev)
{
	struct snd_kcontrol_new *new_kctrl;
	int i, count;

	new_kctrl = devm_kzalloc(frsm_dev->dev,
			sizeof(frsm_codec_controls), GFP_KERNEL);
	if (new_kctrl == NULL)
		return -ENOMEM;

	memcpy(new_kctrl, frsm_codec_controls, sizeof(frsm_codec_controls));
	count = ARRAY_SIZE(frsm_codec_controls);

	for (i = 0; i < count; i++)
		frsm_append_name_prefix(frsm_dev,
				(const char **)&new_kctrl[i].name);

	return snd_soc_add_component_controls(
			frsm_dev->cmpnt, new_kctrl, count);
}

static int frsm_init_dapm_controls(struct frsm_dev *frsm_dev,
		struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_widget *new_widget;
	int i, count;

	new_widget = devm_kzalloc(frsm_dev->dev,
			sizeof(frsm_codec_widgets), GFP_KERNEL);
	if (new_widget == NULL)
		return -ENOMEM;

	memcpy(new_widget, frsm_codec_widgets, sizeof(frsm_codec_widgets));
	count = ARRAY_SIZE(frsm_codec_widgets);

	for (i = 0; i < count; i++) {
		frsm_append_name_prefix(frsm_dev, &new_widget[i].name);
		if (new_widget[i].sname)
			frsm_append_name_prefix(frsm_dev, &new_widget[i].sname);
	}

	return snd_soc_dapm_new_controls(dapm, new_widget, count);
}

static int frsm_init_dapm_route(struct frsm_dev *frsm_dev,
		struct snd_soc_dapm_context *dapm)
{
	struct snd_soc_dapm_route *new_route;
	int i, count;

	new_route = devm_kzalloc(frsm_dev->dev,
			sizeof(frsm_codec_routes), GFP_KERNEL);
	if (new_route == NULL)
		return -ENOMEM;

	memcpy(new_route, frsm_codec_routes, sizeof(frsm_codec_routes));
	count = ARRAY_SIZE(frsm_codec_routes);

	for (i = 0; i < count; i++) {
		frsm_append_name_prefix(frsm_dev, &new_route[i].sink);
		frsm_append_name_prefix(frsm_dev, &new_route[i].source);
	}

	return snd_soc_dapm_add_routes(dapm, new_route, count);
}

static int frsm_codec_probe_no_prefix(struct frsm_dev *frsm_dev,
		struct snd_soc_dapm_context *dapm)
{
	char *name;
	int ret;

	if (frsm_dev == NULL || frsm_dev->cmpnt == NULL || dapm == NULL)
		return -EINVAL;

	if (frsm_dev->pdata == NULL || !frsm_dev->pdata->name_prefix) {
		ret = snd_soc_dapm_new_controls(dapm, frsm_codec_widgets,
				ARRAY_SIZE(frsm_codec_widgets));
		ret |= snd_soc_add_component_controls(frsm_dev->cmpnt,
				frsm_codec_controls,
				ARRAY_SIZE(frsm_codec_controls));
		ret |= snd_soc_dapm_add_routes(dapm, frsm_codec_routes,
				ARRAY_SIZE(frsm_codec_routes));
		snd_soc_dapm_ignore_suspend(dapm, "Playback");
		snd_soc_dapm_ignore_suspend(dapm, "OUT");
		snd_soc_dapm_sync(dapm);
		return ret;
	}

	ret = frsm_init_dapm_controls(frsm_dev, dapm);
	ret |= frsm_init_cmpnt_controls(frsm_dev);
	ret |= frsm_init_dapm_route(frsm_dev, dapm);

	name = kzalloc(FRSM_STRING_MAX, GFP_KERNEL);
	if (name == NULL)
		return -ENOMEM;

	snprintf(name, FRSM_STRING_MAX, "%s Playback",
			frsm_dev->pdata->name_prefix);
	snd_soc_dapm_ignore_suspend(dapm, name);

	snprintf(name, FRSM_STRING_MAX, "%s OUT",
			frsm_dev->pdata->name_prefix);
	snd_soc_dapm_ignore_suspend(dapm, name);
	kfree(name);

	snd_soc_dapm_sync(dapm);

	return ret;
}

static int frsm_codec_probe_with_prefix(struct frsm_dev *frsm_dev,
		struct snd_soc_dapm_context *dapm)
{
	const char *str_prefix;
	char *name;
	int ret;

	if (frsm_dev == NULL || frsm_dev->cmpnt == NULL || dapm == NULL)
		return -EINVAL;

	ret = snd_soc_dapm_new_controls(dapm,
			frsm_codec_widgets, ARRAY_SIZE(frsm_codec_widgets));
	ret |= snd_soc_add_component_controls(frsm_dev->cmpnt,
			frsm_codec_controls, ARRAY_SIZE(frsm_codec_controls));
	ret |= snd_soc_dapm_add_routes(dapm,
			frsm_codec_routes, ARRAY_SIZE(frsm_codec_routes));
	if (ret)
		return ret;

	name = kzalloc(FRSM_STRING_MAX, GFP_KERNEL);
	if (name == NULL)
		return -ENOMEM;

	str_prefix = frsm_dev->cmpnt->name_prefix;
	snprintf(name, FRSM_STRING_MAX, "%s Playback", str_prefix);
	snd_soc_dapm_ignore_suspend(dapm, name);
	snprintf(name, FRSM_STRING_MAX, "%s OUT", str_prefix);
	snd_soc_dapm_ignore_suspend(dapm, name);
	kfree(name);

	snd_soc_dapm_sync(dapm);

	return 0;
}

static int frsm_codec_probe(struct snd_soc_component *cmpnt)
{
	struct snd_soc_dapm_context *dapm;
	struct frsm_dev *frsm_dev;
	int ret;

	if (cmpnt == NULL)
		return -EINVAL;

	frsm_dev = snd_soc_component_get_drvdata(cmpnt);
	if (frsm_dev == NULL || frsm_dev->pdata == NULL) {
		pr_err("codec_probe: frsm_dev is null\n");
		return -EINVAL;
	}

	frsm_dev->cmpnt = cmpnt;
	dapm = snd_soc_component_get_dapm(cmpnt);
	if (dapm == NULL)
		return -EINVAL;

	if (cmpnt->name_prefix) {
		dev_dbg(frsm_dev->dev, "name_prefix: %s\n",
				cmpnt->name_prefix);
		ret = frsm_codec_probe_with_prefix(frsm_dev, dapm);
	} else {
		ret = frsm_codec_probe_no_prefix(frsm_dev, dapm);
	}
	if (ret) {
		dev_err(frsm_dev->dev, "Failed to codec probe:%d\n", ret);
		return ret;
	}

	ret = frsm_mntr_init(frsm_dev);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to init monitor: %d\n", ret);

	ret = frsm_firmware_init_async(frsm_dev,
			frsm_dev->pdata->fwm_name);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to init firmware: %d\n", ret);

#ifdef CONFIG_FRSM_MIXER_DTYPE
	frsm_add_dtype_kcontrol(cmpnt);
#endif
	dev_info(frsm_dev->dev, "codec probed\n");

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static void frsm_codec_remove(struct snd_soc_component *cmpnt)
{
	struct frsm_dev *frsm_dev;

	frsm_dev = snd_soc_component_get_drvdata(cmpnt);
	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return;

	frsm_mntr_deinit(frsm_dev);
	frsm_send_event(frsm_dev, EVENT_SET_IDLE);
	dev_info(frsm_dev->dev, "cmpnt removed\n");
}

#ifdef CONFIG_PM
static int frsm_codec_suspend(struct snd_soc_component *cmpnt)
{
	if (cmpnt == NULL || cmpnt->dev == NULL)
		return -EINVAL;

	dev_info(cmpnt->dev, "cmpnt suspend!\n");

	return 0;
}


static int frsm_codec_resume(struct snd_soc_component *cmpnt)
{
	if (cmpnt == NULL || cmpnt->dev == NULL)
		return -EINVAL;

	dev_info(cmpnt->dev, "cmpnt resume!\n");

	return 0;
}
#endif

static const struct snd_soc_component_driver frsm_codec_cmpnt = {
	.name = "frsm-i2c-codec",
	.probe = frsm_codec_probe,
	.remove = frsm_codec_remove,
#ifdef CONFIG_PM
	.suspend = frsm_codec_suspend,
	.resume = frsm_codec_resume,
#endif
};

static int frsm_codec_add_dai_prefix(struct frsm_dev *frsm_dev)
{
	struct snd_soc_dai_driver *dai_drv;
	struct snd_soc_pcm_stream *stream;
	struct frsm_platform_data *pdata;
	char str[FRSM_STRING_MAX];

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	pdata = frsm_dev->pdata;
	dai_drv = frsm_dev->dai_drv;
	if (pdata->spkr_id <= 0 || !dai_drv->name)
		return 0;

	snprintf(str, sizeof(str), "%s-%d",
			dai_drv->name, pdata->spkr_id);
	dai_drv->name = devm_kstrdup(frsm_dev->dev, str, GFP_KERNEL);
	snprintf(str, sizeof(str), "%s-%d",
			frsm_dev->dev->driver->name, pdata->spkr_id);
	dev_set_name(frsm_dev->dev, "%s", str);

	stream = &dai_drv->playback;
	frsm_append_name_prefix(frsm_dev, &stream->stream_name);
	stream = &dai_drv->capture;
	frsm_append_name_prefix(frsm_dev, &stream->stream_name);

	return 0;
}

static int frsm_codec_init(struct frsm_dev *frsm_dev)
{
	struct snd_soc_dai_driver *dai_drv;
	const char *name;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	dai_drv = devm_kzalloc(frsm_dev->dev,
			sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (dai_drv == NULL)
		return -ENOMEM;

	memcpy(dai_drv, &frsm_codec_dai, sizeof(frsm_codec_dai));
	frsm_dev->dai_drv = dai_drv;

	name = devm_kstrdup(frsm_dev->dev, dev_name(frsm_dev->dev), GFP_KERNEL);
	ret = frsm_codec_add_dai_prefix(frsm_dev);
	if (ret) {
		dev_err(frsm_dev->dev, "Failed to add dai prefix!\n");
		return ret;
	}

	ret = snd_soc_register_component(frsm_dev->dev,
			&frsm_codec_cmpnt, dai_drv, 1);

	dev_dbg(frsm_dev->dev, "codec:%s dai:%s\n",
			dev_name(frsm_dev->dev), dai_drv->name);
	dev_set_name(frsm_dev->dev, "%s", name);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static void frsm_codec_deinit(struct frsm_dev *frsm_dev)
{
	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return;

	snd_soc_unregister_component(frsm_dev->dev);
}

static inline void frsm_codec_unused_func(void)
{
	frsm_get_dapm_from_cmpnt(NULL);
}

#endif // __FRSM_CODEC_H__
