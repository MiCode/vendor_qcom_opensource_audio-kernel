// SPDX-License-Identifier: GPL-2.0

/*
 * cs35l43-cal.c -- CS35l43 calibrate driver
 *
 * Copyright 2023 Cirrus Logic, Inc.
 *
 * Author:	Weiwei Zhang	<weiwei.zhang@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DEBUG

#define DEFAULT_CALR    9790//7*8192/5.85714


static int cs35l43_channel_swap_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
	   snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 =
	   snd_soc_component_get_drvdata(component);
	int val;

	if (cs35l43->dsp.booted) {
	  if (ucontrol->value.integer.value[0])
		 val = 0x7fffff;
	  else
		 val = 0;

	  val = cpu_to_be32(val);
	  wm_adsp_write_ctl(&cs35l43->dsp, "INP_CND_CH_BAL", WMFW_ADSP2_XM, 0x5f20e, (void *)&val, sizeof(__be32));
	}

	return 0;
}

static int cs35l43_channel_swap_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
	   snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 =
	   snd_soc_component_get_drvdata(component);
	int swapped = 0;
	int val;

	if (cs35l43->dsp.booted) {
	  wm_adsp_read_ctl(&cs35l43->dsp, "INP_CND_CH_BAL", WMFW_ADSP2_XM, 0x5f20e, (void *)&val, sizeof(__be32));

	  dev_dbg(cs35l43->dev, "%s: CH_BAL = 0x%08x\n", __func__, be32_to_cpu(val));
	  swapped = (0x7fffff==be32_to_cpu(val)) ? 1 : 0;
	}

	ucontrol->value.integer.value[0] = swapped;

	return 0;
}

static int wm_adsp_cal_ambient_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;
	int ret, val;

	if (!dsp->running) {
		ucontrol->value.enumerated.item[0] = 0;
		return 0;
	}
	
	ret = wm_adsp_read_ctl(dsp, "CAL_AMBIENT", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	if (ret)
		return -EIO;

	//val = be32_to_cpu(val); to compatible with calibration HAL value type
	ucontrol->value.enumerated.item[0] = val;
	dev_dbg(cs35l43->dev, "%s: get cal_ambient = %d, %d\n", __func__, be32_to_cpu(val),cs35l43->ambient);

	return 0;
}

static int wm_adsp_cal_ambient_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;

	int ret, val;
	cs35l43->ambient = ucontrol->value.enumerated.item[0];
	if (!dsp->running) {
		dev_err(cs35l43->dev, "%s: DSP is not running\n", __func__);
		return -EPERM;
	}

	val = ucontrol->value.enumerated.item[0];
	val = cpu_to_be32(val);
	ret = wm_adsp_write_ctl(dsp, "CAL_AMBIENT", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	if (ret)
		dev_err(cs35l43->dev, "%s: Write CAL_AMBIENT, ret=%d\n", __func__, ret);

	dev_dbg(cs35l43->dev, "%s: set cal_ambient = %d\n", __func__, ucontrol->value.enumerated.item[0]);

	return 0;
}

static int wm_adsp_set_cal_r_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;

	int ret, val;
	
	if (!dsp->running) {
		ucontrol->value.enumerated.item[0] = 0;
		return 0;
	}
	
	ret = wm_adsp_read_ctl(dsp, "CAL_R_INIT", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	if (ret)
		return -EIO;
	
	val = be32_to_cpu(val);
	ucontrol->value.enumerated.item[0] = val;
	dev_dbg(cs35l43->dev, "%s: get cal_r_init = %d\n", __func__, val);

	return 0;
}

static int wm_adsp_set_cal_r_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;

	int ret, val;
	cs35l43->calr = ucontrol->value.enumerated.item[0];

	if (!dsp->running) {
		dev_err(cs35l43->dev, "%s: DSP is not running\n", __func__);
		return -EPERM;
	}

	val = ucontrol->value.enumerated.item[0];
	val = cpu_to_be32(val);
	ret = wm_adsp_write_ctl(dsp, "CAL_R_SEL", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	if (ret)
		dev_err(cs35l43->dev, "%s: Write CAL_R_SEL, ret=%d\n", __func__, ret);

	dev_dbg(cs35l43->dev, "%s: set cal_r_sel = %d\n", __func__, ucontrol->value.enumerated.item[0]);

	return 0;
}

 static int wm_adsp_get_cal_r_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
 {
	 struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	 struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	 struct wm_adsp *dsp = &cs35l43->dsp;

	 int ret, val;
 
	 if (!dsp->running) {
		 ucontrol->value.enumerated.item[0] = 0;
		 return 0;
	 }
	 
	 ret = wm_adsp_read_ctl(dsp, "CAL_R", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	 if (ret)
		 return -EIO;
 
	 //val = be32_to_cpu(val);to compatible with calibration HAL value type
	 ucontrol->value.enumerated.item[0] = val;
	 dev_dbg(cs35l43->dev, "%s: get cal_r = %d\n", __func__, be32_to_cpu(val));
 
	 return 0;
 }
 
 static int wm_adsp_get_cal_r_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
 {
	 return 0;
 }

static int wm_adsp_cal_status_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;

	int ret, val;

	if (!dsp->running) {
		ucontrol->value.enumerated.item[0] = 0;
		return 0;
	}
	
	ret = wm_adsp_read_ctl(dsp, "CAL_STATUS", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	if (ret)
		return -EIO;

	//val = be32_to_cpu(val); to compatible with calibration HAL value type
	ucontrol->value.enumerated.item[0] = val;
	dev_dbg(cs35l43->dev, "%s: get cal_status = %d\n", __func__, be32_to_cpu(val));

	return 0;
}

static int wm_adsp_cal_status_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int wm_adsp_dsp_mode_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;

	int ret, val;
	
	if (!dsp->running) {
		ucontrol->value.enumerated.item[0] = 0;
		return 0;
	}
	
	ret = wm_adsp_read_ctl(dsp, "CAL_EN", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	if (ret)
		return -EIO;

	val = be32_to_cpu(val);
	if (val)
		ucontrol->value.enumerated.item[0] = 2;
	else
		ucontrol->value.enumerated.item[0] = 1;
	dev_info(cs35l43->dev, "%s: DSP is running in mode: %d [1:Play, 2:Calib]\n",
					__func__, ucontrol->value.enumerated.item[0]);
	return 0;
}

static int wm_adsp_dsp_mode_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;

	struct snd_kcontrol *kctl;
	struct snd_ctl_elem_value uctl;
	char ctl_name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	int ret, val;

	if (!dsp->running) {
		dev_err(cs35l43->dev, "%s: DSP is not running\n", __func__);
		return -EPERM;
	}

	if (1 == ucontrol->value.enumerated.item[0]) {
		val = cpu_to_be32(0x611);
		ret = wm_adsp_write_ctl(dsp, "PILOT_THRESH", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
		if (ret)
			dev_err(cs35l43->dev, "%s: Write PILOT_THRESH, ret=%d\n", __func__, ret);
		
		val = cpu_to_be32(0);
		ret = wm_adsp_write_ctl(dsp, "CAL_EN", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
		if (ret)
			dev_err(cs35l43->dev, "%s: Write CAL_EN, ret=%d\n", __func__, ret);
	} else if (2 == ucontrol->value.enumerated.item[0]) {

		val = cpu_to_be32(1);
		ret = wm_adsp_write_ctl(dsp, "CAL_EN", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
		if (ret)
			dev_err(cs35l43->dev, "%s: Write CAL_EN, ret=%d\n", __func__, ret);

		val = cpu_to_be32(0);
		ret = wm_adsp_write_ctl(dsp, "PILOT_THRESH", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
		if (ret)
			dev_err(cs35l43->dev, "%s: Write PILOT_THRESH, ret=%d\n", __func__, ret);
		
		val = cpu_to_be32(1);
		ret = wm_adsp_write_ctl(dsp, "CALIB_FIRST_RUN", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
		if (ret)
			dev_err(cs35l43->dev, "%s: Write CALIB_FIRST_RUN, ret=%d\n", __func__, ret);
		

	}

	if (dsp->component->name_prefix)
		snprintf(ctl_name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, "%s %s", dsp->component->name_prefix, "Reinit");
	else
		snprintf(ctl_name, SNDRV_CTL_ELEM_ID_NAME_MAXLEN, "%s", "Reinit");
	
	kctl = snd_soc_card_get_kcontrol(dsp->component->card, ctl_name);
	if (!kctl) {
		dev_err(cs35l43->dev, "Can't find kcontrol %s\n", ctl_name);
		return -EINVAL;
	}

	if (kctl->put) {
		uctl.value.integer.value[0] = 1;
		kctl->put(kctl, &uctl);
	} else {
		dev_err(cs35l43->dev, "Can't write kcontrol %s\n", ctl_name);
		return -EINVAL;
	}
	//read back cal_en value from dsp
	ret = wm_adsp_read_ctl(dsp, "CAL_EN", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&val, sizeof(__be32));
	//original cal_en control value 0 means playback mode, 1 means calibrate mode
	val = be32_to_cpu(val) + 1;
	dev_info(cs35l43->dev, "%s: DSP is running in mode: %d [1:Play, 2:Calib]\n", __func__, val);
	return 0;
}

static void cs35l43_ignore_suspend_widgets(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	struct cs35l43_private *cs35l43 = snd_soc_component_get_drvdata(component);
	char widget[32];
#if LINUX_VERSION_CODE > KERNEL_VERSION(5,10,70)
	if (0) {
#else
	if (component->name_prefix) {
#endif
		dev_info(cs35l43->dev,
        	"Linux kernel version <= 5.10.70, using ignore_suspend with name_prefix");
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "AMP Capture");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "AMP Playback");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASPRX1");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASPRX2");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "DSP RX1 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "DSP RX2 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "DSP RX3 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASPTX1");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASPTX2");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASPTX3");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASPTX4");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASP TX1 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASP TX2 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASP TX3 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "ASP TX4 Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "TEMPMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "IMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "VMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "VPMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "VBSTMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "PCM Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "High Rate PCM Source");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "Hibernate");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "Ultrasonic Mode");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "Main AMP");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "DSP1");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "DSP1 Preload");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "DSP1 Preloader");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "SPK");
		snd_soc_dapm_ignore_suspend(dapm, widget);
		snprintf(widget, sizeof(widget), "%s %s", component->name_prefix, "AMP Enable");
		snd_soc_dapm_ignore_suspend(dapm, widget);
	} else {
		dev_info(cs35l43->dev,
        	"Linux kernel version > 5.10.70, using ignore_suspend WITHOUT name_prefix");
		snd_soc_dapm_ignore_suspend(dapm, "AMP Capture");
		snd_soc_dapm_ignore_suspend(dapm, "AMP Playback");
		snd_soc_dapm_ignore_suspend(dapm, "ASPRX1");
		snd_soc_dapm_ignore_suspend(dapm, "ASPRX2");
		snd_soc_dapm_ignore_suspend(dapm, "DSP RX1 Source");
		snd_soc_dapm_ignore_suspend(dapm, "DSP RX2 Source");
		snd_soc_dapm_ignore_suspend(dapm, "DSP RX3 Source");
		snd_soc_dapm_ignore_suspend(dapm, "ASPTX1");
		snd_soc_dapm_ignore_suspend(dapm, "ASPTX2");
		snd_soc_dapm_ignore_suspend(dapm, "ASPTX3");
		snd_soc_dapm_ignore_suspend(dapm, "ASPTX4");
		snd_soc_dapm_ignore_suspend(dapm, "ASP TX1 Source");
		snd_soc_dapm_ignore_suspend(dapm, "ASP TX2 Source");
		snd_soc_dapm_ignore_suspend(dapm, "ASP TX3 Source");
		snd_soc_dapm_ignore_suspend(dapm, "ASP TX4 Source");
		snd_soc_dapm_ignore_suspend(dapm, "TEMPMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, "IMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, "VMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, "VPMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, "VBSTMON ADC");
		snd_soc_dapm_ignore_suspend(dapm, "PCM Source");
		snd_soc_dapm_ignore_suspend(dapm, "High Rate PCM Source");
		snd_soc_dapm_ignore_suspend(dapm, "Hibernate");
		snd_soc_dapm_ignore_suspend(dapm, "Ultrasonic Mode");
		snd_soc_dapm_ignore_suspend(dapm, "Main AMP");
		snd_soc_dapm_ignore_suspend(dapm, "DSP1");
		snd_soc_dapm_ignore_suspend(dapm, "DSP1 Preload");
		snd_soc_dapm_ignore_suspend(dapm, "DSP1 Preloader");
		snd_soc_dapm_ignore_suspend(dapm, "SPK");
		snd_soc_dapm_ignore_suspend(dapm, "AMP Enable");
	}
}

int cs35l43_apply_calibration(struct snd_soc_dapm_widget *w)
{
    int ret = 0;

	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs35l43_private *cs35l43 =
		snd_soc_component_get_drvdata(component);
	struct wm_adsp *dsp = &cs35l43->dsp;
	//the driver cached calibrated value
	__be32 calr;

	if (!dsp->running) {
		dev_err(cs35l43->dev, "%s: DSP is not running\n", __func__);
		return -EPERM;
	}
	if (cs35l43->calr != 0)
		calr = cpu_to_be32(cs35l43->calr);
	else 
		calr = cpu_to_be32(DEFAULT_CALR);
	// apply it to dsp
	ret = wm_adsp_write_ctl(dsp, "CAL_R_SEL", WMFW_ADSP2_XM, CS35L43_ALG_ID_PROTECT_LITE, (void *)&calr, sizeof(__be32));
	if (ret)
		dev_err(cs35l43->dev, "%s: Write CAL_R_SEL, ret=%d\n", __func__, ret);

	dev_info(cs35l43->dev, "%s: set cal_r_sel = %d, default:%d \n", __func__, cs35l43->calr, DEFAULT_CALR);

	return 0;
}
