/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-12-23 File created.
 */

#ifndef __FS19V2_OPS_H__
#define __FS19V2_OPS_H__

#include <linux/math64.h>
#include "internal.h"

enum fs19v2_param_type {
	FS19V2_PARAM_TCOEF = 0,
	FS19V2_PARAM_RAPP,
	FS19V2_PARAM_RATIO,
	FS19V2_PARAM_MAX,
};

#define FS19V2_00H_STATUS		0x00
#define FS19V2_05H_ANASTAT		0x05
#define FS19V2_06H_DIGSTAT		0x06
#define FS19V2_0BH_ACCKEY		0x0B
#define FS19V2_0CH_I2SDET		0x0C
#define FS19V2_0EH_CHIPINI		0x0E
#define FS19V2_10H_PWRCTRL		0x10
#define FS19V2_11H_SYSCTRL		0x11
#define FS19V2_14H_SPKCOEF		0x14
#define FS19V2_16H_AUDIOCTRL		0x16
#define FS19V2_17H_I2SCTRL		0x17
#define FS19V2_4CH_TSCTRL		0x4C
#define FS19V2_62H_DACEQWL		0x62
#define FS19V2_66H_DACEQA		0x66
#define FS19V2_82H_SDGEQWL		0x82
#define FS19V2_86H_SDGEQA		0x86
#define FS19V2_93H_F0STAT2		0x93
#define FS19V2_94H_TCCTRL1		0x94
#define FS19V2_95H_TCCTRL2		0x95
#define FS19V2_96H_TCTCOEF1		0x96
#define FS19V2_97H_TCTCOEF2		0x97
#define FS19V2_99H_TCSTAT2		0x99
#define FS19V2_9AH_TCSTAT3		0x9A
#define FS19V2_9CH_TCSTAT5		0x9C
#define FS19V2_9DH_TCSTAT6		0x9D
#define FS19V2_9EH_TCSTAT7		0x9E
#define FS19V2_9FH_TCSTAT8		0x9F
#define FS19V2_A1H_PLLCTRL1		0xA1
#define FS19V2_A2H_PLLCTRL2		0xA2
#define FS19V2_A3H_PLLCTRL3		0xA3
#define FS19V2_ABH_INTSTAT		0xAB
#define FS19V2_C3H_CLDCFG		0xC3
#define FS19V2_DCH_OTPCMD		0xDC
#define FS19V2_DDH_OTPADDR		0xDD
#define FS19V2_DEH_OTPWDATA		0xDE
#define FS19V2_DFH_OTPRDATA		0xDF
#define FS19V2_F0H_OTPPG2W0		0xF0
#define FS19V2_F1H_OTPPG2W1		0xF1

#define FS19V2_00H_VDDPS_MASK		BIT(14)
#define FS19V2_00H_BOLDS_MASK		BIT(10)
#define FS19V2_00H_BOPS_MASK		BIT(9)
#define FS19V2_00H_OTWDS_MASK		BIT(8)
#define FS19V2_00H_CLKS_MASK		BIT(6)
#define FS19V2_00H_OCDS_MASK		BIT(5)
#define FS19V2_00H_UVDS_MASK		BIT(4)
#define FS19V2_00H_OVDS_MASK		BIT(3)
#define FS19V2_00H_OTPDS_MASK		BIT(2)
#define FS19V2_00H_PLLS_MASK		BIT(1)
#define FS19V2_00H_BOVDS_MASK		BIT(0)
#define FS19V2_05H_OCDS_MASK		BIT(13)
#define FS19V2_06H_DACRUN_SHIFT		1
#define FS19V2_06H_DACRUN_MASK		BIT(1)
#define FS19V2_06H_RUNOFF_SHIFT		0
#define FS19V2_06H_RUNOFF_MASK		GENMASK(2, 0)
#define FS19V2_0EH_INIST_SHIFT		0
#define FS19V2_0EH_INIST_MASK		GENMASK(1, 0)
#define FS19V2_16H_VOL_SHIFT		6
#define FS19V2_16H_VOL_MASK		GENMASK(15, 6)
#define FS19V2_17H_I2SSR_SHIFT		12
#define FS19V2_17H_I2SSR_MASK		GENMASK(15, 12)
#define FS19V2_17H_CHS12_SHIFT		3
#define FS19V2_17H_CHS12_MASK		GENMASK(4, 3)
#define FS19V2_17H_I2SF_SHIFT		0
#define FS19V2_17H_I2SF_MASK		GENMASK(2, 0)
#define FS19V2_4CH_OFFSTAT_SHIFT	14
#define FS19V2_4CH_OFFSTAT_MASK		BIT(14)
#define FS19V2_94H_CALIBEN_MASK		BIT(11)
#define FS19V2_ABH_INTS14_MASK		BIT(14)
#define FS19V2_ABH_INTS13_MASK		BIT(13)
#define FS19V2_ABH_INTS12_MASK		BIT(12)
#define FS19V2_ABH_INTS11_MASK		BIT(11)
#define FS19V2_ABH_INTS10_MASK		BIT(10)
#define FS19V2_ABH_INTS8_MASK		BIT(8)
#define FS19V2_ABH_INTS6_MASK		BIT(6)
#define FS19V2_ABH_INTS5_MASK		BIT(5)
#define FS19V2_ABH_INTS4_MASK		BIT(4)
#define FS19V2_ABH_INTS1_MASK		BIT(1)
#define FS19V2_C3H_SUBREV_SHIFT		5
#define FS19V2_C3H_SUBREV_MASK		GENMASK(7, 5)
#define FS19V2_DCH_OTPBUSY_SHIFT	2
#define FS19V2_DCH_OTPBUSY_MASK		BIT(2)

#define FS19V2_00H_STATUS_OK		0x0042
#define FS19V2_05H_ANASTAT_OK		0x000F
#define FS19V2_0BH_ACCKEY_ON		0xCA91
#define FS19V2_0BH_ACCKEY_OFF		0x0000
#define FS19V2_0EH_INIST_OK		0x0003
#define FS19V2_10H_POWER_UP		0x0000
#define FS19V2_10H_POWER_DOWN		0x0001
#define FS19V2_10H_I2C_RESET		0x0002
#define FS19V2_11H_SYSTEM_UP		0x00EF
#define FS19V2_11H_SYSTEM_PLL_ON	0x0022
#define FS19V2_11H_SYSTEM_DOWN		0x0022
#define FS19V2_14H_SPKCOEF_DEFALT	0x0000
#define FS19V2_14H_SPKCOEF_INITED	0x5A5A
#define FS19V2_66H_CAM_DEFAULT		0x0000
#define FS19V2_66H_CAM_BURST		0x8000
#define FS19V2_66H_SDG1_BURST		0x803C
#define FS19V2_66H_DAC2_BURST		0x8091
#define FS19V2_86H_CAM_DEFAULT		0x0000
#define FS19V2_86H_CAM_BURST		0x8000
#define FS19V2_C3H_SUBREV_ID		0x0004
#define FS19V2_DCH_OTPCMD_DFT		0x0400
#define FS19V2_DCH_OTPCMD_READ		0x0401
#define FS19V2_DCH_OTPCMD_WRITE		0x0402
#define FS19V2_DCH_OTPCMD_RELOAD	0x0500
#define FS19V2_DDH_OTPADDR_DFT		0x0000
#define FS19V2_DDH_OTPADDR_PG2		0x0010

#define FS19V2_SDG1_RAM_SIZE		0x0154
#define FS19V2_SDG2_RAM_SIZE		0x0168
#define FS19V2_DAC1_RAM_SIZE		0x00F0
#define FS19V2_DAC2_RAM_SIZE		0x03CC

static struct frsm_rate g_fs19v2_rates[] = {
	{   8000, 0x1 }, // RATE_8000
	{  16000, 0x3 }, // RATE_16000
	{  32000, 0x7 }, // RATE_32000
	{  44100, 0x8 }, // RATE_44100
	{  48000, 0x9 }, // RATE_48000
	{  88200, 0xA }, // RATE_88200
	{  96000, 0xB }, // RATE_96000
};

static struct frsm_format g_fs19v2_formats[] = {
	{ 1, 3 }, // SND_SOC_DAIFMT_I2S
	{ 2, 7 }, // SND_SOC_DAIFMT_RIGHT_J
	{ 3, 2 }, // SND_SOC_DAIFMT_LEFT_J
	{ 4, 1 }, // SND_SOC_DAIFMT_DSP_A
	{ 5, 1 }, // SND_SOC_DAIFMT_DSP_B
};

static struct frsm_pll g_fs19v2_plls[] = {
	{  1024000, 0x0264, 0x0B40, 0x000C },
	{  1411200, 0x0260, 0x0540, 0x0006 },
	{  1536000, 0x0260, 0x0540, 0x0006 },
	{  2048000, 0x0264, 0x05A0, 0x000C },
	{  2822400, 0x0260, 0x0540, 0x000C },
	{  3072000, 0x0264, 0x03C0, 0x000C },
	{  5644800, 0x0260, 0x0540, 0x0018 },
	{  6144000, 0x0264, 0x01E0, 0x000C },
	{ 12288000, 0x0260, 0x0540, 0x0030 },
};

static int fs19v2_set_mute(struct frsm_dev *frsm_dev, int mute);
static int fs19v2_shut_down(struct frsm_dev *frsm_dev);
static int fs19v2_set_channel(struct frsm_dev *frsm_dev);

static int fs19v2_i2c_reset(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_write(frsm_dev, FS19V2_10H_PWRCTRL,
			FS19V2_10H_I2C_RESET);
	FRSM_DELAY_MS(5);
	ret |= frsm_reg_wait_stable(frsm_dev, FS19V2_0EH_CHIPINI,
			FS19V2_0EH_INIST_MASK, FS19V2_0EH_INIST_OK);
	if (ret) {
		dev_err(frsm_dev->dev, "Failed to check CHIPINI\n");
		return ret;
	}

	return ret;
}

static int fs19v2_set_i2s_config(struct frsm_dev *frsm_dev)
{
	struct frsm_hw_params *hw_params;
	struct frsm_format *formats;
	uint16_t val, mask;
	int idx;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	hw_params = &frsm_dev->hw_params;
	for (idx = 0; idx < ARRAY_SIZE(g_fs19v2_rates); idx++)
		if (g_fs19v2_rates[idx].rate == hw_params->rate)
			break;

	if (idx == ARRAY_SIZE(g_fs19v2_rates)) {
		dev_err(frsm_dev->dev, "Invalid sample rate:%d\n",
				hw_params->rate);
		return -EINVAL;
	}

	mask = FS19V2_17H_I2SSR_MASK;
	val = g_fs19v2_rates[idx].i2ssr << FS19V2_17H_I2SSR_SHIFT;

	if (hw_params->format != 0xFF) {
		formats = g_fs19v2_formats;
		for (idx = 0; idx < ARRAY_SIZE(g_fs19v2_formats); idx++) {
			if (formats->pcm_format == hw_params->format)
				break;
			formats++;
		}

		if (idx == ARRAY_SIZE(g_fs19v2_formats)) {
			dev_err(frsm_dev->dev, "Invalid pcm format:%d\n",
					hw_params->format);
			return -EINVAL;
		}

		mask |= FS19V2_17H_I2SF_MASK;
		val |= formats->i2sf << FS19V2_17H_I2SF_SHIFT;
	}

	ret = frsm_reg_update_bits(frsm_dev, FS19V2_17H_I2SCTRL, mask, val);

	return ret;
}

static int fs19v2_access_cram(struct frsm_dev *frsm_dev, bool enable)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (enable) {
		ret  = frsm_reg_write(frsm_dev, FS19V2_11H_SYSCTRL,
				FS19V2_11H_SYSTEM_PLL_ON);
		ret |= frsm_reg_write(frsm_dev, FS19V2_10H_PWRCTRL,
				FS19V2_10H_POWER_UP);
		ret |= frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY,
				FS19V2_0BH_ACCKEY_ON);
	} else {
		ret  = frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY,
				FS19V2_0BH_ACCKEY_OFF);
		ret |= frsm_reg_write(frsm_dev, FS19V2_10H_PWRCTRL,
				FS19V2_10H_POWER_DOWN);
		ret |= frsm_reg_write(frsm_dev, FS19V2_11H_SYSCTRL,
				FS19V2_11H_SYSTEM_DOWN);
	}

	ret |= frsm_reg_wait_stable(frsm_dev, FS19V2_06H_DIGSTAT,
			FS19V2_06H_RUNOFF_MASK, 0 << FS19V2_06H_RUNOFF_SHIFT);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to %s access CRAM:%d\n",
				enable ? "enable" : "disable", ret);

	return ret;
}

static int fs19v2_write_model(struct frsm_dev *frsm_dev, uint16_t offset)
{
	const struct scene_table *cur_scene;
	struct file_table *model;
	struct fwm_table *table;
	int i, burst_len;
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	cur_scene = frsm_dev->cur_scene;
	if (offset == 0xFFFF || (cur_scene && cur_scene->model == offset))
		return 0;

	table = frsm_get_fwm_table(frsm_dev, INDEX_MODEL);
	if (table == NULL) {
		dev_err(frsm_dev->dev, "Failed to find model table\n");
		return -EINVAL;
	}

	model = (struct file_table *)((char *)table + offset);
	dev_dbg(frsm_dev->dev, "model table size:%d\n", model->size);
	if (model->size == 0 || (model->size & 0x3)) {
		dev_err(frsm_dev->dev, "Invalid model size:%d\n", model->size);
		return -EINVAL;
	}

	burst_len = frsm_dev->bst_wcam_len;
	if (burst_len > model->size || (burst_len & 0x3)) {
		/* burst_len is 0 or multiples of 4 */
		dev_err(frsm_dev->dev,
				"Invalid burst len:%d\n", burst_len);
		return -EINVAL;
	}

	ret = frsm_reg_write(frsm_dev, FS19V2_66H_DACEQA,
			FS19V2_66H_SDG1_BURST);
	ret |= frsm_reg_write(frsm_dev, FS19V2_86H_SDGEQA,
			FS19V2_86H_CAM_BURST);

	if (burst_len == 0) { // write all one time
		ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_62H_DACEQWL,
				model->buf, FS19V2_SDG1_RAM_SIZE);
		ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_82H_SDGEQWL,
				model->buf + FS19V2_SDG1_RAM_SIZE,
				FS19V2_SDG2_RAM_SIZE);
	} else {
		for (i = 0; i < FS19V2_SDG1_RAM_SIZE; i += burst_len)
			ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_62H_DACEQWL,
					model->buf + i, burst_len);
		for (i = 0; i < FS19V2_SDG2_RAM_SIZE; i += burst_len)
			ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_82H_SDGEQWL,
					model->buf + FS19V2_SDG1_RAM_SIZE + i,
					burst_len);
	}

	ret |= frsm_reg_write(frsm_dev, FS19V2_66H_DACEQA,
			FS19V2_66H_CAM_DEFAULT);
	ret |= frsm_reg_write(frsm_dev, FS19V2_86H_SDGEQA,
			FS19V2_86H_CAM_DEFAULT);

	return ret;
}

static int fs19v2_write_effect(struct frsm_dev *frsm_dev, uint16_t offset)
{
	const struct scene_table *cur_scene;
	struct file_table *effect;
	struct fwm_table *table;
	int i, burst_len;
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	cur_scene = frsm_dev->cur_scene;
	if (offset == 0xFFFF || (cur_scene && cur_scene->effect == offset))
		return 0;

	table = frsm_get_fwm_table(frsm_dev, INDEX_EFFECT);
	if (table == NULL) {
		dev_err(frsm_dev->dev, "Failed to find effect table\n");
		return -EINVAL;
	}

	effect = (struct file_table *)((char *)table + offset);
	dev_dbg(frsm_dev->dev, "effect table size:%d\n", effect->size);
	if (effect->size == 0 || (effect->size & 0x3)) {
		dev_err(frsm_dev->dev,
				"Invalid effect size:%d\n", effect->size);
		return -EINVAL;
	}

	burst_len = frsm_dev->bst_wcam_len;
	if (burst_len > effect->size || (burst_len & 0x3)) {
		/* burst_len is 0 or multiples of 4 */
		dev_err(frsm_dev->dev,
				"Invalid burst len:%d\n", burst_len);
		return -EINVAL;
	}

	ret = frsm_reg_write(frsm_dev, FS19V2_66H_DACEQA,
			FS19V2_66H_CAM_BURST);

	if (burst_len == 0) { // write all one time
		ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_62H_DACEQWL,
				effect->buf, FS19V2_DAC1_RAM_SIZE);
		ret |= frsm_reg_write(frsm_dev, FS19V2_66H_DACEQA,
				FS19V2_66H_DAC2_BURST);
		ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_62H_DACEQWL,
				effect->buf + FS19V2_DAC1_RAM_SIZE,
				FS19V2_DAC2_RAM_SIZE);
	} else {
		for (i = 0; i < FS19V2_DAC1_RAM_SIZE; i += burst_len)
			ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_62H_DACEQWL,
					effect->buf + i, burst_len);
		ret |= frsm_reg_write(frsm_dev, FS19V2_66H_DACEQA,
				FS19V2_66H_DAC2_BURST);
		for (i = 0; i < FS19V2_DAC2_RAM_SIZE; i += burst_len)
			ret |= frsm_reg_bulk_write(frsm_dev, FS19V2_62H_DACEQWL,
					effect->buf + FS19V2_DAC1_RAM_SIZE + i,
					burst_len);
	}

	ret |= frsm_reg_write(frsm_dev, FS19V2_66H_DACEQA,
			FS19V2_66H_CAM_DEFAULT);

	return ret;
}

static int fs19v2_dev_init(struct frsm_dev *frsm_dev)
{
	struct scene_table *scene;
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FS19V2_14H_SPKCOEF, &val);
	if (ret || (!frsm_dev->force_init && val != FS19V2_14H_SPKCOEF_DEFALT))
		return ret;

	frsm_dev->force_init = false;
	frsm_dev->state &= ~(BIT(EVENT_STAT_MNTR) - 1);

	ret = fs19v2_i2c_reset(frsm_dev);
	if (ret)
		return ret;

	if (frsm_dev->tbl_scene == NULL) {
		dev_err(frsm_dev->dev, "Scene table is null\n");
		return -EINVAL;
	}

	scene = (struct scene_table *)frsm_dev->tbl_scene->buf;
	/* write reg table */
	ret = frsm_write_reg_table(frsm_dev, scene->reg);
	if (ret)
		return ret;

	ret = fs19v2_access_cram(frsm_dev, true);
	if (!ret) {
		ret |= fs19v2_write_model(frsm_dev, scene->model);
		ret |= fs19v2_write_effect(frsm_dev, scene->effect);
	}
	frsm_reg_read_status(frsm_dev, FS19V2_ABH_INTSTAT, &val);
	ret |= fs19v2_access_cram(frsm_dev, false);

	ret |= fs19v2_set_mute(frsm_dev, 1);
	ret |= fs19v2_shut_down(frsm_dev);
	if (ret)
		return ret;

	frsm_dev->cur_scene = scene;
	frsm_reg_write(frsm_dev, FS19V2_14H_SPKCOEF,
			FS19V2_14H_SPKCOEF_INITED);

	if (frsm_dev->pdata->rx_volume_v2) {
		frsm_reg_read(frsm_dev, FS19V2_16H_AUDIOCTRL, &val);
		frsm_dev->volume = val >> FS19V2_16H_VOL_SHIFT;
	}

	return ret;
}

static int fs19v2_set_scene(struct frsm_dev *frsm_dev,
		struct scene_table *scene)
{
	const struct scene_table *cur_scene;
	uint16_t val;
	int ret = 0;

	if (frsm_dev == NULL || frsm_dev->dev == NULL || scene == NULL)
		return -EINVAL;

	if (frsm_dev->cur_scene == scene)
		return 0;

	if (frsm_dev->pdata->rx_volume_v2 && frsm_dev->ops.set_volume)
		frsm_dev->ops.set_volume(frsm_dev, frsm_dev->volume);

	cur_scene = frsm_dev->cur_scene;
	if (cur_scene == NULL || cur_scene->reg != scene->reg)
		ret = frsm_write_reg_table(frsm_dev, scene->reg);

	ret = fs19v2_access_cram(frsm_dev, true);
	if (!ret) {
		ret |= fs19v2_write_model(frsm_dev, scene->model);
		ret |= fs19v2_write_effect(frsm_dev, scene->effect);
	}
	ret |= fs19v2_access_cram(frsm_dev, false);

	if (frsm_dev->pdata->rx_volume_v2) {
		frsm_reg_read(frsm_dev, FS19V2_16H_AUDIOCTRL, &val);
		frsm_dev->volume = val >> FS19V2_16H_VOL_SHIFT;
	}

	return ret;
}

static int fs19v2_set_volume(struct frsm_dev *frsm_dev, uint16_t volume)
{
	uint16_t vol;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	vol = volume;
	if (vol > FRSM_VOLUME_MAX)
		vol = FRSM_VOLUME_MAX;

	ret = frsm_reg_write(frsm_dev, FS19V2_16H_AUDIOCTRL,
			vol << FS19V2_16H_VOL_SHIFT);

	return ret;
}

static int fs19v2_hw_params(struct frsm_dev *frsm_dev)
{
	unsigned int bclk;
	int idx;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	bclk = frsm_dev->hw_params.bclk;
	dev_dbg(frsm_dev->dev, "hw_params bclk:%d\n", bclk);
	for (idx = 0; idx < ARRAY_SIZE(g_fs19v2_plls); idx++)
		if (g_fs19v2_plls[idx].bclk == bclk)
			break;

	if (idx == ARRAY_SIZE(g_fs19v2_plls)) {
		dev_err(frsm_dev->dev, "Invalid bclk:%d\n", bclk);
		return -EINVAL;
	}

	ret  = fs19v2_set_i2s_config(frsm_dev);
	ret |= fs19v2_set_channel(frsm_dev);
	ret |= frsm_reg_write(frsm_dev, FS19V2_A1H_PLLCTRL1,
			g_fs19v2_plls[idx].pll1);
	ret |= frsm_reg_write(frsm_dev, FS19V2_A2H_PLLCTRL2,
			g_fs19v2_plls[idx].pll2);
	ret |= frsm_reg_write(frsm_dev, FS19V2_A3H_PLLCTRL3,
			g_fs19v2_plls[idx].pll3);

	return ret;
}

static int fs19v2_start_up(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret  = frsm_reg_write(frsm_dev, FS19V2_11H_SYSCTRL,
			FS19V2_11H_SYSTEM_UP);
	ret |= fs19v2_set_volume(frsm_dev, frsm_dev->volume);

	return ret;
}

static int fs19v2_set_mute(struct frsm_dev *frsm_dev, int mute)
{
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (mute) {
		ret  = frsm_reg_write(frsm_dev, FS19V2_10H_PWRCTRL,
				FS19V2_10H_POWER_DOWN);
		ret |= frsm_reg_wait_stable(frsm_dev, FS19V2_4CH_TSCTRL,
				FS19V2_4CH_OFFSTAT_MASK,
				1 << FS19V2_4CH_OFFSTAT_SHIFT);
		ret |= frsm_reg_wait_stable(frsm_dev, FS19V2_06H_DIGSTAT,
				FS19V2_06H_DACRUN_MASK,
				0 << FS19V2_06H_DACRUN_SHIFT);
	} else {
		ret = frsm_reg_write(frsm_dev, FS19V2_10H_PWRCTRL,
				FS19V2_10H_POWER_UP);
	}
	frsm_reg_read_status(frsm_dev, FS19V2_ABH_INTSTAT, &val);

	return ret;
}

static int fs19v2_set_tsmute(struct frsm_dev *frsm_dev, bool mute)
{
	/* Do nothing */
	return 0;
}

static int fs19v2_set_channel(struct frsm_dev *frsm_dev)
{
	uint16_t chs12, val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	if (frsm_dev->pdata->rx_channel < 0) {
		ret = frsm_reg_read(frsm_dev, FS19V2_17H_I2SCTRL, &val);
		if (ret)
			return ret;
		chs12 = (val & FS19V2_17H_CHS12_MASK) >> FS19V2_17H_CHS12_SHIFT;
		frsm_dev->pdata->rx_channel = chs12;
	} else {
		chs12 = frsm_dev->pdata->rx_channel;
		chs12 &= (FS19V2_17H_CHS12_MASK >> FS19V2_17H_CHS12_SHIFT);
	}

	/* CHS12: Left=0/1, Right=2, Mono=3 */
	if (frsm_dev->swap_channel)
		chs12 = (chs12 == 1) ? 2 : (chs12 == 2) ? 1 : 3;

	ret = frsm_reg_update_bits(frsm_dev, FS19V2_17H_I2SCTRL,
			FS19V2_17H_CHS12_MASK, chs12 << FS19V2_17H_CHS12_SHIFT);

	return ret;
}

static int fs19v2_shut_down(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_write(frsm_dev, FS19V2_11H_SYSCTRL,
			FS19V2_11H_SYSTEM_DOWN);

	return ret;
}

static int fs19v2_irq_handler(struct frsm_dev *frsm_dev)
{
	uint16_t intstat;
	int ret;

	ret = frsm_reg_read(frsm_dev, FS19V2_ABH_INTSTAT, &intstat);

	if (intstat == 0x0000)
		return ret;

	if (intstat & FS19V2_ABH_INTS14_MASK)
		dev_err(frsm_dev->dev, "VDDPS detected\n");
	if (intstat & FS19V2_ABH_INTS13_MASK)
		dev_err(frsm_dev->dev, "OC detected\n");
	if (intstat & FS19V2_ABH_INTS12_MASK)
		dev_err(frsm_dev->dev, "UV detected\n");
	if (intstat & FS19V2_ABH_INTS11_MASK)
		dev_err(frsm_dev->dev, "OV detected\n");
	if (intstat & FS19V2_ABH_INTS10_MASK)
		dev_err(frsm_dev->dev, "OTP detected\n");
	if (intstat & FS19V2_ABH_INTS8_MASK)
		dev_err(frsm_dev->dev, "BOV detected\n");
	if (intstat & FS19V2_ABH_INTS6_MASK)
		dev_err(frsm_dev->dev, "BOL detected\n");
	if (intstat & FS19V2_ABH_INTS5_MASK)
		dev_err(frsm_dev->dev, "BOP detected\n");
	if (intstat & FS19V2_ABH_INTS4_MASK)
		dev_err(frsm_dev->dev, "OTW detected\n");
	if (intstat & FS19V2_ABH_INTS1_MASK)
		dev_err(frsm_dev->dev, "Clock lost detected\n");

	ret = frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs19v2_stat_monitor(struct frsm_dev *frsm_dev)
{
	uint16_t status, anastat;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (frsm_dev->irq_id > 0)
		return fs19v2_irq_handler(frsm_dev);

	ret  = frsm_reg_read_status(frsm_dev, FS19V2_00H_STATUS, &status);
	ret |= frsm_reg_read_status(frsm_dev, FS19V2_05H_ANASTAT, &anastat);
	if (ret)
		return ret;

	if (status == FS19V2_00H_STATUS_OK
			&& (anastat & 0xF) == FS19V2_05H_ANASTAT_OK)
		return 0;

	if (status & FS19V2_00H_VDDPS_MASK)
		dev_err(frsm_dev->dev, "VDDP short detected\n");
	if (status & FS19V2_00H_BOLDS_MASK)
		dev_err(frsm_dev->dev, "Current limit overload\n");
	if (status & FS19V2_00H_BOPS_MASK)
		dev_err(frsm_dev->dev, "Brownout protection active\n");
	if (status & FS19V2_00H_OTWDS_MASK)
		dev_err(frsm_dev->dev, "OT warning detected\n");
	if ((status & FS19V2_00H_CLKS_MASK) == 0)
		dev_err(frsm_dev->dev, "CLK ratio unstable\n");
	if (anastat & FS19V2_05H_OCDS_MASK)
		dev_err(frsm_dev->dev, "OC detected in AMP\n");
	if (status & FS19V2_00H_UVDS_MASK)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if (status & FS19V2_00H_OVDS_MASK)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if (status & FS19V2_00H_OTPDS_MASK)
		dev_err(frsm_dev->dev, "OT detected\n");
	if ((status & FS19V2_00H_PLLS_MASK) == 0)
		dev_err(frsm_dev->dev, "PLL is unlock\n");
	if (status & FS19V2_00H_BOVDS_MASK)
		dev_err(frsm_dev->dev, "OV detected on boost\n");

	ret = frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs19v2_wait_otp_ready(struct frsm_dev *frsm_dev)
{
	int ret;

	ret = frsm_reg_wait_stable(frsm_dev, FS19V2_DCH_OTPCMD,
			FS19V2_DCH_OTPBUSY_MASK, 0 << FS19V2_DCH_OTPBUSY_SHIFT);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to wait OTP ready:%d\n", ret);

	return ret;
}

static int fs19v2_check_otp_repetition(struct frsm_dev *frsm_dev, int spkre)
{
	uint16_t g2w0, g2w1;
	int count;
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FS19V2_F0H_OTPPG2W0, &g2w0);
	if (ret)
		return ret;

	ret |= frsm_reg_write(frsm_dev, FS19V2_DDH_OTPADDR,
			FS19V2_DDH_OTPADDR_PG2);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DCH_OTPCMD,
			FS19V2_DCH_OTPCMD_READ);
	ret = fs19v2_wait_otp_ready(frsm_dev);
	if (ret)
		dev_info(frsm_dev->dev, "Check OTP timeout!\n");

	ret |= frsm_reg_read(frsm_dev, FS19V2_DFH_OTPRDATA, &g2w1);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DCH_OTPCMD,
			FS19V2_DCH_OTPCMD_DFT);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DDH_OTPADDR,
			FS19V2_DDH_OTPADDR_DFT);
	count = g2w0 & 0x000F;
	if (ret || count >= 8) {
		dev_err(frsm_dev->dev, "OTP is used up:%d\n", ret);
		return -EFAULT;
	}

	dev_info(frsm_dev->dev, "Read back OTP_SPKRE:%d\n", g2w1);
	if (count == 0 || 20 * abs(spkre - g2w1) > g2w1) // 1/20=5%
		return 1; // save otp

	return 0;
}

static int fs19v2_save_otp_spkre(struct frsm_dev *frsm_dev, int spkre)
{
	uint16_t value;
	int ret;

	if (!frsm_dev->save_otp_spkre)
		return 0;

	spkre = spkre >> 2; /* Q12 -> Q10 */
	if (spkre <= 0 || spkre >= 0xFFFF) {
		dev_err(frsm_dev->dev, "Invalid spkre:%d\n", spkre);
		return -EINVAL;
	}

	ret = frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY, FS19V2_0BH_ACCKEY_ON);
	ret = fs19v2_check_otp_repetition(frsm_dev, spkre);
	if (ret <= 0)
		goto func_exit;

	ret = fs19v2_wait_otp_ready(frsm_dev);
	if (ret)
		goto func_exit;

	ret = frsm_reg_write(frsm_dev, FS19V2_DDH_OTPADDR,
			FS19V2_DDH_OTPADDR_PG2);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DEH_OTPWDATA, spkre & 0xFFFF);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DCH_OTPCMD,
			FS19V2_DCH_OTPCMD_WRITE);
	ret |= fs19v2_wait_otp_ready(frsm_dev);
	if (ret)
		goto func_exit;

	/* read back and check */
	ret = frsm_reg_write(frsm_dev, FS19V2_DCH_OTPCMD,
			FS19V2_DCH_OTPCMD_RELOAD);
	FRSM_DELAY_MS(1);
	ret |= frsm_reg_read(frsm_dev, FS19V2_F1H_OTPPG2W1, &value);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DCH_OTPCMD,
			FS19V2_DCH_OTPCMD_DFT);
	ret |= frsm_reg_write(frsm_dev, FS19V2_DDH_OTPADDR,
			FS19V2_DDH_OTPADDR_DFT);
	if (spkre != value) {
		dev_err(frsm_dev->dev, "Failed to check OTP\n");
		ret = -EINVAL;
	}

func_exit:
	frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY, FS19V2_0BH_ACCKEY_OFF);

	return ret;
}

static int fs19v2_get_spkre(struct frsm_dev *frsm_dev)
{
	uint16_t p2w0, p2w1;
	int ret;

	if (frsm_dev->spkre > 0)
		return 0;

	ret  = frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY,
			FS19V2_0BH_ACCKEY_ON);
	ret |= frsm_reg_read(frsm_dev, FS19V2_F0H_OTPPG2W0, &p2w0);
	ret |= frsm_reg_read(frsm_dev, FS19V2_F1H_OTPPG2W1, &p2w1);
	ret |= frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY,
			FS19V2_0BH_ACCKEY_OFF);
	if (!ret && (p2w0 & 0x000F) != 0 && p2w1 != 0xFFFF) {
		dev_dbg(frsm_dev->dev, "OTP calire:%d %d\n", p2w0, p2w1);
		frsm_dev->spkre = p2w1 << 2; /* Q10 -> Q12 */
		return 0;
	}

	if (frsm_dev->pdata)
		frsm_dev->spkre = frsm_dev->pdata->ref_rdc;

	return 0;
}

static int fs19v2_get_model_param(void *table, int param_type)
{
	struct file_table *model;
	int *data;

	model = (struct file_table *)table;
	data = (int *)model->buf;

	switch (param_type) {
	case FS19V2_PARAM_RATIO:
		return data[85+24] ^ 0x001801AD; // TC_SPK_R_RATIO
	case FS19V2_PARAM_RAPP:
		return data[85+23] ^ 0x001801AD; // TC_R_APP
	case FS19V2_PARAM_TCOEF:
		return data[85+19] ^ 0x001801AD; // TC_T_COEF
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int fs19v2_update_tc_params(struct frsm_dev *frsm_dev, int spkre)
{
	const struct scene_table *scene;
	struct fwm_table *table;
	uint64_t data;
	int param;
	int ret;

	scene = (struct scene_table *)frsm_dev->tbl_scene->buf;
	if (scene->model == 0 || scene->model == 0xFFFF)
		return -EINVAL;

	table = frsm_get_fwm_table(frsm_dev, INDEX_MODEL);
	param = fs19v2_get_model_param((char *)table + scene->model,
			FS19V2_PARAM_RAPP);
	if (param < 0)
		return -EINVAL;

	data = spkre + param; /* Q12 */
	param = fs19v2_get_model_param((char *)table + scene->model,
			FS19V2_PARAM_RATIO);
	if (param <= 0)
		return -EINVAL;

	dev_dbg(frsm_dev->dev, "spkr:%lld, ratio:%d\n", data, param);
	data = div_u64(data << 18, param); /* 18-12+12=18 */
	if (data > UINT16_MAX)
		data = UINT16_MAX;
	frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY, FS19V2_0BH_ACCKEY_ON);
	ret = frsm_reg_write(frsm_dev, FS19V2_95H_TCCTRL2, (uint16_t)data);

	param = fs19v2_get_model_param((char *)table + scene->model,
			FS19V2_PARAM_TCOEF);
	if (param <= 0)
		return -EINVAL;

	dev_dbg(frsm_dev->dev, "tcoef:%llx\n", data);
	data = div_u64(1llu << 47, spkre); /* 12+23+12=47 */
	data = div_u64(data, param);
	if (data > UINT32_MAX)
		data = UINT32_MAX;
	ret |= frsm_reg_write(frsm_dev, FS19V2_96H_TCTCOEF1, HIGH16(data));
	ret |= frsm_reg_write(frsm_dev, FS19V2_97H_TCTCOEF2, LOW16(data));

	spkre = spkre >> 2; /* Q12 -> Q10 */
	if (spkre > UINT16_MAX)
		spkre = UINT16_MAX;
	ret |= frsm_reg_write(frsm_dev, FS19V2_F1H_OTPPG2W1, (uint16_t)spkre);
	frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY, FS19V2_0BH_ACCKEY_OFF);

	return ret;
}

static int fs19v2_set_spkr_prot(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	ret = fs19v2_save_otp_spkre(frsm_dev, frsm_dev->spkre);
	if (ret)
		return ret;

	ret = fs19v2_get_spkre(frsm_dev);
	if (ret || frsm_dev->spkre == 0) {
		dev_info(frsm_dev->dev, "Disable spkr protection?\n");
		return ret;
	}

	dev_info(frsm_dev->dev, "spkre:%d\n", frsm_dev->spkre);
	ret = fs19v2_update_tc_params(frsm_dev, frsm_dev->spkre);

	return ret;
}

static int fs19v2_get_livedata(struct frsm_dev *frsm_dev,
		struct live_data *data)
{
	uint16_t tcstat[7];
	uint16_t value;
	int ret;

	if (frsm_dev == NULL || data == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FS19V2_93H_F0STAT2, &value);
	data->spkf0 = value << 8; /* Q8 */
	data->spkQ = 0 << 8; /* Q8 */

	ret |= frsm_reg_bulk_read(frsm_dev, FS19V2_99H_TCSTAT2,
			tcstat, sizeof(tcstat));

	data->spkre = frsm_dev->spkre;
	if (frsm_dev->calib_mode)
		data->spkre = (tcstat[0] << 16) | tcstat[1];

	data->spkr0 = (tcstat[3] << 16) | tcstat[4];
	data->spkt0 = (tcstat[5] << 16) | tcstat[6];
	if (data->spkt0 & BIT(23)) // int24_t
		data->spkt0 = (data->spkt0 << 8) >> 8;

	return ret;
}

static int fs19v2_detect_subrev(struct frsm_dev *frsm_dev)
{
	uint16_t subrev;
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (HIGH8(frsm_dev->dev_id) != FRSM_DEVID_FS18DH)
		return 0;

	frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY, FS19V2_0BH_ACCKEY_ON);
	ret = frsm_reg_read(frsm_dev, FS19V2_C3H_CLDCFG, &val);
	frsm_reg_write(frsm_dev, FS19V2_0BH_ACCKEY, FS19V2_0BH_ACCKEY_OFF);

	subrev = (val & FS19V2_C3H_SUBREV_MASK) >> FS19V2_C3H_SUBREV_SHIFT;
	if (ret || subrev != FS19V2_C3H_SUBREV_ID)
		return -ENODEV;

	return 0;
}

static int fs19v2_dev_ops(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = fs19v2_detect_subrev(frsm_dev);
	if (ret)
		return ret;

	frsm_dev->func = FRSM_HAS_IRQ | FRSM_SWAP_CHN;
	frsm_dev->ops.dev_init = fs19v2_dev_init;
	frsm_dev->ops.set_scene = fs19v2_set_scene;
	frsm_dev->ops.set_volume = fs19v2_set_volume;
	frsm_dev->ops.hw_params = fs19v2_hw_params;
	frsm_dev->ops.start_up = fs19v2_start_up;
	frsm_dev->ops.set_mute = fs19v2_set_mute;
	frsm_dev->ops.set_tsmute = fs19v2_set_tsmute;
	frsm_dev->ops.set_channel = fs19v2_set_channel;
	frsm_dev->ops.shut_down = fs19v2_shut_down;
	frsm_dev->ops.stat_monitor = fs19v2_stat_monitor;
	frsm_dev->ops.get_livedata = fs19v2_get_livedata;
	frsm_dev->ops.set_spkr_prot = fs19v2_set_spkr_prot;
	frsm_dev->reg_amp_mute = ((FS19V2_10H_PWRCTRL << 16)
			| FS19V2_10H_POWER_DOWN);

	return 0;
}

#endif // __FS19V2_OPS_H__
