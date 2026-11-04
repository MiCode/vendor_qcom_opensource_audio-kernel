/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-12-12 File created.
 */

#ifndef __FS18V1_OPS_H__
#define __FS18V1_OPS_H__

#include "frsm-dev.h"
#include "internal.h"

#define FS18V1_00H_STATUS			0x00
#define FS18V1_04H_I2SCTRL			0x04
#define FS18V1_05H_ANASTAT			0x05
#define FS18V1_06H_AUDIOCTRL			0x06
#define FS18V1_08H_TEMPSEL			0x08
#define FS18V1_09H_SYSCTRL			0x09
#define FS18V1_0BH_ACCKEY			0x0B
#define FS18V1_58H_INTSTAT			0x58
#define FS18V1_90H_CHIPINI			0x90
#define FS18V1_A2H_DACEQWL			0xA2
#define FS18V1_A6H_DACEQA			0xA6
#define FS18V1_BDH_DIGSTAT			0xBD
#define FS18V1_C1H_PLLCTRL1			0xC1
#define FS18V1_C2H_PLLCTRL2			0xC2
#define FS18V1_C3H_PLLCTRL3			0xC3
#define FS18V1_C4H_PLLCTRL4			0xC4

#define FS18V1_00H_CLKS_MASK		BIT(6)
#define FS18V1_00H_OCDS_MASK		BIT(5)
#define FS18V1_00H_UVDS_MASK		BIT(4)
#define FS18V1_00H_OTPDS_MASK		BIT(2)
#define FS18V1_00H_PLLS_MASK		BIT(1)
#define FS18V1_00H_BOVDS_MASK		BIT(0)
#define FS18V1_04H_I2SSR_SHIFT		12
#define FS18V1_04H_I2SSR_MASK		GENMASK(15, 12)
#define FS18V1_04H_CHS12_SHIFT		3
#define FS18V1_04H_CHS12_MASK		GENMASK(4, 3)
#define FS18V1_04H_I2SF_SHIFT		0
#define FS18V1_04H_I2SF_MASK		GENMASK(2, 0)
#define FS18V1_05H_OCDS_MASK		BIT(13)
#define FS18V1_06H_VOL_SHIFT		6
#define FS18V1_06H_VOL_MASK		GENMASK(15, 6)
#define FS18V1_58H_INTS13_MASK		BIT(13)
#define FS18V1_58H_INTS12_MASK		BIT(12)
#define FS18V1_58H_INTS10_MASK		BIT(10)
#define FS18V1_58H_INTS9_MASK		BIT(9)
#define FS18V1_58H_INTS1_MASK		BIT(1)
#define FS18V1_90H_INIST_SHIFT		0
#define FS18V1_90H_INIST_MASK		GENMASK(1, 0)
#define FS18V1_BDH_DACRUN_SHIFT		1
#define FS18V1_BDH_DACRUN_MASK		BIT(1)

#define FS18V1_00H_STATUS_OK		0x005F
#define FS18V1_05H_ANASTAT_OK		0x000F
#define FS18V1_08H_TEMPSEL_DEFALT	0x0010
#define FS18V1_08H_TEMPSEL_INITED	0x0188
#define FS18V1_09H_POWER_UP		0x0000
#define FS18V1_09H_POWER_UPALL		0x0008
#define FS18V1_09H_POWER_DOWNALL	0x0001
#define FS18V1_09H_I2C_RESET		0x0003
#define FS18V1_0BH_ACCKEY_ON		0xCA91
#define FS18V1_0BH_ACCKEY_OFF		0x0000
#define FS18V1_90H_INIST_OK		0x0003
#define FS18V1_A6H_CAM_DEFAULT		0x0000
#define FS18V1_A6H_CAM_ADD		0x0000
#define FS18V1_C4H_PLLCTRL4_OFF		0x0000
#define FS18V1_C4H_PLLCTRL4_ON		0x000B

#define BURST_WRITE_CAM_LEN		0x0004

static struct frsm_rate g_fs18v1_rates[] = {
	{   8000, 0x0 }, // RATE_8000
	{  16000, 0x3 }, // RATE_16000
	{  44100, 0x7 }, // RATE_44100
	{  48000, 0x8 }, // RATE_48000
};

static struct frsm_format g_fs18v1_formats[] = {
	{ 1, 3 }, // SND_SOC_DAIFMT_I2S
	{ 2, 7 }, // SND_SOC_DAIFMT_RIGHT_J
	{ 3, 2 }, // SND_SOC_DAIFMT_LEFT_J
	{ 4, 1 }, // SND_SOC_DAIFMT_DSP_A
	{ 5, 1 }, // SND_SOC_DAIFMT_DSP_B
};

static int fs18v1_set_mute(struct frsm_dev *frsm_dev, int mute);
static int fs18v1_shut_down(struct frsm_dev *frsm_dev);
static int fs18v1_set_channel(struct frsm_dev *frsm_dev);

static int fs18v1_i2c_reset(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret  = frsm_reg_write(frsm_dev, FS18V1_09H_SYSCTRL,
			FS18V1_09H_I2C_RESET);
	FRSM_DELAY_MS(5);
	ret |= frsm_reg_wait_stable(frsm_dev, FS18V1_90H_CHIPINI,
			FS18V1_90H_INIST_MASK, FS18V1_90H_INIST_OK);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to check CHIPINI\n");

	return ret;
}

static int fs18v1_set_i2s_config(struct frsm_dev *frsm_dev)
{
	struct frsm_hw_params *hw_params;
	struct frsm_format *formats;
	uint16_t val, mask;
	int idx;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	hw_params = &frsm_dev->hw_params;
	for (idx = 0; idx < ARRAY_SIZE(g_fs18v1_rates); idx++)
		if (g_fs18v1_rates[idx].rate == hw_params->rate)
			break;

	if (idx == ARRAY_SIZE(g_fs18v1_rates)) {
		dev_err(frsm_dev->dev,
				"Invalid sample rate:%d\n", hw_params->rate);
		return -EINVAL;
	}

	mask = FS18V1_04H_I2SSR_MASK;
	val = g_fs18v1_rates[idx].i2ssr << FS18V1_04H_I2SSR_SHIFT;

	if (hw_params->format != 0xFF) {
		formats = g_fs18v1_formats;
		for (idx = 0; idx < ARRAY_SIZE(g_fs18v1_formats); idx++) {
			if (formats->pcm_format == hw_params->format)
				break;
			formats++;
		}

		if (idx == ARRAY_SIZE(g_fs18v1_formats)) {
			dev_err(frsm_dev->dev, "Invalid pcm format:%d\n",
					hw_params->format);
			return -EINVAL;
		}

		mask |= FS18V1_04H_I2SF_MASK;
		val |= formats->i2sf << FS18V1_04H_I2SF_SHIFT;
	}

	ret = frsm_reg_update_bits(frsm_dev, FS18V1_04H_I2SCTRL, mask, val);

	return ret;
}

static int fs18v1_access_cram(struct frsm_dev *frsm_dev, bool enable)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (enable) {
		ret = frsm_reg_write(frsm_dev, FS18V1_C4H_PLLCTRL4,
				FS18V1_C4H_PLLCTRL4_ON);
		ret |= frsm_reg_write(frsm_dev, FS18V1_09H_SYSCTRL,
				FS18V1_09H_POWER_UP);
		ret |= frsm_reg_write(frsm_dev, FS18V1_0BH_ACCKEY,
				FS18V1_0BH_ACCKEY_ON);
	} else {
		ret = frsm_reg_write(frsm_dev, FS18V1_0BH_ACCKEY,
				FS18V1_0BH_ACCKEY_OFF);
		ret |= frsm_reg_write(frsm_dev, FS18V1_09H_SYSCTRL,
				FS18V1_09H_POWER_DOWNALL);
		ret |= frsm_reg_write(frsm_dev, FS18V1_C4H_PLLCTRL4,
				FS18V1_C4H_PLLCTRL4_OFF);
	}

	ret |= frsm_reg_wait_stable(frsm_dev, FS18V1_BDH_DIGSTAT,
			FS18V1_BDH_DACRUN_MASK, 0 << FS18V1_BDH_DACRUN_SHIFT);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to %s access CRAM:%d\n",
				enable ? "enable" : "disable", ret);

	return ret;
}

static int fs18v1_write_effect(struct frsm_dev *frsm_dev, uint16_t offset)
{
	const struct scene_table *cur_scene;
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	cur_scene = frsm_dev->cur_scene;
	if (offset == 0xFFFF || (cur_scene && cur_scene->effect == offset))
		return 0;

	ret = frsm_reg_write(frsm_dev, FS18V1_A6H_DACEQA,
			FS18V1_A6H_CAM_ADD);
	ret |= frsm_write_effect_table(frsm_dev, FS18V1_A2H_DACEQWL,
			offset);
	ret |= frsm_reg_write(frsm_dev, FS18V1_A6H_DACEQA,
			FS18V1_A6H_CAM_DEFAULT);

	return ret;
}

static int fs18v1_dev_init(struct frsm_dev *frsm_dev)
{
	struct scene_table *scene;
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FS18V1_08H_TEMPSEL, &val);
	if (ret || (!frsm_dev->force_init && val != FS18V1_08H_TEMPSEL_DEFALT))
		return ret;

	frsm_dev->force_init = false;
	frsm_dev->state &= ~(BIT(EVENT_STAT_MNTR) - 1);

	ret = fs18v1_i2c_reset(frsm_dev);
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

	ret |= fs18v1_access_cram(frsm_dev, true);
	if (!ret)
		ret |= fs18v1_write_effect(frsm_dev, scene->effect);
	ret |= fs18v1_access_cram(frsm_dev, false);

	ret |= fs18v1_set_mute(frsm_dev, 1);
	ret |= fs18v1_shut_down(frsm_dev);
	if (!ret) {
		frsm_dev->cur_scene = scene;
		frsm_reg_write(frsm_dev, FS18V1_08H_TEMPSEL,
				(FS18V1_08H_TEMPSEL_INITED << 1));
	}

	frsm_reg_read_status(frsm_dev, FS18V1_58H_INTSTAT, &val);

	return ret;
}

static int fs18v1_set_scene(struct frsm_dev *frsm_dev,
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
	/* write reg table */
	if (cur_scene == NULL || cur_scene->reg != scene->reg)
		ret = frsm_write_reg_table(frsm_dev, scene->reg);

	if (frsm_dev->pdata->rx_volume_v2) {
		frsm_reg_read(frsm_dev, FS18V1_06H_AUDIOCTRL, &val);
		frsm_dev->volume = (val >> FS18V1_06H_VOL_SHIFT);
	}

	ret |= fs18v1_access_cram(frsm_dev, true);
	if (!ret)
		ret |= fs18v1_write_effect(frsm_dev, scene->effect);
	ret |= fs18v1_access_cram(frsm_dev, false);

	return ret;
}

static int fs18v1_set_volume(struct frsm_dev *frsm_dev, uint16_t volume)
{
	uint16_t vol;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	vol = volume;
	if (vol > FRSM_VOLUME_MAX)
		vol = FRSM_VOLUME_MAX;

	ret = frsm_reg_write(frsm_dev, FS18V1_06H_AUDIOCTRL,
			vol << FS18V1_06H_VOL_SHIFT);

	return ret;
}

static int fs18v1_hw_params(struct frsm_dev *frsm_dev)
{
	uint16_t pll1, pll2, pll3;
	unsigned int bclk;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	bclk = frsm_dev->hw_params.bclk;
	dev_dbg(frsm_dev->dev, "hw_params bclk:%d\n", bclk);
	switch (bclk) {
	case 256000:
		pll1 = 0x01A0; pll2 = 0x0180; pll3 = 0x0001;
		break;
	case 512000:
		pll1 = 0x01A0; pll2 = 0x0180; pll3 = 0x0002;
		break;
	case 1024000:
		pll1 = 0x0260; pll2 = 0x0120; pll3 = 0x0003;
		break;
	case 1411200:
		pll1 = 0x01A0; pll2 = 0x0100; pll3 = 0x0004;
		break;
	case 1536000:
		pll1 = 0x0260; pll2 = 0x0100; pll3 = 0x0004;
		break;
	case 2048000:
		pll1 = 0x0260; pll2 = 0x0120; pll3 = 0x0006;
		break;
	case 2822400:
		pll1 = 0x01A0; pll2 = 0x0100; pll3 = 0x0008;
		break;
	case 3072000:
		pll1 = 0x0260; pll2 = 0x0100; pll3 = 0x0008;
		break;
	case 6144000:
		pll1 = 0x0260; pll2 = 0x0100; pll3 = 0x0010;
		break;
	case 12288000:
		pll1 = 0x0260; pll2 = 0x0100; pll3 = 0x0020;
		break;
	default:
		dev_err(frsm_dev->dev, "Invalid BCLK: %d\n", bclk);
		return -EINVAL;
	}

	ret  = fs18v1_set_i2s_config(frsm_dev);
	ret |= fs18v1_set_channel(frsm_dev);
	ret |= frsm_reg_write(frsm_dev, FS18V1_C1H_PLLCTRL1, pll1);
	ret |= frsm_reg_write(frsm_dev, FS18V1_C2H_PLLCTRL2, pll2);
	ret |= frsm_reg_write(frsm_dev, FS18V1_C3H_PLLCTRL3, pll3);

	return ret;
}

static int fs18v1_start_up(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_write(frsm_dev, FS18V1_C4H_PLLCTRL4,
			FS18V1_C4H_PLLCTRL4_ON);
	ret |= fs18v1_set_volume(frsm_dev, frsm_dev->volume);

	return ret;
}

static int fs18v1_set_mute(struct frsm_dev *frsm_dev, int mute)
{
	uint16_t intstat;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (mute) {
		ret = frsm_reg_write(frsm_dev, FS18V1_09H_SYSCTRL,
				FS18V1_09H_POWER_DOWNALL);
		frsm_reg_wait_stable(frsm_dev, FS18V1_BDH_DIGSTAT,
				FS18V1_BDH_DACRUN_MASK,
				0 << FS18V1_BDH_DACRUN_SHIFT);
	} else {
		ret = frsm_reg_write(frsm_dev, FS18V1_09H_SYSCTRL,
				FS18V1_09H_POWER_UPALL);
	}

	frsm_reg_read_status(frsm_dev, FS18V1_58H_INTSTAT,
			&intstat);

	return ret;
}

static int fs18v1_set_tsmute(struct frsm_dev *frsm_dev, bool mute)
{
	/* Do nothing */
	return 0;
}

static int fs18v1_set_channel(struct frsm_dev *frsm_dev)
{
	uint16_t chs12, val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	if (frsm_dev->pdata->rx_channel < 0) {
		ret = frsm_reg_read(frsm_dev, FS18V1_04H_I2SCTRL, &val);
		if (ret)
			return ret;
		chs12 = (val & FS18V1_04H_CHS12_MASK) >> FS18V1_04H_CHS12_SHIFT;
		frsm_dev->pdata->rx_channel = chs12;
	} else {
		chs12 = frsm_dev->pdata->rx_channel;
		chs12 &= (FS18V1_04H_CHS12_MASK >> FS18V1_04H_CHS12_SHIFT);
	}

	/* CHS12: Left=0/1, Right=2, Mono=3 */
	if (frsm_dev->swap_channel)
		chs12 = (chs12 == 1) ? 2 : (chs12 == 2) ? 1 : 3;

	ret = frsm_reg_update_bits(frsm_dev, FS18V1_04H_I2SCTRL,
			FS18V1_04H_CHS12_MASK, chs12 << FS18V1_04H_CHS12_SHIFT);

	return ret;
}

static int fs18v1_shut_down(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_write(frsm_dev, FS18V1_C4H_PLLCTRL4,
			FS18V1_C4H_PLLCTRL4_OFF);

	return ret;
}

static int fs18v1_irq_handler(struct frsm_dev *frsm_dev)
{
	uint16_t intstat;
	int ret;

	ret = frsm_reg_read(frsm_dev, FS18V1_58H_INTSTAT, &intstat);
	if (intstat & FS18V1_58H_INTS13_MASK)
		dev_err(frsm_dev->dev, "OC detected\n");
	if (intstat & FS18V1_58H_INTS12_MASK)
		dev_err(frsm_dev->dev, "UV detected\n");
	if (intstat & FS18V1_58H_INTS10_MASK)
		dev_err(frsm_dev->dev, "OTP detected\n");
	if (intstat & FS18V1_58H_INTS9_MASK)
		dev_err(frsm_dev->dev, "BOV detected\n");
	if (intstat & FS18V1_58H_INTS1_MASK)
		dev_err(frsm_dev->dev, "Clock lost detected\n");

	ret |= frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs18v1_stat_monitor(struct frsm_dev *frsm_dev)
{
	uint16_t status, anastat;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (frsm_dev->irq_id > 0)
		return fs18v1_irq_handler(frsm_dev);

	ret  = frsm_reg_read_status(frsm_dev, FS18V1_00H_STATUS, &status);
	ret |= frsm_reg_read_status(frsm_dev, FS18V1_05H_ANASTAT, &anastat);
	if (ret)
		return ret;
	if (status == FS18V1_00H_STATUS_OK
			&& (anastat & 0xF) == FS18V1_05H_ANASTAT_OK)
		return 0;

	if ((status & FS18V1_00H_CLKS_MASK) == 0)
		dev_err(frsm_dev->dev, "CLK ratio unstable\n");
	if (anastat & FS18V1_05H_OCDS_MASK)
		dev_err(frsm_dev->dev, "OC detected in AMP\n");
	if ((status & FS18V1_00H_UVDS_MASK) == 0)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if ((status & FS18V1_00H_OTPDS_MASK) == 0)
		dev_err(frsm_dev->dev, "OT detected\n");
	if ((status & FS18V1_00H_PLLS_MASK) == 0)
		dev_err(frsm_dev->dev, "PLL is unlock\n");
	if ((status & FS18V1_00H_BOVDS_MASK) == 0)
		dev_err(frsm_dev->dev, "OV detected on boost\n");

	ret = frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs18v1_dev_ops(struct frsm_dev *frsm_dev)
{
	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	frsm_dev->bst_wcam_len = BURST_WRITE_CAM_LEN;
	frsm_dev->func = FRSM_HAS_IRQ | FRSM_SWAP_CHN;
	frsm_dev->ops.dev_init  = fs18v1_dev_init;
	frsm_dev->ops.set_scene = fs18v1_set_scene;
	frsm_dev->ops.set_volume = fs18v1_set_volume;
	frsm_dev->ops.hw_params = fs18v1_hw_params;
	frsm_dev->ops.start_up = fs18v1_start_up;
	frsm_dev->ops.set_mute  = fs18v1_set_mute;
	frsm_dev->ops.set_tsmute  = fs18v1_set_tsmute;
	frsm_dev->ops.set_channel = fs18v1_set_channel;
	frsm_dev->ops.shut_down = fs18v1_shut_down;
	frsm_dev->ops.stat_monitor = fs18v1_stat_monitor;
	frsm_dev->reg_amp_mute = ((FS18V1_09H_SYSCTRL << 16)
			| FS18V1_09H_POWER_DOWNALL);

	return 0;
}

#endif // __FS18V1_OPS_H__
