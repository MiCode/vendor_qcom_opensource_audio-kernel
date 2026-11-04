/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-12-12 File created.
 */

#ifndef __FS19V1_OPS_H__
#define __FS19V1_OPS_H__

#include "frsm-dev.h"
#include "internal.h"

#define FS19V1_00H_STATUS			0x00
#define FS19V1_05H_ANASTAT			0x05
#define FS19V1_06H_DIGSTAT			0x06
#define FS19V1_0BH_ACCKEY			0x0B
#define FS19V1_0CH_I2SDET			0x0C
#define FS19V1_0EH_CHIPINI			0x0E
#define FS19V1_10H_PWRCTRL			0x10
#define FS19V1_11H_SYSCTRL			0x11
#define FS19V1_14H_SPKCOEF			0x14
#define FS19V1_16H_AUDIOCTRL			0x16
#define FS19V1_17H_I2SCTRL			0x17
#define FS19V1_4CH_TSCTRL			0x4C
#define FS19V1_A1H_PLLCTRL1			0xA1
#define FS19V1_A2H_PLLCTRL2			0xA2
#define FS19V1_A3H_PLLCTRL3			0xA3
#define FS19V1_ABH_INTSTAT			0xAB
#define FS19V1_C3H_CLDCFG			0xC3

#define FS19V1_00H_VDDPS_MASK		BIT(14)
#define FS19V1_00H_INDS_MASK		BIT(11)
#define FS19V1_00H_BOLDS_MASK		BIT(10)
#define FS19V1_00H_BOPS_MASK		BIT(9)
#define FS19V1_00H_OTWDS_MASK		BIT(8)
#define FS19V1_00H_CLKS_MASK		BIT(6)
#define FS19V1_00H_OCDS_MASK		BIT(5)
#define FS19V1_00H_UVDS_MASK		BIT(4)
#define FS19V1_00H_OVDS_MASK		BIT(3)
#define FS19V1_00H_OTPDS_MASK		BIT(2)
#define FS19V1_00H_PLLS_MASK		BIT(1)
#define FS19V1_00H_BOVDS_MASK		BIT(0)
#define FS19V1_05H_OCDS_MASK		BIT(13)
#define FS19V1_06H_DACRUN_SHIFT		1
#define FS19V1_06H_DACRUN_MASK		BIT(1)
#define FS19V1_0EH_INIST_SHIFT		0
#define FS19V1_0EH_INIST_MASK		GENMASK(1, 0)
#define FS19V1_16H_VOL_SHIFT		6
#define FS19V1_16H_VOL_MASK		GENMASK(15, 6)
#define FS19V1_17H_I2SSR_SHIFT		12
#define FS19V1_17H_I2SSR_MASK		GENMASK(15, 12)
#define FS19V1_17H_CHS12_SHIFT		3
#define FS19V1_17H_CHS12_MASK		GENMASK(4, 3)
#define FS19V1_17H_I2SF_SHIFT		0
#define FS19V1_17H_I2SF_MASK		GENMASK(2, 0)
#define FS19V1_4CH_OFFSTAT_SHIFT	14
#define FS19V1_4CH_OFFSTAT_MASK		BIT(14)
#define FS19V1_ABH_INTS14_MASK		BIT(14)
#define FS19V1_ABH_INTS13_MASK		BIT(13)
#define FS19V1_ABH_INTS12_MASK		BIT(12)
#define FS19V1_ABH_INTS11_MASK		BIT(11)
#define FS19V1_ABH_INTS10_MASK		BIT(10)
#define FS19V1_ABH_INTS8_MASK		BIT(8)
#define FS19V1_ABH_INTS7_MASK		BIT(7)
#define FS19V1_ABH_INTS6_MASK		BIT(6)
#define FS19V1_ABH_INTS5_MASK		BIT(5)
#define FS19V1_ABH_INTS4_MASK		BIT(4)
#define FS19V1_ABH_INTS1_MASK		BIT(1)

#define FS19V1_00H_STATUS_OK		0x0042
#define FS19V1_05H_ANASTAT_OK		0x000F
#define FS19V1_0BH_ACCKEY_ON		0xCA91
#define FS19V1_0BH_ACCKEY_OFF		0x0000
#define FS19V1_0EH_INIST_OK		0x0003
#define FS19V1_10H_POWER_UP		0x0000
#define FS19V1_10H_POWER_DOWN		0x0001
#define FS19V1_10H_I2C_RESET		0x0002
#define FS19V1_11H_SYSTEM_UP		0x00EF
#define FS19V1_11H_SYSTEM_DOWN		0x0022
#define FS19V1_14H_SPKCOEF_DEFALT	0x0000
#define FS19V1_14H_SPKCOEF_INITED	0x5A5A

static struct frsm_rate g_fs19v1_rates[] = {
	{   8000, 0x1 }, // RATE_8000
	{  16000, 0x3 }, // RATE_16000
	{  32000, 0x7 }, // RATE_32000
	{  44100, 0x8 }, // RATE_44100
	{  48000, 0x9 }, // RATE_48000
	{  88200, 0xA }, // RATE_88200
	{  96000, 0xB }, // RATE_96000
};

static struct frsm_format g_fs19v1_formats[] = {
	{ 1, 3 }, // SND_SOC_DAIFMT_I2S
	{ 2, 7 }, // SND_SOC_DAIFMT_RIGHT_J
	{ 3, 2 }, // SND_SOC_DAIFMT_LEFT_J
	{ 4, 1 }, // SND_SOC_DAIFMT_DSP_A
	{ 5, 1 }, // SND_SOC_DAIFMT_DSP_B
};

static int fs19v1_set_mute(struct frsm_dev *frsm_dev, int mute);
static int fs19v1_shut_down(struct frsm_dev *frsm_dev);
static int fs19v1_set_channel(struct frsm_dev *frsm_dev);

static int fs19v1_i2c_reset(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret  = frsm_reg_write(frsm_dev, FS19V1_10H_PWRCTRL,
			FS19V1_10H_I2C_RESET);
	FRSM_DELAY_MS(5);
	ret |= frsm_reg_wait_stable(frsm_dev, FS19V1_0EH_CHIPINI,
			FS19V1_0EH_INIST_MASK, FS19V1_0EH_INIST_OK);
	if (ret)
		dev_err(frsm_dev->dev, "Failed to check CHIPINI\n");

	return ret;
}

static int fs19v1_set_i2s_config(struct frsm_dev *frsm_dev)
{
	struct frsm_hw_params *hw_params;
	struct frsm_format *formats;
	uint16_t val, mask;
	int idx;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	hw_params = &frsm_dev->hw_params;
	for (idx = 0; idx < ARRAY_SIZE(g_fs19v1_rates); idx++)
		if (g_fs19v1_rates[idx].rate == hw_params->rate)
			break;

	if (idx == ARRAY_SIZE(g_fs19v1_rates)) {
		dev_err(frsm_dev->dev,
				"Invalid sample rate:%d\n", hw_params->rate);
		return -EINVAL;
	}

	mask = FS19V1_17H_I2SSR_MASK;
	val = g_fs19v1_rates[idx].i2ssr << FS19V1_17H_I2SSR_SHIFT;

	if (hw_params->format != 0xFF) {
		formats = g_fs19v1_formats;
		for (idx = 0; idx < ARRAY_SIZE(g_fs19v1_formats); idx++) {
			if (formats->pcm_format == hw_params->format)
				break;
			formats++;
		}

		if (idx == ARRAY_SIZE(g_fs19v1_formats)) {
			dev_err(frsm_dev->dev, "Invalid pcm format:%d\n",
					hw_params->format);
			return -EINVAL;
		}

		mask |= FS19V1_17H_I2SF_MASK;
		val |= formats->i2sf << FS19V1_17H_I2SF_SHIFT;
	}

	ret = frsm_reg_update_bits(frsm_dev, FS19V1_17H_I2SCTRL, mask, val);

	return ret;
}

static int fs19v1_read_dtype(struct frsm_dev *frsm_dev)
{
	uint16_t val;
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	ret  = frsm_reg_write(frsm_dev, FS19V1_0BH_ACCKEY,
			FS19V1_0BH_ACCKEY_ON);
	ret |= frsm_reg_read(frsm_dev, FS19V1_C3H_CLDCFG, &val);
	ret |= frsm_reg_write(frsm_dev, FS19V1_0BH_ACCKEY,
			FS19V1_0BH_ACCKEY_OFF);
	if (ret)
		return ret;

	frsm_dev->dtype  = (frsm_dev->dev_id & 0xFF00);
	frsm_dev->dtype |= ((val >> 5) & 0x7); // bit[7..5]

	return 0;
}

static int fs19v1_dev_init(struct frsm_dev *frsm_dev)
{
	struct scene_table *scene;
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FS19V1_14H_SPKCOEF, &val);
	if (ret || (!frsm_dev->force_init && val != FS19V1_14H_SPKCOEF_DEFALT))
		return ret;

	frsm_dev->force_init = false;
	frsm_dev->state &= ~(BIT(EVENT_STAT_MNTR) - 1);

	ret = fs19v1_i2c_reset(frsm_dev);
	if (ret)
		return ret;

	if (frsm_dev->tbl_scene == NULL) {
		dev_err(frsm_dev->dev, "Scene table is null\n");
		return -EINVAL;
	}

	scene = (struct scene_table *)frsm_dev->tbl_scene->buf;
	ret = frsm_write_reg_table(frsm_dev, scene->reg);

	ret |= fs19v1_read_dtype(frsm_dev);
	ret |= fs19v1_set_mute(frsm_dev, 1);
	ret |= fs19v1_shut_down(frsm_dev);
	if (!ret) {
		frsm_dev->cur_scene = scene;
		frsm_reg_write(frsm_dev, FS19V1_14H_SPKCOEF,
				FS19V1_14H_SPKCOEF_INITED);
	}

	if (frsm_dev->pdata->rx_volume_v2) {
		frsm_reg_read(frsm_dev, FS19V1_16H_AUDIOCTRL, &val);
		frsm_dev->volume = val >> FS19V1_16H_VOL_SHIFT;
	}

	frsm_reg_read_status(frsm_dev, FS19V1_ABH_INTSTAT, &val);

	return ret;
}

static int fs19v1_set_scene(struct frsm_dev *frsm_dev,
		struct scene_table *scene)
{
	const struct scene_table *cur_scene;
	uint16_t val;
	int ret = 0;

	if (frsm_dev == NULL || frsm_dev->dev == NULL || scene == NULL)
		return -EINVAL;

	dev_dbg(frsm_dev->dev, "reg:%d\n", scene->reg);

	if (frsm_dev->pdata->rx_volume_v2 && frsm_dev->ops.set_volume)
		frsm_dev->ops.set_volume(frsm_dev, frsm_dev->volume);

	cur_scene = frsm_dev->cur_scene;
	if (cur_scene == NULL || cur_scene->reg != scene->reg)
		ret |= frsm_write_reg_table(frsm_dev, scene->reg);

	if (frsm_dev->pdata->rx_volume_v2) {
		frsm_reg_read(frsm_dev, FS19V1_16H_AUDIOCTRL, &val);
		frsm_dev->volume = val >> FS19V1_16H_VOL_SHIFT;
	}

	return ret;
}

static int fs19v1_set_volume(struct frsm_dev *frsm_dev, uint16_t volume)
{
	uint16_t vol;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	vol = volume;
	if (vol > FRSM_VOLUME_MAX)
		vol = FRSM_VOLUME_MAX;

	ret = frsm_reg_write(frsm_dev, FS19V1_16H_AUDIOCTRL,
			vol << FS19V1_16H_VOL_SHIFT);

	return ret;
}

static int fs19v1_hw_params(struct frsm_dev *frsm_dev)
{
	uint16_t pll1, pll2, pll3;
	unsigned int bclk;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	bclk = frsm_dev->hw_params.bclk;
	dev_dbg(frsm_dev->dev, "hw_params bclk:%d\n", bclk);
	switch (bclk) {
	case 1411200:
	case 1536000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0006;
		break;
	case 2822400:
	case 3072000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x000C;
		break;
	case 4233600:
	case 4608000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0012;
		break;
	case 5644800:
	case 6144000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0018;
		break;
	case 8467200:
	case 9216000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0024;
		break;
	case 12288000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0030;
		break;
	case 512000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0002;
		break;
	case 1024000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0004;
		break;
	case 2048000:
		pll1 = 0x0260; pll2 = 0x0540; pll3 = 0x0008;
		break;
	default:
		dev_err(frsm_dev->dev, "Invalid BCLK: %d\n", bclk);
		return -EINVAL;
	}

	ret  = fs19v1_set_i2s_config(frsm_dev);
	ret |= fs19v1_set_channel(frsm_dev);
	ret |= frsm_reg_write(frsm_dev, FS19V1_A1H_PLLCTRL1, pll1);
	ret |= frsm_reg_write(frsm_dev, FS19V1_A2H_PLLCTRL2, pll2);
	ret |= frsm_reg_write(frsm_dev, FS19V1_A3H_PLLCTRL3, pll3);

	return ret;
}

static int fs19v1_start_up(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret  = frsm_reg_write(frsm_dev, FS19V1_11H_SYSCTRL,
			FS19V1_11H_SYSTEM_UP);

	return ret;
}

static int fs19v1_set_mute(struct frsm_dev *frsm_dev, int mute)
{
	uint16_t intstat;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (mute) {
		ret  = frsm_reg_write(frsm_dev, FS19V1_10H_PWRCTRL,
				FS19V1_10H_POWER_DOWN);
		frsm_reg_wait_stable(frsm_dev, FS19V1_4CH_TSCTRL,
				FS19V1_4CH_OFFSTAT_MASK,
				1 << FS19V1_4CH_OFFSTAT_SHIFT);
		frsm_reg_wait_stable(frsm_dev, FS19V1_06H_DIGSTAT,
				FS19V1_06H_DACRUN_MASK,
				0 << FS19V1_06H_DACRUN_SHIFT);
	} else {
		ret = frsm_reg_write(frsm_dev, FS19V1_10H_PWRCTRL,
				FS19V1_10H_POWER_UP);
	}

	frsm_reg_read_status(frsm_dev, FS19V1_ABH_INTSTAT,
			&intstat); // clear

	return ret;
}

static int fs19v1_set_tsmute(struct frsm_dev *frsm_dev, bool mute)
{
	/* Do nothing */
	return 0;
}

static int fs19v1_set_channel(struct frsm_dev *frsm_dev)
{
	uint16_t chs12, val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	if (frsm_dev->pdata->rx_channel < 0) {
		ret = frsm_reg_read(frsm_dev, FS19V1_17H_I2SCTRL, &val);
		if (ret)
			return ret;
		chs12 = (val & FS19V1_17H_CHS12_MASK) >> FS19V1_17H_CHS12_SHIFT;
		frsm_dev->pdata->rx_channel = chs12;
	} else {
		chs12 = frsm_dev->pdata->rx_channel;
		chs12 &= (FS19V1_17H_CHS12_MASK >> FS19V1_17H_CHS12_SHIFT);
	}

	/* CHS12: Left=0/1, Right=2, Mono=3 */
	if (frsm_dev->swap_channel)
		chs12 = (chs12 == 1) ? 2 : (chs12 == 2) ? 1 : 3;

	ret = frsm_reg_update_bits(frsm_dev, FS19V1_17H_I2SCTRL,
			FS19V1_17H_CHS12_MASK, chs12 << FS19V1_17H_CHS12_SHIFT);

	return ret;
}

static int fs19v1_shut_down(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_write(frsm_dev, FS19V1_11H_SYSCTRL,
			FS19V1_11H_SYSTEM_DOWN);

	return ret;
}

static int fs19v1_irq_handler(struct frsm_dev *frsm_dev)
{
	uint16_t intstat;
	int ret;

	ret = frsm_reg_read(frsm_dev, FS19V1_ABH_INTSTAT, &intstat);
	if (intstat & FS19V1_ABH_INTS14_MASK)
		dev_err(frsm_dev->dev, "VDDPS detected\n");
	if (intstat & FS19V1_ABH_INTS13_MASK)
		dev_err(frsm_dev->dev, "OC detected\n");
	if (intstat & FS19V1_ABH_INTS12_MASK)
		dev_err(frsm_dev->dev, "UV detected\n");
	if (intstat & FS19V1_ABH_INTS11_MASK)
		dev_err(frsm_dev->dev, "OV detected\n");
	if (intstat & FS19V1_ABH_INTS10_MASK)
		dev_err(frsm_dev->dev, "OTP detected\n");
	if (intstat & FS19V1_ABH_INTS8_MASK)
		dev_err(frsm_dev->dev, "BOV detected\n");
	if (intstat & FS19V1_ABH_INTS7_MASK)
		dev_err(frsm_dev->dev, "IND error detected\n");
	if (intstat & FS19V1_ABH_INTS6_MASK)
		dev_err(frsm_dev->dev, "BOL detected\n");
	if (intstat & FS19V1_ABH_INTS5_MASK)
		dev_err(frsm_dev->dev, "BOP detected\n");
	if (intstat & FS19V1_ABH_INTS4_MASK)
		dev_err(frsm_dev->dev, "OTW detected\n");
	if (intstat & FS19V1_ABH_INTS1_MASK)
		dev_err(frsm_dev->dev, "Clock lost detected\n");

	ret |= frsm_reg_read(frsm_dev, FS19V1_ABH_INTSTAT, &intstat);

	return ret;
}

static int fs19v1_stat_monitor(struct frsm_dev *frsm_dev)
{
	uint16_t status, anastat;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (frsm_dev->irq_id > 0)
		return fs19v1_irq_handler(frsm_dev);

	ret  = frsm_reg_read_status(frsm_dev, FS19V1_00H_STATUS, &status);
	ret |= frsm_reg_read_status(frsm_dev, FS19V1_05H_ANASTAT, &anastat);
	if (ret)
		return ret;
	if (status == FS19V1_00H_STATUS_OK
			&& (anastat & 0xF) == FS19V1_05H_ANASTAT_OK)
		return 0;

	if (status & FS19V1_00H_VDDPS_MASK)
		dev_err(frsm_dev->dev, "VDDP short detected\n");
	if (status & FS19V1_00H_INDS_MASK)
		dev_err(frsm_dev->dev, "Inductance diagnose error\n");
	if (status & FS19V1_00H_BOLDS_MASK)
		dev_err(frsm_dev->dev, "Current limit overload\n");
	if (status & FS19V1_00H_BOPS_MASK)
		dev_err(frsm_dev->dev, "Brownout protection active\n");
	if (status & FS19V1_00H_OTWDS_MASK)
		dev_err(frsm_dev->dev, "OT warning detected\n");
	if ((status & FS19V1_00H_CLKS_MASK) == 0)
		dev_err(frsm_dev->dev, "CLK ratio unstable\n");
	if (anastat & FS19V1_05H_OCDS_MASK)
		dev_err(frsm_dev->dev, "OC detected in AMP\n");
	if (status & FS19V1_00H_UVDS_MASK)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if (status & FS19V1_00H_OVDS_MASK)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if (status & FS19V1_00H_OTPDS_MASK)
		dev_err(frsm_dev->dev, "OT detected\n");
	if ((status & FS19V1_00H_PLLS_MASK) == 0)
		dev_err(frsm_dev->dev, "PLL unlock\n");
	if (status & FS19V1_00H_BOVDS_MASK)
		dev_err(frsm_dev->dev, "OV detected on boost\n");

	ret = frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs19v1_dev_ops(struct frsm_dev *frsm_dev)
{
	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	frsm_dev->func = FRSM_HAS_IRQ;
	frsm_dev->ops.dev_init  = fs19v1_dev_init;
	frsm_dev->ops.set_scene = fs19v1_set_scene;
	frsm_dev->ops.set_volume = fs19v1_set_volume;
	frsm_dev->ops.hw_params = fs19v1_hw_params;
	frsm_dev->ops.start_up = fs19v1_start_up;
	frsm_dev->ops.set_mute  = fs19v1_set_mute;
	frsm_dev->ops.set_tsmute  = fs19v1_set_tsmute;
	frsm_dev->ops.set_channel = fs19v1_set_channel;
	frsm_dev->ops.shut_down = fs19v1_shut_down;
	frsm_dev->ops.stat_monitor = fs19v1_stat_monitor;
	frsm_dev->reg_amp_mute = ((FS19V1_10H_PWRCTRL << 16)
			| FS19V1_10H_POWER_DOWN);

	return 0;
}

#endif // __FS19V1_OPS_H__
