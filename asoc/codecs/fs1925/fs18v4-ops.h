/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-12-23 File created.
 */

#ifndef __FS18V4_OPS_H__
#define __FS18V4_OPS_H__

#include "internal.h"

#define FS18V4_05H_ANASTAT			0x05
#define FS18V4_05H_OCDL_SHIFT			13
#define FS18V4_05H_OCDL_MASK			0x2000
#define FS18V4_05H_UVDL_SHIFT			12
#define FS18V4_05H_UVDL_MASK			0x1000
#define FS18V4_05H_OVDL_SHIFT			11
#define FS18V4_05H_OVDL_MASK			0x0800
#define FS18V4_05H_OTPDL_SHIFT			10
#define FS18V4_05H_OTPDL_MASK			0x0400
#define FS18V4_05H_BOVDL_SHIFT			8
#define FS18V4_05H_BOVDL_MASK			0x0100
#define FS18V4_05H_BOLDL_SHIFT			6
#define FS18V4_05H_BOLDL_MASK			0x0040
#define FS18V4_05H_BOPDL_SHIFT			5
#define FS18V4_05H_BOPDL_MASK			0x0020
#define FS18V4_05H_OTWDL_SHIFT			4
#define FS18V4_05H_OTWDL_MASK			0x0010
#define FS18V4_05H_AMPS_SHIFT			3
#define FS18V4_05H_AMPS_MASK			0x0008
#define FS18V4_05H_BSTS_SHIFT			2
#define FS18V4_05H_BSTS_MASK			0x0004
#define FS18V4_05H_PLLS_SHIFT			1
#define FS18V4_05H_PLLS_MASK			0x0002
#define FS18V4_05H_VBGS_SHIFT			0
#define FS18V4_05H_VBGS_MASK			0x0001
#define FS18V4_05H_ANASTAT_OK			0x000F

#define FS18V4_06H_DIGSTAT			0x06
#define FS18V4_06H_DACRUN_SHIFT			1
#define FS18V4_06H_DACRUN_MASK			0x0002

#define FS18V4_08H_STATUS			0x08
#define FS18V4_08H_BOLDS_SHIFT			10
#define FS18V4_08H_BOLDS_MASK			0x0400
#define FS18V4_08H_BOPS_SHIFT			9
#define FS18V4_08H_BOPS_MASK			0x0200
#define FS18V4_08H_OTWDS_SHIFT			8
#define FS18V4_08H_OTWDS_MASK			0x0100
#define FS18V4_08H_CLKS_SHIFT			6
#define FS18V4_08H_CLKS_MASK			0x0040
#define FS18V4_08H_OCDS_SHIFT			5
#define FS18V4_08H_OCDS_MASK			0x0020
#define FS18V4_08H_UVDS_SHIFT			4
#define FS18V4_08H_UVDS_MASK			0x0010
#define FS18V4_08H_OVDS_SHIFT			3
#define FS18V4_08H_OVDS_MASK			0x0008
#define FS18V4_08H_OTPDS_SHIFT			2
#define FS18V4_08H_OTPDS_MASK			0x0004
#define FS18V4_08H_PLLS_SHIFT			1
#define FS18V4_08H_PLLS_MASK			0x0002
#define FS18V4_08H_BOVDS_SHIFT			0
#define FS18V4_08H_BOVDS_MASK			0x0001
#define FS18V4_08H_STATUS_OK			0x0042

#define FS18V4_0BH_ACCKEY			0x0B
#define FS18V4_0BH_ACCKEY_DFT			0x0000
#define FS18V4_0BH_ACCKEY_ON			0xCA91
#define FS18V4_0BH_ACCKEY_OFF			0x5A5A

#define FS18V4_10H_PWRCTRL			0x10
#define FS18V4_10H_I2C_RESET			0x0003
#define FS18V4_10H_POWER_UP			0x0000
#define FS18V4_10H_POWER_DOWN			0x0001

#define FS18V4_11H_SYSCTRL			0x11
#define FS18V4_11H_SYSTEM_UP			0x00EB
#define FS18V4_11H_SYSTEM_DOWN			0x0000

#define FS18V4_16H_AUDIOCTRL			0x16
#define FS18V4_16H_VOL_SHIFT			5
#define FS18V4_16H_VOL_MASK			0xFFE0

#define FS18V4_17H_I2SCTRL			0x17
#define FS18V4_17H_I2SSR_SHIFT			12
#define FS18V4_17H_I2SSR_MASK			0xF000
#define FS18V4_17H_I2SDOE_SHIFT			11
#define FS18V4_17H_I2SDOE_MASK			0x0800
#define FS18V4_17H_CHS12_SHIFT			3
#define FS18V4_17H_CHS12_MASK			0x0018
#define FS18V4_17H_I2SF_SHIFT			0
#define FS18V4_17H_I2SF_MASK			0x0007

#define FS18V4_A1H_PLLCTRL1			0xA1
#define FS18V4_A2H_PLLCTRL2			0xA2
#define FS18V4_A3H_PLLCTRL3			0xA3

#define FS18V4_ABH_INTSTAT			0xAB
#define FS18V4_ABH_INTS13_SHIFT			13
#define FS18V4_ABH_INTS13_MASK			0x2000
#define FS18V4_ABH_INTS12_SHIFT			12
#define FS18V4_ABH_INTS12_MASK			0x1000
#define FS18V4_ABH_INTS11_SHIFT			11
#define FS18V4_ABH_INTS11_MASK			0x0800
#define FS18V4_ABH_INTS10_SHIFT			10
#define FS18V4_ABH_INTS10_MASK			0x0400
#define FS18V4_ABH_INTS8_SHIFT			8
#define FS18V4_ABH_INTS8_MASK			0x0100
#define FS18V4_ABH_INTS5_SHIFT			5
#define FS18V4_ABH_INTS5_MASK			0x0020
#define FS18V4_ABH_INTS4_SHIFT			4
#define FS18V4_ABH_INTS4_MASK			0x0010
#define FS18V4_ABH_INTS1_SHIFT			1
#define FS18V4_ABH_INTS1_MASK			0x0002

#define FS18V4_VOL_OFFSET			(0x57F - 0x3FF)

static struct frsm_rate g_fs18v4_rates[] = {
	{   8000, 0x0 }, // RATE_8000
	{  16000, 0x3 }, // RATE_16000
	{  32000, 0x7 }, // RATE_16000
	{  44100, 0x8 }, // RATE_44100
	{  48000, 0x9 }, // RATE_48000
	{  88200, 0xA }, // RATE_88200
	{  96000, 0xB }, // RATE_96000
};

static struct frsm_format g_fs18v4_formats[] = {
	{ 1, 3 }, // SND_SOC_DAIFMT_I2S
	{ 2, 7 }, // SND_SOC_DAIFMT_RIGHT_J
	{ 3, 2 }, // SND_SOC_DAIFMT_LEFT_J
	{ 4, 1 }, // SND_SOC_DAIFMT_DSP_A
	{ 5, 1 }, // SND_SOC_DAIFMT_DSP_B
};

static struct frsm_pll g_fs18v4_plls[] = {
	{   512000, 0x0260, 0x0180, 0x0002 },
	{   768000, 0x0260, 0x0180, 0x0003 },
	{  1024000, 0x0260, 0x0180, 0x0004 },
	{  1536000, 0x0260, 0x0180, 0x0006 },
	{  2048000, 0x0260, 0x0180, 0x0008 },
	{  2304000, 0x0260, 0x0180, 0x0008 },
	{  3072000, 0x0260, 0x0180, 0x000C },
	{  4096000, 0x0260, 0x0180, 0x0010 },
	{  4608000, 0x0260, 0x0180, 0x0012 },
	{  6144000, 0x0260, 0x0180, 0x0018 },
	{  8192000, 0x0260, 0x0180, 0x0020 },
	{  9216000, 0x0260, 0x0180, 0x0024 },
	{ 12288000, 0x0260, 0x0180, 0x0030 },
	{ 16384000, 0x0260, 0x0180, 0x0040 },
	{ 18432000, 0x0260, 0x0180, 0x0048 },
	{ 24576000, 0x0260, 0x0180, 0x0060 },
	{  1411200, 0x0260, 0x0180, 0x0006 },
	{  2822400, 0x0260, 0x0180, 0x000C },
	{  4233600, 0x0260, 0x0180, 0x0012 },
	{  5644800, 0x0260, 0x0180, 0x0018 },
	{  8467200, 0x0260, 0x0180, 0x0024 },
	{ 11289600, 0x0260, 0x0180, 0x0030 },
	{ 16934400, 0x0260, 0x0180, 0x0048 },
	{ 22579200, 0x0260, 0x0180, 0x0060 },
	{   960000, 0x0270, 0x0133, 0x0003 },
};

static int fs18v4_set_mute(struct frsm_dev *frsm_dev, int mute);
static int fs18v4_shut_down(struct frsm_dev *frsm_dev);
static int fs18v4_set_channel(struct frsm_dev *frsm_dev);

static int fs18v4_set_i2s_config(struct frsm_dev *frsm_dev)
{
	struct frsm_hw_params *hw_params;
	struct frsm_format *formats;
	uint16_t val, mask;
	int idx;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	hw_params = &frsm_dev->hw_params;
	for (idx = 0; idx < ARRAY_SIZE(g_fs18v4_rates); idx++)
		if (g_fs18v4_rates[idx].rate == hw_params->rate)
			break;

	if (idx == ARRAY_SIZE(g_fs18v4_rates)) {
		dev_err(frsm_dev->dev, "Invalid sample rate:%d\n",
				hw_params->rate);
		return -EINVAL;
	}

	mask = FS18V4_17H_I2SSR_MASK;
	val = g_fs18v4_rates[idx].i2ssr << FS18V4_17H_I2SSR_SHIFT;

	if (hw_params->format != 0xFF) {
		formats = g_fs18v4_formats;
		for (idx = 0; idx < ARRAY_SIZE(g_fs18v4_formats); idx++) {
			if (formats->pcm_format == hw_params->format)
				break;
			formats++;
		}

		if (idx == ARRAY_SIZE(g_fs18v4_formats)) {
			dev_err(frsm_dev->dev, "Invalid pcm format:%d\n",
					hw_params->format);
			return -EINVAL;
		}

		mask |= FS18V4_17H_I2SF_MASK;
		val |= formats->i2sf << FS18V4_17H_I2SF_SHIFT;
	}

	ret = frsm_reg_update_bits(frsm_dev, FS18V4_17H_I2SCTRL, mask, val);

	return ret;
}

static int fs18v4_dev_init(struct frsm_dev *frsm_dev)
{
	struct scene_table *scene;
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FS18V4_0BH_ACCKEY, &val);
	if (ret || (!frsm_dev->force_init && val != FS18V4_0BH_ACCKEY_DFT))
		return ret;

	frsm_dev->force_init = false;
	frsm_dev->state &= ~(BIT(EVENT_STAT_MNTR) - 1);

	ret = frsm_reg_write(frsm_dev, FS18V4_10H_PWRCTRL,
			     FS18V4_10H_I2C_RESET);
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

	ret |= fs18v4_set_mute(frsm_dev, 1);
	ret |= fs18v4_shut_down(frsm_dev);
	if (!ret) {
		frsm_dev->cur_scene = scene;
		frsm_reg_write(frsm_dev, FS18V4_0BH_ACCKEY,
			       FS18V4_0BH_ACCKEY_OFF);
	}

	if (frsm_dev->pdata->rx_volume_v2) {
		frsm_reg_read(frsm_dev, FS18V4_16H_AUDIOCTRL, &val);
		frsm_dev->volume = (val >> FS18V4_16H_VOL_SHIFT)
				- FS18V4_VOL_OFFSET;
	}

	frsm_reg_read_status(frsm_dev, FS18V4_ABH_INTSTAT, &val);

	return ret;
}

static int fs18v4_set_scene(struct frsm_dev *frsm_dev,
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
		frsm_reg_read(frsm_dev, FS18V4_16H_AUDIOCTRL, &val);
		frsm_dev->volume = (val >> FS18V4_16H_VOL_SHIFT)
				- FS18V4_VOL_OFFSET;
	}

	return ret;
}

static int fs18v4_set_volume(struct frsm_dev *frsm_dev, uint16_t volume)
{
	uint16_t vol;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	vol = volume;
	if (vol > FRSM_VOLUME_MAX)
		vol = FRSM_VOLUME_MAX;

	vol += FS18V4_VOL_OFFSET;
	ret = frsm_reg_write(frsm_dev, FS18V4_16H_AUDIOCTRL,
			     vol << FS18V4_16H_VOL_SHIFT);

	return ret;
}

static int fs18v4_hw_params(struct frsm_dev *frsm_dev)
{
	unsigned int bclk;
	int idx;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	bclk = frsm_dev->hw_params.bclk;
	dev_dbg(frsm_dev->dev, "hw_params bclk:%d\n", bclk);
	for (idx = 0; idx < ARRAY_SIZE(g_fs18v4_plls); idx++)
		if (g_fs18v4_plls[idx].bclk == bclk)
			break;

	if (idx == ARRAY_SIZE(g_fs18v4_plls)) {
		dev_err(frsm_dev->dev, "Invalid bclk:%d\n", bclk);
		return -EINVAL;
	}

	ret  = fs18v4_set_i2s_config(frsm_dev);
	ret |= fs18v4_set_channel(frsm_dev);
	ret |= frsm_reg_write(frsm_dev, FS18V4_A1H_PLLCTRL1,
			g_fs18v4_plls[idx].pll1);
	ret |= frsm_reg_write(frsm_dev, FS18V4_A2H_PLLCTRL2,
			g_fs18v4_plls[idx].pll2);
	ret |= frsm_reg_write(frsm_dev, FS18V4_A3H_PLLCTRL3,
			g_fs18v4_plls[idx].pll3);

	return ret;
}

static int fs18v4_start_up(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret  = frsm_reg_write(frsm_dev, FS18V4_11H_SYSCTRL,
			FS18V4_11H_SYSTEM_UP);
	ret |= fs18v4_set_volume(frsm_dev, frsm_dev->volume);

	return ret;
}

static int fs18v4_set_mute(struct frsm_dev *frsm_dev, int mute)
{
	uint16_t val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (mute) {
		ret  = frsm_reg_write(frsm_dev, FS18V4_10H_PWRCTRL,
				FS18V4_10H_POWER_DOWN);
		ret |= frsm_reg_wait_stable(frsm_dev, FS18V4_06H_DIGSTAT,
				FS18V4_06H_DACRUN_MASK,
				0 << FS18V4_06H_DACRUN_SHIFT);
	} else {
		ret = frsm_reg_write(frsm_dev, FS18V4_10H_PWRCTRL,
				FS18V4_10H_POWER_UP);
	}
	frsm_reg_read_status(frsm_dev, FS18V4_ABH_INTSTAT, &val);

	return ret;
}

static int fs18v4_set_tsmute(struct frsm_dev *frsm_dev, bool mute)
{
	return 0;
}

static int fs18v4_set_channel(struct frsm_dev *frsm_dev)
{
	uint16_t chs12, val;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	if (frsm_dev->pdata->rx_channel < 0) {
		ret = frsm_reg_read(frsm_dev, FS18V4_17H_I2SCTRL, &val);
		if (ret)
			return ret;
		chs12 = (val & FS18V4_17H_CHS12_MASK) >> FS18V4_17H_CHS12_SHIFT;
		frsm_dev->pdata->rx_channel = chs12;
	} else {
		chs12 = frsm_dev->pdata->rx_channel;
		chs12 &= (FS18V4_17H_CHS12_MASK >> FS18V4_17H_CHS12_SHIFT);
	}

	/* CHS12: Left=0/1, Right=2, Mono=3 */
	if (frsm_dev->swap_channel)
		chs12 = (chs12 == 1) ? 2 : (chs12 == 2) ? 1 : 3;

	ret = frsm_reg_update_bits(frsm_dev, FS18V4_17H_I2SCTRL,
			FS18V4_17H_CHS12_MASK, chs12 << FS18V4_17H_CHS12_SHIFT);

	return ret;
}

static int fs18v4_shut_down(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_write(frsm_dev, FS18V4_11H_SYSCTRL,
			FS18V4_11H_SYSTEM_DOWN);

	return ret;
}

static int fs18v4_irq_handler(struct frsm_dev *frsm_dev)
{
	uint16_t intstat;
	int ret;

	ret = frsm_reg_read(frsm_dev, FS18V4_ABH_INTSTAT, &intstat);
	if (ret || intstat == 0x0000)
		return ret;

	if (intstat & FS18V4_ABH_INTS13_MASK)
		dev_err(frsm_dev->dev, "OC detected\n");
	if (intstat & FS18V4_ABH_INTS12_MASK)
		dev_err(frsm_dev->dev, "UV detected\n");
	if (intstat & FS18V4_ABH_INTS11_MASK)
		dev_err(frsm_dev->dev, "OV detected\n");
	if (intstat & FS18V4_ABH_INTS10_MASK)
		dev_err(frsm_dev->dev, "OTP detected\n");
	if (intstat & FS18V4_ABH_INTS8_MASK)
		dev_err(frsm_dev->dev, "BOV detected\n");
	if (intstat & FS18V4_ABH_INTS5_MASK)
		dev_err(frsm_dev->dev, "BOP detected\n");
	if (intstat & FS18V4_ABH_INTS4_MASK)
		dev_err(frsm_dev->dev, "OTW detected\n");
	if (intstat & FS18V4_ABH_INTS1_MASK)
		dev_err(frsm_dev->dev, "Clock lost detected\n");

	ret = frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs18v4_stat_monitor(struct frsm_dev *frsm_dev)
{
	uint16_t status, anastat;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (frsm_dev->irq_id > 0)
		return fs18v4_irq_handler(frsm_dev);

	ret = frsm_reg_read_status(frsm_dev, FS18V4_05H_ANASTAT, &anastat);
	ret |= frsm_reg_read_status(frsm_dev, FS18V4_08H_STATUS, &status);
	if (ret)
		return ret;

	if (status == FS18V4_08H_STATUS_OK
			&& (anastat & 0xF) == FS18V4_05H_ANASTAT_OK)
		return 0;

	if (status & FS18V4_08H_BOPS_MASK)
		dev_err(frsm_dev->dev, "Brownout protection active\n");
	if (status & FS18V4_08H_OTWDS_MASK)
		dev_err(frsm_dev->dev, "OT warning detected\n");
	if ((status & FS18V4_08H_CLKS_MASK) == 0)
		dev_err(frsm_dev->dev, "CLK ratio unstable\n");
	if (anastat & FS18V4_08H_OCDS_MASK)
		dev_err(frsm_dev->dev, "OC detected in AMP\n");
	if (status & FS18V4_08H_UVDS_MASK)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if (status & FS18V4_08H_OVDS_MASK)
		dev_err(frsm_dev->dev, "UV detected on VBAT\n");
	if (status & FS18V4_08H_OTPDS_MASK)
		dev_err(frsm_dev->dev, "OT detected\n");
	if ((status & FS18V4_08H_PLLS_MASK) == 0)
		dev_err(frsm_dev->dev, "PLL is unlock\n");
	if (status & FS18V4_08H_BOVDS_MASK)
		dev_err(frsm_dev->dev, "OV detected on boost\n");

	ret = frsm_reg_dump(frsm_dev, 0xCF); // dump reg: 00~CF

	return ret;
}

static int fs18v4_dev_ops(struct frsm_dev *frsm_dev)
{
	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	frsm_dev->func = FRSM_HAS_IRQ | FRSM_SWAP_CHN;
	frsm_dev->ops.dev_init = fs18v4_dev_init;
	frsm_dev->ops.set_scene = fs18v4_set_scene;
	frsm_dev->ops.set_volume = fs18v4_set_volume;
	frsm_dev->ops.hw_params = fs18v4_hw_params;
	frsm_dev->ops.start_up = fs18v4_start_up;
	frsm_dev->ops.set_mute = fs18v4_set_mute;
	frsm_dev->ops.set_tsmute = fs18v4_set_tsmute;
	frsm_dev->ops.set_channel = fs18v4_set_channel;
	frsm_dev->ops.shut_down = fs18v4_shut_down;
	frsm_dev->ops.stat_monitor = fs18v4_stat_monitor;
	frsm_dev->reg_amp_mute = (FS18V4_10H_PWRCTRL << 16)
			| FS18V4_10H_POWER_DOWN;

	return 0;
}

#endif // __FS18V4_OPS_H__
