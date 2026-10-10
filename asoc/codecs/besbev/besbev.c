// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/component.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <soc/soundwire.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include "internal.h"
#include "besbev.h"
#include "besbev-registers.h"
#include <asoc/wcdcal-hwdep.h>
#include "pmw5100-spmi.h"
#include <asoc/msm-cdc-pinctrl.h>
#include <bindings/audio-codec-port-types.h>
#include <asoc/msm-cdc-supply.h>
#include <linux/power_supply.h>
#include "asoc/bolero-slave-internal.h"

#define NUM_SWRS_DT_PARAMS 5

#define BESBEV_VERSION_1_0 1
#define BESBEV_VERSION_ENTRY_SIZE 32

#define NUM_ATTEMPTS 5
#define SOC_THRESHOLD_LEVEL 25
#define LOW_SOC_MBIAS_REG_MIN_VOLTAGE 2850000

#define FOUNDRY_ID_SEC 0x5

#define T1_TEMP -10
#define T2_TEMP 150
#define LOW_TEMP_THRESHOLD 5
#define UNLOCK_CONTIN_TEMP_READ 0x01
#define LOCK_CONTIN_TEMP_READ 0x00
#define HIGH_TEMP_THRESHOLD 45
#define TEMP_INVALID	0xFFFF
#define BESBEV_TEMP_RETRY 3

#define MAX_NAME_LEN	40
#define BESBEV_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			SNDRV_PCM_RATE_384000)
/* Fractional Rates */
#define BESBEV_FRAC_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800)

#define BESBEV_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE |\
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

enum {
	CODEC_TX = 0,
	CODEC_RX,
};

enum {
	MIC_BIAS_1 = 1,
	MIC_BIAS_2
};

enum {
	MICB_PULLUP_ENABLE,
	MICB_PULLUP_DISABLE,
	MICB_ENABLE,
	MICB_DISABLE,
};

/* TODO: Check on the step values */
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

static int besbev_handle_post_irq(void *data);
static int besbev_reset(struct device *dev, int val);
static int besbev_get_temperature(struct snd_soc_component *component,
				   int *temp);
enum {
	SPKR_STATUS = 0,
	SPKR_ADIE_LB,
	WCD_SUPPLIES_LPM_MODE,
	BESBEV_SPKR_BOOST_ENABLE,
};

enum {
	BESBEV_IRQ_INT_SAF2WAR = 0,
	BESBEV_IRQ_INT_WAR2SAF,
	BESBEV_IRQ_INT_DISABLE,
	BESBEV_IRQ_INT_OCP,
	BESBEV_IRQ_INT_CLIP,
	BESBEV_IRQ_INT_PDM_WD,
	BESBEV_IRQ_INT_CLK_WD,
	BESBEV_IRQ_INT_INTR_PIN,
	BESBEV_IRQ_INT_UVLO,
	BESBEV_IRQ_INT_PA_ON_ERR,
	BESBEV_NUM_IRQS,
};

struct bb_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

static const struct bb_reg_mask_val bb_init[] = {
	{BESBEV_IVSENSE_ADC_5,                  0x30,   0x05},
	{BESBEV_IVSENSE_ADC_4,                  0x30,   0x05},
	{BESBEV_IVSENSE_ISENSE2,                0x0F,   0x0A},
	{BESBEV_IVSENSE_ADC_6,                  0x02,   0x02},
	{BESBEV_IVSENSE_ADC_2,                  0x40,   0x00},
	{BESBEV_IVSENSE_ADC_7,                  0x04,   0x04},
	{BESBEV_IVSENSE_ADC_7,                  0x02,   0x02},
	{BESBEV_SPK_TOP_DAC_CTRL_REG,           0x01,   0x01},
	{BESBEV_DRE_CTL_1,                      0x01,   0x01},
	{BESBEV_VAGC_TIME,                      0x0C,   0x0C},
	{BESBEV_VAGC_TIME,                      0x03,   0x03},
	{BESBEV_VAGC_ATTN_LVL_1_2,              0x30,   0x11},
	{BESBEV_VAGC_ATTN_LVL_3,                0x03,   0x02},
	// Enable BCL or Enable "VBAT_AGC_EN (BOP)"
	{BESBEV_VAGC_CTL,                       0x71,   0x41},
	{BESBEV_TAGC_CTL,                       0x0E,   0x0A},
	{BESBEV_TAGC_TIME,                      0x30,   0x10},
	{BESBEV_TAGC_E2E_GAIN,                  0x07,   0x04},
	{BESBEV_TAGC_CTL,                       0x01,   0x01},
	{BESBEV_TEMP_CONFIG0,                   0x07,   0x02},
	{BESBEV_TEMP_CONFIG1,                   0x07,   0x02},
	{BESBEV_PA_OTP_LOW_M,                   0xFF,   0x49},
	{BESBEV_PA_OTP_LOW_L,                   0xC0,   0x80},
	{BESBEV_PA_OTP_HIGH_M,                  0xFF,   0xC9},
	{BESBEV_PA_OTP_HIGH_L,                  0xC0,   0x40},
};

static const struct regmap_irq BESBEV_IRQs[BESBEV_NUM_IRQS] = {
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_SAF2WAR, 0, 0x01),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_WAR2SAF, 0, 0x02),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_DISABLE, 0, 0x04),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_OCP, 0, 0x08),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_CLIP, 0, 0x10),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_PDM_WD, 0, 0x20),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_CLK_WD, 0, 0x40),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_INTR_PIN, 0, 0x80),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_UVLO, 1, 0x01),
	REGMAP_IRQ_REG(BESBEV_IRQ_INT_PA_ON_ERR, 1, 0x02),
};

static struct regmap_irq_chip besbev_regmap_irq_chip = {
	.name = "besbev",
	.irqs = BESBEV_IRQs,
	.num_irqs = ARRAY_SIZE(BESBEV_IRQs),
	.num_regs = 2,
	.status_base = BESBEV_INTR_STATUS0,
	.mask_base = BESBEV_INTR_MASK0,
	.ack_base = BESBEV_INTR_CLEAR0,
	.use_ack = 1,
	.type_base = BESBEV_INTR_LEVEL0,
	.runtime_pm = false,
	.handle_post_irq = besbev_handle_post_irq,
	.irq_drv_data = NULL,
};

static int besbev_handle_post_irq(void *data)
{
	struct besbev_priv *besbev = data;
	u32 sts1 = 0, sts2 = 0;

	regmap_read(besbev->regmap, BESBEV_INTR_STATUS0, &sts1);
	regmap_read(besbev->regmap, BESBEV_INTR_STATUS1, &sts2);

	besbev->swr_dev->slave_irq_pending =
			((sts1 || sts2) ? true : false);

	return IRQ_HANDLED;
}

static int besbev_init_reg(struct snd_soc_component *component,
				bool speaker_present)
{
	int i;

	if (speaker_present == true) {
		for (i = 0; i < ARRAY_SIZE(bb_init); i++) {
			snd_soc_component_update_bits(component, bb_init[i].reg,
						bb_init[i].mask, bb_init[i].val);
		}
	}
	/* Disable mic bias pull down */
	snd_soc_component_update_bits(component, BESBEV_ANA_MICBIAS_MICB_1_2_EN,
								0x01, 0x00);
	snd_soc_component_update_bits(component, BESBEV_CKWD_CKWD_CTL_1,
								0x1F, 0x1B);
	return 0;
}

static int besbev_set_port_params(struct snd_soc_component *component,
				u8 slv_prt_type, u8 *port_id, u8 *num_ch,
				u8 *ch_mask, u32 *ch_rate,
				u8 *port_type, u8 path)
{
	int i, j;
	u8 num_ports = 0;
	struct codec_port_info (*map)[MAX_PORT][MAX_CH_PER_PORT] = NULL;
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	switch (path) {
	case CODEC_RX:
		map = &besbev->rx_port_mapping;
		num_ports = besbev->num_rx_ports;
		break;
	case CODEC_TX:
		map = &besbev->tx_port_mapping;
		num_ports = besbev->num_tx_ports;
		break;
	default:
		dev_err(component->dev, "%s: Invalid path: %d\n",
			__func__, path);
		return -EINVAL;
	}

	for (i = 0; i <= num_ports; i++) {
		for (j = 0; j < MAX_CH_PER_PORT; j++) {
			if ((*map)[i][j].slave_port_type == slv_prt_type)
				goto found;
		}
	}

	dev_err(component->dev, "%s: Failed to find slave port for type %u\n",
					__func__, slv_prt_type);
	return -EINVAL;
found:
	*port_id = i;
	*num_ch = (*map)[i][j].num_ch;
	*ch_mask = (*map)[i][j].ch_mask;
	*ch_rate = (*map)[i][j].ch_rate;
	*port_type = (*map)[i][j].master_port_type;

	return 0;
}

static int besbev_parse_port_mapping(struct device *dev,
			char *prop, u8 path)
{
	u32 *dt_array, map_size, map_length;
	u32 port_num = 0, ch_mask, ch_rate, old_port_num = 0;
	u32 slave_port_type, master_port_type;
	u32 i, ch_iter = 0;
	int ret = 0;
	u8 *num_ports = NULL;
	struct codec_port_info (*map)[MAX_PORT][MAX_CH_PER_PORT] = NULL;
	struct besbev_priv *besbev = dev_get_drvdata(dev);

	switch (path) {
	case CODEC_RX:
		map = &besbev->rx_port_mapping;
		num_ports = &besbev->num_rx_ports;
		break;
	case CODEC_TX:
		map = &besbev->tx_port_mapping;
		num_ports = &besbev->num_tx_ports;
		break;
	default:
		dev_err(dev, "%s: Invalid path: %d\n",
			__func__, path);
		return -EINVAL;
	}

	if (!of_find_property(dev->of_node, prop,
				&map_size)) {
		dev_err(dev, "%s: missing port mapping prop %s\n",
				__func__, prop);
		ret = -EINVAL;
		goto err;
	}

	map_length = map_size / (NUM_SWRS_DT_PARAMS * sizeof(u32));

	dt_array = kzalloc(map_size, GFP_KERNEL);

	if (!dt_array) {
		ret = -ENOMEM;
		goto err;
	}
	ret = of_property_read_u32_array(dev->of_node, prop, dt_array,
				NUM_SWRS_DT_PARAMS * map_length);
	if (ret) {
		dev_err(dev, "%s: Failed to read  port mapping from prop %s\n",
					__func__, prop);
		ret = -EINVAL;
		goto err_pdata_fail;
	}

	for (i = 0; i < map_length; i++) {
		port_num = dt_array[NUM_SWRS_DT_PARAMS * i];

		if (port_num >= MAX_PORT || ch_iter >= MAX_CH_PER_PORT) {
			dev_err(dev, "%s: Invalid port or channel number\n", __func__);
			goto err_pdata_fail;
		}

		slave_port_type = dt_array[NUM_SWRS_DT_PARAMS * i + 1];
		ch_mask = dt_array[NUM_SWRS_DT_PARAMS * i + 2];
		ch_rate = dt_array[NUM_SWRS_DT_PARAMS * i + 3];
		master_port_type = dt_array[NUM_SWRS_DT_PARAMS * i + 4];

		if (port_num != old_port_num)
			ch_iter = 0;

		(*map)[port_num][ch_iter].slave_port_type = slave_port_type;
		(*map)[port_num][ch_iter].ch_mask = ch_mask;
		(*map)[port_num][ch_iter].master_port_type = master_port_type;
		(*map)[port_num][ch_iter].num_ch = __sw_hweight8(ch_mask);
		(*map)[port_num][ch_iter++].ch_rate = ch_rate;
		old_port_num = port_num;
	}
	*num_ports = port_num;

err_pdata_fail:
	kfree(dt_array);
err:
	return ret;
}

static int besbev_tx_connect_port(struct snd_soc_component *component,
					u8 slv_port_type, u8 enable)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	u8 port_id;
	u8 num_ch;
	u8 ch_mask;
	u32 ch_rate;
	u8 port_type;
	u8 num_port = 1;
	int ret = 0;

	ret = besbev_set_port_params(component, slv_port_type, &port_id,
				&num_ch, &ch_mask, &ch_rate,
				&port_type, CODEC_TX);

	if (ret) {
		dev_err(besbev->dev, "%s: Failed to set port params: %d\n",
			__func__, ret);
		return ret;
	}

	if (enable)
		ret = swr_connect_port(besbev->swr_dev, &port_id,
					num_port, &ch_mask, &ch_rate,
					 &num_ch, &port_type);
	else
		ret = swr_disconnect_port(besbev->swr_dev, &port_id,
					num_port, &ch_mask, &port_type);
	return ret;

}
static int besbev_rx_connect_port(struct snd_soc_component *component,
					u8 slv_port_type, u8 enable)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	u8 port_id;
	u8 num_ch;
	u8 ch_mask;
	u32 ch_rate;
	u8 port_type;
	u8 num_port = 1;
	int ret = 0;

	ret = besbev_set_port_params(component, slv_port_type, &port_id,
				&num_ch, &ch_mask, &ch_rate,
				&port_type, CODEC_RX);

	if (ret) {
		dev_err(besbev->dev, "%s: Failed to set port params: %d\n",
			__func__, ret);
		return ret;
	}

	if (enable)
		ret = swr_connect_port(besbev->swr_dev, &port_id,
					num_port, &ch_mask, &ch_rate,
					&num_ch, &port_type);
	else
		ret = swr_disconnect_port(besbev->swr_dev, &port_id,
					num_port, &ch_mask, &port_type);
	return ret;
}

static int besbev_swr_ctrl(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	int ret = 0;
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	if (!besbev) {
		dev_err(component->dev, "%s: besbev is NULL\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = swr_slvdev_datapath_control(besbev->swr_dev,
					besbev->swr_dev->dev_num, true);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (besbev->speaker_present == true)
			besbev_rx_connect_port(component, ADC1 + (w->shift),
						false);
		else
			besbev_tx_connect_port(component, ADC1 + (w->shift),
						false);
		if (w->shift) {
			snd_soc_component_update_bits(component,
				BESBEV_ANA_TX_MISC_CTL, 0x02, 0x00);
			snd_soc_component_update_bits(component,
				BESBEV_DIG_SWR_CDC_TX_MODE, 0x30, 0x00);
		} else {
			snd_soc_component_update_bits(component,
				BESBEV_ANA_TX_MISC_CTL, 0x04, 0x00);
			snd_soc_component_update_bits(component,
				BESBEV_DIG_SWR_CDC_TX_MODE, 0x03, 0x00);
		}
		besbev_global_mbias_disable(component);
		break;
	};

	return ret;
}

int besbev_global_mbias_enable(struct snd_soc_component *component)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	mutex_lock(&besbev->main_bias_lock);
	if (besbev->mbias_cnt == 0) {
		usleep_range(6000, 6100);
		snd_soc_component_update_bits(component,
				BESBEV_ANA_MBIAS_TSADC_EN, 0x20, 0x20);
		snd_soc_component_update_bits(component,
				BESBEV_ANA_MBIAS_TSADC_EN, 0x10, 0x10);
	}
	besbev->mbias_cnt++;
	mutex_unlock(&besbev->main_bias_lock);

	return 0;
}

int besbev_global_mbias_disable(struct snd_soc_component *component)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	mutex_lock(&besbev->main_bias_lock);
	if (besbev->mbias_cnt == 0) {
		dev_dbg(besbev->dev, "%s: mbias already disabled\n", __func__);
		mutex_unlock(&besbev->main_bias_lock);
		return 0;
	}
	besbev->mbias_cnt--;
	if (besbev->mbias_cnt == 0) {
		snd_soc_component_update_bits(component,
				BESBEV_ANA_MBIAS_TSADC_EN, 0x10, 0x00);
		snd_soc_component_update_bits(component,
				BESBEV_ANA_MBIAS_TSADC_EN, 0x20, 0x00);
	}
	mutex_unlock(&besbev->main_bias_lock);

	return 0;
}

/*
 * besbev_disable_visense - Disable VISense for amic variant
 * @component: component instance
 *
 * Return: 0 on success or negative error code on failure.
 */
int besbev_disable_visense(struct snd_soc_component *component)
{
	/* Disable VISense for AMIC variant */
	snd_soc_component_update_bits(component,
				BESBEV_DIG_SWR_CDC_RX_MODE, 0x4, 0x4);
	return 0;
}
EXPORT_SYMBOL(besbev_disable_visense);

static int besbev_codec_enable_adc(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol,
				    int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct besbev_priv *besbev =
			snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (besbev->speaker_present == true)
			besbev_rx_connect_port(component, ADC1 + (w->shift),
						true);
		else
			besbev_tx_connect_port(component, ADC1 + (w->shift),
						true);
		besbev_global_mbias_enable(component);
		if (w->shift) {
			snd_soc_component_update_bits(component,
				BESBEV_ANA_TX_MISC_CTL, 0x12, 0x12);
			snd_soc_component_update_bits(component,
				BESBEV_DIG_SWR_CDC_TX_MODE, 0x30, 0x30);
		} else {
			snd_soc_component_update_bits(component,
				BESBEV_ANA_TX_MISC_CTL, 0x44, 0x44);
			snd_soc_component_update_bits(component,
				BESBEV_DIG_SWR_CDC_TX_MODE, 0x03, 0x03);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		swr_slvdev_datapath_control(besbev->swr_dev,
					besbev->swr_dev->dev_num, false);
		break;
	};

	return 0;
}

/*
 * besbev_get_micb_vout_ctl_val: converts micbias from volts to register value
 * @micb_mv: micbias in mv
 *
 * return register value converted
 */
int besbev_get_micb_vout_ctl_val(u32 micb_mv)
{
	/* min micbias voltage is 1.6V and maximum is 2.85V */
	if (micb_mv < 1600 || micb_mv > 2850) {
		pr_err("%s: unsupported micbias voltage\n", __func__);
		return -EINVAL;
	}

	return (micb_mv - 1600) / 50;
}
EXPORT_SYMBOL(besbev_get_micb_vout_ctl_val);

int besbev_micbias_control(struct snd_soc_component *component,
				int micb_num, int req, bool is_dapm)
{

	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	int micb_index = micb_num - 1;
	u16 micb_reg;
	u8 pullup_mask = 0, enable_mask = 0;
	int ret = 0;

	if ((micb_index < 0) || (micb_index > BESBEV_MAX_MICBIAS - 1)) {
		dev_err(component->dev, "%s: Invalid micbias index, micb_ind: %d\n",
			__func__, micb_index);
		return -EINVAL;
	}
	micb_reg = BESBEV_ANA_MICBIAS_MICB_1_2_EN;
	switch (micb_num) {
	case MIC_BIAS_1:
		pullup_mask = 0x20;
		enable_mask = 0x40;
		break;
	case MIC_BIAS_2:
		pullup_mask = 0x02;
		enable_mask = 0x04;
		break;
	default:
		dev_err(component->dev, "%s: Invalid micbias number: %d\n",
			__func__, micb_num);
		return -EINVAL;
	};
	mutex_lock(&besbev->micb_lock);

	switch (req) {
	case MICB_PULLUP_ENABLE:
		if (!besbev->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		besbev->pullup_ref[micb_index]++;
		if ((besbev->pullup_ref[micb_index] == 1) &&
		    (besbev->micb_ref[micb_index] == 0))
			snd_soc_component_update_bits(component, micb_reg,
						pullup_mask, pullup_mask);
		break;
	case MICB_PULLUP_DISABLE:
		if (!besbev->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		if (besbev->pullup_ref[micb_index] > 0)
			besbev->pullup_ref[micb_index]--;
		if ((besbev->pullup_ref[micb_index] == 0) &&
		    (besbev->micb_ref[micb_index] == 0))
			snd_soc_component_update_bits(component, micb_reg,
						pullup_mask, 0x00);
		break;
	case MICB_ENABLE:
		if (!besbev->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
						__func__, req);
			ret = -ENODEV;
			goto done;
		}
		besbev->micb_ref[micb_index]++;
		if (besbev->micb_ref[micb_index] == 1) {
			besbev_global_mbias_enable(component);
			snd_soc_component_update_bits(component,
					micb_reg, enable_mask, enable_mask);
		}
		break;
	case MICB_DISABLE:
		if (besbev->micb_ref[micb_index] > 0)
			besbev->micb_ref[micb_index]--;
		if (!besbev->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		if ((besbev->micb_ref[micb_index] == 0) &&
		    (besbev->pullup_ref[micb_index] > 0)) {
			snd_soc_component_update_bits(component, micb_reg,
						pullup_mask, pullup_mask);
			snd_soc_component_update_bits(component, micb_reg,
						enable_mask, 0x00);
			besbev_global_mbias_disable(component);
		} else if ((besbev->micb_ref[micb_index] == 0) &&
			   (besbev->pullup_ref[micb_index] == 0)) {
			snd_soc_component_update_bits(component, micb_reg,
						enable_mask, 0x00);
			besbev_global_mbias_disable(component);
		}
		break;
	};

	dev_dbg(component->dev, "%s: micb_num:%d, micb_ref: %d, pullup_ref: %d\n",
		__func__, micb_num, besbev->micb_ref[micb_index],
		besbev->pullup_ref[micb_index]);
done:
	mutex_unlock(&besbev->micb_lock);
	return 0;
}
EXPORT_SYMBOL(besbev_micbias_control);

static int besbev_get_logical_addr(struct swr_device *swr_dev)
{
	int ret = 0;
	uint8_t devnum = 0;
	int num_retry = NUM_ATTEMPTS;

	do {
		ret = swr_get_logical_dev_num(swr_dev, swr_dev->addr, &devnum);
		if (ret) {
			dev_err(&swr_dev->dev,
				"%s get devnum %d for dev addr %lx failed\n",
				__func__, devnum, swr_dev->addr);
			/* retry after 1ms */
			usleep_range(1000, 1010);
		}
	} while (ret && --num_retry);
	swr_dev->dev_num = devnum;
	return 0;
}

static int besbev_tx_event_notify(struct notifier_block *block,
				unsigned long val,
				void *data)
{
	u16 event = (val & 0xffff);
	struct besbev_priv *besbev = dev_get_drvdata((struct device *)data);
	struct snd_soc_component *component = besbev->component;

	switch (event) {
	case BOLERO_SLV_EVT_PA_OFF_PRE_SSR:
		break;
	case BOLERO_SLV_EVT_SSR_DOWN:
		besbev->dev_up = false;
		besbev_reset(besbev->dev, 0x01);
		break;
	case BOLERO_SLV_EVT_SSR_UP:
		besbev_reset(besbev->dev, 0x00);
		/* allow reset to take effect */
		usleep_range(10000, 10010);
		besbev_get_logical_addr(besbev->swr_dev);

		besbev_init_reg(component, false);
		mutex_lock(&besbev->res_lock);
		regcache_mark_dirty(besbev->regmap);
		regcache_sync(besbev->regmap);
		mutex_unlock(&besbev->res_lock);
		besbev->dev_up = true;
		break;
	default:
		dev_err(component->dev, "%s: invalid event %d\n", __func__,
			event);
		break;
	}
	return 0;
}

static int besbev_rx_event_notify(struct notifier_block *block,
				unsigned long val,
				void *data)
{
	u16 event = (val & 0xffff);
	struct besbev_priv *besbev = dev_get_drvdata((struct device *)data);
	struct snd_soc_component *component = besbev->component;

	switch (event) {
	case BOLERO_SLV_EVT_PA_OFF_PRE_SSR:
		if (test_bit(SPKR_STATUS, &besbev->status_mask))
			snd_soc_component_update_bits(besbev->component,
						BESBEV_PA_FSM_CTL,
						0x01, 0x00);
		break;
	case BOLERO_SLV_EVT_SSR_DOWN:
		besbev->dev_up = false;
		besbev_reset(besbev->dev, 0x01);
		break;
	case BOLERO_SLV_EVT_SSR_UP:
		besbev_reset(besbev->dev, 0x00);
		besbev_init_reg(component, true);
		mutex_lock(&besbev->res_lock);
		regcache_mark_dirty(besbev->regmap);
		regcache_sync(besbev->regmap);
		mutex_unlock(&besbev->res_lock);
		besbev->dev_up = true;
		usleep_range(20000, 20010);
		break;
	case BOLERO_SLV_EVT_PA_ON_POST_FSCLK:
		if (test_bit(SPKR_STATUS, &besbev->status_mask)) {
			snd_soc_component_update_bits(besbev->component,
						BESBEV_PDM_WD_CTL,
						0x01, 0x01);
			snd_soc_component_update_bits(besbev->component,
						BESBEV_PA_FSM_CTL,
						0x01, 0x01);
			wcd_enable_irq(&besbev->irq_info,
					BESBEV_IRQ_INT_PDM_WD);
			/* Added delay as per HW sequence */
			usleep_range(3000, 3100);
			if (besbev->comp_support)
				snd_soc_component_update_bits(besbev->component,
						BESBEV_DRE_CTL_1,
						0x01, 0x00);
			/* Added delay as per HW sequence */
			usleep_range(5000, 5050);
		}
		break;
	case BOLERO_SLV_EVT_PA_ON_POST_FSCLK_ADIE_LB:
		if (test_bit(SPKR_STATUS, &besbev->status_mask))
			set_bit(SPKR_ADIE_LB, &besbev->status_mask);
	default:
		dev_err(component->dev, "%s: invalid event %d\n", __func__,
			event);
		break;
	}
	return 0;
}

static int __besbev_codec_enable_micbias(struct snd_soc_dapm_widget *w,
					  int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	int micb_num;

	dev_dbg(component->dev, "%s: wname: %s, event: %d\n",
		__func__, w->name, event);

	if (strnstr(w->name, "MIC BIAS1", sizeof("MIC BIAS1")))
		micb_num = MIC_BIAS_1;
	else if (strnstr(w->name, "MIC BIAS2", sizeof("MIC BIAS2")))
		micb_num = MIC_BIAS_2;
	else
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		besbev_micbias_control(component, micb_num,
			MICB_ENABLE, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(1000, 1100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		besbev_micbias_control(component, micb_num,
			MICB_DISABLE, true);
		break;
	};

	return 0;

}

static int besbev_codec_enable_micbias(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	return __besbev_codec_enable_micbias(w, event);
}

static int __besbev_codec_enable_micbias_pullup(struct snd_soc_dapm_widget *w,
						 int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	int micb_num;

	dev_dbg(component->dev, "%s: wname: %s, event: %d\n",
		__func__, w->name, event);

	if (strnstr(w->name, "VA MIC BIAS1", sizeof("VA MIC BIAS1")))
		micb_num = MIC_BIAS_1;
	else if (strnstr(w->name, "VA MIC BIAS2", sizeof("VA MIC BIAS2")))
		micb_num = MIC_BIAS_2;
	else
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		besbev_micbias_control(component, micb_num,
					MICB_PULLUP_ENABLE, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 1 msec delay as per HW requirement */
		usleep_range(1000, 1100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		besbev_micbias_control(component, micb_num,
					MICB_PULLUP_DISABLE, true);
		break;
	};

	return 0;

}

static int besbev_codec_enable_micbias_pullup(struct snd_soc_dapm_widget *w,
					       struct snd_kcontrol *kcontrol,
					       int event)
{
	return __besbev_codec_enable_micbias_pullup(w, event);
}

static const char * const besbev_dev_mode_text[] = {
	"speaker", "receiver", "ultrasound"
};

static const struct soc_enum besbev_dev_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(besbev_dev_mode_text),
					besbev_dev_mode_text);

static int besbev_dev_mode_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = besbev->dev_mode;

	dev_dbg(component->dev, "%s: mode = 0x%x\n", __func__,
			besbev->dev_mode);

	return 0;
}

static int besbev_dev_mode_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	besbev->dev_mode =  ucontrol->value.integer.value[0];

	return 0;
}

static int besbev_get_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: compander = 0x%x\n", __func__,
			besbev->comp_enable);
	ucontrol->value.integer.value[0] = besbev->comp_enable;
	return 0;
}

static int besbev_set_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: Compander enable current %d, new %d\n",
		 __func__, besbev->comp_enable, value);
	besbev->comp_enable = value;
	return 0;
}

static int besbev_get_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: VIsense = 0x%x\n", __func__,
			besbev->visense_enable);
	ucontrol->value.integer.value[0] = besbev->visense_enable;
	return 0;
}

static int besbev_set_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: VIsense enable current %d, new %d\n",
		 __func__, besbev->visense_enable, value);
	besbev->visense_enable = value;
	return 0;
}

static const char * const besbev_pa_gain_text[] = {
	"G_18_DB", "G_16P5_DB", "G_15_DB", "G_13P5_DB", "G_12_DB", "G_10P5_DB",
	"G_9_DB", "G_7P5_DB", "G_6_DB", "G_4P5_DB", "G_3_DB", "G_1P5_DB",
	"G_0_DB"
};

static const struct soc_enum besbev_pa_gain_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(besbev_pa_gain_text),
					besbev_pa_gain_text);

static int besbev_pa_gain_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = besbev->pa_gain;

	dev_dbg(component->dev, "%s: PA gain = 0x%x\n", __func__,
			besbev->pa_gain);

	return 0;
}

static int besbev_pa_gain_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	besbev->pa_gain =  ucontrol->value.integer.value[0];

	return 0;
}

static int besbev_get_mute(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: pa_mute = 0x%x\n", __func__,
			besbev->pa_mute);
	ucontrol->value.integer.value[0] = besbev->pa_mute;

	return 0;
}

static int besbev_set_mute(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: mute current %d, new %d\n",
		__func__, besbev->pa_mute, value);

	besbev->pa_mute = value;

	return 0;
}

static int besbev_get_temp(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	int temp = 0;

	if (test_bit(SPKR_STATUS, &besbev->status_mask))
		temp = besbev->curr_temp;
	else
		besbev_get_temperature(component, &temp);

	dev_dbg(component->dev, "%s: temp = 0x%x\n", __func__, temp);
	ucontrol->value.integer.value[0] = temp;

	return 0;
}

static int besbev_get_ext_vdd_spk(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: Ext VDD SPK = 0x%x\n", __func__,
			besbev->ext_vdd_spk);
	ucontrol->value.integer.value[0] = besbev->ext_vdd_spk;

	return 0;
}

static int besbev_put_ext_vdd_spk(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: Ext VDD SPK enable current %d, new %d\n",
		 __func__, besbev->ext_vdd_spk, value);
	besbev->ext_vdd_spk = value;

	return 0;
}

static const struct snd_kcontrol_new besbev_snd_controls_tx[] = {
	SOC_SINGLE_TLV("ADC1 Volume", BESBEV_ANA_TX_AMIC1, 0, 8, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", BESBEV_ANA_TX_AMIC2, 0, 8, 0,
			analog_gain),
};

static const struct snd_kcontrol_new besbev_snd_controls_rx[] = {
	SOC_SINGLE_TLV("ADC1 Volume", BESBEV_ANA_TX_AMIC1, 0, 8, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", BESBEV_ANA_TX_AMIC2, 0, 8, 0,
			analog_gain),
	SOC_ENUM_EXT("BESBEV PA Gain", besbev_pa_gain_enum,
		     besbev_pa_gain_get, besbev_pa_gain_put),

	SOC_SINGLE_EXT("BESBEV PA Mute", SND_SOC_NOPM, 0, 1, 0,
		besbev_get_mute, besbev_set_mute),

	SOC_SINGLE_EXT("BESBEV Temp", SND_SOC_NOPM, 0, UINT_MAX, 0,
			besbev_get_temp, NULL),

	SOC_ENUM_EXT("BESBEV MODE", besbev_dev_mode_enum,
		     besbev_dev_mode_get, besbev_dev_mode_put),

	SOC_SINGLE_EXT("COMP Switch", SND_SOC_NOPM, 0, 1, 0,
		besbev_get_compander, besbev_set_compander),

	SOC_SINGLE_EXT("VISENSE Switch", SND_SOC_NOPM, 0, 1, 0,
		besbev_get_visense, besbev_set_visense),

	SOC_SINGLE_EXT("External VDD_SPK", SND_SOC_NOPM, 0, 1, 0,
		besbev_get_ext_vdd_spk, besbev_put_ext_vdd_spk),
};

static const struct snd_kcontrol_new adc1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new adc2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new swr_dac_port[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static int besbev_enable_swr_dac_port(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	u8 port_id[BESBEV_MAX_SWR_PORTS];
	u8 num_ch[BESBEV_MAX_SWR_PORTS];
	u8 ch_mask[BESBEV_MAX_SWR_PORTS];
	u32 ch_rate[BESBEV_MAX_SWR_PORTS];
	u8 port_type[BESBEV_MAX_SWR_PORTS];
	u8 num_port = 0;

	dev_dbg(component->dev, "%s: event %d name %s\n", __func__,
		event, w->name);
	if (besbev == NULL)
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		besbev_set_port_params(component, SPKR_L,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port],
				&port_type[num_port], CODEC_RX);
		++num_port;

		if (besbev->comp_enable) {
			besbev_set_port_params(component, SWR_COMP_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port], CODEC_RX);
			++num_port;
		}
		if (besbev->visense_enable) {
			besbev_set_port_params(component, SPKR_L_VI,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port], CODEC_RX);
			++num_port;
		}
		swr_connect_port(besbev->swr_dev, &port_id[0], num_port,
				&ch_mask[0], &ch_rate[0], &num_ch[0],
					&port_type[0]);
		break;
	case SND_SOC_DAPM_POST_PMU:
		set_bit(SPKR_STATUS, &besbev->status_mask);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		besbev_set_port_params(component, SPKR_L,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port],
				&port_type[num_port], CODEC_RX);
		++num_port;

		if (besbev->comp_enable) {
			besbev_set_port_params(component, SWR_COMP_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port], CODEC_RX);
			++num_port;
		}
		if (besbev->visense_enable) {
			besbev_set_port_params(component, SPKR_L_VI,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port], CODEC_RX);
			++num_port;
		}
		swr_disconnect_port(besbev->swr_dev, &port_id[0], num_port,
				&ch_mask[0], &port_type[0]);
		besbev_global_mbias_disable(component);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (swr_set_device_group(besbev->swr_dev, SWR_GROUP_NONE))
			dev_err(component->dev,
				"%s: set num ch failed\n", __func__);

		swr_slvdev_datapath_control(besbev->swr_dev,
					    besbev->swr_dev->dev_num,
					    false);
		break;
	default:
		break;
	}
	return 0;
}

static int besbev_spkr_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	struct besbev_pdata *pdata = NULL;
	int ret = 0;

	pdata = dev_get_platdata(besbev->dev);

	dev_dbg(component->dev, "%s: %s %d\n", __func__, w->name, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (test_bit(BESBEV_SPKR_BOOST_ENABLE, &besbev->status_mask)) {
			dev_dbg(component->dev,
				"%s: vdd spkr is already in enabled state\n",
				__func__);
		} else {
			ret = msm_cdc_enable_ondemand_supply(besbev->dev,
						besbev->supplies,
						pdata->regulator,
						pdata->num_supplies,
						"cdc-vdd-spkr");
			if (ret == -EINVAL) {
				dev_err(component->dev, "%s: vdd spkr is not enabled\n",
					__func__);
				return ret;
			}
			set_bit(BESBEV_SPKR_BOOST_ENABLE, &besbev->status_mask);
		}
		swr_slvdev_datapath_control(besbev->swr_dev,
					    besbev->swr_dev->dev_num,
					    true);
		besbev_global_mbias_enable(component);
		/* Set Gain from SWR */
		if (besbev->comp_support)
			snd_soc_component_update_bits(component,
						BESBEV_DRE_CTL_1,
						0x01, 0x00);
		snd_soc_component_update_bits(component, BESBEV_PDM_WD_CTL,
						0x01, 0x01);
		/* Force remove group */
		swr_remove_from_group(besbev->swr_dev,
				      besbev->swr_dev->dev_num);
		/* VBat adc filter control */
		snd_soc_component_update_bits(component,
					BESBEV_VBAT_ADC_FLT_CTL, 0x0E, 0x06);
		snd_soc_component_update_bits(component,
					BESBEV_VBAT_ADC_FLT_CTL, 0x01, 0x01);

		snd_soc_component_update_bits(component,
				BESBEV_PA_FSM_CTL, 0x01, 0x01);
		if (besbev->update_wcd_event)
			besbev->update_wcd_event(besbev->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX3 << 0x10));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (besbev->update_wcd_event)
			besbev->update_wcd_event(besbev->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX3 << 0x10 | 0x1));
		if (!test_bit(SPKR_ADIE_LB, &besbev->status_mask))
			wcd_disable_irq(&besbev->irq_info,
					BESBEV_IRQ_INT_PDM_WD);
		snd_soc_component_update_bits(component,
					BESBEV_VBAT_ADC_FLT_CTL, 0x01, 0x00);
		snd_soc_component_update_bits(component,
					BESBEV_VBAT_ADC_FLT_CTL, 0x0E, 0x00);
		snd_soc_component_update_bits(component, BESBEV_PA_FSM_CTL,
						0x01, 0x00);

		/* As part of usecase stop to disable the PA we update
		 * BESBEV_PA_FSM_CTL to 0x00 which triggers the Global PA_EN to 0
		 * It takes some time to completely disable the PA.
		 * During this PA disabling time if we suddenly cut-off/disable the
		 * speaker VDD it will result in POP sound.
		 * Adding sleep for PA to get disabled and then disable supply.
		 */
		msleep(10);

		/* PDM watchdog control disable*/
		snd_soc_component_update_bits(component, BESBEV_PDM_WD_CTL,
						0x01, 0x00);
		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_UVLO);

		if (test_bit(BESBEV_SPKR_BOOST_ENABLE, &besbev->status_mask)) {
			ret = msm_cdc_disable_ondemand_supply(besbev->dev,
						besbev->supplies,
						pdata->regulator,
						pdata->num_supplies,
						"cdc-vdd-spkr");
			if (ret == -EINVAL) {
				dev_err(component->dev, "%s: vdd spkr is not enabled\n",
					__func__);
			}
			clear_bit(BESBEV_SPKR_BOOST_ENABLE, &besbev->status_mask);
		}

		clear_bit(SPKR_STATUS, &besbev->status_mask);
		clear_bit(SPKR_ADIE_LB, &besbev->status_mask);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget besbev_dapm_widgets_tx[] = {

	/*input widgets*/
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),

	/*tx widgets*/
	SND_SOC_DAPM_ADC_E("ADC1", NULL, SND_SOC_NOPM, 0, 0,
				besbev_swr_ctrl,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, SND_SOC_NOPM, 0, 0,
				besbev_swr_ctrl,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/*tx mixers*/
	SND_SOC_DAPM_MIXER_E("ADC1_MIXER", SND_SOC_NOPM, 0, 0,
				adc1_switch, ARRAY_SIZE(adc1_switch),
				besbev_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC2_MIXER", SND_SOC_NOPM, 1, 0,
				adc2_switch, ARRAY_SIZE(adc2_switch),
				besbev_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),

	/* micbias widgets*/
	SND_SOC_DAPM_SUPPLY("MIC BIAS1", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

	/*output widgets tx*/

	SND_SOC_DAPM_OUTPUT("ADC1_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("ADC2_OUTPUT"),

	/* micbias pull up widgets*/
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS1", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS2", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

};

static const struct snd_soc_dapm_widget besbev_dapm_widgets_rx[] = {

	/*input widgets*/
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),

	/*tx widgets*/
	SND_SOC_DAPM_ADC_E("ADC1", NULL, SND_SOC_NOPM, 0, 0,
				besbev_swr_ctrl,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, SND_SOC_NOPM, 0, 0,
				besbev_swr_ctrl,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/*tx mixers*/
	SND_SOC_DAPM_MIXER_E("ADC1_MIXER", SND_SOC_NOPM, 0, 0,
				adc1_switch, ARRAY_SIZE(adc1_switch),
				besbev_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC2_MIXER", SND_SOC_NOPM, 1, 0,
				adc2_switch, ARRAY_SIZE(adc2_switch),
				besbev_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),

	/* micbias widgets*/
	SND_SOC_DAPM_SUPPLY("MIC BIAS1", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

	/*rx widgets*/
	SND_SOC_DAPM_INPUT("SPKR_IN"),
	SND_SOC_DAPM_MIXER_E("SWR DAC_Port", SND_SOC_NOPM, 0, 0, swr_dac_port,
		ARRAY_SIZE(swr_dac_port), besbev_enable_swr_dac_port,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* rx mixer widgets*/
	SND_SOC_DAPM_SPK("SPKR", besbev_spkr_event),
	/*output widgets tx*/

	SND_SOC_DAPM_OUTPUT("ADC1_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("ADC2_OUTPUT"),

	/* micbias pull up widgets*/
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS1", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS2", SND_SOC_NOPM, 0, 0,
				besbev_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

};

static const struct snd_soc_dapm_route besbev_audio_map_tx[] = {
	{"ADC1_OUTPUT", NULL, "ADC1_MIXER"},
	{"ADC1_MIXER", "Switch", "ADC1"},
	{"ADC1", NULL, "AMIC1"},

	{"ADC2_OUTPUT", NULL, "ADC2_MIXER"},
	{"ADC2_MIXER", "Switch", "ADC2"},
	{"ADC2", NULL, "AMIC2"},
};

static const struct snd_soc_dapm_route besbev_audio_map_rx[] = {
	{"ADC1_OUTPUT", NULL, "ADC1_MIXER"},
	{"ADC1_MIXER", "Switch", "ADC1"},
	{"ADC1", NULL, "AMIC1"},

	{"ADC2_OUTPUT", NULL, "ADC2_MIXER"},
	{"ADC2_MIXER", "Switch", "ADC2"},
	{"ADC2", NULL, "AMIC2"},

	{"SWR DAC_Port", "Switch", "SPKR_IN"},
	{"SPKR", NULL, "SWR DAC_Port"},
};

static ssize_t besbev_version_read(struct snd_info_entry *entry,
				   void *file_private_data,
				   struct file *file,
				   char __user *buf, size_t count,
				   loff_t pos)
{
	struct besbev_priv *priv;
	char buffer[BESBEV_VERSION_ENTRY_SIZE];
	int len = 0;

	priv = (struct besbev_priv *) entry->private_data;
	if (!priv) {
		pr_err("%s: besbev priv is null\n", __func__);
		return -EINVAL;
	}

	switch (priv->version) {
	case BESBEV_VERSION_1_0:
		len = snprintf(buffer, sizeof(buffer), "BESBEV_1_0\n");
		break;
	default:
		len = snprintf(buffer, sizeof(buffer), "VER_UNDEFINED\n");
	}

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops besbev_info_ops = {
	.read = besbev_version_read,
};

static irqreturn_t besbev_saf2war_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_war2saf_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_otp_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_ocp_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_clip_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_pdm_wd_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_clk_wd_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_ext_int_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_uvlo_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t besbev_pa_on_err_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

/*
 * besbev_info_create_codec_entry - creates besbev module
 * @codec_root: The parent directory
 * @component: component instance
 *
 * Creates besbev module and version entry under the given
 * parent directory.
 *
 * Return: 0 on success or negative error code on failure.
 */
int besbev_info_create_codec_entry(struct snd_info_entry *codec_root,
				   struct snd_soc_component *component)
{
	struct snd_info_entry *version_entry;
	struct besbev_priv *priv;
	struct snd_soc_card *card;

	if (!codec_root || !component)
		return -EINVAL;

	priv = snd_soc_component_get_drvdata(component);
	if (priv->entry) {
		dev_dbg(priv->dev,
			"%s:besbev module already created\n", __func__);
		return 0;
	}
	card = component->card;
	priv->entry = snd_info_create_module_entry(codec_root->module,
					     "besbev", codec_root);
	if (!priv->entry) {
		dev_dbg(component->dev, "%s: failed to create besbev entry\n",
			__func__);
		return -ENOMEM;
	}
	version_entry = snd_info_create_card_entry(card->snd_card,
						   "version",
						   priv->entry);
	if (!version_entry) {
		dev_dbg(component->dev, "%s: failed to create besbev version entry\n",
			__func__);
		return -ENOMEM;
	}

	version_entry->private_data = priv;
	version_entry->size = BESBEV_VERSION_ENTRY_SIZE;
	version_entry->content = SNDRV_INFO_CONTENT_DATA;
	version_entry->c.ops = &besbev_info_ops;

	if (snd_info_register(version_entry) < 0) {
		snd_info_free_entry(version_entry);
		return -ENOMEM;
	}
	priv->version_entry = version_entry;

	return 0;
}
EXPORT_SYMBOL(besbev_info_create_codec_entry);

static int besbev_set_micbias_data(struct besbev_priv *besbev,
			      struct besbev_pdata *pdata)
{
	int vout_ctl = 0;
	int rc = 0;

	if (!pdata) {
		dev_err(besbev->dev, "%s: NULL pdata\n", __func__);
		return -ENODEV;
	}

	/* set micbias voltage */
	vout_ctl = besbev_get_micb_vout_ctl_val(pdata->micbias.micb1_mv);
	if (vout_ctl < 0) {
		rc = -EINVAL;
		goto done;
	}
	regmap_update_bits(besbev->regmap, BESBEV_ANA_MICBIAS_LDO_1_SETTING,
			   0xF8, vout_ctl << 3);

done:
	return rc;
}

/*
 * besbev_amic_init - amic init for amic variant
 * @component: component instance
 *
 * Return: 0 on success or negative error code on failure.
 */
int besbev_amic_init(struct snd_soc_component *component)
{
	int ret = 0;
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	struct besbev_pdata *pdata = NULL;

	if (!besbev || besbev->visense_support) {
		dev_err(component->dev, "%s: besbev is NULL or VI sense is"
			" supported,so avoid AMIC init \n", __func__);
		return -EINVAL;
	}

	pdata = dev_get_platdata(besbev->dev);

	/* Disable VISense for AMIC variant */
	snd_soc_component_update_bits(component,
				BESBEV_DIG_SWR_CDC_RX_MODE, 0x4, 0x4);

	mutex_init(&besbev->micb_lock);
	mutex_init(&besbev->main_bias_lock);

	ret = besbev_set_micbias_data(besbev, pdata);
	if (ret < 0) {
		dev_err(besbev->dev, "%s: bad micbias pdata\n", __func__);
		goto err_irq;
	}

	return 0;

err_irq:
	mutex_destroy(&besbev->micb_lock);
	mutex_destroy(&besbev->main_bias_lock);

	return ret;
}
EXPORT_SYMBOL(besbev_amic_init);

static int besbev_battery_supply_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct besbev_priv *besbev =
		container_of(nb, struct besbev_priv, psy_nb);

	if (strcmp(psy->desc->name, "battery"))
		return NOTIFY_OK;
	queue_work(system_freezable_wq, &besbev->soc_eval_work);

	return NOTIFY_OK;
}

static int besbev_read_battery_soc(struct besbev_priv *besbev, int *soc_val)
{
	static struct power_supply *batt_psy;
	union power_supply_propval ret = {0,};
	int err = 0;

	*soc_val = 100;
	if (!batt_psy)
		batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		err = power_supply_get_property(batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (err) {
			pr_err("%s: battery SoC read error:%d\n",
				__func__, err);
			return err;
		}
		*soc_val = ret.intval;
	}

	return err;
}

static void besbev_evaluate_soc(struct work_struct *work)
{
	struct besbev_priv *besbev =
		container_of(work, struct besbev_priv, soc_eval_work);
	int soc_val = 0, ret = 0;
	struct besbev_pdata *pdata = NULL;

	pdata = dev_get_platdata(besbev->dev);
	if (!pdata) {
		dev_err(besbev->dev, "%s: pdata is NULL\n", __func__);
		return;
	}

	if (besbev_read_battery_soc(besbev, &soc_val) < 0) {
		dev_err(besbev->dev, "%s: unable to read battery SoC\n",
			__func__);
		return;
	}

	if (soc_val < SOC_THRESHOLD_LEVEL) {
		dev_dbg(besbev->dev,
			"%s: battery SoC less than threshold soc_val = %d\n",
			__func__, soc_val);
		/* Reduce PA Gain by 6DB for low SoC */
		if (besbev->update_wcd_event)
			besbev->update_wcd_event(besbev->handle,
					SLV_BOLERO_EVT_RX_PA_GAIN_UPDATE,
					true);
		besbev->low_soc = true;
		ret = msm_cdc_set_supply_min_voltage(besbev->dev,
						 besbev->supplies,
						 pdata->regulator,
						 pdata->num_supplies,
						 "cdc-vdd-mic-bias",
						 LOW_SOC_MBIAS_REG_MIN_VOLTAGE,
						 true);
		if (ret < 0)
			dev_err(besbev->dev,
				"%s unable to set mbias min voltage\n",
				__func__);
	} else {
		if (besbev->low_soc == true) {
			/* Reset PA Gain to default for normal SoC */
			if (besbev->update_wcd_event)
				besbev->update_wcd_event(besbev->handle,
					SLV_BOLERO_EVT_RX_PA_GAIN_UPDATE,
					false);
			ret = msm_cdc_set_supply_min_voltage(besbev->dev,
						besbev->supplies,
						pdata->regulator,
						pdata->num_supplies,
						"cdc-vdd-mic-bias",
						LOW_SOC_MBIAS_REG_MIN_VOLTAGE,
						false);
			if (ret < 0)
				dev_err(besbev->dev,
					"%s unable to set mbias min voltage\n",
					__func__);
			besbev->low_soc = false;
		}
	}
}

static void besbev_get_foundry_id(struct besbev_priv *besbev)
{
	int ret = 0;

	if (besbev->foundry_id_reg == 0) {
		pr_debug("%s: foundry id not defined\n", __func__);
		return;
	}

	ret = pmw5100_spmi_read(besbev->spmi_dev, besbev->foundry_id_reg,
				&besbev->foundry_id);
	if (ret == 0)
		pr_debug("%s: besbev foundry id = %x\n", __func__,
			besbev->foundry_id);
	else
		pr_debug("%s: besbev error spmi read ret = %d\n",
			 __func__, ret);
}

static int32_t besbev_temp_reg_read(struct snd_soc_component *component,
				     struct besbev_temp_register *besbev_temp_reg)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	if (!besbev) {
		dev_err(component->dev, "%s: besbev is NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&besbev->res_lock);
	besbev_global_mbias_enable(component);

	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0x01, 0x01);
	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0x04, 0x04);
	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0x02, 0x02);
	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0x80, 0x80);
	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0x20, 0x20);
	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0x40, 0x40);
	/**
	 * Disable read continous temp value
	 */
	snd_soc_component_update_bits(component, BESBEV_TADC_VALUE_CTL,
				0x01, LOCK_CONTIN_TEMP_READ);

	besbev_temp_reg->dmeas_msb = snd_soc_component_read(
					component, BESBEV_TEMP_MSB);
	besbev_temp_reg->dmeas_lsb = snd_soc_component_read(
					component, BESBEV_TEMP_LSB);
	 /**
      * Enable read continous temp value
      */
	snd_soc_component_update_bits(component, BESBEV_TADC_VALUE_CTL,
				0x01, UNLOCK_CONTIN_TEMP_READ);

	besbev_temp_reg->d1_msb = snd_soc_component_read(
					component, BESBEV_PA_OTP_LOW_M);
	besbev_temp_reg->d1_lsb = snd_soc_component_read(
					component, BESBEV_PA_OTP_LOW_L);
	besbev_temp_reg->d2_msb = snd_soc_component_read(
					component, BESBEV_PA_OTP_HIGH_M);
	besbev_temp_reg->d2_lsb = snd_soc_component_read(
					component, BESBEV_PA_OTP_HIGH_L);

	snd_soc_component_update_bits(component, BESBEV_PA_FSM_BYP,
				0xE7, 0x00);
	besbev_global_mbias_disable(component);
	mutex_unlock(&besbev->res_lock);

	return 0;
}

static int besbev_get_temperature(struct snd_soc_component *component,
				   int *temp)
{
	struct besbev_temp_register reg;
	int dmeas, d1, d2;
	int ret = 0;
	int temp_val = 0;
	int t1 = T1_TEMP;
	int t2 = T2_TEMP;
	u8 retry = BESBEV_TEMP_RETRY;
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	if (!besbev)
		return -EINVAL;

	do {
		ret = besbev_temp_reg_read(component, &reg);
		if (ret) {
			pr_err("%s: temp read failed: %d, current temp: %d\n",
				__func__, ret, besbev->curr_temp);
			if (temp)
				*temp = besbev->curr_temp;
			return 0;
		}
		/*
		 * Temperature register values are expected to be in the
		 * following range.
		 * d1_msb  = 68 - 92 and d1_lsb  = 0, 64, 128, 192
		 * d2_msb  = 185 -218 and  d2_lsb  = 0, 64, 128, 192
		 */
		if ((reg.d1_msb < 68 || reg.d1_msb > 92) ||
		    (!(reg.d1_lsb == 0 || reg.d1_lsb == 64 || reg.d1_lsb == 128 ||
			reg.d1_lsb == 192)) ||
		    (reg.d2_msb < 185 || reg.d2_msb > 218) ||
		    (!(reg.d2_lsb == 0 || reg.d2_lsb == 64 || reg.d2_lsb == 128 ||
			reg.d2_lsb == 192))) {
			printk_ratelimited("%s: Temperature registers[%d %d %d %d]"
				"are out of range\n", __func__, reg.d1_msb,
				reg.d1_lsb, reg.d2_msb, reg.d2_lsb);
		}

		dmeas = ((reg.dmeas_msb << 0x8) | reg.dmeas_lsb) >> 0x6;
		d1 = ((reg.d1_msb << 0x8) | reg.d1_lsb) >> 0x6;
		d2 = ((reg.d2_msb << 0x8) | reg.d2_lsb) >> 0x6;

		if (d1 == d2)
			temp_val = TEMP_INVALID;
		else
			temp_val = t1 + (((dmeas - d1) * (t2 - t1))/(d2 - d1));

		if (temp_val <= LOW_TEMP_THRESHOLD ||
			temp_val >= HIGH_TEMP_THRESHOLD) {
			pr_debug("%s: T0: %d is out of range[%d, %d]\n", __func__,
				 temp_val, LOW_TEMP_THRESHOLD, HIGH_TEMP_THRESHOLD);
			if (retry--)
				msleep(10);
		} else {
			break;
		}
	} while (retry);

	besbev->curr_temp = temp_val;
	if (temp)
		*temp = temp_val;
	pr_debug("%s: t0 measured: %d dmeas = %d, d1 = %d, d2 = %d\n",
		  __func__, temp_val, dmeas, d1, d2);

	return ret;
}

static int besbev_soc_codec_probe(struct snd_soc_component *component)
{
	char w_name[MAX_NAME_LEN];
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);
	int ret = -EINVAL, version = 0;

	dev_info(component->dev, "%s()\n", __func__);
	besbev = snd_soc_component_get_drvdata(component);

	if (!besbev)
		return -EINVAL;

	besbev->component = component;
	snd_soc_component_init_regmap(component, besbev->regmap);

	besbev->fw_data = devm_kzalloc(component->dev,
					sizeof(*(besbev->fw_data)),
					GFP_KERNEL);
	if (!besbev->fw_data) {
		dev_err(component->dev, "%s: Failed to allocate fw_data\n",
					__func__);
		ret = -ENOMEM;
		goto done;
	}

	ret = wcd_cal_create_hwdep(besbev->fw_data,
				   WCD9XXX_CODEC_HWDEP_NODE, component);

	if (ret < 0) {
		dev_err(component->dev, "%s hwdep failed %d\n", __func__, ret);
		goto done;
	}

	snd_soc_dapm_ignore_suspend(dapm, "BESBEV_AIF Capture");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "ADC1_OUTPUT");
	snd_soc_dapm_ignore_suspend(dapm, "ADC2_OUTPUT");

	besbev_init_reg(component, besbev->speaker_present);

	/* Get besbev foundry id */
	besbev_get_foundry_id(besbev);

	version = (snd_soc_component_read(component, BESBEV_DIG_SWR_CHIP_ID0)
					    & 0xFF);
	besbev->version = version;

	if (besbev->speaker_present == true) {
		snd_soc_dapm_ignore_suspend(dapm, "BESBEV_AIF Playback");

		memset(w_name, 0, sizeof(w_name));
		strlcpy(w_name, "SPKR_IN", sizeof(w_name));
		snd_soc_dapm_ignore_suspend(dapm, w_name);

		memset(w_name, 0, sizeof(w_name));
		strlcpy(w_name, "SWR DAC_PORT", sizeof(w_name));
		snd_soc_dapm_ignore_suspend(dapm, w_name);

		memset(w_name, 0, sizeof(w_name));
		strlcpy(w_name, "SPKR", sizeof(w_name));
		snd_soc_dapm_ignore_suspend(dapm, w_name);

		/* Register event notifier */
		besbev->nblock.notifier_call = besbev_rx_event_notify;
		if (besbev->register_notifier) {
			ret = besbev->register_notifier(besbev->handle,
						&besbev->nblock,
						true);
			if (ret) {
				dev_err(component->dev,
					"%s: Failed to register notifier %d\n",
					__func__, ret);
				return ret;
			}
		}
	} else {
		/* Register event notifier */
		besbev->nblock.notifier_call = besbev_tx_event_notify;
		if (besbev->register_notifier) {
			ret = besbev->register_notifier(besbev->handle,
						&besbev->nblock,
						true);
			if (ret) {
				dev_err(component->dev,
					"%s: Failed to register notifier %d\n",
					__func__, ret);
				return ret;
			}
		}
	}
	snd_soc_dapm_sync(dapm);

	besbev->low_soc = false;
	besbev->dev_up = true;
	/* Register notifier to change gain based on state of charge */
	INIT_WORK(&besbev->soc_eval_work, besbev_evaluate_soc);
	besbev->psy_nb.notifier_call = besbev_battery_supply_cb;
	if (power_supply_reg_notifier(&besbev->psy_nb) < 0)
		dev_dbg(besbev->dev,
			"%s: could not register pwr supply notifier\n",
			__func__);
	queue_work(system_freezable_wq, &besbev->soc_eval_work);
done:
	return ret;
}

static void besbev_soc_codec_remove(struct snd_soc_component *component)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	if (!besbev)
		return;

	if (besbev->register_notifier)
		besbev->register_notifier(besbev->handle,
						&besbev->nblock,
						false);
}

static int besbev_soc_codec_suspend(struct snd_soc_component *component)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	if (!besbev)
		return 0;
	besbev->dapm_bias_off = true;
	return 0;
}

static int besbev_soc_codec_resume(struct snd_soc_component *component)
{
	struct besbev_priv *besbev = snd_soc_component_get_drvdata(component);

	if (!besbev)
		return 0;
	besbev->dapm_bias_off = false;
	return 0;
}

static struct snd_soc_component_driver soc_codec_dev_besbev = {
	.name = BESBEV_DRV_NAME,
	.probe = besbev_soc_codec_probe,
	.remove = besbev_soc_codec_remove,
	.suspend = besbev_soc_codec_suspend,
	.resume = besbev_soc_codec_resume,
};

#ifdef CONFIG_PM_SLEEP
static int besbev_suspend(struct device *dev)
{
	struct besbev_priv *besbev = NULL;
	struct besbev_pdata *pdata = NULL;

	if (!dev)
		return -ENODEV;

	besbev = dev_get_drvdata(dev);
	if (!besbev)
		return -EINVAL;

	pdata = dev_get_platdata(besbev->dev);

	if (!pdata) {
		dev_err(dev, "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	if (besbev->dapm_bias_off) {
		 msm_cdc_set_supplies_lpm_mode(besbev->dev,
					      besbev->supplies,
					      pdata->regulator,
					      pdata->num_supplies,
					      true);
		set_bit(WCD_SUPPLIES_LPM_MODE, &besbev->status_mask);
	}
	return 0;
}

static int besbev_resume(struct device *dev)
{
	struct besbev_priv *besbev = NULL;
	struct besbev_pdata *pdata = NULL;

	if (!dev)
		return -ENODEV;

	besbev = dev_get_drvdata(dev);
	if (!besbev)
		return -EINVAL;

	pdata = dev_get_platdata(besbev->dev);

	if (!pdata) {
		dev_err(dev, "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	if (test_bit(WCD_SUPPLIES_LPM_MODE, &besbev->status_mask)) {
		msm_cdc_set_supplies_lpm_mode(besbev->dev,
						besbev->supplies,
						pdata->regulator,
						pdata->num_supplies,
						false);
		clear_bit(WCD_SUPPLIES_LPM_MODE, &besbev->status_mask);
	}

	return 0;
}
#endif

static int besbev_reset(struct device *dev, int reset_val)
{
	struct besbev_priv *besbev = NULL;

	if (!dev)
		return -ENODEV;

	besbev = dev_get_drvdata(dev);
	if (!besbev)
		return -EINVAL;

	pmw5100_spmi_write(besbev->spmi_dev, besbev->reset_reg, reset_val);

	return 0;
}

static int besbev_read_of_property_u32(struct device *dev, const char *name,
					u32 *val)
{
	int rc = 0;

	rc = of_property_read_u32(dev->of_node, name, val);
	if (rc)
		dev_err(dev, "%s: Looking up %s property in node %s failed\n",
			__func__, name, dev->of_node->full_name);

	return rc;
}

static void besbev_dt_parse_micbias_info(struct device *dev,
					  struct besbev_micbias_setting *mb)
{
	u32 prop_val = 0;
	int rc = 0;

	/* MB1 */
	if (of_find_property(dev->of_node, "qcom,cdc-micbias1-mv",
				    NULL)) {
		rc = besbev_read_of_property_u32(dev,
						  "qcom,cdc-micbias1-mv",
						  &prop_val);
		if (!rc)
			mb->micb1_mv = prop_val;
	} else {
		dev_info(dev, "%s: Micbias1 DT property not found\n",
			__func__);
	}

	/* MB2 */
	if (of_find_property(dev->of_node, "qcom,cdc-micbias2-mv",
				    NULL)) {
		rc = besbev_read_of_property_u32(dev,
						  "qcom,cdc-micbias2-mv",
						  &prop_val);
		if (!rc)
			mb->micb2_mv = prop_val;
	} else {
		dev_info(dev, "%s: Micbias2 DT property not found\n",
			__func__);
	}

}

struct besbev_pdata *besbev_populate_dt_data(struct device *dev)
{
	struct besbev_pdata *pdata = NULL;
	u32 reg;
	int ret = 0;

	pdata = kzalloc(sizeof(struct besbev_pdata),
				GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->spmi_np = of_parse_phandle(dev->of_node,
					"qcom,pmic-spmi-node", 0);
	if (!pdata->spmi_np) {
		dev_err(dev, "%s: Looking up %s property in node %s failed\n",
				__func__, "qcom,pmic-spmi-node",
				dev->of_node->full_name);
		kfree(pdata);
		return NULL;
	}

	ret = of_property_read_u32(dev->of_node, "qcom,wcd-reset-reg", &reg);
	if (ret) {
		dev_err(dev, "%s: Failed to obtain reset reg value %d\n",
			__func__, ret);
		kfree(pdata);
		return NULL;
	}
	pdata->reset_reg = reg;

	if (of_property_read_u32(dev->of_node, "qcom,foundry-id-reg", &reg))
		dev_dbg(dev, "%s: Failed to obtain foundry id\n",
			__func__);
	else
		pdata->foundry_id_reg = reg;

	/* Parse power supplies */
	msm_cdc_get_power_supplies(dev, &pdata->regulator,
				   &pdata->num_supplies);
	if (!pdata->regulator || (pdata->num_supplies <= 0)) {
		dev_err(dev, "%s: no power supplies defined for codec\n",
			__func__);
		kfree(pdata);
		return NULL;
	}

	pdata->besbev_slave = of_parse_phandle(dev->of_node,
						"qcom,besbev-slave", 0);

	/*
	 * speaker_present is a flag to differentiate weather
	 * besbev is connected to bolero RX SWR or TX SWR.
	 */
	pdata->speaker_present = of_property_read_bool(dev->of_node,
					"qcom,speaker-present");

	besbev_dt_parse_micbias_info(dev, &pdata->micbias);

	return pdata;
}

static int besbev_wakeup(void *handle, bool enable)
{
	struct besbev_priv *priv;

	if (!handle) {
		pr_err("%s: NULL handle\n", __func__);
		return -EINVAL;
	}
	priv = (struct besbev_priv *)handle;
	if (!priv->swr_dev) {
		pr_err("%s: swr dev is NULL\n", __func__);
		return -EINVAL;
	}
	if (enable)
		return swr_device_wakeup_vote(priv->swr_dev);
	else
		return swr_device_wakeup_unvote(priv->swr_dev);
}

static struct snd_soc_dai_driver besbev_dai[] = {
	{
		.name = "besbev_tx",
		.capture = {
			.stream_name = "BESBEV_AIF Capture",
			.rates = BESBEV_RATES | BESBEV_FRAC_RATES,
			.formats = BESBEV_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
	},
	{
		.name = "besbev_rx",
		.playback = {
			.stream_name = "BESBEV_AIF Playback",
			.rates = BESBEV_RATES | BESBEV_FRAC_RATES,
			.formats = BESBEV_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
	},
};

static int besbev_bind(struct device *dev)
{
	int ret = 0, i = 0, comp_support = 0, val = 0;
	struct besbev_priv *besbev = NULL;
	struct besbev_pdata *pdata = NULL;
	struct wcd_ctrl_platform_data *plat_data = NULL;
	struct platform_device *pdev = NULL;

	besbev = kzalloc(sizeof(struct besbev_priv), GFP_KERNEL);
	if (!besbev)
		return -ENOMEM;

	dev_set_drvdata(dev, besbev);

	pdata = besbev_populate_dt_data(dev);
	if (!pdata) {
		dev_err(dev, "%s: Fail to obtain platform data\n", __func__);
		kfree(besbev);
		return -EINVAL;
	}
	besbev->dev = dev;
	besbev->dev->platform_data = pdata;
	pdev = of_find_device_by_node(pdata->spmi_np);
	if (!pdev) {
		dev_err(dev, "%s: platform device from SPMI node is NULL\n",
				__func__);
		ret = -EINVAL;
		goto err_bind_all;
	}

	besbev->spmi_dev = &pdev->dev;
	besbev->reset_reg = pdata->reset_reg;
	besbev->foundry_id_reg = pdata->foundry_id_reg;
	besbev->speaker_present = pdata->speaker_present;
	ret = msm_cdc_init_supplies(dev, &besbev->supplies,
				    pdata->regulator, pdata->num_supplies);
	if (!besbev->supplies) {
		dev_err(dev, "%s: Cannot init wcd supplies\n",
			__func__);
		goto err_bind_all;
	}

	plat_data = dev_get_platdata(dev->parent);
	if (!plat_data) {
		dev_err(dev, "%s: platform data from parent is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_bind_all;
	}
	besbev->handle = (void *)plat_data->handle;
	if (!besbev->handle) {
		dev_err(dev, "%s: handle is NULL\n", __func__);
		ret = -EINVAL;
		goto err_bind_all;
	}
	besbev->update_wcd_event = plat_data->update_wcd_event;
	if (!besbev->update_wcd_event) {
		dev_err(dev, "%s: update_wcd_event api is null!\n",
			__func__);
		ret = -EINVAL;
		goto err_bind_all;
	}
	besbev->register_notifier = plat_data->register_notifier;
	if (!besbev->register_notifier) {
		dev_err(dev, "%s: register_notifier api is null!\n",
			__func__);
		ret = -EINVAL;
		goto err_bind_all;
	}

	ret = msm_cdc_enable_static_supplies(dev, besbev->supplies,
					     pdata->regulator,
					     pdata->num_supplies);
	if (ret) {
		dev_err(dev, "%s: wcd static supply enable failed!\n",
			__func__);
		goto err_bind_all;
	}

	ret = of_property_read_u32(dev->of_node, "qcom,comp-support", &comp_support);
	if (ret) {
		dev_dbg(dev, "%s: Failed to obtain comp-support %d\n",
			__func__, ret);
		besbev->comp_support = 0;
	} else {
		besbev->comp_support = comp_support;
	}

	besbev_reset(dev, 0x01);
	usleep_range(20, 30);
	besbev_reset(dev, 0x00);
	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * as per HW requirement.
	 */
	usleep_range(5000, 5010);
	besbev->wakeup = besbev_wakeup;

	ret = component_bind_all(dev, besbev);
	if (ret) {
		dev_err(dev, "%s: Slave bind failed, ret = %d\n",
			__func__, ret);
		goto err_bind_all;
	}

	if (besbev->speaker_present == true)
		ret = besbev_parse_port_mapping(dev, "qcom,swr_ch_map",
						CODEC_RX);
	else
		ret = besbev_parse_port_mapping(dev, "qcom,swr_ch_map",
						CODEC_TX);

	if (ret) {
		dev_err(dev, "%s: Failed to read port mapping\n", __func__);
		goto err;
	}

	besbev->swr_dev = get_matching_swr_slave_device(pdata->besbev_slave);
	if (!besbev->swr_dev) {
		dev_err(dev, "%s: Could not find besbev swr slave device\n",
			 __func__);
		ret = -ENODEV;
		goto err;
	}

	mutex_init(&besbev->res_lock);

	besbev->regmap = devm_regmap_init_swr(besbev->swr_dev,
						       &besbev_regmap_config);
	if (!besbev->regmap) {
		dev_err(dev, "%s: Regmap init failed\n",
				__func__);
		goto err;
	}

	ret = of_property_read_u32(dev->of_node, "qcom,visense-support", &val);

	if (ret)
		besbev->visense_support = 1;
	else
		besbev->visense_support = val;

	if (besbev->speaker_present == true) {
		/* Set all interupts as edge triggered */
		for (i = 0; i < besbev_regmap_irq_chip.num_regs; i++) {
			regmap_write(besbev->regmap,
				     (BESBEV_INTR_LEVEL0 + i), 0);
		}

		besbev_regmap_irq_chip.irq_drv_data = besbev;
		besbev->irq_info.wcd_regmap_irq_chip = &besbev_regmap_irq_chip;
		besbev->irq_info.codec_name = "besbev";
		besbev->irq_info.regmap = besbev->regmap;
		besbev->irq_info.dev = dev;
		ret = wcd_irq_init(&besbev->irq_info, &besbev->virq);

		if (ret) {
			dev_err(dev, "%s: IRQ init failed: %d\n",
				__func__, ret);
			goto err;
		}
		besbev->swr_dev->slave_irq = besbev->virq;

		mutex_init(&besbev->rx_clk_lock);
		/* Request for watchdog interrupt */
		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_SAF2WAR,
				"BESBEV SAF2WAR", besbev_saf2war_handle_irq,
				NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_SAF2WAR);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_WAR2SAF,
				"BESBEV WAR2SAF", besbev_war2saf_handle_irq,
				NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_WAR2SAF);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_DISABLE,
				"BESBEV OTP", besbev_otp_handle_irq, NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_DISABLE);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_OCP,
				"BESBEV OCP", besbev_ocp_handle_irq, NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_OCP);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLIP,
				"BESBEV CLIP", besbev_clip_handle_irq, NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLIP);

		/*
		 * PDM_WD irq is handled to detect disruption in rx data
		 * during playback
		 */
		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_PDM_WD,
				"BESBEV PDM WD", besbev_pdm_wd_handle_irq,
				NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_PDM_WD);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLK_WD,
				"BESBEV CLK WD", besbev_clk_wd_handle_irq,
				NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLK_WD);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_INTR_PIN,
				"BESBEV EXT INT", besbev_ext_int_handle_irq,
				NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_INTR_PIN);

		/* Under Voltage Lock out (UVLO) interrupt handle */
		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_UVLO,
				"BESBEV UVLO", besbev_uvlo_handle_irq, NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_UVLO);

		wcd_request_irq(&besbev->irq_info, BESBEV_IRQ_INT_PA_ON_ERR,
				"BESBEV PA ERR", besbev_pa_on_err_handle_irq,
				NULL);

		wcd_disable_irq(&besbev->irq_info, BESBEV_IRQ_INT_PA_ON_ERR);

		besbev->dai_driver = besbev_dai;

		/*  snd soc register for rx swr */
		soc_codec_dev_besbev.controls = besbev_snd_controls_rx;
		soc_codec_dev_besbev.num_controls =
					ARRAY_SIZE(besbev_snd_controls_rx);
		soc_codec_dev_besbev.dapm_widgets = besbev_dapm_widgets_rx;
		soc_codec_dev_besbev.num_dapm_widgets =
					ARRAY_SIZE(besbev_dapm_widgets_rx);
		soc_codec_dev_besbev.dapm_routes = besbev_audio_map_rx;
		soc_codec_dev_besbev.num_dapm_routes =
					ARRAY_SIZE(besbev_audio_map_rx);
		ret = snd_soc_register_component(dev,
					&soc_codec_dev_besbev,
					besbev->dai_driver, 2);
		if (ret) {
			dev_err(dev, "%s: Codec registration failed\n",
					__func__);
			goto err_irq;
		}
	} else {
		mutex_init(&besbev->micb_lock);
		mutex_init(&besbev->main_bias_lock);

		ret = besbev_set_micbias_data(besbev, pdata);
		if (ret < 0) {
			dev_err(dev, "%s: bad micbias pdata\n", __func__);
			goto err_irq;
		}

		/* snd soc register for tx swr */
		besbev->dai_driver = besbev_dai;

		soc_codec_dev_besbev.controls = besbev_snd_controls_tx;
		soc_codec_dev_besbev.num_controls =
					ARRAY_SIZE(besbev_snd_controls_tx);
		soc_codec_dev_besbev.dapm_widgets = besbev_dapm_widgets_tx;
		soc_codec_dev_besbev.num_dapm_widgets =
					ARRAY_SIZE(besbev_dapm_widgets_tx);
		soc_codec_dev_besbev.dapm_routes = besbev_audio_map_tx;
		soc_codec_dev_besbev.num_dapm_routes =
					ARRAY_SIZE(besbev_audio_map_tx);
		ret = snd_soc_register_component(dev, &soc_codec_dev_besbev,
				     besbev->dai_driver, 1);
		if (ret) {
			dev_err(dev, "%s: Codec registration failed\n",
					__func__);
			goto err_irq;
		}
	}
	return ret;

err_irq:
	if (besbev->speaker_present == true) {
		mutex_destroy(&besbev->rx_clk_lock);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_SAF2WAR, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_WAR2SAF, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_DISABLE, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_OCP, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLIP, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_PDM_WD, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLK_WD, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_INTR_PIN, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_UVLO, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_PA_ON_ERR, NULL);
		wcd_irq_exit(&besbev->irq_info, besbev->virq);
	} else {
		mutex_destroy(&besbev->micb_lock);
		mutex_destroy(&besbev->main_bias_lock);
	}
err:
	component_unbind_all(dev, besbev);
err_bind_all:
	dev_set_drvdata(dev, NULL);
	kfree(pdata);
	kfree(besbev);
	return ret;
}

static void besbev_unbind(struct device *dev)
{
	struct besbev_priv *besbev = dev_get_drvdata(dev);
	struct besbev_pdata *pdata = dev_get_platdata(besbev->dev);

	wcd_irq_exit(&besbev->irq_info, besbev->virq);
	snd_soc_unregister_component(dev);
	if (besbev->speaker_present == true) {
		mutex_destroy(&besbev->rx_clk_lock);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_SAF2WAR, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_WAR2SAF, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_DISABLE, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_OCP, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLIP, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_PDM_WD, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_CLK_WD, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_INTR_PIN, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_UVLO, NULL);
		wcd_free_irq(&besbev->irq_info, BESBEV_IRQ_INT_PA_ON_ERR, NULL);
		wcd_irq_exit(&besbev->irq_info, besbev->virq);
	} else {
		mutex_destroy(&besbev->micb_lock);
		mutex_destroy(&besbev->main_bias_lock);
	}
	mutex_destroy(&besbev->res_lock);
	component_unbind_all(dev, besbev);
	dev_set_drvdata(dev, NULL);
	kfree(pdata);
	kfree(besbev);
}

static const struct of_device_id besbev_dt_match[] = {
	{ .compatible = "qcom,besbev-codec", .data = "besbev" },
	{}
};

static const struct component_master_ops besbev_comp_ops = {
	.bind   = besbev_bind,
	.unbind = besbev_unbind,
};

static int besbev_compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static void besbev_release_of(struct device *dev, void *data)
{
	of_node_put(data);
}

static int besbev_add_slave_components(struct device *dev,
				struct component_match **matchptr)
{
	struct device_node *np, *besbev_node;

	np = dev->of_node;

	besbev_node = of_parse_phandle(np, "qcom,besbev-slave", 0);
	if (!besbev_node) {
		dev_err(dev, "%s: slave node not defined\n", __func__);
		return -ENODEV;
	}

	if (besbev_node) {
		of_node_get(besbev_node);
		component_match_add_release(dev, matchptr,
				besbev_release_of,
				besbev_compare_of,
				besbev_node);
	}

	return 0;
}

static int besbev_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	int ret;

	ret = besbev_add_slave_components(&pdev->dev, &match);
	if (ret)
		return ret;

	return component_master_add_with_match(&pdev->dev,
					&besbev_comp_ops, match);
}

static int besbev_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &besbev_comp_ops);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops besbev_dev_pm_ops = {
	.suspend_late = besbev_suspend,
	.resume_early = besbev_resume
};
#endif

static struct platform_driver besbev_codec_driver = {
	.probe = besbev_probe,
	.remove = besbev_remove,
	.driver = {
		.name = "besbev_codec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(besbev_dt_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &besbev_dev_pm_ops,
#endif
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(besbev_codec_driver);
MODULE_DESCRIPTION("Besbev Codec driver");
MODULE_LICENSE("GPL v2");
