// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/of_device.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <soc/snd_event.h>
#include <dsp/audio_prm.h>
#include <soc/swr-common.h>
#include <soc/soundwire.h>
#include "device_event.h"
#include "asoc/msm-cdc-pinctrl.h"
#include "msm-audio-defs.h"
#include "msm_common.h"
#include "sa525m_dailink.h"
#include <soc/qcom/boot_stats.h>

#define DRV_NAME "sdx-asoc-snd"
#define __CHIPSET__ "LAHAINA "
#define MSM_DAILINK_NAME(name) (__CHIPSET__#name)

#define WCD9XXX_MBHC_DEF_RLOADS     5
#define WCD9XXX_MBHC_DEF_BUTTONS    8
#define CODEC_EXT_CLK_RATE          9600000
#define DEV_NAME_STR_LEN            32
#define WCD_MBHC_HS_V_MAX           1600

#define MSM_LL_QOS_VALUE	300 /* time in us to ensure LPM doesn't go in C3/C4 */
#define TLV_CLKIN_MCLK 0

#define WCN_CDC_SLIM_RX_CH_MAX 2
#define WCN_CDC_SLIM_TX_CH_MAX 2
#define WCN_CDC_SLIM_TX_CH_MAX_LITO 3

#define LPAIF_OFFSET 0x01a00000
#define LPAIF_PRI_MODE_MUXSEL (LPAIF_OFFSET + 0x2008)
#define LPAIF_SEC_MODE_MUXSEL (LPAIF_OFFSET + 0x200c)
#define LPAIF_TERTIARY_MODE_MUXSEL (LPAIF_OFFSET + 0x2010)
#define LPASS_CSR_GP_IO_MUX_SPKR_CTL (LPAIF_OFFSET + 0x2004)
#define LPASS_CSR_GP_IO_MUX_MIC_CTL  (LPAIF_OFFSET + 0x2000)
#define LPASS_CSR_GP_IO_MUX_TER_CTL (LPAIF_OFFSET + 0x2014)

#define I2S_SEL 0
#define PCM_SEL 1
#define I2S_PCM_SEL_OFFSET 0
#define I2S_PCM_MASTER_MODE 1
#define I2S_PCM_SLAVE_MODE 0

#define PRI_TLMM_CLKS_EN_MASTER 0x4
#define SEC_TLMM_CLKS_EN_MASTER 0x2
#define TERT_TLMM_CLKS_EN_MASTER 0x1
#define PRI_TLMM_CLKS_EN_SLAVE 0x100000
#define SEC_TLMM_CLKS_EN_SLAVE 0x800000
#define TERT_TLMM_CLKS_EN_SLAVE 0x2
#define CLOCK_ON  1
#define CLOCK_OFF 0


#define SDX_MCLK_CLK_12P288MHZ 12288000

struct msm_asoc_mach_data {
	struct snd_info_entry *codec_root;
    struct msm_common_pdata *common_pdata;
	int usbc_en2_gpio; /* used by gpio driver API */
	struct device_node *dmic01_gpio_p; /* used by pinctrl API */
	struct device_node *dmic23_gpio_p; /* used by pinctrl API */
	struct device_node *dmic45_gpio_p; /* used by pinctrl API */
	struct device_node *us_euro_gpio_p; /* used by pinctrl API */
	struct pinctrl *usbc_en2_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en1_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en0_gpio_p; /* used by pinctrl API */
	bool is_afe_config_done;
	struct device_node *fsa_handle;
	struct clk *lpass_audio_hw_vote;
	int core_audio_vote_count;
	u32 wsa_max_devs;
	u16 prim_mi2s_mode;
	u16 sec_mi2s_mode;
	u16 tert_mi2s_mode;
	u16 prim_auxpcm_mode;
	u16 sec_auxpcm_mode;
	u16 tert_auxpcm_mode;
	struct device_node *prim_master_slave_p;
	struct device_node *sec_master_slave_p;
	struct device_node *tert_master_slave_p;
	void __iomem *lpaif_pri_muxsel_virt_addr;
	void __iomem *lpaif_sec_muxsel_virt_addr;
	void __iomem *lpaif_tert_muxsel_virt_addr;
	void __iomem *lpass_mux_spkr_ctl_virt_addr;
	void __iomem *lpass_sec_mux_spkr_ctl_virt_addr;
	void __iomem *lpass_tert_mux_spkr_ctl_virt_addr;
	void __iomem *lpass_mux_mic_ctl_virt_addr;
};

static bool is_initial_boot;
static atomic_t mi2s_ref_count;
static atomic_t sec_mi2s_ref_count;
static atomic_t tert_mi2s_ref_count;
static int sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
static int sdx_sec_mi2s_mode = I2S_PCM_MASTER_MODE;
static int sdx_tert_mi2s_mode = I2S_PCM_MASTER_MODE;
static int sdx_auxpcm_mode = I2S_PCM_MASTER_MODE;
static int sdx_sec_auxpcm_mode = I2S_PCM_MASTER_MODE;
static int sdx_tert_auxpcm_mode = I2S_PCM_MASTER_MODE;
static struct snd_soc_card snd_soc_card_sdx_msm;

static int msm_aux_codec_init(struct snd_soc_pcm_runtime*);
static int msm_int_audrx_init(struct snd_soc_pcm_runtime*);

/* set audio task affinity to core 1 & 2 */
static const unsigned int audio_core_list[] = {1, 2};

static struct snd_soc_ops msm_common_be_ops = {
	.hw_params = msm_common_snd_hw_params,
	.startup = msm_common_snd_startup,
	.shutdown = msm_common_snd_shutdown,
};

static void sdx_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&mi2s_ref_count) == 0) {
		struct clk_cfg intf_clk_cfg;
		ret = msm_cdc_pinctrl_select_sleep_state
					(pdata->prim_master_slave_p);
		if (ret)
			pr_err("%s: failed to set pri gpios to sleep: %d\n",
			   __func__, ret);

		if (pdata->prim_mi2s_mode == 1) {
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 0);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
			}
		}
	}
}

static int sdx_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->prim_mi2s_mode = sdx_mi2s_mode;
	if (atomic_inc_return(&mi2s_ref_count) == 1) {
		if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {

			ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
			if (ret < 0) {
				dev_err(card->dev,
				"%s:set lpass core clk failed ret: %d\n",
				__func__, ret);
				ret = -EINVAL;
				goto done;
			}

			iowrite32(I2S_SEL << I2S_PCM_SEL_OFFSET,
				  pdata->lpaif_pri_muxsel_virt_addr);
			if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
				if (pdata->prim_mi2s_mode == 1)
					iowrite32(PRI_TLMM_CLKS_EN_MASTER,
					pdata->lpass_mux_spkr_ctl_virt_addr);
				else
					iowrite32(PRI_TLMM_CLKS_EN_SLAVE,
					pdata->lpass_mux_spkr_ctl_virt_addr);
			} else {
				dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
					__func__);

				ret = -EINVAL;
				goto done;
			}
		} else {
			dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto done;
		}
		/*
		 * This sets the CONFIG PARAMETER WS_SRC.
		 * 1 means internal clock master mode.
		 * 0 means external clock slave mode.
		 */
		if (pdata->prim_mi2s_mode == 1) {
			struct clk_cfg intf_clk_cfg;

			ret = msm_cdc_pinctrl_select_active_state
					(pdata->prim_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 1);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
				goto done;
			}
			ret = snd_soc_dai_set_fmt(codec_dai,
						SND_SOC_DAIFMT_CBS_CFS |
						SND_SOC_DAIFMT_I2S);
			if (ret < 0) {
				dev_err(card->dev,
					"%s Set fmt for codec dai failed\n",
					__func__);
				//Disable mlck here
			}

			ret = snd_soc_dai_set_sysclk(codec_dai,
					TLV_CLKIN_MCLK,
					SDX_MCLK_CLK_12P288MHZ,
					SND_SOC_CLOCK_OUT);
			if (ret < 0) {
				pr_err("%s Set sysclk for codec dai failed 0x%8x\n",
					__func__, ret);
			}
		} else {
			ret = msm_cdc_pinctrl_select_alt_active_state
					(pdata->prim_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
		}
		audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
	}

done:
	if (ret)
		atomic_dec_return(&mi2s_ref_count);
	return ret;
}

static struct snd_soc_ops sdx_mi2s_be_ops = {
	.startup = sdx_mi2s_startup,
	.shutdown = sdx_mi2s_shutdown,
};

static void sdx_tert_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&tert_mi2s_ref_count) == 0) {
		struct clk_cfg intf_clk_cfg;

		ret = msm_cdc_pinctrl_select_sleep_state
					(pdata->tert_master_slave_p);
		if (ret)
			pr_err("%s: failed to set tert gpios to sleep: %d\n",
			   __func__, ret);
		if (pdata->tert_mi2s_mode == 1) {
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 0);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
			}
		}
	}
}

static int sdx_tert_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->tert_mi2s_mode = sdx_tert_mi2s_mode;
	if (atomic_inc_return(&tert_mi2s_ref_count) == 1) {
		if (pdata->lpaif_tert_muxsel_virt_addr != NULL) {

			ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
			if (ret < 0) {
				dev_err(card->dev,
				"%s:set lpass core clk failed ret: %d\n",
				__func__, ret);
				ret = -EINVAL;
				goto done;
			}

			iowrite32(I2S_SEL << I2S_PCM_SEL_OFFSET,
				  pdata->lpaif_tert_muxsel_virt_addr);
			if (pdata->lpass_tert_mux_spkr_ctl_virt_addr != NULL) {
				if (pdata->tert_mi2s_mode == 1)
					iowrite32(TERT_TLMM_CLKS_EN_MASTER,
					pdata->lpass_tert_mux_spkr_ctl_virt_addr);
				else
					iowrite32(TERT_TLMM_CLKS_EN_SLAVE,
					pdata->lpass_tert_mux_spkr_ctl_virt_addr);
			} else {
				dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
					__func__);

				ret = -EINVAL;
				goto done;
			}
		} else {
			dev_err(card->dev, "%s lpaif_tert_muxsel_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto done;
		}
		/*
		 * This sets the CONFIG PARAMETER WS_SRC.
		 * 1 means internal clock master mode.
		 * 0 means external clock slave mode.
		 */
		if (pdata->tert_mi2s_mode == 1) {
			struct clk_cfg intf_clk_cfg;

			ret = msm_cdc_pinctrl_select_active_state
					(pdata->tert_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 1);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
				goto done;
			}

		} else {
			ret = msm_cdc_pinctrl_select_alt_active_state
					(pdata->tert_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
		}
		audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
	}

done:
	if (ret)
		atomic_dec_return(&tert_mi2s_ref_count);
	return ret;
}

static struct snd_soc_ops sdx_tert_mi2s_be_ops = {
	.startup = sdx_tert_mi2s_startup,
	.shutdown = sdx_tert_mi2s_shutdown,
};

static void sdx_sec_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&sec_mi2s_ref_count) == 0) {
		struct clk_cfg intf_clk_cfg;

		ret = msm_cdc_pinctrl_select_sleep_state
					(pdata->sec_master_slave_p);
		if (ret)
			pr_err("%s: failed to set sec gpios to sleep: %d\n",
			   __func__, ret);
		if (pdata->sec_mi2s_mode == 1) {
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 0);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
			}
		}
	}
}

static int sdx_sec_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->sec_mi2s_mode = sdx_sec_mi2s_mode;
	if (atomic_inc_return(&sec_mi2s_ref_count) == 1) {
		if (pdata->lpaif_sec_muxsel_virt_addr != NULL) {

			ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
			if (ret < 0) {
				dev_err(card->dev,
				"%s:set lpass core clk failed ret: %d\n",
				__func__, ret);
				ret = -EINVAL;
				goto done;
			}

			iowrite32(I2S_SEL << I2S_PCM_SEL_OFFSET,
				  pdata->lpaif_sec_muxsel_virt_addr);
			if (pdata->lpass_sec_mux_spkr_ctl_virt_addr != NULL) {
				if (pdata->sec_mi2s_mode == 1)
					iowrite32(SEC_TLMM_CLKS_EN_MASTER,
					pdata->lpass_sec_mux_spkr_ctl_virt_addr);
				else
					iowrite32(SEC_TLMM_CLKS_EN_SLAVE,
					pdata->lpass_sec_mux_spkr_ctl_virt_addr);
			} else {
				dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
					__func__);

				ret = -EINVAL;
				goto done;
			}
		} else {
			dev_err(card->dev, "%s lpaif_sec_muxsel_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto done;
		}
		/*
		 * This sets the CONFIG PARAMETER WS_SRC.
		 * 1 means internal clock master mode.
		 * 0 means external clock slave mode.
		 */
		if (pdata->sec_mi2s_mode == 1) {
			struct clk_cfg intf_clk_cfg;

			ret = msm_cdc_pinctrl_select_active_state
					(pdata->sec_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 1);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
				goto done;
			}

		} else {
			ret = msm_cdc_pinctrl_select_alt_active_state
					(pdata->sec_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
		}
		audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
	}

done:
	if (ret)
		atomic_dec_return(&sec_mi2s_ref_count);
	return ret;
}

static struct snd_soc_ops sdx_sec_mi2s_be_ops = {
	.startup = sdx_sec_mi2s_startup,
	.shutdown = sdx_sec_mi2s_shutdown,
};

static void sdx_tdm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&mi2s_ref_count) == 0) {
		struct clk_cfg intf_clk_cfg;
		ret = msm_cdc_pinctrl_select_sleep_state
					(pdata->prim_master_slave_p);
		if (ret)
			pr_err("%s: failed to set pri gpios to sleep: %d\n",
			   __func__, ret);

		if (pdata->prim_mi2s_mode == 1) {
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 0);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
			}
		}
	}
}

static int sdx_mi2s_mode_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_mi2s_mode %d\n", __func__, sdx_mi2s_mode);
	ucontrol->value.integer.value[0] = sdx_mi2s_mode;
	return 0;
}

static int sdx_mi2s_mode_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	case 0:
		sdx_mi2s_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_mi2s_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_mi2s_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_sec_mi2s_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_sec_mi2s_mode %d\n", __func__, sdx_sec_mi2s_mode);
	ucontrol->value.integer.value[0] = sdx_sec_mi2s_mode;
	return 0;
}

static int sdx_sec_mi2s_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_sec_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	case 0:
		sdx_sec_mi2s_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_sec_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_sec_mi2s_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_sec_mi2s_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_tert_mi2s_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_tert_mi2s_mode %d\n", __func__, sdx_tert_mi2s_mode);
	ucontrol->value.integer.value[0] = sdx_tert_mi2s_mode;
	return 0;
}

static int sdx_tert_mi2s_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_tert_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	case 0:
		sdx_tert_mi2s_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_tert_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_tert_mi2s_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_tert_mi2s_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_auxpcm_mode_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_auxpcm_mode %d\n", __func__, sdx_auxpcm_mode);
	ucontrol->value.integer.value[0] = sdx_auxpcm_mode;
	return 0;
}

static int sdx_auxpcm_mode_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	case 0:
		sdx_auxpcm_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_auxpcm_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_auxpcm_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_sec_auxpcm_mode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_sec_auxpcm_mode %d\n", __func__, sdx_sec_auxpcm_mode);
	ucontrol->value.integer.value[0] = sdx_sec_auxpcm_mode;
	return 0;
}

static int sdx_sec_auxpcm_mode_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_sec_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	case 0:
		sdx_sec_auxpcm_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_sec_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_sec_auxpcm_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_sec_auxpcm_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_tert_auxpcm_mode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_tert_auxpcm_mode %d\n", __func__, sdx_tert_auxpcm_mode);
	ucontrol->value.integer.value[0] = sdx_tert_auxpcm_mode;
	return 0;
}

static int sdx_tert_auxpcm_mode_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_tert_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	case 0:
		sdx_tert_auxpcm_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_tert_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_tert_auxpcm_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_tert_auxpcm_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_tdm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->prim_mi2s_mode = sdx_mi2s_mode;
	if (atomic_inc_return(&mi2s_ref_count) == 1) {
		if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {

			ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
			if (ret < 0) {
				dev_err(card->dev,
				"%s:set lpass core clk failed ret: %d\n",
				__func__, ret);
				ret = -EINVAL;
				goto done;
			}

			iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
				  pdata->lpaif_pri_muxsel_virt_addr);
			if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
				if (pdata->prim_mi2s_mode == 1)
					iowrite32(PRI_TLMM_CLKS_EN_MASTER,
					pdata->lpass_mux_spkr_ctl_virt_addr);
				else
					iowrite32(PRI_TLMM_CLKS_EN_SLAVE,
					pdata->lpass_mux_spkr_ctl_virt_addr);
			} else {
				dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
					__func__);

				ret = -EINVAL;
				goto done;
			}
		} else {
			dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto done;
		}
		/*
		 * This sets the CONFIG PARAMETER WS_SRC.
		 * 1 means internal clock master mode.
		 * 0 means external clock slave mode.
		 */
		if (pdata->prim_mi2s_mode == 1) {
			struct clk_cfg intf_clk_cfg;

			ret = msm_cdc_pinctrl_select_active_state
					(pdata->prim_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
			intf_clk_cfg.clk_id = CLOCK_ID_MCLK_3 ;
			intf_clk_cfg.clk_freq_in_hz = SDX_MCLK_CLK_12P288MHZ;
			intf_clk_cfg.clk_attri = CLOCK_ATTRIBUTE_COUPLE_NO;
			intf_clk_cfg.clk_root = 0;
			ret = audio_prm_set_lpass_clk_cfg(&intf_clk_cfg, 1);
			if (ret < 0) {
				pr_err("%s set lpass clk set failed\n", __func__);
				goto done;
			}

			if (ret < 0) {
				dev_err(card->dev,
					"%s Set fmt for codec dai failed\n",
					__func__);
				//Disable mlck here
			}
		} else {
			ret = msm_cdc_pinctrl_select_alt_active_state
				(pdata->prim_master_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto done;
			}
		}
		audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
	}

done:
	if (ret)
		atomic_dec_return(&mi2s_ref_count);
	return ret;
}

static struct snd_soc_ops sdx_tdm_be_ops = {
	.startup = sdx_tdm_startup,
	.shutdown = sdx_tdm_shutdown,
};

static void sdx_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->prim_auxpcm_mode == 1) {
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->prim_master_slave_p);
		if (ret)
			pr_err("%s: failed to set pri gpios to sleep: %d\n",
		        __func__, ret);
	}
}

static int sdx_auxpcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->prim_auxpcm_mode = sdx_auxpcm_mode;
	if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {

		ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
		if (ret < 0) {
			dev_err(card->dev,
			"%s:set lpass core clk failed ret: %d\n",
			__func__, ret);
			ret = -EINVAL;
			goto done;
		}

		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
			  pdata->lpaif_pri_muxsel_virt_addr);
		if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
			if (pdata->prim_auxpcm_mode == 1)
				iowrite32(PRI_TLMM_CLKS_EN_MASTER,
				pdata->lpass_mux_spkr_ctl_virt_addr);
			else
				iowrite32(PRI_TLMM_CLKS_EN_SLAVE,
				pdata->lpass_mux_spkr_ctl_virt_addr);
		} else {
			dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
				__func__);

			ret = -EINVAL;
			goto done;
		}
	} else {
		dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
			__func__);
		ret = -EINVAL;
		goto done;
	}
	/*
	 * This sets the CONFIG PARAMETER WS_SRC.
	 * 1 means internal clock master mode.
	 * 0 means external clock slave mode.
	 */
	if (pdata->prim_auxpcm_mode == 1) {

		ret = msm_cdc_pinctrl_select_active_state(pdata->prim_master_slave_p);
		if (ret < 0) {
			pr_err("%s pinctrl set failed\n", __func__);
			goto done;
		}
	} else {
		/*
		 * Disable bit clk in slave mode for QC codec.
		 * Enable only mclk.
		 */
		ret = msm_cdc_pinctrl_select_alt_active_state(pdata->prim_master_slave_p);
		if (ret < 0) {
			pr_err("%s pinctrl set failed\n", __func__);
			goto done;
		}
	}
	audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
done:
	return ret;
}

static void sdx_sec_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->sec_auxpcm_mode == 1) {
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->sec_master_slave_p);
		if (ret)
			pr_err("%s: failed to set sec gpios to sleep: %d\n",
		       __func__, ret);
	}
}

static int sdx_sec_auxpcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->sec_auxpcm_mode = sdx_sec_auxpcm_mode;
	if (pdata->lpaif_sec_muxsel_virt_addr != NULL) {

		ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
		if (ret < 0) {
			dev_err(card->dev,
			"%s:set lpass core clk failed ret: %d\n",
			__func__, ret);
			ret = -EINVAL;
			goto done;
		}

		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
			  pdata->lpaif_sec_muxsel_virt_addr);
		if (pdata->lpass_sec_mux_spkr_ctl_virt_addr != NULL) {
			if (pdata->sec_auxpcm_mode == 1)
				iowrite32(SEC_TLMM_CLKS_EN_MASTER,
				pdata->lpass_sec_mux_spkr_ctl_virt_addr);
			else
				iowrite32(SEC_TLMM_CLKS_EN_SLAVE,
				pdata->lpass_sec_mux_spkr_ctl_virt_addr);
		} else {
			dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
				__func__);

			ret = -EINVAL;
			goto done;
		}
	} else {
		dev_err(card->dev, "%s lpaif_sec_muxsel_virt_addr is NULL\n",
			__func__);
		ret = -EINVAL;
		goto done;
	}
	/*
	 * This sets the CONFIG PARAMETER WS_SRC.
	 * 1 means internal clock master mode.
	 * 0 means external clock slave mode.
	 */
	if (pdata->sec_auxpcm_mode == 1) {

		ret = msm_cdc_pinctrl_select_active_state(pdata->sec_master_slave_p);
		if (ret < 0) {
			pr_err("%s pinctrl set failed\n", __func__);
			goto done;
		}
	} else {
		/*
		 * Disable bit clk in slave mode for QC codec.
		 * Enable only mclk.
		 */
		ret = msm_cdc_pinctrl_select_alt_active_state(pdata->sec_master_slave_p);
		if (ret < 0) {
			pr_err("%s pinctrl set failed\n", __func__);
			goto done;
		}
	}
	audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
done:
	return ret;
}

static void sdx_tert_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->tert_auxpcm_mode == 1) {
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->tert_master_slave_p);
		if (ret)
			pr_err("%s: failed to set sec gpios to sleep: %d\n",
		       __func__, ret);
	}
}

static int sdx_tert_auxpcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->tert_auxpcm_mode = sdx_tert_auxpcm_mode;
	if (pdata->lpaif_tert_muxsel_virt_addr != NULL) {
		ret = audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
		if (ret < 0) {
			dev_err(card->dev,
			"%s:set lpass core clk failed ret: %d\n",
			__func__, ret);
			ret = -EINVAL;
			goto done;
		}

		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
			  pdata->lpaif_tert_muxsel_virt_addr);

		if (pdata->lpass_tert_mux_spkr_ctl_virt_addr != NULL) {
			if (pdata->tert_auxpcm_mode == 1)
				iowrite32(TERT_TLMM_CLKS_EN_MASTER,
				pdata->lpass_tert_mux_spkr_ctl_virt_addr);
			else
				iowrite32(TERT_TLMM_CLKS_EN_SLAVE,
				pdata->lpass_tert_mux_spkr_ctl_virt_addr);
		} else {
			dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
				__func__);

			ret = -EINVAL;
			goto done;
		}
	} else {
		dev_err(card->dev, "%s lpaif_tert_muxsel_virt_addr is NULL\n",
			__func__);
		ret = -EINVAL;
		goto done;
	}
	/*
	 * This sets the CONFIG PARAMETER WS_SRC.
	 * 1 means internal clock master mode.
	 * 0 means external clock slave mode.
	 */
	if (pdata->tert_auxpcm_mode == 1) {
		ret = msm_cdc_pinctrl_select_active_state(pdata->tert_master_slave_p);
		if (ret < 0) {
			pr_err("%s pinctrl set failed\n", __func__);
			goto done;
		}
	} else {
		/*
		 * Disable bit clk in slave mode for QC codec.
		 * Enable only mclk.
		 */
		ret = msm_cdc_pinctrl_select_alt_active_state
				(pdata->tert_master_slave_p);
		if (ret < 0) {
			pr_err("%s pinctrl set failed\n", __func__);
			goto done;
		}
	}
	audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
done:
	return ret;
}

static struct snd_soc_ops sdx_auxpcm_be_ops = {
	.startup = sdx_auxpcm_startup,
	.shutdown = sdx_auxpcm_shutdown,
};

static struct snd_soc_ops sdx_sec_auxpcm_be_ops = {
	.startup = sdx_sec_auxpcm_startup,
	.shutdown = sdx_sec_auxpcm_shutdown,
};

static struct snd_soc_ops sdx_tert_auxpcm_be_ops = {
	.startup = sdx_tert_auxpcm_startup,
	.shutdown = sdx_tert_auxpcm_shutdown,
};

static struct snd_info_entry *msm_snd_info_create_subdir(struct module *mod,
				const char *name,
				struct snd_info_entry *parent)
{
	struct snd_info_entry *entry;

	entry = snd_info_create_module_entry(mod, name, parent);
	if (!entry)
		return NULL;
	entry->mode = S_IFDIR | 0555;
	if (snd_info_register(entry) < 0) {
		snd_info_free_entry(entry);
		return NULL;
	}
	return entry;
}

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm_common_be_dai_links[] = {
	/* Proxy Tx BACK END DAI Link */
	{
		.name = LPASS_BE_RT_PROXY_PCM_TX,
		.stream_name = LPASS_BE_RT_PROXY_PCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(proxy_tx),
	},
	/* Proxy Rx BACK END DAI Link */
	{
		.name = LPASS_BE_RT_PROXY_PCM_RX,
		.stream_name = LPASS_BE_RT_PROXY_PCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(proxy_rx),
	},
	{
		.name = LPASS_BE_USB_AUDIO_RX,
		.stream_name = LPASS_BE_USB_AUDIO_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(usb_audio_rx),
	},
	{
		.name = LPASS_BE_USB_AUDIO_TX,
		.stream_name = LPASS_BE_USB_AUDIO_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(usb_audio_tx),
	},
};

static struct snd_soc_dai_link msm_rx_tx_cdc_dma_be_dai_links[] = {
	/* WSA CDC DMA Backend DAI Links */
	{
		.name = LPASS_BE_PRI_MI2S_RX,
		.stream_name = LPASS_BE_PRI_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_mi2s_be_ops,
		SND_SOC_DAILINK_REG(tlv320aic3x_codec),
		.init = &msm_aux_codec_init,
	},
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = LPASS_BE_PRI_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_mi2s_be_ops,
		SND_SOC_DAILINK_REG(tlv320aic3x_codec),
	},
	{
		.name = LPASS_BE_TERT_MI2S_RX,
		.stream_name = LPASS_BE_TERT_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_tert_mi2s_be_ops,
		SND_SOC_DAILINK_REG(pcm_rx),
		.init = &msm_aux_codec_init,
	},
	{
		.name = LPASS_BE_TERT_MI2S_TX,
		.stream_name = LPASS_BE_TERT_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_tert_mi2s_be_ops,
		SND_SOC_DAILINK_REG(pcm_tx),
	},
    {
		.name = LPASS_BE_SEC_MI2S_RX,
		.stream_name = LPASS_BE_SEC_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_sec_mi2s_be_ops,
		SND_SOC_DAILINK_REG(pcm_rx),
		.init = &msm_aux_codec_init,
	},
	{
		.name = LPASS_BE_SEC_MI2S_TX,
		.stream_name = LPASS_BE_SEC_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_sec_mi2s_be_ops,
		SND_SOC_DAILINK_REG(pcm_tx),
	},
};

static struct snd_soc_dai_link sdx_auxpcm_be_dai_links[] = {
	/* Primary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_PRI_AUXPCM_RX,
		.stream_name = LPASS_BE_PRI_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_auxpcm_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_PRI_AUXPCM_TX,
		.stream_name = LPASS_BE_PRI_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_auxpcm_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
	{
		.name = LPASS_BE_SEC_AUXPCM_RX,
		.stream_name = LPASS_BE_SEC_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_sec_auxpcm_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_SEC_AUXPCM_TX,
		.stream_name = LPASS_BE_SEC_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_sec_auxpcm_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
	{
		.name = LPASS_BE_TERT_AUXPCM_RX,
		.stream_name = LPASS_BE_TERT_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_tert_auxpcm_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_TERT_AUXPCM_TX,
		.stream_name = LPASS_BE_TERT_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_tert_auxpcm_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
};

static struct snd_soc_dai_link msm_tdm_dai_links[] = {
	{
		.name = LPASS_BE_PRI_TDM_RX_0,
		.stream_name = LPASS_BE_PRI_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
		SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_tdm_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0),
	},
	{
		.name = LPASS_BE_PRI_TDM_TX_0,
		.stream_name = LPASS_BE_PRI_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
		SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_tdm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0),
	},
		{
		.name = LPASS_BE_SEC_TDM_RX_0,
		.stream_name = LPASS_BE_SEC_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
		SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_sec_auxpcm_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0),
	},
	{
		.name = LPASS_BE_SEC_TDM_TX_0,
		.stream_name = LPASS_BE_SEC_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
		SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_sec_auxpcm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0),
	},	{
		.name = LPASS_BE_TERT_TDM_RX_0,
		.stream_name = LPASS_BE_TERT_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
		SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_tert_auxpcm_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0),
	},
	{
		.name =  LPASS_BE_TERT_TDM_TX_0,
		.stream_name =  LPASS_BE_TERT_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
		SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_tert_auxpcm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0),
	},
};

static struct snd_soc_dai_link msm_sdx_dai_links[
			ARRAY_SIZE(msm_rx_tx_cdc_dma_be_dai_links) +
			ARRAY_SIZE(sdx_auxpcm_be_dai_links) +
			ARRAY_SIZE(msm_common_be_dai_links) +
			ARRAY_SIZE(msm_tdm_dai_links)];


static int msm_populate_dai_link_component_of_node(
					struct snd_soc_card *card)
{
	int i, j, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np = NULL;
	int codecs_enabled = 0;
	struct snd_soc_dai_link_component *codecs_comp = NULL;

	if (!cdev) {
		dev_err(cdev, "%s: Sound card device memory NULL\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].init == NULL)
			dai_link[i].init = &msm_common_dai_link_init;

		/* populate codec_of_node for snd card dai links */
		if (dai_link[i].num_codecs > 0) {
			for (j = 0; j < dai_link[i].num_codecs; j++) {
				if (dai_link[i].codecs[j].of_node ||
						!dai_link[i].codecs[j].name)
					continue;

				index = of_property_match_string(cdev->of_node,
						"asoc-codec-names",
						dai_link[i].codecs[j].name);
				if (index < 0)
					continue;
				np = of_parse_phandle(cdev->of_node,
						      "asoc-codec",
						      index);
				if (!np) {
					dev_err(cdev, "%s: retrieving phandle for codec %s failed\n",
						__func__,
						dai_link[i].codecs[j].name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].codecs[j].of_node = np;
				dai_link[i].codecs[j].name = NULL;
			}
		}
	}

	/* In multi-codec scenario, check if codecs are enabled for this platform */
	for (i = 0; i < card->num_links; i++) {
		codecs_enabled = 0;
		if (dai_link[i].num_codecs > 1) {
			for (j = 0; j < dai_link[i].num_codecs; j++) {
				if (!dai_link[i].codecs[j].of_node)
					continue;

				np = dai_link[i].codecs[j].of_node;
                if (!of_device_is_available(np)) {
                    dev_err(cdev, "%s: codec is disabled: %s\n",
						__func__,
						np->full_name);
					dai_link[i].codecs[j].of_node = NULL;
					continue;
                }

				codecs_enabled++;
			}
			if (codecs_enabled > 0 &&
				    codecs_enabled < dai_link[i].num_codecs) {
				codecs_comp = devm_kzalloc(cdev,
				    sizeof(struct snd_soc_dai_link_component)
				    * codecs_enabled, GFP_KERNEL);
				if (!codecs_comp) {
					dev_err(cdev, "%s: %s dailink codec component alloc failed\n",
						__func__, dai_link[i].name);
					ret = -ENOMEM;
					goto err;
				}
				index = 0;
				for (j = 0; j < dai_link[i].num_codecs; j++) {
					if(dai_link[i].codecs[j].of_node) {
						codecs_comp[index].of_node =
						  dai_link[i].codecs[j].of_node;
						codecs_comp[index].dai_name =
						  dai_link[i].codecs[j].dai_name;
						codecs_comp[index].name = NULL;
						index++;
					}
				}
				dai_link[i].codecs = codecs_comp;
				dai_link[i].num_codecs = codecs_enabled;
			}
		}
	}

err:
	return ret;
}

static int msm_audrx_stub_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int msm_snd_stub_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops msm_stub_be_ops = {
	.hw_params = msm_snd_stub_hw_params,
};

struct snd_soc_card snd_soc_card_stub_msm = {
	.name		= "sdx-stub-snd-card",
};

static struct snd_soc_dai_link msm_stub_be_dai_links[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_PRI_AUXPCM_RX,
		.stream_name = LPASS_BE_PRI_AUXPCM_RX,
		.playback_only = 1,
		.init = &msm_audrx_stub_init,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_PRI_AUXPCM_TX,
		.stream_name = LPASS_BE_PRI_AUXPCM_TX,
		.capture_only = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
};

static struct snd_soc_dai_link msm_stub_dai_links[
			 ARRAY_SIZE(msm_stub_be_dai_links)];

static const struct of_device_id sdx_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,sdx-asoc-snd-auto",
	  .data = "codec"},
	{},
};

static const char *const mode_text[] = {"slave", "master"};

static const struct soc_enum sdx_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, mode_text),
};

static const struct snd_kcontrol_new sdx_snd_controls[] = {
	SOC_ENUM_EXT("MI2S Mode", sdx_enum[0],
				 sdx_mi2s_mode_get,
				 sdx_mi2s_mode_put),
	SOC_ENUM_EXT("SEC_MI2S Mode", sdx_enum[0],
				 sdx_sec_mi2s_mode_get,
				 sdx_sec_mi2s_mode_put),
	SOC_ENUM_EXT("TERT_MI2S Mode", sdx_enum[0],
				 sdx_tert_mi2s_mode_get,
				 sdx_tert_mi2s_mode_put),
	SOC_ENUM_EXT("AUXPCM Mode", sdx_enum[0],
				 sdx_auxpcm_mode_get,
				 sdx_auxpcm_mode_put),
	SOC_ENUM_EXT("SEC_AUXPCM Mode", sdx_enum[0],
				 sdx_sec_auxpcm_mode_get,
				 sdx_sec_auxpcm_mode_put),
	SOC_ENUM_EXT("TERT_AUXPCM Mode", sdx_enum[0],
				 sdx_tert_auxpcm_mode_get,
				 sdx_tert_auxpcm_mode_put),
};


static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *dailink = NULL;
	int total_links = 0;
	const struct of_device_id *match;

	match = of_match_node(sdx_asoc_machine_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "%s: No DT match found for sound card\n",
			__func__);
		return NULL;
	}

	if (!strcmp(match->data, "codec")) {
		card = &snd_soc_card_sdx_msm;

		memcpy(msm_sdx_dai_links + total_links,
		       msm_rx_tx_cdc_dma_be_dai_links,
		       sizeof(msm_rx_tx_cdc_dma_be_dai_links));
		total_links +=
			ARRAY_SIZE(msm_rx_tx_cdc_dma_be_dai_links);

		memcpy(msm_sdx_dai_links + total_links,
		       sdx_auxpcm_be_dai_links,
		       sizeof(sdx_auxpcm_be_dai_links));
		total_links +=
			ARRAY_SIZE(sdx_auxpcm_be_dai_links);

		memcpy(msm_sdx_dai_links + total_links,
		       msm_common_be_dai_links,
		       sizeof(msm_common_be_dai_links));
		total_links +=
			ARRAY_SIZE(msm_common_be_dai_links);

		memcpy(msm_sdx_dai_links + total_links,
				msm_tdm_dai_links,
				sizeof(msm_tdm_dai_links));
		total_links += ARRAY_SIZE(msm_tdm_dai_links);


		dailink = msm_sdx_dai_links;
	} else if(!strcmp(match->data, "stub_codec")) {
		card = &snd_soc_card_stub_msm;

		memcpy(msm_stub_dai_links,
		       msm_stub_be_dai_links,
		       sizeof(msm_stub_be_dai_links));

		dailink = msm_stub_dai_links;
		total_links = ARRAY_SIZE(msm_stub_be_dai_links);
	}

	if (card) {
		card->dai_link = dailink;
		card->num_links = total_links;
	}

	return card;
}

static int msm_int_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = NULL;
	struct snd_info_entry *entry = NULL;
	struct msm_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *component = NULL;

	card = rtd->card->snd_card;
	if (!pdata->codec_root) {
		entry = msm_snd_info_create_subdir(card->module, "codecs",
						 card->proc_root);
		if (!entry) {
			pr_debug("%s: Cannot create codecs module entry\n",
				 __func__);
			ret = 0;
			goto err;
		}
		pdata->codec_root = entry;
	}
	msm_common_dai_link_init(rtd);

	component = codec_dai->component;

	ret = snd_soc_add_component_controls(component, sdx_snd_controls,
					 ARRAY_SIZE(sdx_snd_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, %d\n",
		       __func__, ret);
		goto err;
	}
err:
	return ret;
}

static int msm_aux_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	msm_common_dai_link_init(rtd);
	msm_int_audrx_init(rtd);
	return 0;
}

static int sdx_ssr_enable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	if (!strcmp(card->name, "sdx-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO \n", __func__);
	}

	snd_card_notify_user(SND_CARD_STATUS_ONLINE);

	dev_info(dev, "%s: setting snd_card to ONLINE\n", __func__);

err:
	return ret;
}

static void sdx_ssr_disable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		return;
	}

	dev_info(dev, "%s: setting snd_card to OFFLINE\n", __func__);


	snd_card_notify_user(SND_CARD_STATUS_OFFLINE);
	if (!strcmp(card->name, "sdx-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO \n", __func__);
	}
}

static const struct snd_event_ops sdx_ssr_ops = {
	.enable = sdx_ssr_enable,
	.disable = sdx_ssr_disable,
};

static int msm_audio_ssr_compare(struct device *dev, void *data)
{
	struct device_node *node = data;

	dev_dbg(dev, "%s: dev->of_node = 0x%p, node = 0x%p\n",
		__func__, dev->of_node, node);
	return (dev->of_node && dev->of_node == node);
}

static int msm_audio_ssr_register(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct snd_event_clients *ssr_clients = NULL;
	struct device_node *node = NULL;
	int ret = 0;
	int i = 0;

	for (i = 0; ; i++) {
		node = of_parse_phandle(np, "qcom,msm_audio_ssr_devs", i);
		if (!node)
			break;
		snd_event_mstr_add_client(&ssr_clients,
					msm_audio_ssr_compare, node);
	}

	ret = snd_event_master_register(dev, &sdx_ssr_ops,
					ssr_clients, NULL);
	if (!ret)
		snd_event_notify(dev, SND_EVENT_UP);

	return ret;
}

struct msm_common_pdata *msm_common_get_pdata(struct snd_soc_card *card)
{
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return NULL;

	return pdata->common_pdata;
}

void msm_common_set_pdata(struct snd_soc_card *card,
			  struct msm_common_pdata *common_pdata)
{
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return;

	pdata->common_pdata = common_pdata;
}

static int msm_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = NULL;
	struct msm_asoc_mach_data *pdata = NULL;
	int ret = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s: No platform supplied from device tree\n", __func__);
		return -EINVAL;
	}
	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_asoc_mach_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	card = populate_snd_card_dailinks(&pdev->dev);
	if (!card) {
		dev_err(&pdev->dev, "%s: Card uninitialized\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "%s: parse card name failed, err:%d\n",
			__func__, ret);
		goto err;
	}

	ret = msm_populate_dai_link_component_of_node(card);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "%s: snd_soc_register_card failed (%d)\n",
			__func__, ret);
		goto err;
	}
	dev_info(&pdev->dev, "%s: Sound card %s registered\n",
		 __func__, card->name);

	pdata->fsa_handle = of_parse_phandle(pdev->dev.of_node,
					"fsa4480-i2c-handle", 0);
	if (!pdata->fsa_handle)
		dev_dbg(&pdev->dev, "property %s not detected in node %s\n",
			"fsa4480-i2c-handle", pdev->dev.of_node->full_name);

	msm_common_snd_init(pdev, card);

	pdata->prim_master_slave_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,prim_mi2s_aux_master",
						0);
	atomic_set(&mi2s_ref_count, 0);
	pdata->sec_master_slave_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,sec_mi2s_aux_master",
						0);
	atomic_set(&sec_mi2s_ref_count, 0);

	pdata->tert_master_slave_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,tert_mi2s_aux_master",
						0);
	atomic_set(&tert_mi2s_ref_count, 0);

	pdata->lpaif_pri_muxsel_virt_addr = ioremap(LPAIF_PRI_MODE_MUXSEL, 4);
	if (pdata->lpaif_pri_muxsel_virt_addr == NULL) {
		pr_err("%s Pri muxsel virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err;
	}

	pdata->lpass_mux_spkr_ctl_virt_addr =
				ioremap(LPASS_CSR_GP_IO_MUX_SPKR_CTL, 4);
	if (pdata->lpass_mux_spkr_ctl_virt_addr == NULL) {
		pr_err("%s lpass spkr ctl virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err1;
	}

	pdata->lpaif_sec_muxsel_virt_addr = ioremap(LPAIF_SEC_MODE_MUXSEL, 4);
	if (pdata->lpaif_sec_muxsel_virt_addr == NULL) {
		pr_err("%s Sec muxsel virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err2;
	}

	pdata->lpass_sec_mux_spkr_ctl_virt_addr =
				ioremap(LPASS_CSR_GP_IO_MUX_MIC_CTL, 4);
	if (pdata->lpass_sec_mux_spkr_ctl_virt_addr == NULL) {
		pr_err("%s lpass secondary spkr ctl virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err3;
	}

	pdata->lpaif_tert_muxsel_virt_addr = ioremap(LPAIF_TERTIARY_MODE_MUXSEL, 4);
	if (pdata->lpaif_tert_muxsel_virt_addr == NULL) {
		pr_err("%s Tertiary muxsel virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err4;
	}

	pdata->lpass_tert_mux_spkr_ctl_virt_addr =
				ioremap(LPASS_CSR_GP_IO_MUX_TER_CTL, 4);
	if (pdata->lpass_tert_mux_spkr_ctl_virt_addr == NULL) {
		pr_err("%s lpass spkr ctl virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err5;
	}

	snd_card_sysfs_init();
	ret = msm_audio_ssr_register(&pdev->dev);
	if (ret)
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",
			__func__, ret);

	is_initial_boot = true;
	update_marker("M - DRIVER Audio Ready");

	return 0;
err5:
	iounmap(pdata->lpaif_tert_muxsel_virt_addr);
err4:
	iounmap(pdata->lpass_sec_mux_spkr_ctl_virt_addr);
err3:
	iounmap(pdata->lpaif_sec_muxsel_virt_addr);
err2:
	iounmap(pdata->lpass_mux_spkr_ctl_virt_addr);
err1:
	iounmap(pdata->lpaif_pri_muxsel_virt_addr);
err:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int msm_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = NULL;
	struct msm_common_pdata *common_pdata = NULL;

	if (card)
		pdata = snd_soc_card_get_drvdata(card);

	if (pdata)
		common_pdata = pdata->common_pdata;

	msm_common_snd_deinit(common_pdata);
	snd_event_master_deregister(&pdev->dev);
	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver sdx_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = sdx_asoc_machine_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_asoc_machine_probe,
	.remove = msm_asoc_machine_remove,
};
module_platform_driver(sdx_asoc_machine_driver);

MODULE_SOFTDEP("pre: bt_fm_slim");
MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, sdx_asoc_machine_of_match);
