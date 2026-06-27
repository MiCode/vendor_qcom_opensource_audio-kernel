// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
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
#include <linux/nvmem-consumer.h>
#include <linux/soc/qcom/slatecom_intf.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <soc/snd_event.h>
#include <soc/swr-common.h>
#include <soc/soundwire.h>
#include <dsp/spf-core.h>
#include <dsp/msm_audio_ion.h>
#include <dsp/audio_notifier.h>
#include "device_event.h"
#include "asoc/msm-cdc-pinctrl.h"
#include "asoc/wcd-mbhc-v2.h"
#include "codecs/besbev/besbev.h"
#include "codecs/wsa883x/wsa883x.h"
#include "codecs/bolero/bolero-cdc.h"
#include <bindings/audio-codec-port-types.h>
#include "monaco-port-config.h"
#include "msm-audio-defs.h"
#include "msm_common.h"
#include "msm_monaco_dailink.h"
#include "dsp/audio_prm.h"

#define DRV_NAME "monaco-asoc-snd"
#define __CHIPSET__ "MONACO "
#define MSM_DAILINK_NAME(name) (__CHIPSET__#name)

#define CODEC_EXT_CLK_RATE          9600000
#define ADSP_STATE_READY_TIMEOUT_MS 3000
#define DEV_NAME_STR_LEN            32
#define WCN_CDC_SLIM_RX_CH_MAX 2
#define WCN_CDC_SLIM_TX_CH_MAX 3

#ifdef LPASS_BE_PRI_MI2S_TX
#undef LPASS_BE_PRI_MI2S_TX
#define LPASS_BE_PRI_MI2S_TX "MI2S-LPAIF_VA-TX-PRIMARY"
#endif
#ifdef LPASS_BE_PRI_MI2S_RX
#undef LPASS_BE_PRI_MI2S_RX
#define LPASS_BE_PRI_MI2S_RX "MI2S-LPAIF_VA-RX-PRIMARY"
#endif

#ifdef LPASS_BE_QUAT_MI2S_RX
#undef LPASS_BE_QUAT_MI2S_RX
#define LPASS_BE_QUAT_MI2S_RX "MI2S-LPAIF_RXTX-RX-PRIMARY"
#endif
#ifdef LPASS_BE_QUAT_MI2S_TX
#undef LPASS_BE_QUAT_MI2S_TX
#define LPASS_BE_QUAT_MI2S_TX "MI2S-LPAIF_RXTX-TX-PRIMARY"
#endif
#define BT_SLIMBUS_CLK_STR "BT SLIMBUS CLK SRC"
/* Slimbus device id for SLIMBUS_DEVICE_1 */
#define BT_SLIMBUS_DEV_ID 0

#ifdef LPASS_BE_PRI_TDM_TX_0
#undef LPASS_BE_PRI_TDM_TX_0
#define LPASS_BE_PRI_TDM_TX_0 "TDM-LPAIF_VA-TX-PRIMARY"
#endif
#ifdef LPASS_BE_PRI_TDM_RX_0
#undef LPASS_BE_PRI_TDM_RX_0
#define LPASS_BE_PRI_TDM_RX_0 "TDM-LPAIF_VA-RX-PRIMARY"
#endif
#define BT_SLIMBUS_CLK_STR "BT SLIMBUS CLK SRC"
/* Slimbus device id for SLIMBUS_DEVICE_1 */
#define BT_SLIMBUS_DEV_ID 0


struct card_status {
	int state_counter;
	int prev_state_counter;
	struct mutex lock;
};

struct msm_asoc_mach_data {
	struct snd_info_entry *codec_root;
	struct msm_common_pdata *common_pdata;
	struct device_node *dmic01_gpio_p; /* used by pinctrl API */
	struct device_node *dmic23_gpio_p; /* used by pinctrl API */
	struct device_node *us_euro_gpio_p; /* used by pinctrl API */
	struct pinctrl *usbc_en2_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en1_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en0_gpio_p; /* used by pinctrl API */
	struct device_node *fsa_handle;
	struct srcu_notifier_head *slatecom_notifier_chain;
	struct notifier_block notifier_cc_dsp_nb;
	struct card_status cs;
	bool is_standby_mode_supported;
};

enum bt_slimbus_clk_src {
	SLIMBUS_CLOCK_SRC_XO = 1,
	SLIMBUS_CLOCK_SRC_RCO = 2
};

static bool is_initial_boot;
static bool codec_reg_done;
static struct snd_soc_card snd_soc_card_monaco_msm;
static int dmic_0_1_gpio_cnt;
static int dmic_2_3_gpio_cnt;
static atomic_t bt_slim_clk_src_value = ATOMIC_INIT(SLIMBUS_CLOCK_SRC_XO);

static const char *get_domain_str(int domain)
{
	const char *domain_name = NULL;

	switch (domain) {
	case AUDIO_NOTIFIER_ADSP_DOMAIN:
		domain_name = "ADSP";
		break;
	case AUDIO_NOTIFIER_MODEM_DOMAIN:
		domain_name = "MODEM";
		break;
	case AUDIO_NOTIFIER_CC_DOMAIN:
		domain_name = "CC";
		break;
	case AUDIO_NOTIFIER_CCDSP_DOMAIN:
		domain_name = "CCDSP";
		break;
	default:
		domain_name = "UNKNOWN";
		break;
	}

	return domain_name;
}

static const char *get_opcode_str(int opcode)
{
	const char *notifier_state = NULL;

	switch (opcode) {
	case AUDIO_NOTIFIER_SERVICE_DOWN:
		notifier_state = "DOWN";
		break;
	case AUDIO_NOTIFIER_SERVICE_UP:
		notifier_state = "UP";
		break;
	default:
		notifier_state = "UNKNOWN";
		break;
	}

	return notifier_state;
}

static const char *get_snd_card_state_str(int cs)
{
	const char *card_state = NULL;

	switch (cs) {
	case SND_CARD_STATUS_OFFLINE:
		card_state = "OFFLINE";
		break;
	case SND_CARD_STATUS_ONLINE:
		card_state = "ONLINE";
		break;
	case SND_CARD_STATUS_STANDBY:
		card_state = "STANDBY";
		break;
	default:
		card_state = "INVALID";
		break;
	}

	return card_state;
}

static void check_userspace_service_state(struct snd_soc_pcm_runtime *rtd,
						struct msm_common_pdata *pdata)
{
	dev_info(rtd->card->dev,"%s: pcm_id %d state %d\n", __func__,
				rtd->num, pdata->aud_dev_state[rtd->num]);

	if (pdata->aud_dev_state[rtd->num] == DEVICE_ENABLE) {
		dev_info(rtd->card->dev, "%s userspace service crashed\n",
					__func__);
		if (pdata->dsp_sessions_closed == 0) {
			/*Issue close all graph cmd to DSP*/
			spf_core_apm_close_all();
			/*unmap all dma mapped buffers*/
			msm_audio_ion_crash_handler();
			pdata->dsp_sessions_closed = 1;
		}
		/*Reset the state as sysfs node wont be triggred*/
		pdata->aud_dev_state[rtd->num] = 0;
	}
}

static int get_intf_index(const char *stream_name)
{
	if (strnstr(stream_name, "LPAIF_VA", strlen(stream_name)))
		return PRI_MI2S_TDM_AUXPCM;
	else if (strnstr(stream_name, "LPAIF_RXTX", strlen(stream_name)))
		return QUAT_MI2S_TDM_AUXPCM;
	else {
		pr_debug("%s: stream name %s does not match\n", __func__, stream_name);
		return -EINVAL;
	}
}

static int msm_monaco_snd_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_common_pdata *pdata = msm_common_get_pdata(card);
	const char *stream_name = rtd->dai_link->stream_name;
	int index = get_intf_index(stream_name);

	dev_dbg(rtd->card->dev,
		"%s: substream = %s  stream = %d\n",
		__func__, substream->name, substream->stream);

	if (!pdata) {
		dev_err(rtd->card->dev, "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	if (index >= 0) {
		mutex_lock(&pdata->lock[index]);
		if (pdata->mi2s_gpio_p[index]) {
			if (atomic_read(&(pdata->mi2s_gpio_ref_cnt[index])) == 0) {
				ret = msm_cdc_pinctrl_select_active_state(
						pdata->mi2s_gpio_p[index]);
				if (ret) {
				  pr_err("%s:pinctrl set actve fail with %d\n",
							__func__, ret);
					goto done;
				}
			}
			atomic_inc(&(pdata->mi2s_gpio_ref_cnt[index]));
		}
done:
		mutex_unlock(&pdata->lock[index]);
	}
	return ret;
}

static void msm_monaco_snd_shutdown(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm_common_pdata *pdata = msm_common_get_pdata(card);
	const char *stream_name = rtd->dai_link->stream_name;
	int index = get_intf_index(stream_name);

	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
			substream->name, substream->stream);

	if (!pdata) {
		dev_err(card->dev, "%s: pdata is NULL\n", __func__);
		return;
	}

	check_userspace_service_state(rtd, pdata);

	if (index >= 0) {
		mutex_lock(&pdata->lock[index]);
		if (pdata->mi2s_gpio_p[index]) {
			atomic_dec(&pdata->mi2s_gpio_ref_cnt[index]);
			if (atomic_read(&pdata->mi2s_gpio_ref_cnt[index]) == 0) {
				ret = msm_cdc_pinctrl_select_sleep_state(
						pdata->mi2s_gpio_p[index]);
				if (ret)
					dev_err(card->dev,
					"%s: pinctrl set actv fail %d\n",
					__func__, ret);
			}
		}
		mutex_unlock(&pdata->lock[index]);
	}
}

/*
 * Need to report LINEIN
 * if R/L channel impedance is larger than 5K ohm
 */
static struct wcd_mbhc_config wcd_mbhc_cfg = {
	.read_fw_bin = false,
	.calibration = NULL,
	.detect_extn_cable = true,
	.mono_stero_detection = false,
	.swap_gnd_mic = NULL,
	.hs_ext_micbias = true,
	.key_code[0] = KEY_MEDIA,
	.key_code[1] = KEY_VOICECOMMAND,
	.key_code[2] = KEY_VOLUMEUP,
	.key_code[3] = KEY_VOLUMEDOWN,
	.key_code[4] = 0,
	.key_code[5] = 0,
	.key_code[6] = 0,
	.key_code[7] = 0,
	.linein_th = 5000,
	.moisture_en = false,
	.mbhc_micbias = MIC_BIAS_2,
	.anc_micbias = MIC_BIAS_2,
	.enable_anc_mic_detect = false,
	.moisture_duty_cycle_en = true,
};

static bool msm_usbc_swap_gnd_mic(struct snd_soc_component *component,
				  bool active)
{
	struct snd_soc_card *card = component->card;
	struct msm_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(card);

	if (!pdata->fsa_handle)
		return false;

	return fsa4480_switch_event(pdata->fsa_handle, FSA_MIC_GND_SWAP);
}

static bool msm_swap_gnd_mic(struct snd_soc_component *component, bool active)
{
	int value = 0;
	bool ret = false;
	struct snd_soc_card *card;
	struct msm_asoc_mach_data *pdata;

	if (!component) {
		pr_err("%s component is NULL\n", __func__);
		return false;
	}
	card = component->card;
	pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return false;

	if (wcd_mbhc_cfg.enable_usbc_analog)
		return msm_usbc_swap_gnd_mic(component, active);

	/* if usbc is not defined, swap using us_euro_gpio_p */
	if (pdata->us_euro_gpio_p) {
		value = msm_cdc_pinctrl_get_state(
				pdata->us_euro_gpio_p);
		if (value)
			msm_cdc_pinctrl_select_sleep_state(
					pdata->us_euro_gpio_p);
		else
			msm_cdc_pinctrl_select_active_state(
					pdata->us_euro_gpio_p);
		dev_dbg(component->dev, "%s: swap select switch %d to %d\n",
			__func__, value, !value);
		ret = true;
	}

	return ret;
}

static int msm_dmic_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct msm_asoc_mach_data *pdata = NULL;
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	int ret = 0;
	u32 dmic_idx;
	int *dmic_gpio_cnt;
	struct device_node *dmic_gpio;
	char  *wname;

	wname = strpbrk(w->name, "0123");
	if (!wname) {
		dev_err(component->dev, "%s: widget not found\n", __func__);
		return -EINVAL;
	}

	ret = kstrtouint(wname, 10, &dmic_idx);
	if (ret < 0) {
		dev_err(component->dev, "%s: Invalid DMIC line on the codec\n",
			__func__);
		return -EINVAL;
	}

	pdata = snd_soc_card_get_drvdata(component->card);

	switch (dmic_idx) {
	case 0:
	case 1:
		dmic_gpio_cnt = &dmic_0_1_gpio_cnt;
		dmic_gpio = pdata->dmic01_gpio_p;
		break;
	case 2:
	case 3:
		dmic_gpio_cnt = &dmic_2_3_gpio_cnt;
		dmic_gpio = pdata->dmic23_gpio_p;
		break;
	default:
		dev_err(component->dev, "%s: Invalid DMIC Selection\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(component->dev, "%s: event %d DMIC%d dmic_gpio_cnt %d\n",
			__func__, event, dmic_idx, *dmic_gpio_cnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		(*dmic_gpio_cnt)++;
		if (*dmic_gpio_cnt == 1) {
			ret = msm_cdc_pinctrl_select_active_state(
						dmic_gpio);
			if (ret < 0) {
				pr_err("%s: gpio set cannot be activated %sd",
					__func__, "dmic_gpio");
				return ret;
			}
		}

		break;
	case SND_SOC_DAPM_POST_PMD:
		(*dmic_gpio_cnt)--;
		if (*dmic_gpio_cnt == 0) {
			ret = msm_cdc_pinctrl_select_sleep_state(
					dmic_gpio);
			if (ret < 0) {
				pr_err("%s: gpio set cannot be de-activated %sd",
					__func__, "dmic_gpio");
				return ret;
			}
		}
		break;
	default:
		pr_err("%s: invalid DAPM event %d\n", __func__, event);
		return -EINVAL;
	}
	return 0;
}

static const struct snd_soc_dapm_widget msm_int_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Analog Mic1", NULL),
	SND_SOC_DAPM_MIC("Analog Mic2", NULL),
	SND_SOC_DAPM_MIC("Analog Mic3", NULL),
	SND_SOC_DAPM_MIC("Analog Mic4", NULL),
	SND_SOC_DAPM_MIC("Digital Mic0", msm_dmic_event),
	SND_SOC_DAPM_MIC("Digital Mic1", msm_dmic_event),
	SND_SOC_DAPM_MIC("Digital Mic2", msm_dmic_event),
	SND_SOC_DAPM_MIC("Digital Mic3", msm_dmic_event),
};

static int msm_int_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = -EINVAL;
	struct snd_soc_component *bolero_component = NULL;
	struct snd_soc_component *component;
	struct snd_soc_dapm_context *dapm;
	struct snd_card *card;
	struct snd_info_entry *entry;
	u8 spkleft_ports[WSA883X_MAX_SWR_PORTS] = {0, 1};
	u8 spkleft_port_types[WSA883X_MAX_SWR_PORTS] = {LO, SPKR_L_VI};
	unsigned int ch_rate[WSA883X_MAX_SWR_PORTS] = {SWR_CLK_RATE_9P6MHZ,
							SWR_CLK_RATE_1P2MHZ};
	unsigned int ch_mask[WSA883X_MAX_SWR_PORTS] = {0x1, 0x3};
	struct msm_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

	bolero_component = snd_soc_rtdcom_lookup(rtd, "bolero_codec");
	if (!bolero_component) {
		pr_err("%s: could not find component for bolero_codec\n",
			__func__);
		return ret;
	}

	dapm = snd_soc_component_get_dapm(bolero_component);

	snd_soc_dapm_new_controls(dapm, msm_int_dapm_widgets,
				ARRAY_SIZE(msm_int_dapm_widgets));

	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic0");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic1");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic2");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic3");

	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic1");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic2");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic3");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic4");

	snd_soc_dapm_sync(dapm);

	card = rtd->card->snd_card;
	if (!pdata->codec_root) {
		entry = snd_info_create_module_entry(card->module, "codecs",
						 card->proc_root);
		if (!entry) {
			pr_debug("%s: Cannot create codecs module entry\n",
				 __func__);
			ret = 0;
			goto err;
		}
		entry->mode = S_IFDIR | 0555;
		if (snd_info_register(entry) < 0) {
			snd_info_free_entry(entry);
			goto err;
		}
		pdata->codec_root = entry;
	}
	bolero_info_create_codec_entry(pdata->codec_root, bolero_component);
	bolero_register_wake_irq(bolero_component, false);

	component = snd_soc_rtdcom_lookup(rtd, BESBEV_DRV_NAME);
	if (!component)
		component = snd_soc_rtdcom_lookup(rtd, "wsa-codec.1");
	if (!component) {
		pr_err("%s component is NULL\n", __func__);
		return -EINVAL;
	}

	dapm = snd_soc_component_get_dapm(component);
	card = component->card->snd_card;

	snd_soc_dapm_ignore_suspend(dapm, "AUX");
	snd_soc_dapm_ignore_suspend(dapm, "LO");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_sync(dapm);

	if (!strncmp(component->driver->name, BESBEV_DRV_NAME,
						strlen(BESBEV_DRV_NAME))) {
		besbev_info_create_codec_entry(pdata->codec_root, component);
		bolero_set_port_map(bolero_component,
			ARRAY_SIZE(sm_port_map_besbev), sm_port_map_besbev);
		/* SWR port 7 is used for AMIC IN and VI SENSE IN,
		 * first set port map with VI SENSE port config and
		 * if amic_init is success overwrite with AMIC IN port config
		 */
		ret = besbev_amic_init(component);
		if (!ret) {
			/* For AMIC Variant Speaker Protection is not supported
			 * thus VI SENSE port config is not updated here
			 */
			bolero_set_port_map(bolero_component,
			ARRAY_SIZE(sm_port_map_besbev_amic), sm_port_map_besbev_amic);
		}
	} else if (!strncmp(component->driver->name, "wsa-codec.1",
						strlen("wsa-codec.1"))) {
		bolero_set_port_map(bolero_component,
			ARRAY_SIZE(sm_port_map_besbev), sm_port_map_besbev);
		wsa883x_set_channel_map(component, &spkleft_ports[0],
					WSA883X_MAX_SWR_PORTS, &ch_mask[0],
					&ch_rate[0], &spkleft_port_types[0]);
		wsa883x_codec_info_create_codec_entry(pdata->codec_root,
							component);
	} else {
		bolero_set_port_map(bolero_component, ARRAY_SIZE(sm_port_map),
						sm_port_map);
	}
	codec_reg_done = true;
	msm_common_dai_link_init(rtd);
	return 0;
err:
	return ret;
}

int slimbus_clock_src_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = SLIMBUS_CLOCK_SRC_XO;
	uinfo->value.integer.max = SLIMBUS_CLOCK_SRC_RCO;

	return 0;
}

int slimbus_clock_src_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = atomic_read(&bt_slim_clk_src_value);
	return 0;
}

int slimbus_clock_src_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	int ret = 0;

	if (ucontrol->value.integer.value[0] < SLIMBUS_CLOCK_SRC_XO ||
			ucontrol->value.integer.value[0] > SLIMBUS_CLOCK_SRC_RCO) {
		pr_err("%s: Invalid ctrl value %s=%d\n", __func__, BT_SLIMBUS_CLK_STR,
				ucontrol->value.integer.value[0]);
		return -EINVAL;
	}

	dev_dbg(component->dev, "%s: old_clock = %d : new_clock = %d\n", __func__,
			atomic_read(&bt_slim_clk_src_value),
			ucontrol->value.integer.value[0]);

	if (atomic_read(&bt_slim_clk_src_value) != ucontrol->value.integer.value[0]) {
		ret = audio_prm_set_slimbus_clock_src(ucontrol->value.integer.value[0],
			BT_SLIMBUS_DEV_ID);
		if (!ret) {
			atomic_set(&bt_slim_clk_src_value,
					ucontrol->value.integer.value[0]);
		} else {
			pr_err("%s: audio_prm_set_slimbus_clock_src value = %d failed : ret = %d\n",
					__func__, ucontrol->value.integer.value[0], ret);
		}
	}
	return ret;
}

static struct snd_kcontrol_new slimbus_clock_src_ctrls[1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = BT_SLIMBUS_CLK_STR,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = slimbus_clock_src_info,
		.get = slimbus_clock_src_get,
		.put = slimbus_clock_src_put,
		.private_value = 0,
	}
};

static int msm_wcn_init(struct snd_soc_pcm_runtime *rtd)
{
	unsigned int rx_ch[WCN_CDC_SLIM_RX_CH_MAX] = {157, 158};
	unsigned int tx_ch[WCN_CDC_SLIM_TX_CH_MAX]  = {159, 160};
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int ret = 0, rc = 0;
	u32 bt_slim_clk_src_ctrl = 0;

	ret = snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
					   tx_ch, ARRAY_SIZE(rx_ch), rx_ch);
	if (ret)
		return ret;

	msm_common_dai_link_init(rtd);


	rc = of_property_read_u32(rtd->card->dev->of_node,
			"qcom,bt-slim-clk-src-ctrl", &bt_slim_clk_src_ctrl);

	if (!rc && bt_slim_clk_src_ctrl) {
		ret = snd_soc_add_card_controls(rtd->card,
				slimbus_clock_src_ctrls,
				ARRAY_SIZE(slimbus_clock_src_ctrls));
		if (ret)
			dev_err(rtd->card->dev, "unable to add %s mixer control\n",
				BT_SLIMBUS_CLK_STR);
	}

	return ret;
}

static struct snd_soc_ops msm_common_be_ops = {
	.startup = msm_monaco_snd_startup,
	.shutdown = msm_monaco_snd_shutdown,
};

static struct snd_soc_dai_link msm_common_dai_links[] = {
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_0,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.init = &msm_int_audrx_init,
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(rx_dma_rx0),
	},
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_1,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_1,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(rx_dma_rx1),
	},
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_2,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_2,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(rx_dma_rx2),
	},
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_3,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_3,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(rx_dma_rx3),
	},
	{
		.name = LPASS_BE_TX_CDC_DMA_TX_3,
		.stream_name = LPASS_BE_TX_CDC_DMA_TX_3,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tx_dma_tx3),
	},
	{
		.name = LPASS_BE_TX_CDC_DMA_TX_4,
		.stream_name = LPASS_BE_TX_CDC_DMA_TX_4,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tx_dma_tx4),
	},
	{
		.name = LPASS_BE_TX_CDC_DMA_TX_5,
		.stream_name = LPASS_BE_TX_CDC_DMA_TX_5,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tx_dma_tx5),
	},
	{
		.name = LPASS_BE_VA_CDC_DMA_TX_0,
		.stream_name = LPASS_BE_VA_CDC_DMA_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(va_dma_tx0),
	},
};

static struct snd_soc_dai_link msm_wcn_be_dai_links[] = {
	{
		.name = LPASS_BE_SLIMBUS_7_RX,
		.stream_name = LPASS_BE_SLIMBUS_7_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.init = &msm_wcn_init,
		.ops = &msm_common_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(slimbus_7_rx),
	},
	{
		.name = LPASS_BE_SLIMBUS_7_TX,
		.stream_name = LPASS_BE_SLIMBUS_7_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(slimbus_7_tx),
	},
};

static struct snd_soc_dai_link msm_va_dai_links[] = {
	{
		.name = LPASS_BE_VA_CDC_CC,
		.stream_name = LPASS_BE_VA_CDC_CC,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(va_cap0),
	}
};

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
					 __func__, dai_link[i].codecs[j].name);
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
	.name		= "monaco-stub-snd-card",
};

static struct snd_soc_dai_link msm_stub_be_dai_links[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_PRI_AUXPCM_RX,
		.stream_name = LPASS_BE_PRI_AUXPCM_RX,
		.dpcm_playback = 1,
		.init = &msm_audrx_stub_init,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_PRI_AUXPCM_TX,
		.stream_name = LPASS_BE_PRI_AUXPCM_TX,
		.dpcm_playback = 1,
		.init = &msm_audrx_stub_init,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
};

static struct snd_soc_dai_link msm_stub_dai_links[
			 ARRAY_SIZE(msm_stub_be_dai_links)];

static struct snd_soc_dai_link msm_mi2s_be_dai_links[] = {
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = LPASS_BE_PRI_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(pri_mi2s_tx),
	},
	{
		.name = LPASS_BE_QUAT_MI2S_RX,
		.stream_name = LPASS_BE_QUAT_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(quat_mi2s_rx),
	},
	{
		.name = LPASS_BE_QUAT_MI2S_TX,
		.stream_name = LPASS_BE_QUAT_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(quat_mi2s_tx),
	},
};

static struct snd_soc_dai_link msm_pri_mi2s_be_dai_links[] = {
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = LPASS_BE_PRI_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(pri_mi2s_tx2),
	},
	{
		.name = LPASS_BE_PRI_MI2S_RX,
		.stream_name = LPASS_BE_PRI_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(pri_mi2s_rx),
	},
};

static struct snd_soc_dai_link msm_pri_tdm_be_dai_links[] = {
	{
		.name = LPASS_BE_PRI_TDM_TX_0,
		.stream_name = LPASS_BE_PRI_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(pri_tdm_tx),
	},
	{
		.name = LPASS_BE_PRI_TDM_RX_0,
		.stream_name = LPASS_BE_PRI_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(pri_tdm_rx),
	},
};

static struct snd_soc_dai_link msm_monaco_dai_links[
	ARRAY_SIZE(msm_common_dai_links) +
	ARRAY_SIZE(msm_va_dai_links) +
	ARRAY_SIZE(msm_mi2s_be_dai_links) +
	ARRAY_SIZE(msm_wcn_be_dai_links) +
	ARRAY_SIZE(msm_pri_mi2s_be_dai_links) +
	ARRAY_SIZE(msm_pri_tdm_be_dai_links)
];

static const struct of_device_id monaco_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,monaco-asoc-snd",
	  .data = "codec"},
	{ .compatible = "qcom,monaco-asoc-snd-stub",
	  .data = "stub_codec"},
	{},
};

static int msm_snd_card_monaco_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = NULL;

	rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[0]);
	if (!rtd) {
		dev_err(card->dev,
			"%s: snd_soc_get_pcm_runtime for %s failed!\n",
			__func__, card->dai_link[0].name);
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *dailink = NULL;
	int total_links = 0;
	int rc = 0;
	u32 val = 0;
	const struct of_device_id *match;
	const char *codec_name = NULL;

	match = of_match_node(monaco_asoc_machine_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "%s: No DT match found for sound card\n",
			__func__);
		return NULL;
	}

	if (!strcmp(match->data, "codec")) {
		card = &snd_soc_card_monaco_msm;

		rc = of_property_read_string(dev->of_node,
                        "asoc-codec-names", &codec_name);

		if (!rc && !strcmp(codec_name, "cc_codec")) {
			rc = of_property_read_u32(dev->of_node,
				"qcom,cc-va-intf-enable", &val);
			if (!rc && val) {
				dev_dbg(dev, "%s(): CC-VA support present\n",
					__func__);
				memcpy(msm_monaco_dai_links + total_links,
					msm_va_dai_links,
					sizeof(msm_va_dai_links));
				total_links += ARRAY_SIZE(msm_va_dai_links);
			}

			rc = of_property_read_u32(dev->of_node,
				"qcom,mi2s-audio-intf", &val);
			if (!rc && val) {
				dev_dbg(dev, "%s(): CC-MI2S support present\n",
					__func__);
				memcpy(msm_monaco_dai_links + total_links,
					msm_mi2s_be_dai_links,
					sizeof(msm_mi2s_be_dai_links));
				total_links += ARRAY_SIZE(msm_mi2s_be_dai_links);
			}


		} else {
			dev_dbg(dev, "%s(): Bolero support present\n",
				__func__);
			memcpy(msm_monaco_dai_links + total_links,
				msm_common_dai_links,
				sizeof(msm_common_dai_links));
			total_links += ARRAY_SIZE(msm_common_dai_links);

			rc = of_property_read_u32(dev->of_node,
				"qcom,mi2s-audio-intf", &val);
			if (!rc && val) {
				dev_dbg(dev, "%s(): MI2S support present\n",
					__func__);
				memcpy(msm_monaco_dai_links + total_links,
					msm_pri_mi2s_be_dai_links,
					sizeof(msm_pri_mi2s_be_dai_links));
				total_links += ARRAY_SIZE(msm_pri_mi2s_be_dai_links);
			}

			rc = of_property_read_u32(dev->of_node,
				"qcom,tdm-audio-intf", &val);
			if (!rc && val) {
				dev_dbg(dev, "%s(): TDM support present\n",
					__func__);
				memcpy(msm_monaco_dai_links + total_links,
					msm_pri_tdm_be_dai_links,
					sizeof(msm_pri_tdm_be_dai_links));
				total_links += ARRAY_SIZE(msm_pri_tdm_be_dai_links);
			}
		}

		// WCN BT is common for both ATH & ATH+SL variants
		rc = of_property_read_u32(dev->of_node, "qcom,wcn-btfm", &val);
		if (!rc && val) {
			dev_info(dev, "%s(): WCN BT support present\n",
				__func__);
			memcpy(msm_monaco_dai_links + total_links,
				msm_wcn_be_dai_links,
				sizeof(msm_wcn_be_dai_links));
			total_links += ARRAY_SIZE(msm_wcn_be_dai_links);
		}

		dailink = msm_monaco_dai_links;
	} else if (!strcmp(match->data, "stub_codec")) {
		dev_dbg(dev, "%s(): STUB support present\n",
			__func__);
		card = &snd_soc_card_stub_msm;

		memcpy(msm_stub_dai_links,
		       msm_stub_be_dai_links,
		       sizeof(msm_stub_be_dai_links));

		dailink = msm_stub_dai_links;
		total_links = ARRAY_SIZE(msm_stub_be_dai_links);;
	}
	if (card) {
		card->dai_link = dailink;
		card->num_links = total_links;
		card->late_probe = msm_snd_card_monaco_late_probe;
	}

	return card;
}

static void monaco_update_snd_card_status(struct msm_asoc_mach_data *pdata,
								int domain, int opcode,
								bool is_standby_mode_supported)
{
	int cur_state = SND_CARD_STATUS_INVALID;

	pr_debug("%s: Subsys-domain %s(%d), Service-opcode %s(%d)\n", __func__,
			get_domain_str(domain), domain, get_opcode_str(opcode), opcode);

	if (!pdata)
		return;

	mutex_lock(&pdata->cs.lock);
	switch (opcode) {
	case AUDIO_NOTIFIER_SERVICE_DOWN:
		pdata->cs.state_counter--;
		/**
		 * On 1st service down(ADSP SSR) event, MSM which has companion
		 * chip, API updates soundcard status as STANDBY so that userspace
		 * handles stream accordingly.
		 */
		if (pdata->cs.state_counter == SND_CARD_STATUS_OFFLINE) {
			if (domain == AUDIO_NOTIFIER_ADSP_DOMAIN &&
				is_standby_mode_supported) {
				cur_state = SND_CARD_STATUS_STANDBY;
			} else {
				cur_state = SND_CARD_STATUS_OFFLINE;
			}
		/**
		 * On 2nd service down event(When CC goes down post ADSP SSR),
		 * API updates current card state as OFFLINE.
		 */
		} else if (pdata->cs.state_counter == SND_CARD_STATUS_INVALID) {
			if (pdata->cs.prev_state_counter == SND_CARD_STATUS_STANDBY)
				cur_state = SND_CARD_STATUS_OFFLINE;
		} else {
			break;
		}

		/* Update previous card state */
		if (cur_state == SND_CARD_STATUS_OFFLINE ||
			cur_state == SND_CARD_STATUS_STANDBY) {
			pdata->cs.prev_state_counter = cur_state;
			snd_card_notify_user(cur_state);
			pr_info("%s: Sound card is in %s\n", __func__,
					get_snd_card_state_str(cur_state));
		}
		break;
	case AUDIO_NOTIFIER_SERVICE_UP:
		/**
		 * Up notification comes once for ADSP and CC/CC_DSP as part
		 * of bootup. Post bootup as part of SSR, API counts ADSP and
		 * CC/CC_DSP's notification independently.
		 */
		pdata->cs.state_counter++;
		if (pdata->cs.state_counter == SND_CARD_STATUS_ONLINE) {
			snd_card_notify_user(SND_CARD_STATUS_ONLINE);
			pr_info("%s: Sound card is in ONLINE\n", __func__);
		}
		break;
	default:
		break;
	}
	mutex_unlock(&pdata->cs.lock);
}

static int monaco_cc_dsp_notifier_service_cb(struct notifier_block *this,
					 unsigned long opcode, void *ptr)
{
	struct msm_asoc_mach_data *pdata = NULL;

	pr_debug("%s: Service opcode 0x%lx\n", __func__, opcode);

	if (!this)
		return NOTIFY_STOP;

	pdata = container_of(this, struct msm_asoc_mach_data,
							notifier_cc_dsp_nb);

	switch (opcode) {
	case DSP_ERROR:
		monaco_update_snd_card_status(pdata, AUDIO_NOTIFIER_CCDSP_DOMAIN,
								AUDIO_NOTIFIER_SERVICE_DOWN, false);
		break;
	case DSP_READY:
		monaco_update_snd_card_status(pdata, AUDIO_NOTIFIER_CCDSP_DOMAIN,
								AUDIO_NOTIFIER_SERVICE_UP, false);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int monaco_ssr_enable(struct device *dev, void *data, int domain)
{
	struct msm_asoc_mach_data *pdata = NULL;
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	pdata = snd_soc_card_get_drvdata(card);

	if (!strcmp(card->name, "monaco-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO\n", __func__);
	}

	atomic_set(&bt_slim_clk_src_value, SLIMBUS_CLOCK_SRC_XO);
	dev_dbg(dev, "%s: reset bt_slim_clk_src_value = %d, domain %d\n", __func__,
		atomic_read(&bt_slim_clk_src_value), domain);
	monaco_update_snd_card_status(pdata, domain, AUDIO_NOTIFIER_SERVICE_UP, false);

err:
	return ret;
}

static void monaco_ssr_disable(struct device *dev, void *data, int domain)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = NULL;

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		return;
	}

	pdata = snd_soc_card_get_drvdata(card);

	if (pdata)
		monaco_update_snd_card_status(pdata, domain, AUDIO_NOTIFIER_SERVICE_DOWN,
							pdata->is_standby_mode_supported);

	if (!strcmp(card->name, "monaco-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO\n", __func__);
	}
}

static const struct snd_event_ops_v2 monaco_ssr_ops = {
	.enable = monaco_ssr_enable,
	.disable = monaco_ssr_disable,
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

	ret = snd_event_master_register_v2(dev, &monaco_ssr_ops,
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
	const char *codec_name = NULL;
	const char *mbhc_audio_jack_type = NULL;
	int ret = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s: No platform supplied from device tree\n",
								__func__);
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

	ret = of_property_read_string(pdev->dev.of_node, "asoc-codec-names", &codec_name);
	if (!ret && !strcmp(codec_name, "cc_codec")) {
		pdata->is_standby_mode_supported = true;
		dev_info(&pdev->dev, "%s: Routing comes from companion chip\n", __func__);
	} else {
		ret = snd_soc_of_parse_audio_routing(card, "qcom,audio-routing");
		if (ret) {
			dev_err(&pdev->dev, "%s: parse audio routing failed, err:%d\n",
				__func__, ret);
			goto err;
		}
	}

	ret = msm_populate_dai_link_component_of_node(card);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		if (codec_reg_done)
			ret = -EINVAL;
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "%s: snd_soc_register_card failed (%d)\n",
			__func__, ret);
		goto err;
	}
	dev_info(&pdev->dev, "%s: Sound card %s registered\n",
		 __func__, card->name);

	msm_common_snd_init(pdev, card);

	pdata->hph_en1_gpio_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,hph-en1-gpio", 0);
	if (!pdata->hph_en1_gpio_p) {
		dev_dbg(&pdev->dev, "%s: property %s not detected in node %s\n",
			__func__, "qcom,hph-en1-gpio",
			pdev->dev.of_node->full_name);
	}

	pdata->hph_en0_gpio_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,hph-en0-gpio", 0);
	if (!pdata->hph_en0_gpio_p) {
		dev_dbg(&pdev->dev, "%s: property %s not detected in node %s\n",
			__func__, "qcom,hph-en0-gpio",
			pdev->dev.of_node->full_name);
	}

	ret = of_property_read_string(pdev->dev.of_node,
		"qcom,mbhc-audio-jack-type", &mbhc_audio_jack_type);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: Looking up %s property in node %s failed\n",
			__func__, "qcom,mbhc-audio-jack-type",
			pdev->dev.of_node->full_name);
		dev_dbg(&pdev->dev, "Jack type properties set to default\n");
	} else {
		if (!strcmp(mbhc_audio_jack_type, "4-pole-jack")) {
			wcd_mbhc_cfg.enable_anc_mic_detect = false;
			dev_dbg(&pdev->dev, "This hardware has 4 pole jack");
		} else if (!strcmp(mbhc_audio_jack_type, "5-pole-jack")) {
			wcd_mbhc_cfg.enable_anc_mic_detect = true;
			dev_dbg(&pdev->dev, "This hardware has 5 pole jack");
		} else if (!strcmp(mbhc_audio_jack_type, "6-pole-jack")) {
			wcd_mbhc_cfg.enable_anc_mic_detect = true;
			dev_dbg(&pdev->dev, "This hardware has 6 pole jack");
		} else {
			wcd_mbhc_cfg.enable_anc_mic_detect = false;
			dev_dbg(&pdev->dev, "Unknown value, set to default\n");
		}
	}
	/*
	 * Parse US-Euro gpio info from DT. Report no error if us-euro
	 * entry is not found in DT file as some targets do not support
	 * US-Euro detection
	 */
	pdata->us_euro_gpio_p = of_parse_phandle(pdev->dev.of_node,
					"qcom,us-euro-gpios", 0);
	if (!pdata->us_euro_gpio_p) {
		dev_dbg(&pdev->dev, "property %s not detected in node %s",
			"qcom,us-euro-gpios", pdev->dev.of_node->full_name);
	} else {
		dev_dbg(&pdev->dev, "%s detected\n",
			"qcom,us-euro-gpios");
		wcd_mbhc_cfg.swap_gnd_mic = msm_swap_gnd_mic;
	}

	if (wcd_mbhc_cfg.enable_usbc_analog)
		wcd_mbhc_cfg.swap_gnd_mic = msm_usbc_swap_gnd_mic;

	pdata->fsa_handle = of_parse_phandle(pdev->dev.of_node,
					"fsa4480-i2c-handle", 0);
	if (!pdata->fsa_handle)
		dev_dbg(&pdev->dev, "property %s not detected in node %s\n",
			"fsa4480-i2c-handle", pdev->dev.of_node->full_name);

	pdata->dmic01_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic01-gpios",
					       0);
	pdata->dmic23_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic23-gpios",
					       0);
	mutex_init(&pdata->cs.lock);
	/**
	 * By default state is OFFLINE, once ADSP is up post registration
	 * to snd event, state_counter moves to ONLINE using
	 * monaco_update_snd_card_status API.
	 */
	pdata->cs.state_counter = SND_CARD_STATUS_OFFLINE;

	ret = msm_audio_ssr_register(&pdev->dev);
	if (ret)
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",
			__func__, ret);

	pdata->notifier_cc_dsp_nb.notifier_call  = monaco_cc_dsp_notifier_service_cb;
	pdata->notifier_cc_dsp_nb.priority = 0;
	pdata->slatecom_notifier_chain =
		(struct srcu_notifier_head *)slatecom_register_notifier(
			&pdata->notifier_cc_dsp_nb);
	is_initial_boot = true;
	return 0;
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
	{
		common_pdata = pdata->common_pdata;
		slatecom_unregister_notifier(pdata->slatecom_notifier_chain,
			&pdata->notifier_cc_dsp_nb);
		if (common_pdata)
			msm_common_snd_deinit(common_pdata);

	}
	snd_event_master_deregister(&pdev->dev);
	if (card)
		snd_soc_unregister_card(card);

	if (pdata) {
		pdata->cs.state_counter = SND_CARD_STATUS_OFFLINE;
		mutex_destroy(&pdata->cs.lock);
	}

	return 0;
}

static struct platform_driver monaco_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = monaco_asoc_machine_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_asoc_machine_probe,
	.remove = msm_asoc_machine_remove,
};

static int __init msm_asoc_machine_init(void)
{
	snd_card_sysfs_init();
	return platform_driver_register(&monaco_asoc_machine_driver);
}
module_init(msm_asoc_machine_init);

static void __exit msm_asoc_machine_exit(void)
{
	platform_driver_unregister(&monaco_asoc_machine_driver);
}
module_exit(msm_asoc_machine_exit);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, monaco_asoc_machine_of_match);
