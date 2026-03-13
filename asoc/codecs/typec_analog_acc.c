// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024, Xiaomi Corporation, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/usb/ucsi_glink.h>
#include <linux/usb/typec.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include "typec_analog_acc.h"

/**
 * need match kernel define
 * use SND_JACK_VIDEOOUT for unsupported bit
 */
#define JACK_TYPEC_ANALOG_MASK (SND_JACK_VIDEOOUT | SND_JACK_HEADSET)

struct typec_analog_acc_priv {
    int current_jack_type;
    struct notifier_block ucsi_nb;
    struct snd_soc_jack typec_analog_jack;
};

static struct typec_analog_acc_priv *acc_priv = NULL;

static void typec_analog_acc_jack_report(struct snd_soc_jack *jack, int status, int mask)
{
	pr_info("%s: enter, jack->status: %d, status: %d, mask: %d\n", __func__, jack->status, status, mask);
	snd_soc_jack_report(jack, status, mask);
}

static int typec_analog_acc_event_changed(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct typec_analog_acc_priv *priv = NULL;
	enum typec_accessory acc = TYPEC_ACCESSORY_NONE;

	pr_info("%s: enter\n", __func__);

	priv = container_of(nb, struct typec_analog_acc_priv, ucsi_nb);
	if (!priv) {
		pr_err("%s: priv is NULL\n", __func__);
	}

	acc = ((struct ucsi_glink_constat_info *)ptr)->acc;

	pr_info("%s: acc = %d", __func__, acc);

	switch (acc) {
	case TYPEC_ACCESSORY_AUDIO:
		if (priv->current_jack_type != TYPEC_ACCESSORY_AUDIO) {
			typec_analog_acc_jack_report(&priv->typec_analog_jack,
			            (SND_JACK_HEADSET | SND_JACK_VIDEOOUT), JACK_TYPEC_ANALOG_MASK);
			priv->current_jack_type = TYPEC_ACCESSORY_AUDIO;
		}
		break;
	case TYPEC_ACCESSORY_NONE:
		if (priv->current_jack_type != TYPEC_ACCESSORY_NONE) {
			typec_analog_acc_jack_report(&priv->typec_analog_jack, 0, JACK_TYPEC_ANALOG_MASK);
			priv->current_jack_type = TYPEC_ACCESSORY_NONE;
		}
		break;
	default:
		break;
	}

	pr_info("%s: exit\n", __func__);
	return 0;
}

int typec_analog_acc_init(struct snd_soc_component *component)
{
	int ret = 0;

	pr_info("%s: enter\n", __func__);

	acc_priv = kzalloc(sizeof(struct typec_analog_acc_priv), GFP_KERNEL);
	if (!acc_priv) {
		pr_err("%s: acc_priv is NULL\n", __func__);
	}

	acc_priv->ucsi_nb.notifier_call = typec_analog_acc_event_changed;
	acc_priv->ucsi_nb.priority = 0;
	ret = register_ucsi_glink_notifier(&acc_priv->ucsi_nb);
	if (ret) {
		pr_err("%s: ucsi glink notifier registration failed: %d\n", __func__, ret);
		goto err;
	}

	ret = snd_soc_card_jack_new(component->card, "Typec_analog Jack",
	            JACK_TYPEC_ANALOG_MASK, &acc_priv->typec_analog_jack);
	if (ret) {
		pr_err("%s: Failed to create new jack Typec_analog Jack: %d\n", __func__, ret);
		goto err;
	}

	pr_info("%s: exit\n", __func__);

err:
	return ret;
}
EXPORT_SYMBOL(typec_analog_acc_init);

static int __init analog_acc_init(void)
{
	return 0;
}

static void __exit analog_acc_exit(void)
{
	kfree(acc_priv);
}

module_init(analog_acc_init);
module_exit(analog_acc_exit);

MODULE_DESCRIPTION("type analog accessory module");
MODULE_LICENSE("GPL v2");