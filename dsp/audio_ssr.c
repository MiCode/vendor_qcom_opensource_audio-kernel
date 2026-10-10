// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016, 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/qcom_rproc.h>
#include "audio_ssr.h"

static char *audio_ssr_domains[] = {
	"lpass",
	"mpss",
	"slatefw"
};

/**
 * audio_ssr_register -
 *        register to SSR framework
 *
 * @domain_id: Domain ID to register with
 * @nb: notifier block
 *
 * Returns handle pointer on success or error PTR on failure
 */
void *audio_ssr_register(int domain_id, struct notifier_block *nb)
{
	if ((domain_id < 0) ||
		(domain_id >= AUDIO_SSR_DOMAIN_MAX)) {
		pr_err("%s: Invalid domain ID %d\n", __func__, domain_id);
		return ERR_PTR(-EINVAL);
	}

	return qcom_register_ssr_notifier(audio_ssr_domains[domain_id], nb);
}
EXPORT_SYMBOL(audio_ssr_register);

/**
 * audio_ssr_deregister -
 *        Deregister handle from SSR framework
 *
 * @handle: SSR handle
 * @nb: notifier block
 *
 * Returns 0 on success or error on failure
 */
int audio_ssr_deregister(void *handle, struct notifier_block *nb)
{
	return qcom_unregister_ssr_notifier(handle, nb);
}
EXPORT_SYMBOL(audio_ssr_deregister);

