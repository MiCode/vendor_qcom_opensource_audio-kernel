// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/workqueue.h>

struct coupled_ssr_private {
	const char *ssr_name;
	struct rproc *ssr_rproc;
	void *ssr_notify_handler;
	struct list_head list;
};

struct coupled_ssr_private_list {
	struct list_head priv_list;
};

static struct coupled_ssr_private_list coupled_ssr_list = {0,};

static int strlcmp(const char *s, const char *t, size_t n)
{
	while (n-- && *t != '\0') {
		if (*s != *t) {
			return ((unsigned char)*s - (unsigned char)*t);
		} else {
			++s, ++t;
		}
	}
	return (unsigned char)*s;
}

static int ssr_notifier_general(struct notifier_block *this,
					 unsigned long opcode, void *ptr);

static struct notifier_block ssr_notifier_general_nb = {
	.notifier_call  = ssr_notifier_general,
	.priority = -INT_MAX,
};


/* main entry for all subsystem ssr events */
static int ssr_notifier_general(struct notifier_block *nb,
					 unsigned long opcode, void *ptr)
{
	struct coupled_ssr_private *coupled_ssr_data_priv = NULL;
	struct qcom_ssr_notify_data *data = ptr;
	size_t ssr_name_len = 0;

	pr_debug("%s: ssr name %s opcode 0x%lx \n", __func__, data->name, opcode);

	/*
	 * restart dsps in serial order to avoid coccurency of gpr remove and probe
	 */
	if (data->crashed && opcode == QCOM_SSR_BEFORE_SHUTDOWN) {
		pr_debug("%s: coupled ssr start from %s\n", __func__, data->name);
		list_for_each_entry(coupled_ssr_data_priv,
				&coupled_ssr_list.priv_list, list) {
			ssr_name_len = strlen(coupled_ssr_data_priv->ssr_name);
			if (strlcmp(data->name, coupled_ssr_data_priv->ssr_name, ssr_name_len) &&
				coupled_ssr_data_priv->ssr_rproc) {
				rproc_shutdown(coupled_ssr_data_priv->ssr_rproc);
				rproc_boot(coupled_ssr_data_priv->ssr_rproc);
			}
		}
	}

	return NOTIFY_OK;
}

static void coupled_ssr_list_update(struct coupled_ssr_private *coupled_ssr_data)
{
	struct coupled_ssr_private *coupled_ssr_data_priv = NULL;

	list_for_each_entry(coupled_ssr_data_priv,
			&coupled_ssr_list.priv_list, list) {
		if (coupled_ssr_data_priv->ssr_rproc == coupled_ssr_data->ssr_rproc) {
			pr_err("%s dsp already present, not updating the list",
				__func__);
			return;
		}
	}
	list_add_tail(&coupled_ssr_data->list, &coupled_ssr_list.priv_list);
}

static void coupled_ssr_cleanup(void) {
	struct coupled_ssr_private *coupled_ssr_data_priv = NULL;

	list_for_each_entry(coupled_ssr_data_priv,
			&coupled_ssr_list.priv_list, list) {
		if (coupled_ssr_data_priv->ssr_notify_handler) {
			qcom_unregister_ssr_notifier(coupled_ssr_data_priv->ssr_notify_handler,
						&ssr_notifier_general_nb);
		}
		coupled_ssr_data_priv->ssr_rproc = NULL;
	}
}

static int coupled_ssr_remove(struct platform_device *pdev)
{
	coupled_ssr_cleanup();
	return 0;
}

static int coupled_ssr_probe(struct platform_device *pdev)
{
	int rproc_count = 0;
	int i;
	phandle rproc_phandle;
	struct device_node *rproc_device_node = NULL;
	struct device_node *np = NULL;
	struct rproc *ssr_rproc = NULL;
	void *ssr_notify_handler = NULL;
	const char *subsys_ssr_name = NULL;
	struct coupled_ssr_private* coupled_ssr_data = NULL;

	rproc_count = of_count_phandle_with_args(pdev->dev.of_node,
				"qcom,msm-coupled-ssr-group",
				NULL);
	if (!rproc_count) {
		dev_err(&pdev->dev, "Missing coupled remoteproc\n");
		return -ENOPARAM;
	}
	INIT_LIST_HEAD(&coupled_ssr_list.priv_list);

	for (i = 0; i < rproc_count; i++) {
		of_property_read_u32_index(pdev->dev.of_node,
					"qcom,msm-coupled-ssr-group",
					i,
					&rproc_phandle);

		ssr_rproc = rproc_get_by_phandle(rproc_phandle);
		if (!ssr_rproc) {
			dev_err(&pdev->dev, "%s: fail to get rproc\n", __func__);
			coupled_ssr_cleanup();
			return -EPROBE_DEFER;
		}

		rproc_device_node = of_find_node_by_phandle(rproc_phandle);
		for_each_child_of_node(rproc_device_node, np) {
			if (!strlcmp("glink-edge", np->name, strlen(np->name))) {
				/* glink-labal should be identical with ssr name */
				of_property_read_string(np, "qcom,glink-label", &subsys_ssr_name);
			}
		}

		if (!subsys_ssr_name) {
			dev_err(&pdev->dev, "missing ssr name from glink-label for rproc %s\n", ssr_rproc->name);
			return -ENOPARAM;
		}

		ssr_notify_handler = qcom_register_ssr_notifier(subsys_ssr_name,
						&ssr_notifier_general_nb);
		if (!ssr_notify_handler) {
			dev_err(&pdev->dev, "%s fails to register notifier for %s\n",
						__func__, subsys_ssr_name);
			return -ENOPARAM;
		}

		coupled_ssr_data = devm_kzalloc(&pdev->dev, (sizeof(struct coupled_ssr_private)),
                                          GFP_KERNEL);
		if (!coupled_ssr_data) {
			dev_err(&pdev->dev, "%s: no memory\n", __func__);
			return -ENOMEM;
		}
		coupled_ssr_data->ssr_name = subsys_ssr_name;
		coupled_ssr_data->ssr_rproc = ssr_rproc;
		coupled_ssr_data->ssr_notify_handler = ssr_notify_handler;
		coupled_ssr_list_update(coupled_ssr_data);
	}

	return 0;
}

static const struct of_device_id coupled_ssr_dt_match[] = {
	{ .compatible = "qcom,coupled_ssr" },
	{ }
};
MODULE_DEVICE_TABLE(of, coupled_ssr_dt_match);

static struct platform_driver coupled_ssr_driver = {
	.driver = {
		.name = "coupled_ssr",
		.owner = THIS_MODULE,
		.of_match_table = coupled_ssr_dt_match,
	},
	.probe = coupled_ssr_probe,
	.remove = coupled_ssr_remove,
};

static int __init coupled_ssr_init(void)
{
	return platform_driver_register(&coupled_ssr_driver);
}
module_init(coupled_ssr_init);

static void __exit coupled_ssr_exit(void)
{
	platform_driver_unregister(&coupled_ssr_driver);
}
module_exit(coupled_ssr_exit);

MODULE_DESCRIPTION("coupled ssr module");
MODULE_LICENSE("GPL v2");
