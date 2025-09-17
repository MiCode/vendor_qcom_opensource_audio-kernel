// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/component.h>
#include <soc/soundwire.h>

struct besbev_slave_priv {
	struct swr_device *swr_slave;
};

static int besbev_slave_bind(struct device *dev,
				struct device *master, void *data)
{
	int ret = 0;
	struct besbev_slave_priv *besbev_slave = NULL;
	uint8_t devnum = 0;
	struct swr_device *pdev = to_swr_device(dev);

	if (pdev == NULL) {
		dev_err(dev, "%s: pdev is NULL\n", __func__);
		return -EINVAL;
	}

	besbev_slave = devm_kzalloc(&pdev->dev,
				sizeof(struct besbev_slave_priv), GFP_KERNEL);
	if (!besbev_slave)
		return -ENOMEM;

	swr_set_dev_data(pdev, besbev_slave);

	besbev_slave->swr_slave = pdev;

	ret = swr_get_logical_dev_num(pdev, pdev->addr, &devnum);
	if (ret) {
		dev_dbg(&pdev->dev,
				"%s get devnum %d for dev addr %lx failed\n",
				__func__, devnum, pdev->addr);
		swr_remove_device(pdev);
		return ret;
	}
	pdev->dev_num = devnum;

	return ret;
}

static void besbev_slave_unbind(struct device *dev,
				struct device *master, void *data)
{
	struct besbev_slave_priv *besbev_slave = NULL;
	struct swr_device *pdev = to_swr_device(dev);

	if (pdev == NULL) {
		dev_err(dev, "%s: pdev is NULL\n", __func__);
		return;
	}

	besbev_slave = swr_get_dev_data(pdev);
	if (!besbev_slave) {
		dev_err(&pdev->dev, "%s: besbev_slave is NULL\n", __func__);
		return;
	}

	swr_set_dev_data(pdev, NULL);
}

static const struct swr_device_id besbev_swr_id[] = {
	{"besbev-slave", 0},
	{}
};

static const struct of_device_id besbev_swr_dt_match[] = {
	{
		.compatible = "qcom,besbev-slave",
	},
	{}
};

static const struct component_ops besbev_slave_comp_ops = {
	.bind   = besbev_slave_bind,
	.unbind = besbev_slave_unbind,
};

static int besbev_swr_up(struct swr_device *pdev)
{
	return 0;
}

static int besbev_swr_down(struct swr_device *pdev)
{
	return 0;
}

static int besbev_swr_reset(struct swr_device *pdev)
{
	return 0;
}

static int besbev_swr_probe(struct swr_device *pdev)
{
	return component_add(&pdev->dev, &besbev_slave_comp_ops);
}

static int besbev_swr_remove(struct swr_device *pdev)
{
	component_del(&pdev->dev, &besbev_slave_comp_ops);
	return 0;
}

static struct swr_driver besbev_slave_driver = {
	.driver = {
		.name = "besbev-slave",
		.owner = THIS_MODULE,
		.of_match_table = besbev_swr_dt_match,
	},
	.probe = besbev_swr_probe,
	.remove = besbev_swr_remove,
	.id_table = besbev_swr_id,
	.device_up = besbev_swr_up,
	.device_down = besbev_swr_down,
	.reset_device = besbev_swr_reset,
};

static int __init besbev_slave_init(void)
{
	return swr_driver_register(&besbev_slave_driver);
}

static void __exit besbev_slave_exit(void)
{
	swr_driver_unregister(&besbev_slave_driver);
}

module_init(besbev_slave_init);
module_exit(besbev_slave_exit);

MODULE_DESCRIPTION("Besbev Slave driver");
MODULE_LICENSE("GPL v2");
