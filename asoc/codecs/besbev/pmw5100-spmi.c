// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

/**
 * @regmap: regmap used to access PMIC registers
 */
struct pmw5100_spmi {
	struct regmap *regmap;
};

static const struct of_device_id pmw5100_id_table[] = {
	{ .compatible = "qcom,pm5100-spmi" },
	{ },
};
MODULE_DEVICE_TABLE(of, pmw5100_id_table);

/**
 * pmw5100_spmi_write: Function to write PMIC register
 * @device: node for besbev device
 * @reg: PMIC register to write value
 * @value: Value of reg to write
 */
int pmw5100_spmi_write(struct device *dev, int reg, int value)
{
	int rc;
	struct pmw5100_spmi *spmi_dd;

	if (!of_device_is_compatible(dev->of_node, "qcom,pm5100-spmi")) {
		pr_err("%s: Device node is invalid\n", __func__);
		return -EINVAL;
	}

	spmi_dd = dev_get_drvdata(dev);
	if (!spmi_dd)
		return -EINVAL;

	rc = regmap_write(spmi_dd->regmap, reg, value);
	if (rc)
		dev_err(dev, "%s: Write to PMIC register failed\n", __func__);

	return rc;
}
EXPORT_SYMBOL(pmw5100_spmi_write);

/**
 * pmw5100_spmi_read: Function to read PMIC register
 * @device: node for besbev device
 * @reg: PMIC register to read value
 * @value: Pointer to value of reg to be read
 */
int pmw5100_spmi_read(struct device *dev, int reg, int *value)
{
	int rc;
	struct pmw5100_spmi *spmi_dd;

	if (!of_device_is_compatible(dev->of_node, "qcom,pm5100-spmi")) {
		pr_err("%s: Device node is invalid\n", __func__);
		return -EINVAL;
	}

	spmi_dd = dev_get_drvdata(dev);
	if (!spmi_dd)
		return -EINVAL;

	rc = regmap_read(spmi_dd->regmap, reg, value);
	if (rc)
		dev_err(dev, "%s: Read from PMIC register failed\n", __func__);

	return rc;
}
EXPORT_SYMBOL(pmw5100_spmi_read);

static int pmw5100_spmi_probe(struct platform_device *pdev)
{
	struct pmw5100_spmi *spmi_dd;
	const struct of_device_id *match;

	match = of_match_node(pmw5100_id_table, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	spmi_dd = devm_kzalloc(&pdev->dev, sizeof(*spmi_dd), GFP_KERNEL);
	if (spmi_dd == NULL)
		return -ENOMEM;

	spmi_dd->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!spmi_dd->regmap) {
		dev_err(&pdev->dev, "%s: Parent regmap unavailable.\n", __func__);
		return -ENXIO;
	}

	platform_set_drvdata(pdev, spmi_dd);

	dev_dbg(&pdev->dev, "%s: Probe success !!\n", __func__);

	return 0;
}

static int pmw5100_spmi_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	return 0;
}

static struct platform_driver pmw5100_spmi_driver = {
	.probe		= pmw5100_spmi_probe,
	.remove		= pmw5100_spmi_remove,
	.driver	= {
		.name		= "pm5100-spmi",
		.of_match_table	= pmw5100_id_table,
	},
};
module_platform_driver(pmw5100_spmi_driver);

MODULE_ALIAS("platform:pmw5100-spmi");
MODULE_DESCRIPTION("PMIC SPMI driver");
MODULE_LICENSE("GPL v2");
