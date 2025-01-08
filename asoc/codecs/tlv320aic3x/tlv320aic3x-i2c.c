/* SPDX-License-Identifier: GPL-2.0-only
 *
 * ALSA SoC TLV320AIC3x codec driver I2C interface
 *
 * Author:      Arun KS, <arunks@mistralsolutions.com>
 * Copyright:   (C) 2008 Mistral Solutions Pvt Ltd.,
 *
 * Based on sound/soc/codecs/wm8731.c by Richard Purdie
 *
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "tlv320aic3x.h"
#include "tas5411.h"

/* aic3x driver start*/
static int aic3x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct regmap_config config;

	dev_info(&i2c->dev, "%s: aic3x i2c probe, addr = %x\n", __func__, i2c->addr);

	config = aic3x_regmap;
	config.reg_bits = 8;
	config.val_bits = 8;

	regmap = devm_regmap_init_i2c(i2c, &config);
	return aic3x_probe(&i2c->dev, regmap, id->driver_data);
}

static void aic3x_i2c_remove(struct i2c_client *i2c)
{
	aic3x_remove(&i2c->dev);
	return;
}

static const struct i2c_device_id aic3x_i2c_id[] = {
	{ "tlv320aic3x", AIC3X_MODEL_3X },
	{ "tlv320aic33", AIC3X_MODEL_33 },
	{ "tlv320aic3007", AIC3X_MODEL_3007 },
	{ "tlv320aic3104", AIC3X_MODEL_3104 },
	{ "tlv320aic3106", AIC3X_MODEL_3106 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aic3x_i2c_id);

static const struct of_device_id aic3x_of_id[] = {
	{ .compatible = "ti,tlv320aic3x", },
	{ .compatible = "ti,tlv320aic33" },
	{ .compatible = "ti,tlv320aic3007" },
	{ .compatible = "ti,tlv320aic3104" },
	{ .compatible = "ti,tlv320aic3106" },
	{},
};

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops tlv320aic3x_dev_pm_ops = {
	.suspend = aic3x_suspend,
	.resume = aic3x_resume,
};
#endif

MODULE_DEVICE_TABLE(of, aic3x_of_id);

static struct i2c_driver aic3x_i2c_driver = {
	.driver = {
		.name = "tlv320aic3x",
		.of_match_table = aic3x_of_id,
#ifdef CONFIG_PM_SLEEP
 		.pm = &tlv320aic3x_dev_pm_ops,
#endif
	},
	.probe = aic3x_i2c_probe,
	.remove = aic3x_i2c_remove,
	.id_table = aic3x_i2c_id,
}; /* aic3x driver end*/

/* tas5411 driver start*/
static int tas5411_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	dev_info(&i2c->dev, "%s: tas5411 i2c probe, addr = %x\n", __func__, i2c->addr);
	return tas5411_probe(i2c, id);
}

static void tas5411_i2c_remove(struct i2c_client *i2c)
{
	tas5411_remove(i2c);
	return;
}

static const struct i2c_device_id tas5411_i2c_id[] = {
	{ "tas5411", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tas5411_i2c_id);

static struct of_device_id tas5411_dt_match[] = {
	{ .compatible = "ti,tas5411" },
	{ },
};
MODULE_DEVICE_TABLE(of, tas5411_dt_match);

static struct i2c_driver tas5411_i2c_driver = {
	.driver = {
		.name		= "tas5411",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tas5411_dt_match),
	},
	.id_table	= tas5411_i2c_id,
	.probe		= tas5411_i2c_probe,
	.remove		= tas5411_i2c_remove,
}; /* tas5411 driver end*/

/* Driver Enter */
static int __init aic3x_tas_i2c_init(void)
{
	int ret = -1;
	printk("add tas5411 device into i2c\n");
	ret = i2c_add_driver(&tas5411_i2c_driver);
	if (ret) {
		printk(KERN_ERR "fail to add tas5411 device into i2c, ret = %d\n", ret);
		return ret;
	}
	printk("add tlv320aic3x device into i2c\n");
	ret = i2c_add_driver(&aic3x_i2c_driver);
	if (ret) {
		printk(KERN_ERR "fail to add aic3x device into i2c, ret = %d\n", ret);
		i2c_del_driver(&tas5411_i2c_driver);
	}
	printk("tlv and tas i2c init success\n");

	return ret;
}
module_init(aic3x_tas_i2c_init);

/* Driver Exit */
static void __exit aic3x_tas_i2c_exit(void)
{
	i2c_del_driver(&aic3x_i2c_driver);
	i2c_del_driver(&tas5411_i2c_driver);
}
module_exit(aic3x_tas_i2c_exit);


MODULE_DESCRIPTION("ASoC TLV320AIC3x codec driver I2C");
MODULE_AUTHOR("Arun KS <arunks@mistralsolutions.com>");
MODULE_LICENSE("GPL");
