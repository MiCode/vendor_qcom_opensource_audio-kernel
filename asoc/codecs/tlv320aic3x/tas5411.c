/*
 * 1. Audio driver for the TI TAS5411-Q1 automotive class-D amplifier.
 * 2. No front end，so no need to register Asoc component
 * 3. Perfect gpio operation
 * 4. Perfect read and write register interface
 * 5. Call each other with tlv320
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <sound/soc.h>

#include "tas5411.h"

struct tas5411_data *tas5411_priv = NULL;

static const char * const mute_pctl_names[] = {
	"tas_mute_io_high",
	"tas_mute_io_low",
};

const struct reg_default tas5411_reg[] = {
	{ TAS5411_REG_FALTE_STATUS,			0x00 },
	{ TAS5411_REG_DIAGNOSTIC,			0x00 },
	{ TAS5411_REG_CONTROL,				0x78 },
};

const struct regmap_config tas5411_regmap = {
	.max_register = TAS5411_REG_CONTROL,
	.reg_defaults = tas5411_reg,
	.num_reg_defaults = ARRAY_SIZE(tas5411_reg),

	.cache_type = REGCACHE_RBTREE,
};
EXPORT_SYMBOL_GPL(tas5411_regmap);

/* tas5411 dev date keep private, only Open interface to aic3x codec */
int tas5411_pinctrl_controller(bool amp_mute_status) {
	if (tas5411_priv == NULL)
		return -1;

	if (tas5411_priv->tas5411_mute_pinctrl && tas5411_priv->enable_mute_pin_control) {
		if (amp_mute_status) {
			dev_info(tas5411_priv->dev, "%s: amp_mute_status %d, amp mute gpio output high\n", __func__, amp_mute_status);
			pinctrl_select_state(tas5411_priv->tas5411_mute_pinctrl,
					tas5411_priv->pinctrl_state[0]);
		} else {
			dev_info(tas5411_priv->dev, "%s: amp_mute_status %d, amp mute gpio output low\n", __func__, amp_mute_status);
			pinctrl_select_state(tas5411_priv->tas5411_mute_pinctrl,
					tas5411_priv->pinctrl_state[1]);
		}
	}

	return 0;
}

int tas5411_i2c_writes(struct tas5411_data *tas5411,
	unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data = NULL;

	data = kmalloc(len+1, GFP_KERNEL);
	if (data == NULL) {
		dev_err(tas5411->dev, "can not allocate memory");
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(tas5411->client, data, len+1);
	if (ret < 0)
		dev_err(tas5411->dev, "i2c master send error");

	kfree(data);

	return ret;
}

int tas5411_i2c_reads(struct tas5411_data *tas5411,
	unsigned char reg_addr, unsigned char *data_buf, unsigned int data_len)
{
	int ret;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = tas5411->client->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = tas5411->client->addr,
			.flags = I2C_M_RD,
			.len = data_len,
			.buf = data_buf,
			},
	};

	ret = i2c_transfer(tas5411->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(tas5411->dev, "transfer failed.");
		return ret;
	}

	return 0;
}

int tas5411_i2c_write(struct tas5411_data *tas5411,
	unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char buf[1];

	buf[0] = reg_data;

	while (cnt < AMP_I2C_RETRIES) {
		ret = tas5411_i2c_writes(tas5411, reg_addr, buf, 1);
		if (ret < 0) {
			dev_err(tas5411->dev, "i2c_write cnt=%d error=%d",
					cnt, ret);
		} else {
			dev_info(tas5411->dev, "reg_addr: 0x%02x, reg_data :0x%02x",
						(uint8_t)reg_addr, (uint8_t)reg_data);
			break;
		}
		cnt++;
	}

	if(ret < 0){
		dev_err(tas5411->dev, "retray 5 times,still error,try usleep 3s, i2c_write cnt=%d error=%d",
					cnt, ret);
		usleep_range(AMP_RETRY_WAIT_TIME, AMP_RETRY_WAIT_TIME + 100);
		ret = tas5411_i2c_writes(tas5411, reg_addr, buf, 1);
		if (ret < 0) {
			dev_err(tas5411->dev, "usleep 3s still ereror, i2c_write cnt=%d error=%d",
					cnt, ret);
		}
	}

	return ret;
}

int tas5411_i2c_read(struct tas5411_data *tas5411,
	unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char buf[1];

	while (cnt < AMP_I2C_RETRIES) {
		ret = tas5411_i2c_reads(tas5411, reg_addr, buf, 1);
		if (ret < 0) {
			dev_err(tas5411->dev, "i2c_read cnt=%d error=%d",
						cnt, ret);
		} else {
			dev_err(tas5411->dev, "reg_addr: 0x%02x, reg_data :0x%02x, buf[0] :0x%02x",
						(uint8_t)reg_addr, (uint8_t)(*reg_data), buf[0]);
			*reg_data = (buf[0]<<0);
			dev_err(tas5411->dev, "reg_addr: 0x%02x, reg_data :0x%02x, buf[0] :0x%02x",
						(uint8_t)reg_addr, (uint8_t)(*reg_data), buf[0]);
			break;
		}
		cnt++;
	}
	if(ret < 0) {
		dev_err(tas5411->dev, "retray 5 times,still error,try usleep 3s, i2c_write cnt=%d error=%d",
					cnt, ret);
		usleep_range(AMP_RETRY_WAIT_TIME, AMP_RETRY_WAIT_TIME + 100);
		ret = tas5411_i2c_reads(tas5411, reg_addr, buf, 1);
		if (ret < 0) {
			dev_err(tas5411->dev, "usleep 3s still ereror, i2c_read cnt=%d error=%d",
						cnt, ret);
		}
	}

	return ret;
}

static ssize_t tas5411_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tas5411_data *tas5411 = dev_get_drvdata(dev);
	unsigned char databuf[2] = {0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		dev_info(tas5411->dev, "buf %s databuf[0] %#x, databuf[1] %#x", buf, databuf[0], databuf[1]);
		tas5411_i2c_write(tas5411, databuf[0], databuf[1]);
	}

	return count;
}

static ssize_t tas5411_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tas5411_data *tas5411 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 1;
	unsigned int reg_val = 0;
	int reg_num = AMP_ADJUT_REG;

	for (i = 1; i <= reg_num; i++) {
		tas5411_i2c_read(tas5411, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

int tas5411_pa_mute_switch_init(struct tas5411_data *tas5411)
{
	int ret = 0, i =0;

	tas5411->enable_mute_pin_control = 0;
	tas5411->tas5411_mute_pinctrl = devm_pinctrl_get(tas5411->dev);
	if (IS_ERR(tas5411->tas5411_mute_pinctrl)) {
		if (PTR_ERR(tas5411->tas5411_mute_pinctrl) == -EPROBE_DEFER) {
			dev_err(tas5411->dev, "pinctrl not ready\n");
			ret = -EPROBE_DEFER;
			return ret;
		}
		dev_err(tas5411->dev, "Target does not use pinctrl\n");
		tas5411->tas5411_mute_pinctrl = NULL;
		ret = -EINVAL;
		return ret;
	}
	for (i = 0; i < ARRAY_SIZE(mute_pctl_names); i++) {
		const char *n = mute_pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(tas5411->tas5411_mute_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(tas5411->dev, "cannot find '%s'\n", n);
			ret = -EINVAL;
			//goto exit;
			return ret;
		}
		tas5411->pinctrl_state[i] = state;
		tas5411->enable_mute_pin_control = 1;
		dev_err(tas5411->dev, "found mute pin control %s, enable_mute_pin_control %d\n", n, tas5411->enable_mute_pin_control);
	}
	return ret;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
	tas5411_reg_show, tas5411_reg_store);

static struct attribute *tas5411_attributes[] = {
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group tas5411_attribute_group = {
	.attrs = tas5411_attributes,
};

static int tas5411_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct regmap_config amp_config;
	int ret = 0;
	struct tas5411_data *tas5411;

	dev_info(&client->dev, "%s: tas5411 i2c probe E, addr = %x\n", __func__, client->addr);

	/* Allocate driver data */
	tas5411 = devm_kzalloc(&client->dev, sizeof(struct tas5411_data),
			       GFP_KERNEL);
	if (!tas5411)
		return -ENOMEM;

	/* Initialize client driver structure */
	tas5411->client = client;
	tas5411->dev = &client->dev;

	/* regmap config */
	amp_config = tas5411_regmap;
	amp_config.reg_bits = 8;
	amp_config.val_bits = 8;
	i2c_set_clientdata(client, tas5411);
	tas5411->regmap = devm_regmap_init_i2c(client, &amp_config);
	dev_info(tas5411->dev, "allocate register map done\n");
	if (IS_ERR(tas5411->regmap)) {
		ret = PTR_ERR(tas5411->regmap);
		dev_err(tas5411->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	/* create attr */
	ret = sysfs_create_group(&client->dev.kobj, &tas5411_attribute_group);
	if (ret < 0) {
		dev_err(tas5411->dev, "error creating sysfs attr files");
	}

	/* 1. accroding to i2c devive parse dtsi
	   2. gpio control in tlv320 driver use mixer kcontrol implement */
	tas5411_priv = tas5411;
	tas5411_pa_mute_switch_init(tas5411_priv);

	/* 14-V peak output */
	regmap_write(tas5411_priv->regmap, TAS5411_REG_CONTROL, 0x70);

	dev_info(&client->dev, "%s: tas5411 i2c probe X, ret %d\n", __func__, ret);

	return ret;
}

static void tas5411_i2c_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "%s: tas5411 i2c remove\n", __func__);
	/*rm attr node*/
	sysfs_remove_group(&client->dev.kobj, &tas5411_attribute_group);
	return;
}

/* TBD */
static const struct i2c_device_id tas5411_i2c_id[] = {
	{ "tas5411", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tas5411_i2c_id);

/* match dtsi */
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
};
// module_i2c_driver(tas5411_i2c_driver);

// Driver Enter
int __init tas5411_i2c_init(void)
{
	int ret = -1;

	ret = i2c_add_driver(&tas5411_i2c_driver);
	printk("add tas5411 device into i2c, ret = %d\n", ret);
	if (ret)
		printk("fail to add tas5411 device into i2c\n");

	return ret;
}
// module_init(tas5411_i2c_init);

// Driver Exit
void __exit tas5411_i2c_exit(void)
{
	i2c_del_driver(&tas5411_i2c_driver);
}
// module_exit(tas5411_i2c_exit);

MODULE_DESCRIPTION("TAS5411 PA driver Register R/W & GPIO control");
MODULE_AUTHOR("N/A");
MODULE_LICENSE("GPL");
