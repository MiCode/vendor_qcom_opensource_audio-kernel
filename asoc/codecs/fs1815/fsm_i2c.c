/*
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 */
#include "fsm_public.h"
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#if defined(CONFIG_REGULATOR)
#include <linux/regulator/consumer.h>
static struct regulator *g_fsm_vdd = NULL;
#endif

static DEFINE_MUTEX(g_fsm_mutex);
static struct device *g_fsm_pdev = NULL;
static int g_probe_complete_flag = 0;

/* customize configrature */
#include "fsm_firmware.c"
#include "fsm_misc.c"
//#include "fsm_codec.c"

void fsm_mutex_lock()
{
	mutex_lock(&g_fsm_mutex);
}

void fsm_mutex_unlock()
{
	mutex_unlock(&g_fsm_mutex);
}

int fsm_i2c_reg_read(fsm_dev_t *fsm_dev, uint8_t reg, uint16_t *pVal)
{
	struct i2c_msg msgs[2];
	uint8_t retries = 0;
	uint8_t buffer[2];
	int ret;

	if (!fsm_dev || !fsm_dev->i2c || !pVal) {
		return -EINVAL;
	}

	// write register address.
	msgs[0].addr = fsm_dev->i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	// read register buffer.
	msgs[1].addr = fsm_dev->i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 2;
	msgs[1].buf = &buffer[0];

	do {
		mutex_lock(&fsm_dev->i2c_lock);
		ret = i2c_transfer(fsm_dev->i2c->adapter, &msgs[0], ARRAY_SIZE(msgs));
		mutex_unlock(&fsm_dev->i2c_lock);
		if (ret != ARRAY_SIZE(msgs)) {
			fsm_delay_ms(5);
			retries++;
		}
	} while (ret != ARRAY_SIZE(msgs) && retries < FSM_I2C_RETRY);

	if (ret != ARRAY_SIZE(msgs)) {
		pr_err("read %02x transfer error: %d", reg, ret);
		return -EIO;
	}

	*pVal = ((buffer[0] << 8) | buffer[1]);

	return 0;
}

int fsm_i2c_reg_write(fsm_dev_t *fsm_dev, uint8_t reg, uint16_t val)
{
	struct i2c_msg msgs[1];
	uint8_t retries = 0;
	uint8_t buffer[3];
	int ret;

	if (!fsm_dev || !fsm_dev->i2c) {
		return -EINVAL;
	}

	buffer[0] = reg;
	buffer[1] = (val >> 8) & 0x00ff;
	buffer[2] = val & 0x00ff;
	msgs[0].addr = fsm_dev->i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(buffer);
	msgs[0].buf = &buffer[0];

	do {
		mutex_lock(&fsm_dev->i2c_lock);
		ret = i2c_transfer(fsm_dev->i2c->adapter, &msgs[0], ARRAY_SIZE(msgs));
		mutex_unlock(&fsm_dev->i2c_lock);
		if (ret != ARRAY_SIZE(msgs)) {
			fsm_delay_ms(5);
			retries++;
		}
	} while (ret != ARRAY_SIZE(msgs) && retries < FSM_I2C_RETRY);

	if (ret != ARRAY_SIZE(msgs)) {
		pr_err("write %02x transfer error: %d", reg, ret);
		return -EIO;
	}

	return 0;
}

int fsm_i2c_bulkwrite(fsm_dev_t *fsm_dev, uint8_t reg,
				uint8_t *data, int len)
{
	uint8_t retries = 0;
	uint8_t *buf;
	int size;
	int ret;

	if (!fsm_dev || !fsm_dev->i2c || !data) {
		return -EINVAL;
	}

	size = sizeof(uint8_t) + len;
	buf = (uint8_t *)fsm_alloc_mem(size);
	if (!buf) {
		pr_err("alloc memery failed");
		return -ENOMEM;
	}

	buf[0] = reg;
	memcpy(&buf[1], data, len);
	do {
		mutex_lock(&fsm_dev->i2c_lock);
		ret = i2c_master_send(fsm_dev->i2c, buf, size);
		mutex_unlock(&fsm_dev->i2c_lock);
		if (ret < 0) {
			fsm_delay_ms(5);
			retries++;
		} else if (ret == size) {
			break;
		}
	} while (ret != size && retries < FSM_I2C_RETRY);

	fsm_free_mem((void **)&buf);

	if (ret != size) {
		pr_err("write %02x transfer error: %d", reg, ret);
		return -EIO;
	}

	return 0;
}

bool fsm_set_pdev(struct device *dev)
{
	if (g_fsm_pdev == NULL || dev == NULL) {
		g_fsm_pdev = dev;
		// pr_debug("dev_name: %s", dev_name(dev));
		return true;
	}
	return false; // already got device
}

struct device *fsm_get_pdev(void)
{
	return g_fsm_pdev;
}

int fsm_vddd_on(struct device *dev)
{
	fsm_config_t *cfg = fsm_get_config();
	int ret = 0;

	if (!cfg || cfg->vddd_on) {
		return 0;
	}
#if defined(CONFIG_REGULATOR)
	g_fsm_vdd = regulator_get(dev, "fsm_vddd");
	if (IS_ERR(g_fsm_vdd) != 0) {
		pr_err("error getting fsm_vddd regulator");
		ret = PTR_ERR(g_fsm_vdd);
		g_fsm_vdd = NULL;
		return ret;
	}
	pr_info("enable regulator");
	regulator_set_voltage(g_fsm_vdd, 1800000, 1800000);
	ret = regulator_enable(g_fsm_vdd);
	if (ret < 0) {
		pr_err("enabling fsm_vddd failed: %d", ret);
	}
#endif
	cfg->vddd_on = 1;
	fsm_delay_ms(10);

	return ret;
}

void fsm_vddd_off(void)
{
	fsm_config_t *cfg = fsm_get_config();

	if (!cfg || !cfg->vddd_on || cfg->dev_count > 0) {
		return;
	}
#if defined(CONFIG_REGULATOR)
	if (g_fsm_vdd) {
		pr_info("disable regulator");
		regulator_disable(g_fsm_vdd);
		regulator_put(g_fsm_vdd);
		g_fsm_vdd = NULL;
	}
#endif
	cfg->vddd_on = 0;
}

int fsm_get_amb_tempr(void)
{
	union power_supply_propval psp = { 0 };
	struct power_supply *psy;
	int tempr = FSM_DFT_AMB_TEMPR;
	int vbat = FSM_DFT_AMB_VBAT;

	psy = power_supply_get_by_name("battery");
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	if (psy && psy->get_property) {
		// battery temperatrue
		psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &psp);
		tempr = DIV_ROUND_CLOSEST(psp.intval, 10);
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &psp);
		vbat = DIV_ROUND_CLOSEST(psp.intval, 1000);
	}
#else
	if (psy && psy->desc && psy->desc->get_property) {
		// battery temperatrue
		psy->desc->get_property(psy, POWER_SUPPLY_PROP_TEMP, &psp);
		tempr = DIV_ROUND_CLOSEST(psp.intval, 10);
		psy->desc->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &psp);
		vbat = DIV_ROUND_CLOSEST(psp.intval, 1000);
	}
#endif
	pr_info("vbat:%d, tempr:%d", vbat, tempr);

	return tempr;
}

void *fsm_devm_kstrdup(struct device *dev, void *buf, size_t size)
{
	char *devm_buf = devm_kzalloc(dev, size + 1, GFP_KERNEL);

	if (!devm_buf) {
		return devm_buf;
	}
	memcpy(devm_buf, buf, size);

	return devm_buf;
}

static int fsm_set_irq(fsm_dev_t *fsm_dev, bool enable)
{
	if (!fsm_dev || fsm_dev->irq_id <= 0) {
		return -EINVAL;
	}
	if (enable)
		enable_irq(fsm_dev->irq_id);
	else
		disable_irq(fsm_dev->irq_id);

	return 0;
}

int fsm_set_monitor(fsm_dev_t *fsm_dev, bool enable)
{
	fsm_config_t *cfg = fsm_get_config();

	if (!cfg || !fsm_dev || !fsm_dev->fsm_wq) {
		return -EINVAL;
	}
	if (!cfg->use_monitor) {
		return 0;
	}
	if (fsm_dev->use_irq) {
		return fsm_set_irq(fsm_dev, enable);
	}
	if (enable) {
		queue_delayed_work(fsm_dev->fsm_wq,
				&fsm_dev->monitor_work, 5*HZ);
	}
	else {
		if (delayed_work_pending(&fsm_dev->monitor_work)) {
			cancel_delayed_work_sync(&fsm_dev->monitor_work);
		}
	}

	return 0;
}

static int fsm_ext_reset(fsm_dev_t *fsm_dev)
{
	fsm_config_t *cfg = fsm_get_config();
	uint16_t id = 0;
	int ret;

	if (!cfg || !fsm_dev) {
		return -EINVAL;
	}
	if (cfg->reset_chip) {
		return 0;
	}
	if (gpio_is_valid(fsm_dev->rst_gpio)) {
		gpio_set_value(fsm_dev->rst_gpio, 0);
		fsm_delay_ms(10); // mdelay
		gpio_set_value(fsm_dev->rst_gpio, 1);
		fsm_delay_ms(1); // mdelay
		// TODO: for sharing reset pin in multi-pa application
		// cfg->reset_chip = true;
	}
	fsm_mutex_lock();
	/* soft reset */
	fsm_dev->addr = fsm_dev->i2c->addr;
	ret = fsm_reg_read(fsm_dev, 0x01, &id);
	if (ret && fsm_dev->addr != 0x34) {
		fsm_dev->i2c->addr = 0x34;
		ret = fsm_reg_read(fsm_dev, 0x01, &id);
	}
	if (!ret && LOW8(id) == FS1815_DEV_ID) {
		fsm_reg_write(fsm_dev, 0x10, 0x0002);
		fsm_delay_ms(15);
	} else {
		ret = -ENODEV;
	}
	fsm_dev->i2c->addr = fsm_dev->addr;
	fsm_dev->addr = 0;
	fsm_mutex_unlock();

	return 0;
}

static irqreturn_t fsm_irq_hander(int irq, void *data)
{
	fsm_dev_t *fsm_dev = data;

	queue_delayed_work(fsm_dev->fsm_wq, &fsm_dev->interrupt_work, 0);

	return IRQ_HANDLED;
}

static void fsm_work_monitor(struct work_struct *work)
{
	fsm_config_t *cfg = fsm_get_config();
	fsm_dev_t *fsm_dev;
	//int ret;

	fsm_dev = container_of(work, struct fsm_dev, monitor_work.work);
	if (!cfg || cfg->skip_monitor || !fsm_dev) {
		return;
	}
	fsm_mutex_lock();
	//ret = fsm_dev_recover(fsm_dev);
	fsm_mutex_unlock();
	if (fsm_dev->rec_count >= 5) { // 5 time max
		pr_addr(debug, "recover max time, stop it");
		return;
	}
	/* reschedule */
	queue_delayed_work(fsm_dev->fsm_wq, &fsm_dev->monitor_work,
			5*HZ);

}

static void fsm_work_interrupt(struct work_struct *work)
{
	fsm_config_t *cfg = fsm_get_config();
	fsm_dev_t *fsm_dev;
	//int ret;

	fsm_dev = container_of(work, struct fsm_dev, interrupt_work.work);
	if (!cfg || cfg->skip_monitor || !fsm_dev) {
		return;
	}
	fsm_mutex_lock();
	//ret = fsm_dev_recover(fsm_dev);
	//fsm_get_spkr_tempr(fsm_dev);

	fsm_mutex_unlock();
}

static int fsm_request_irq(fsm_dev_t *fsm_dev)
{
	struct i2c_client *i2c;
	int irq_flags;
	int ret;

	if (fsm_dev == NULL || fsm_dev->i2c == NULL) {
		return -EINVAL;
	}
	fsm_dev->irq_id = -1;
	if (!fsm_dev->use_irq || !gpio_is_valid(fsm_dev->irq_gpio)) {
		pr_addr(info, "skip to request irq");
		return 0;
	}
	i2c = fsm_dev->i2c;
	/* register irq handler */
	fsm_dev->irq_id = gpio_to_irq(fsm_dev->irq_gpio);
	if (fsm_dev->irq_id <= 0) {
		dev_err(&i2c->dev, "invalid irq %d\n", fsm_dev->irq_id);
		return -EINVAL;
	}
	irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	ret = devm_request_threaded_irq(&i2c->dev, fsm_dev->irq_id,
				NULL, fsm_irq_hander, irq_flags, "fs16xx", fsm_dev);
	if (ret) {
		dev_err(&i2c->dev, "failed to request IRQ %d: %d\n",
				fsm_dev->irq_id, ret);
		return ret;
	}
	disable_irq(fsm_dev->irq_id);

	return 0;
}

static int fsm_i2c_pre_check(fsm_dev_t *fsm_dev)
{
	uint16_t id = 0;
	uint16_t temp_addr = 0;
	int ret;

	if (!fsm_dev) {
		return -EINVAL;
	}

	fsm_mutex_lock();
	temp_addr = fsm_dev->i2c->addr;
	ret = fsm_reg_read(fsm_dev, 0x01, &id);
	if (ret && (temp_addr != 0x34)) {
		fsm_dev->i2c->addr = 0x34;
		ret = fsm_reg_read(fsm_dev, 0x01, &id);
	}
	if (ret) {
		ret = -ENODEV;
		dev_err(&fsm_dev->i2c->dev, "precheck fsm devcie failed");
	}
	fsm_dev->i2c->addr = temp_addr;
	fsm_mutex_unlock();

	return ret;
}

#ifdef CONFIG_OF
static int fsm_parse_dts(struct i2c_client *i2c, fsm_dev_t *fsm_dev)
{
	struct device_node *np = i2c->dev.of_node;
	char *str_name;
	int ret;

	if (fsm_dev == NULL || np == NULL) {
		return -EINVAL;
	}

	str_name = devm_kzalloc(&i2c->dev, 32, GFP_KERNEL);
	fsm_dev->rst_gpio = of_get_named_gpio(np, "fsm,rst-gpio", 0);
	if (gpio_is_valid(fsm_dev->rst_gpio)) {
		snprintf(str_name, 32, "FS1815_SDZ_%02X", i2c->addr);
		ret = devm_gpio_request_one(&i2c->dev, fsm_dev->rst_gpio,
			GPIOF_OUT_INIT_LOW, str_name);
		if (ret)
			return ret;
	}

	return 0;
}

static struct of_device_id fsm_match_tbl[] =
{
	{ .compatible = "foursemi,fs16xx" },
	{ .compatible = "foursemi,fs1815" },
	{},
};
MODULE_DEVICE_TABLE(of, fsm_match_tbl);
#endif

static int fsm_i2c_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	fsm_config_t *cfg = fsm_get_config();
	fsm_dev_t *fsm_dev;
	int ret = 0;

	pr_debug("enter");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check I2C_FUNC_I2C failed");
		g_probe_complete_flag = -EIO;
		return -EIO;
	}

	fsm_dev = devm_kzalloc(&i2c->dev, sizeof(struct fsm_dev), GFP_KERNEL);
	if (fsm_dev == NULL) {
		dev_err(&i2c->dev, "alloc memory fialed");
		g_probe_complete_flag = -ENOMEM;
		return -ENOMEM;
	}

	memset(fsm_dev, 0, sizeof(struct fsm_dev));
	mutex_init(&fsm_dev->i2c_lock);
	fsm_dev->i2c = i2c;

	ret = fsm_i2c_pre_check(fsm_dev);
	if (ret) {
		devm_kfree(&i2c->dev, fsm_dev);
		dev_err(&i2c->dev, " fsm pre check failed");
		g_probe_complete_flag = ret;
		return ret;
	}

#ifdef CONFIG_OF
	ret = fsm_parse_dts(i2c, fsm_dev);
	if (ret) {
		devm_kfree(&i2c->dev, fsm_dev);
		dev_err(&i2c->dev, "failed to parse DTS node");
		g_probe_complete_flag = ret;
		return ret;
	}
#endif
#if defined(CONFIG_FSM_REGMAP)
	fsm_dev->regmap = fsm_regmap_i2c_init(i2c);
	if (fsm_dev->regmap == NULL) {
		devm_kfree(&i2c->dev, fsm_dev);
		dev_err(&i2c->dev, "regmap init fialed");
		g_probe_complete_flag = -EINVAL;
		return -EINVAL;
	}
#endif

	fsm_vddd_on(&i2c->dev);
	fsm_ext_reset(fsm_dev);
	ret = fsm_probe(fsm_dev, i2c->addr);
	if (ret) {
		dev_err(&i2c->dev, "detect device failed");
#if defined(CONFIG_FSM_REGMAP)
		fsm_regmap_i2c_deinit(fsm_dev->regmap);
#endif
		gpio_set_value(fsm_dev->rst_gpio, 0);
		devm_gpio_free(&i2c->dev, fsm_dev->rst_gpio);
		devm_kfree(&i2c->dev, fsm_dev);
		g_probe_complete_flag = ret;
		return ret;
	}
	fsm_dev->id = cfg->dev_count - 1;
	i2c_set_clientdata(i2c, fsm_dev);
	pr_addr(info, "index:%d", fsm_dev->id);
	fsm_dev->fsm_wq = create_singlethread_workqueue("fs16xx");
	INIT_DELAYED_WORK(&fsm_dev->monitor_work, fsm_work_monitor);
	INIT_DELAYED_WORK(&fsm_dev->interrupt_work, fsm_work_interrupt);
	fsm_request_irq(fsm_dev);

	if(fsm_dev->id == 0) {
		// reigster only in the first device
#if !defined(CONFIG_FSM_CODEC)
		fsm_set_pdev(&i2c->dev);
#endif
		fsm_misc_init();
		fsm_sysfs_init(&i2c->dev);
		fsm_codec_register(&i2c->dev, fsm_dev->id);
	}

	dev_info(&i2c->dev, "i2c probe completed");

	g_probe_complete_flag = 1;
	return 0;
}

int get_fsm_probe_complete_flag(void)
{
	return g_probe_complete_flag;
}
EXPORT_SYMBOL(get_fsm_probe_complete_flag);

static int fsm_i2c_remove(struct i2c_client *i2c)
{
	fsm_dev_t *fsm_dev = i2c_get_clientdata(i2c);

	pr_debug("enter");
	if (fsm_dev == NULL) {
		pr_err("bad parameter");
		return -EINVAL;
	}
	if (fsm_dev->fsm_wq) {
		cancel_delayed_work_sync(&fsm_dev->interrupt_work);
		cancel_delayed_work_sync(&fsm_dev->monitor_work);
		destroy_workqueue(fsm_dev->fsm_wq);
	}
#if defined(CONFIG_FSM_REGMAP)
	fsm_regmap_i2c_deinit(fsm_dev->regmap);
#endif
	if (fsm_dev->id == 0) {
		fsm_codec_unregister(&i2c->dev);
		fsm_sysfs_deinit(&i2c->dev);
		fsm_misc_deinit();
		fsm_set_pdev(NULL);
	}

	fsm_remove(fsm_dev);
	fsm_vddd_off();
	if (gpio_is_valid(fsm_dev->irq_gpio)) {
		devm_gpio_free(&i2c->dev, fsm_dev->irq_gpio);
	}
	if (gpio_is_valid(fsm_dev->rst_gpio)) {
		devm_gpio_free(&i2c->dev, fsm_dev->rst_gpio);
	}
	devm_kfree(&i2c->dev, fsm_dev);
	dev_info(&i2c->dev, "i2c removed");

	return 0;
}

static void fsm_i2c_shutdown(struct i2c_client *i2c)
{
	fsm_dev_t *fsm_dev = i2c_get_clientdata(i2c);

	pr_debug("enter");
	if (fsm_dev == NULL) {
		pr_err("bad parameter");
		return;
	}

	fsm_stub_shut_down(fsm_dev);
	if (gpio_is_valid(fsm_dev->rst_gpio))
		gpio_set_value(fsm_dev->rst_gpio, 0);

	dev_info(&i2c->dev, "i2c shutdowned");
}

static const struct i2c_device_id fsm_i2c_id[] =
{
	{ "fs16xx", 0 },
	{ "fs1815", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fsm_i2c_id);

static struct i2c_driver fsm_i2c_driver =
{
	.driver = {
		.name  = FSM_DRV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(fsm_match_tbl),
#endif
	},
	.probe    = fsm_i2c_probe,
	.remove   = fsm_i2c_remove,
	.shutdown = fsm_i2c_shutdown,
	.id_table = fsm_i2c_id,
};

int exfsm_i2c_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	return fsm_i2c_probe(i2c, id);
}
EXPORT_SYMBOL(exfsm_i2c_probe);

int exfsm_i2c_remove(struct i2c_client *i2c)
{
	return fsm_i2c_remove(i2c);
}
EXPORT_SYMBOL(exfsm_i2c_remove);

int fsm_i2c_init(void)
{
	return i2c_add_driver(&fsm_i2c_driver);
}

void fsm_i2c_exit(void)
{
	i2c_del_driver(&fsm_i2c_driver);
}

static int __init fsm_mod_init(void)
{
	int ret;

	ret = fsm_i2c_init();
	if (ret) {
		pr_err("init fail: %d", ret);
		return ret;
	}

	return 0;
}

static void __exit fsm_mod_exit(void)
{
	fsm_i2c_exit();
}

//module_i2c_driver(fsm_i2c_driver);
module_init(fsm_mod_init);
module_exit(fsm_mod_exit);

MODULE_AUTHOR("FourSemi SW <support@foursemi.com>");
MODULE_DESCRIPTION("FourSemi Smart PA Driver");
MODULE_VERSION(FSM_CODE_VERSION);
MODULE_ALIAS("foursemi:"FSM_DRV_NAME);
MODULE_LICENSE("GPL");

