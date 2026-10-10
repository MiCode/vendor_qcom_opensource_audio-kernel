/*
 * oca72xxx_device.c  oca72xxx pa module
 *
 * Copyright (c) 2025 OCS Technology CO., LTD
 *
 * Author: Wall <Wall@orient-chip.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/timer.h>
#include "ocspa.h"
#include "ocspa_device.h"
#include "ocspa_log.h"
#include "oca72390_11_reg.h"
#include "oca72559_09_reg.h"

/*************************************************************************
 * oca72xxx variable
 ************************************************************************/
const char *g_oca_pid_09_product[] = {
	"oca72559",
};

const char *g_oca_pid_11_product[] = {
	"oca72390",
};

static int oca72xxx_dev_get_chipid(struct oca_device *oca_dev);

/***************************************************************************
 *
 * reading and writing of I2C bus
 *
 ***************************************************************************/
int oca72xxx_dev_i2c_write_byte(struct oca_device *oca_dev,
			uint8_t reg_addr, uint8_t reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < OCA_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(oca_dev->i2c, reg_addr, reg_data);
		if (ret < 0) {
			OCA_DEV_LOGE(oca_dev->dev, "i2c_write cnt=%d error=%d",
				cnt, ret);
			if (oca_dev->i2c->addr == 0x58) {
				oca_dev->i2c->addr = 0x5A;
				OCA_DEV_LOGE(oca_dev->dev, "i2c_write i2c addr 0x5A");
			} else if (oca_dev->i2c->addr == 0x5A) {
				oca_dev->i2c->addr = 0x58;
				OCA_DEV_LOGE(oca_dev->dev, "i2c_write i2c addr 0x58");
			}
		} else {
			break;
                }
		cnt++;
		msleep(OCA_I2C_RETRY_DELAY);
	}

	return ret;
}

int oca72xxx_dev_i2c_read_byte(struct oca_device *oca_dev,
			uint8_t reg_addr, uint8_t *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < OCA_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(oca_dev->i2c, reg_addr);
		if (ret < 0) {
			OCA_DEV_LOGE(oca_dev->dev, "i2c_read cnt=%d error=%d",
				cnt, ret);
			if (oca_dev->i2c->addr == 0x58) {
				oca_dev->i2c->addr = 0x5A;
				OCA_DEV_LOGE(oca_dev->dev, "i2c_read i2c addr 0x5A");
			} else if (oca_dev->i2c->addr == 0x5A) {
				oca_dev->i2c->addr = 0x58;
				OCA_DEV_LOGE(oca_dev->dev, "i2c_read i2c addr 0x58");
			}
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(OCA_I2C_RETRY_DELAY);
	}

	return ret;
}

int oca72xxx_dev_i2c_read_msg(struct oca_device *oca_dev,
	uint8_t reg_addr, uint8_t *data_buf, uint32_t data_len)
{
	int ret = -1;

	struct i2c_msg msg[] = {
	[0] = {
		.addr = oca_dev->i2c_addr,
		.flags = 0,
		.len = sizeof(uint8_t),
		.buf = &reg_addr,
		},
	[1] = {
		.addr = oca_dev->i2c_addr,
		.flags = I2C_M_RD,
		.len = data_len,
		.buf = data_buf,
		},
	};

	ret = i2c_transfer(oca_dev->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		OCA_DEV_LOGE(oca_dev->dev, "transfer failed");
		return ret;
	} else if (ret != OCA_I2C_READ_MSG_NUM) {
		OCA_DEV_LOGE(oca_dev->dev, "transfer failed(size error)");
		return -ENXIO;
	}

	return 0;
}

int oca72xxx_dev_i2c_write_bits(struct oca_device *oca_dev,
	uint8_t reg_addr, uint8_t mask, uint8_t reg_data)
{
	int ret = -1;
	unsigned char reg_val = 0;

	ret = oca72xxx_dev_i2c_read_byte(oca_dev, reg_addr, &reg_val);
	if (ret < 0) {
		OCA_DEV_LOGE(oca_dev->dev, "i2c read error, ret=%d", ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = oca72xxx_dev_i2c_write_byte(oca_dev, reg_addr, reg_val);
	if (ret < 0) {
		OCA_DEV_LOGE(oca_dev->dev, "i2c write error, ret=%d", ret);
		return ret;
	}

	return 0;
}

/************************************************************************
 *
 * oca72xxx device update profile data to registers
 *
 ************************************************************************/
static int oca72xxx_dev_reg_update(struct oca_device *oca_dev,
			struct oca_data_container *profile_data)
{
	int i = 0;
	int ret = -1;

	if (profile_data == NULL)
		return -EINVAL;

	if (oca_dev->hwen_status == OCA_DEV_HWEN_OFF) {
		OCA_DEV_LOGE(oca_dev->dev, "dev is pwr_off,can not update reg");
		return -EINVAL;
	}

	for (i = 0; i < profile_data->len; i = i + 2) {
		OCA_DEV_LOGD(oca_dev->dev, "reg=0x%02x, val = 0x%02x",
			profile_data->data[i], profile_data->data[i + 1]);

		ret = oca72xxx_dev_i2c_write_byte(oca_dev, profile_data->data[i],
				profile_data->data[i + 1]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/************************************************************************
 *
 * oca72xxx device hadware and soft contols
 *
 ************************************************************************/
static bool oca72xxx_dev_gpio_is_valid(struct oca_device *oca_dev)
{
	if (gpio_is_valid(oca_dev->rst_gpio))
		return true;
	else
		return false;
}

void oca72xxx_dev_hw_pwr_ctrl(struct oca_device *oca_dev, bool enable)
{
	if (oca_dev->hwen_status == OCA_DEV_HWEN_INVALID) {
		OCA_DEV_LOGD(oca_dev->dev, "product not have reset-pin,hardware pwd control invalid");
		return;
	}
	if (enable) {
		if (oca72xxx_dev_gpio_is_valid(oca_dev)) {
			gpio_set_value_cansleep(oca_dev->rst_gpio, OCA_GPIO_LOW_LEVEL);
			mdelay(2);
			gpio_set_value_cansleep(oca_dev->rst_gpio, OCA_GPIO_HIGHT_LEVEL);
			mdelay(2);
			oca_dev->hwen_status = OCA_DEV_HWEN_ON;
			OCA_DEV_LOGI(oca_dev->dev, "hw power on");
		} else {
			OCA_DEV_LOGI(oca_dev->dev, "hw already power on");
		}
	} else {
		if (oca72xxx_dev_gpio_is_valid(oca_dev)) {
			gpio_set_value_cansleep(oca_dev->rst_gpio, OCA_GPIO_LOW_LEVEL);
			mdelay(2);
			oca_dev->hwen_status = OCA_DEV_HWEN_OFF;
			OCA_DEV_LOGI(oca_dev->dev, "hw power off");
		} else {
			OCA_DEV_LOGI(oca_dev->dev, "hw already power off");
		}
	}
}

void oca72xxx_dev_soft_reset(struct oca_device *oca_dev)
{
	int i = 0;
	int ret = -1;
	struct oca_soft_rst_desc *soft_rst = &oca_dev->soft_rst_desc;

	OCA_DEV_LOGD(oca_dev->dev, "enter");

	if (oca_dev->hwen_status == OCA_DEV_HWEN_OFF) {
		OCA_DEV_LOGE(oca_dev->dev, "hw is off,can not softrst");
		return;
	}

	if (oca_dev->soft_rst_enable == OCA_DEV_SOFT_RST_DISENABLE) {
		OCA_DEV_LOGD(oca_dev->dev, "softrst is disenable");
		return;
	}

	if (soft_rst->access == NULL || soft_rst->len == 0) {
		OCA_DEV_LOGE(oca_dev->dev, "softrst_info not init");
		return;
	}

	if (soft_rst->len % 2) {
		OCA_DEV_LOGE(oca_dev->dev, "softrst data_len[%d] is odd number,data not available",
			oca_dev->soft_rst_desc.len);
		return;
	}

	for (i = 0; i < soft_rst->len; i += 2) {
		OCA_DEV_LOGD(oca_dev->dev, "softrst_reg=0x%02x, val = 0x%02x",
			soft_rst->access[i], soft_rst->access[i + 1]);

		ret = oca72xxx_dev_i2c_write_byte(oca_dev, soft_rst->access[i],
				soft_rst->access[i + 1]);
		if (ret < 0) {
			OCA_DEV_LOGE(oca_dev->dev, "write failed,ret = %d,cnt=%d",
				ret, i);
			return;
		}
	}
	OCA_DEV_LOGD(oca_dev->dev, "down");
}


int oca72xxx_dev_default_pwr_off(struct oca_device *oca_dev,
		struct oca_data_container *profile_data)
{
	int ret = 0;

	OCA_DEV_LOGD(oca_dev->dev, "enter");
	if (oca_dev->hwen_status == OCA_DEV_HWEN_OFF) {
		OCA_DEV_LOGE(oca_dev->dev, "hwen is already off");
		return 0;
	}

	if (oca_dev->soft_off_enable && profile_data) {
		ret = oca72xxx_dev_reg_update(oca_dev, profile_data);
		if (ret < 0) {
			OCA_DEV_LOGE(oca_dev->dev, "update profile[Off] fw config failed");
			goto reg_off_update_failed;
		}
	}

	oca72xxx_dev_hw_pwr_ctrl(oca_dev, false);
	OCA_DEV_LOGD(oca_dev->dev, "down");
	return 0;

reg_off_update_failed:
	oca72xxx_dev_hw_pwr_ctrl(oca_dev, false);
	return ret;
}


/************************************************************************
 *
 * oca72xxx device power on process function
 *
 ************************************************************************/

int oca72xxx_dev_default_pwr_on(struct oca_device *oca_dev,
			struct oca_data_container *profile_data)
{
	int ret = 0;

	/*hw power on*/
	oca72xxx_dev_hw_pwr_ctrl(oca_dev, true);

	ret = oca72xxx_dev_reg_update(oca_dev, profile_data);
	if (ret < 0)
		return ret;

	return 0;
}

/****************************************************************************
 *
 * oca72xxx product attributes init info
 *
 ****************************************************************************/

/********************** oca72xxx_pid_09 attributes ****************************/
static void oca_dev_chipid_09_init(struct oca_device *oca_dev)
{
	/* Product register permission info */
	oca_dev->reg_max_addr = OCA72XXX_PID_09_REG_MAX;
	oca_dev->reg_access = oca72xxx_pid_09_reg_access;

	/* software reset control info */
	oca_dev->soft_rst_desc.len = sizeof(oca72xxx_pid_09_softrst_access);
	oca_dev->soft_rst_desc.access = oca72xxx_pid_09_softrst_access;
	oca_dev->soft_rst_enable = OCA_DEV_SOFT_RST_ENABLE;

	/* Whether to allow register operation to power off */
	oca_dev->soft_off_enable = OCA_DEV_SOFT_OFF_ENABLE;

	oca_dev->product_tab = g_oca_pid_09_product;
	oca_dev->product_cnt = OCA72XXX_PID_09_PRODUCT_MAX;

	oca_dev->vol_desc.addr = OCA_REG_NONE;
}
/********************** oca72xxx_pid_09 attributes end ************************/

/********************** oca72xxx_pid_11 attributes ****************************/
static void oca_dev_chipid_11_init(struct oca_device *oca_dev)
{
	/* Product register permission info */
	oca_dev->reg_max_addr = OCA72XXX_PID_11_REG_MAX;
	oca_dev->reg_access = oca72xxx_pid_11_reg_access;

	/* software reset control info */
	oca_dev->soft_rst_desc.len = sizeof(oca72xxx_pid_11_softrst_access);
	oca_dev->soft_rst_desc.access = oca72xxx_pid_11_softrst_access;
	oca_dev->soft_rst_enable = OCA_DEV_SOFT_RST_ENABLE;

	/* software power off control info */
	oca_dev->soft_off_enable = OCA_DEV_SOFT_OFF_ENABLE;

	oca_dev->product_tab = g_oca_pid_11_product;
	oca_dev->product_cnt = OCA72XXX_PID_11_PROFUCT_MAX;

	oca_dev->vol_desc.addr = OCA72XXX_PID_11_REG03;
}
/********************** oca72xxx_pid_11 attributes end ************************/

static int oca_dev_chip_init(struct oca_device *oca_dev)
{
	/*get info by chipid*/
	switch (oca_dev->chipid) {
	case OCA_DEV_CHIPID_09:
		oca_dev_chipid_09_init(oca_dev);
		OCA_DEV_LOGI(oca_dev->dev, "product is pid_09 class");
		break;
	case OCA_DEV_CHIPID_11:
		oca_dev_chipid_11_init(oca_dev);
		OCA_DEV_LOGI(oca_dev->dev, "product is pid_11 class");
		break;
	default:
		OCA_DEV_LOGE(oca_dev->dev, "unsupported device revision [0x%x]",
			oca_dev->chipid);
		return -EINVAL;
	}

	return 0;
}

static int oca72xxx_dev_get_chipid(struct oca_device *oca_dev)
{
	int ret = -1;
	unsigned int cnt = 0;
	unsigned char reg_val = 0;

	for (cnt = 0; cnt < OCA_READ_CHIPID_RETRIES; cnt++) {
		ret = oca72xxx_dev_i2c_read_byte(oca_dev, OCA_DEV_REG_CHIPID, &reg_val);
		if (ret < 0) {
			OCA_DEV_LOGE(oca_dev->dev, "[%d] read chip is failed, ret=%d",
				cnt, ret);
			continue;
		}
		break;
	}


	if (cnt == OCA_READ_CHIPID_RETRIES) {
		OCA_DEV_LOGE(oca_dev->dev, "read chip is failed,cnt=%d", cnt);
		return -EINVAL;
	}

	OCA_DEV_LOGI(oca_dev->dev, "read chipid[0x%x] succeed", reg_val);
	oca_dev->chipid = reg_val;

	return 0;
}

int oca72xxx_dev_init(struct oca_device *oca_dev)
{
	int ret = -1;

	ret = oca72xxx_dev_get_chipid(oca_dev);
	if (ret < 0) {
		OCA_DEV_LOGE(oca_dev->dev, "read chipid is failed,ret=%d", ret);
		return ret;
	}

	ret = oca_dev_chip_init(oca_dev);

	return ret;
}


