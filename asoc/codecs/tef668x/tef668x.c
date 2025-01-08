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
#include <linux/stdarg.h>
#include "tef668x.h"
#include "tef668x_params_config.h"

struct tef668x_data *tef668x_priv = NULL;
static TEF668x_STATE tef668x_state;

eDev_Type RadioDev = Radio_Atomic2;
byte_field TunerFlag1;
const size_t patch_size = sizeof(PatchByteValues);
const uint8_t *p_patch_bytes = &PatchByteValues[0];

const size_t lut_size = sizeof(LutByteValues);
const uint8_t *p_lut_bytes = &LutByteValues[0];

static struct reg_default tef668x_reg[] = {
	{ TEF668x_REG_START, 0x14 },	   { TEF668x_REG_INIT, 0x1B },
	{ TEF668x_REG_CONTROL, 0x1C },	   { TEF668x_REG_FM_TUNE_TO, 0x20 },
	{ TEF668x_REG_AM_TUNE_TO, 0x21 },  { TEF668x_REG_AUDIO_SET_MUTE, 0x30 },
	{ TEF668x_REG_MODULE_INIT, 0x40 }, { TEF668x_REG_GET_PROPERTY, 0xFA },
};

typedef enum {
	PARAM_FM_SCAN_LEVEL = 0,
	PARAM_FM_SCAN_LEVEL_HI,
	PARAM_FM_USN_DISTURBANCE,
	PARAM_FM_WAM_DISTURBANCE,
	PARAM_FM_FREQ_OFFSET,
	PARAM_FM_LEVEL_AVAILABLE_TIME,
	PARAM_FM_BAND_WIDTH,
	PARAM_STEREO_Separation_Level_H,
	PARAM_STEREO_Separation_Level_L,
	PARAM_AM_SCAN_LEVEL,
	PARAM_AM_SCAN_LEVEL_HI,
	PARAM_AM_FREQ_OFFSET,
	PARAM_AM_LEVEL_AVAILABLE_TIME,
	PARAM_RADIO_USN_AVAILABLE_TIME,

	PARAM_FM_AM_MAX
} RADIO_TUNNING_PARA;

static Radio_CheckPara radio_para[] = {
	{ .name = "fm_scan_level", .value = FM_SCAN_LEVEL },
	{ .name = "fm_scan_level_hi", .value = FM_SCAN_LEVEL_HI },
	{ .name = "fm_usn_disturbance", .value = FM_USN_DISTURBANCE },
	{ .name = "fm_wam_disturbance", .value = FM_WAM_DISTURBANCE },
	{ .name = "fm_freq_offset", .value = FM_FREQ_OFFSET },
	{ .name = "fm_level_available_time", .value = RADIO_FM_LEVEL_AVAILABLE_TIME },
	{ .name = "fm_band_width", .value = FM_BAND_WIDTH },
	{ .name = "stereo_separation_level_h", .value = STEREO_Separation_Level_H },
	{ .name = "stereo_separation_level_l", .value = STEREO_Separation_Level_L },

	{ .name = "am_scan_level", .value = AM_SCAN_LEVEL },
	{ .name = "am_scan_level_hi", .value = AM_SCAN_LEVEL_HI },
	{ .name = "am_freq_offset", .value = AM_FREQ_OFFSET },
	{ .name = "am_level_available_time", .value = RADIO_AM_LEVEL_AVAILABLE_TIME },

	{ .name = "radio_usn_available_time", .value = RADIO_USN_AVAILABLE_TIME }
};

/* station */
static StationMemType station_record[MaxStationNum] = { 0 };
#ifdef CONFIG_PM_SLEEP
static int tef668x_resume(struct device *dev);
static int tef668x_suspend(struct device *dev);
#endif
static bool tef668x_addr_is_in_range(uint32_t addr, uint32_t addr_min, uint32_t addr_max)
{
	if ((addr >= addr_min) && (addr <= addr_max))
		return true;
	else
		return false;
}

static bool tef668x_volatile_register(struct device *dev, uint32_t reg)
{
	/* coeff registers */
	if (tef668x_addr_is_in_range(reg, TEF668x_REG_START, TEF668x_REG_MAX))
		return true;

	/* control registers */
	switch (reg) {
	case TEF668x_REG_START:
	case TEF668x_REG_INIT:
	case TEF668x_REG_CONTROL:
	case TEF668x_REG_FM_TUNE_TO:
	case TEF668x_REG_AM_TUNE_TO:
	case TEF668x_REG_AUDIO_SET_MUTE:
	case TEF668x_REG_MODULE_INIT:
	case TEF668x_REG_GET_PROPERTY:
		return true;
	default:
		return false;
	}
}

static bool tef668x_writeable_registers(struct device *dev, uint32_t reg)
{
	if (tef668x_addr_is_in_range(reg, TEF668x_REG_START, TEF668x_REG_GET_PROPERTY) ||
	    tef668x_addr_is_in_range(reg, TEF668x_REG_START, TEF668x_REG_MAX))
		return true;
	else
		return false;
}

static bool tef668x_readable_registers(struct device *dev, uint32_t reg)
{
	if (tef668x_addr_is_in_range(reg, TEF668x_REG_START, TEF668x_REG_GET_PROPERTY) ||
	    tef668x_addr_is_in_range(reg, TEF668x_REG_START, TEF668x_REG_MAX))
		return true;
	else
		return false;
}

static struct regmap_config tef668x_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.reg_defaults = tef668x_reg,
	.num_reg_defaults = ARRAY_SIZE(tef668x_reg),
	.max_register = TEF668x_REG_MAX,
	.volatile_reg = tef668x_volatile_register,
	.writeable_reg = tef668x_writeable_registers,
	.readable_reg = tef668x_readable_registers,
};

static int tef668x_i2c_writes(struct tef668x_data *tef668x, const uint8_t *tab)
{
	int ret = -1, i = 0;
	size_t len = 0;
	uint8_t buffer[TEF668x_BUF_LEN_MAX] = { 0 };

	if ((!tef668x) || (!tab)) {
		return -EPERM;
	}

	len = tab[0];

	ret = i2c_master_send(tef668x->client, &tab[1], len);
	if (ret < 0) {
		rdev_err(tef668x->dev, "i2c master send error\n");
		return ret;
	}

	if (log_debug & RADIO_LOG_REG)
		memcpy(buffer, &tab[1], len);
	rdev_reg_dbg(tef668x->dev, "%s: addr = 0x%02x, ", __func__, tef668x->client->addr);
	for (i = 1; i <= len; i++) {
		if (i == 1)
			rdev_reg_dbg(tef668x->dev, "reg_addr = 0x%02x, ", buffer[i]);
		else
			rdev_reg_dbg(tef668x->dev, "reg_data[%d] = 0x%02x, ", i, buffer[i]);
	}

	return ret;
};

static int tef668x_i2c_reads(struct tef668x_data *tef668x, uint8_t reg_addr, uint8_t *data_buf,
			     uint32_t data_len)
{
	int ret = -1, i = 0;
	struct i2c_msg msg[] = {
		[0] = {
			.addr = tef668x->client->addr,
			.flags = 0,
			.len = sizeof(uint8_t),
			.buf = &reg_addr,
			},
		[1] = {
			.addr = tef668x->client->addr,
			.flags = I2C_M_RD,
			.len = data_len,
			.buf = data_buf,
			},
	};

	ret = i2c_transfer(tef668x->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		rdev_err(tef668x->dev, "transfer failed.");
		return ret;
	}

	rdev_reg_dbg(tef668x->dev, "%s: addr = 0x%02x,", __func__, tef668x->client->addr);
	rdev_reg_dbg(tef668x->dev, "reg_addr = 0x%02x, ", reg_addr);
	for (i = 0; i < data_len; i++) {
		rdev_reg_dbg(tef668x->dev, "reg_data[%d] = 0x%02x, ", i, data_buf[i]);
	}

	return ret;
}

/*
Function: tef668x_set_cmd
notes: Change from devTEF668x_Set_Cmd in tef668x_sourcecode
*/
static int tef668x_set_cmd(struct tef668x_data *tef668x, TEF668x_MODULE module, uint8_t cmd,
			   int len, ...)
{
	int ret = -1;
	int i = 0;
	uint8_t buf[TEF668x_CMD_LEN_MAX] = { 0 };
	uint16_t temp = 0;
	va_list vArgs;
	va_start(vArgs, len);

	buf[0] = module; //module,		FM/AM/APP
	buf[1] = cmd; //cmd,		1,2,10,...
	buf[2] = 1; //index, 		always 1

	//fill buffer with 16bits one by one
	for (i = 3; i < len; i++) {
		temp = va_arg(vArgs, int); //the size only int valid for compile

		buf[i++] = High_16bto8b(temp);
		buf[i] = Low_16bto8b(temp);
	}
	va_end(vArgs);

	rdev_cmd_dbg(tef668x->dev,
		     "%s: addr = %x reg_addr: 0x%x, reg_data :0x%x,"
		     "0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x",
		     __func__, tef668x->client->addr, buf[0], buf[1], buf[2], buf[3], buf[4],
		     buf[5], buf[6], buf[7], buf[8]);

	ret = i2c_master_send(tef668x->client, buf, len);
	if (ret < 0)
		rdev_err(tef668x->dev, "i2c master send error");

	return ret;
}

/*
Function: tef668x_get_cmd
notes: Change from devTEF668x_Get_Cmd in tef668x_sourcecode
*/
static int tef668x_get_cmd(struct tef668x_data *tef668x, TEF668x_MODULE module, uint8_t cmd,
			   uint8_t *receive, int len)
{
	uint8_t buf[3] = { 0 };
	buf[0] = module; //module,		FM/AM/APP
	buf[1] = cmd; //cmd,		1,2,10,...
	buf[2] = 1; //index, 		always 1

	rdev_cmd_dbg(tef668x->dev, "%s: addr = %x reg_addr: 0x%x, reg_data :0x%x,0x%x", __func__,
		     tef668x->client->addr, buf[0], buf[1], buf[2]);

	i2c_master_send(tef668x->client, buf, 3);
	tef668x_i2c_reads(tef668x, module, receive, len);

	return SUCCESS;
}

/*
Function: tef668x_radio_tune_to
notes: Change from devTEF668x_Radio_Tune_To in tef668x_sourcecode
*/
static int tef668x_radio_tune_to(struct tef668x_data *tef668x, bool fm, uint16_t mode,
				 uint16_t frequency)
{
	int ret = -1;

	rdev_debug(tef668x->dev, "%s enter, mode = %d, frequency = %d\n", __func__, mode,
		   frequency);

	ret = tef668x_set_cmd(tef668x, fm ? TEF668x_MODULE_FM : TEF668x_MODULE_AM,
			      TEF668x_Cmd_Tune_To, (mode <= 5) ? 7 : 5, mode, frequency);

	return ret;
}

static int tef668x_write_register(struct tef668x_data *tef668x, uint8_t reg_addr, uint8_t reg_data0,
				  uint8_t reg_data1)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t Tab[] = { 3, 0x00, 0x00, 0x00 };

	Tab[1] = reg_addr;
	Tab[2] = reg_data0;
	Tab[3] = reg_data1;

	while (cnt < TEF_I2C_RETRIES) {
		ret = tef668x_i2c_writes(tef668x, Tab);
		if (ret < 0) {
			rdev_err(tef668x->dev, "%s cnt=%d error=%d", __func__, cnt, ret);
		} else {
			rdev_reg_dbg(tef668x->dev,
				     "%s reg_addr: 0x%x, reg_data :0x%x,reg_data :0x%x", __func__,
				     (uint8_t)reg_addr, (uint8_t)reg_data0, (uint8_t)reg_data1);
			break;
		}
		cnt++;
	}

	if (ret < 0) {
		rdev_err(tef668x->dev,
			 "retray 5 times,still error,try usleep 3s, i2c_write cnt=%d error=%d", cnt,
			 ret);
		usleep_range(TEF_RETRY_WAIT_TIME, TEF_RETRY_WAIT_TIME + 100);
		ret = tef668x_i2c_writes(tef668x, Tab);
		if (ret < 0) {
			rdev_err(tef668x->dev, "usleep 3s still ereror, i2c_write cnt=%d error=%d",
				 cnt, ret);
		}
	}

	return ret;
}

static int tef668x_read_register(struct tef668x_data *tef668x, uint8_t reg_addr, uint32_t *reg_data)
{
	int ret = -1;
	uint8_t cnt = 0;
	uint8_t buf[1];

	while (cnt < TEF_I2C_RETRIES) {
		ret = tef668x_i2c_reads(tef668x, reg_addr, buf, 1);
		if (ret < 0) {
			rdev_err(tef668x->dev, "i2c_read cnt=%d error=%d", cnt, ret);
		} else {
			rdev_err(tef668x->dev, "reg_addr: 0x%02x, reg_data :0x%02x, buf[0] :0x%02x",
				 (uint8_t)reg_addr, (uint8_t)(*reg_data), buf[0]);
			*reg_data = (buf[0] << 0);
			rdev_err(tef668x->dev, "reg_addr: 0x%02x, reg_data :0x%02x, buf[0] :0x%02x",
				 (uint8_t)reg_addr, (uint8_t)(*reg_data), buf[0]);
			break;
		}
		cnt++;
	}
	if (ret < 0) {
		rdev_err(tef668x->dev,
			 "retray 5 times,still error,try usleep 3s, i2c_write cnt=%d error=%d", cnt,
			 ret);
		usleep_range(TEF_RETRY_WAIT_TIME, TEF_RETRY_WAIT_TIME + 100);
		ret = tef668x_i2c_reads(tef668x, reg_addr, buf, 1);
		if (ret < 0) {
			rdev_err(tef668x->dev, "usleep 3s still ereror, i2c_read cnt=%d error=%d",
				 cnt, ret);
		}
	}

	return ret;
}

/*
Function: tef668x_appl_get_operation_status
notes: Change from devTEF668x_APPL_Get_Operation_Status in tef668x_sourcecode
*/
static int tef668x_appl_get_operation_status(struct tef668x_data *tef668x, uint16_t status)
{
	uint8_t buf[2] = { 0 };
	int r = ERROR;
	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter, state= %d\n", __func__, status);

	r = tef668x_get_cmd(tef668x, TEF668x_MODULE_APPL, TEF668x_Cmd_Get_Operation_Status, buf,
			    sizeof(buf));

	if (r == SUCCESS) {
		status = Convert8bto16b(buf);
	}
	tef668x_state = status;

	rdev_info(tef668x->dev, "%s leave, state= %d\n", __func__, tef668x_state);
	return r;
}

/*
Function: tef668x_para_load
notes: Change from devTEF668x_Para_Load in tef668x_sourcecode
*/
static int tef668x_para_load(struct tef668x_data *tef668x, bool fm, bool upate_all)
{
	int i = 0, ret = -1;
	const uint8_t *p;

	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter,ret = %d \n", __func__, ret);

	if (fm) {
		p = fm_init_para;

		for (i = 0; i < sizeof(fm_init_para); i += (p[i] + 1)) {
			ret = tef668x_i2c_writes(tef668x, (p + i));
			if (ret < 0) {
				rdev_err(tef668x->dev, "%s , fm_init_para fail, ret = %d \n",
					 __func__, ret);
				goto fail;
			}
		}
	} else {
		p = am_init_para;

		for (i = 0; i < sizeof(am_init_para); i += (p[i] + 1)) {
			ret = tef668x_i2c_writes(tef668x, (p + i));
			if (ret < 0) {
				rdev_err(tef668x->dev, "%s , am_init_para fail, ret = %d \n",
					 __func__, ret);
				goto fail;
			}
		}
	}

	if (upate_all) {
		p = audio_init_para;

		for (i = 0; i < sizeof(audio_init_para); i += (p[i] + 1)) {
			usleep_range(1000, 1000 + 100); //1 ms
			ret = tef668x_i2c_writes(tef668x, (p + i));
			if (ret < 0) {
				rdev_err(tef668x->dev, "%s , audio_init_para fail, ret = %d \n",
					 __func__, ret);
				goto fail;
			}
		}
		// xiaomi add to init volume after load para
		tef668x->cur_volume = AUDIO_VOLUME_INIT;
	}
fail:
	rdev_info(tef668x->dev, "%s leave,ret = %d \n", __func__, ret);
	return ret;
}

/*
notes: Change form devTEF668x_Radio_Get_Quality_Status in tef668x_sourcecode

module 32 / 33 FM / AM
cmd 128 / 129 Get_Quality_Status/ Get_Quality_Data
FM : | status, level, usn, wam, offset, bandwidth, modulation
AM : | status, level, -, -, offset, bandwidth, modulation

index
1 status
	[ 15:0 ]
	quality detector status
	[15] =
	AF_update flag
	0 = continuous quality data with time stamp
	1 = AF_Update sampled data
	[14:10] = reserved
	[9:0] = quality time stamp
	0 = tuning is in progress, no quality data available
	1 ... 320 (* 0.1 ms) = 0.1 ... 32 ms after tuning,
	quality data available, reliability depending on time stamp
	1000 = > 32 ms after tuning, quality data continuously updated
2 level
	[ 15:0 ] (signed)
	level detector result
	-200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
	actual range and accuracy is limited by noise and agc
3 usn
	[ 15:0 ]
	FM ultrasonic noise detector
	0 ... 1000 (*0.1 %) = 0 ... 100% relative usn detector result
4 wam
	[ 15:0 ]
	FM wideband-AM multipath detector
	0 ... 1000 (*0.1 %) = 0 ... 100% relative wam detector result
5 offset
	[ 15:0 ] (signed)
	radio frequency offset
	-1200 ... 1200 (*0.1 kHz) = -120 kHz ... 120 kHz radio frequency error
	actual range and accuracy is limited by noise and bandwidth
6 bandwidth
	[ 15:0 ]
	IF bandwidth
	FM 560 ...3110 [*0.1 kHz] = IF bandwidth 56 ... 311 kHz; narrow ... wide
	AM 30 ... 80 [*0.1 kHz] = IF bandwidth 3 ... 8 kHz; narrow ... wide
7 modulation
	[ 15:0 ]
	modulation detector
	FM 0 ... 1000 [*0.1 %] = 0 ... 100% modulation = 0 ... 75 kHz FM dev.
	1000 ... 2000 [*0.1 %] = 100% ... 200% over-modulation range
	(modulation results are an approximate indication of actual FM dev.)
	AM 0 ... 1000 [*0.1 %] = 0 ... 100% AM modulation index
	1000 ... 2000 [*0.1 %] = 100% ... 200% peak modulation range
*/
static int tef668x_radio_get_quality_status(struct tef668x_data *tef668x, bool fm, uint8_t *status)
{
	uint8_t buf[2] = { 0 };
	int r = ERROR;

	if (!tef668x)
		return -EPERM;

	r = tef668x_get_cmd(tef668x, fm ? TEF668x_MODULE_FM : TEF668x_MODULE_AM,
			    TEF668x_Cmd_Get_Operation_Status, buf, sizeof(buf));

	if (r == SUCCESS) {
		*status = ((0x3fff & Convert8bto16b(buf)) / 10);
	}

	return r;
}

/*
Function: tef668x_radio_get_quality_level
notes: Change from devTEF668x_Radio_Get_Quality_Level in tef668x_sourcecode
*/
static int tef668x_radio_get_quality_level(struct tef668x_data *tef668x, bool fm, uint8_t *status,
					   int16_t *level)
{
	uint8_t buf[4] = { 0 };
	int r = ERROR;

	if (!tef668x)
		return -EPERM;

	r = tef668x_get_cmd(tef668x, fm ? TEF668x_MODULE_FM : TEF668x_MODULE_AM,
			    TEF668x_Cmd_Get_Quality_Data, buf, sizeof(buf));

	if (r == SUCCESS) {
		*status = (int16_t)((0x3fff & Convert8bto16b(buf)) / 10);
		*level = (int16_t)(((int)Convert8bto16b(buf + 2)) / 10);
	}

	return r;
}

/*
Function: tef668x_radio_get_quality_data
notes: Change from devTEF668x_Radio_Get_Quality_Data in tef668x_sourcecode

status	= 1 ... 320 (* 0.1 ms) = 0.1 ... 32 ms after tuning
level		= -200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
usn		=  0 ... 1000 (*0.1 %) = 0 ... 100% relative usn detector result
wam		=  0 ... 1000 (*0.1 %) = 0 ... 100% relative wam detector result
offset	 = -1200 ... 1200 (*0.1 kHz) = -120 kHz ... 120 kHz radio frequency error
bandwith	 = FM 560 ... 3110 [*0.1 kHz] = IF bandwidth 56 ... 311 kHz; narrow ... wide
			AM 30 ... 80 [*0.1 kHz] = IF bandwidth 3 ... 8 kHz; narrow ... wide
modulation	 = FM 0 ... 1000 [*0.1 %] = 0 ... 100% modulation = 0 ... 75 kHz FM dev.
				1000 ... 2000 [*0.1 %] = 100% ... 200% over-modulation range
			AM 0 ... 1000 [*0.1 %] = 0 ... 100% AM modulation index
			1000 ... 2000 [*0.1 %] = 100% ... 200% peak modulation range

*/
static int tef668x_radio_get_quality_data(struct tef668x_data *tef668x, bool fm, uint8_t *status,
					  int16_t *level, uint8_t *usn, uint8_t *wam,
					  int16_t *offset, int16_t *bandwidth, uint8_t *modulation)
{
	uint8_t buf[14] = { 0 };
	int r = ERROR;

	if (!tef668x)
		return -EPERM;

	r = tef668x_get_cmd(tef668x, fm ? TEF668x_MODULE_FM : TEF668x_MODULE_AM,
			    TEF668x_Cmd_Get_Quality_Data, buf, sizeof(buf));

	if (r == SUCCESS) {
		*status = ((0x3fff & Convert8bto16b(buf)) / 10);
		*level = (((int)Convert8bto16b(buf + 2)) / 10);
		*usn = (Convert8bto16b(buf + 4) / 10);
		*wam = (Convert8bto16b(buf + 6) / 10);
		*offset = (((int)Convert8bto16b(buf + 8)) / 1);
		*bandwidth = (Convert8bto16b(buf + 10) / 1);
		*modulation = (Convert8bto16b(buf + 12) / 10);
	}

	return r;
}

/*
Function: tef668x_active_state
notes: Change from DevTEF668x_Active_state in tef668x_sourcecode
*/
static int tef668x_active_state(struct tef668x_data *tef668x)
{
	int ret = -1;
	if (!tef668x)
		return -EPERM;

	ret = tef668x_appl_get_operation_status(tef668x, tef668x_state);
	return ret;
}

static int tef668x_wait_active(struct tef668x_data *tef668x)
{
	int ret = -1;
	uint16_t freq = 8870;

	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter,tef668x_state= %d\n", __func__, tef668x_state);

	if (SUCCESS == tef668x_appl_get_operation_status(tef668x, tef668x_state)) {
		if ((tef668x_state != eDevTEF668x_Boot_state) &&
		    (tef668x_state != eDevTEF668x_Idle_state)) {
			usleep_range(TEF668x_TIMER_WAIT_ACTIVE,
				     TEF668x_TIMER_WAIT_ACTIVE + 100); //100 ms
			//try to tune GUI
			ret = tef668x_radio_tune_to(tef668x, TEF668x_Is_FM_Freq(freq),
						    eAR_TuningAction_Preset, freq);
			if (ret < 0) {
				rdev_err(tef668x->dev, "tef668x_radio_tune_to error\n");
				return ret;
			}
			ret = tef668x_para_load(tef668x, 1, 1);
			if (ret < 0) {
				rdev_err(tef668x->dev, "tef668x_para_load error\n");
				return ret;
			}
		}
	}
	return ret;
}

/*
Function: tef668x_appl_activate
notes: Change from devTEF668x_APPL_Activate in tef668x_sourcecode

module 64 APPL
cmd 5 Activate mode
index
1 mode
	[ 15:0 ]
	1 = goto active state with operation mode of radio standby
*/
static int tef668x_appl_activate(struct tef668x_data *tef668x, uint16_t mode)
{
	int ret = -1;
	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter\n", __func__);

	ret = tef668x_set_cmd(tef668x, TEF668x_MODULE_APPL, TEF668x_Cmd_Activate, 5, mode);
	return ret;
}

/*
Function: tef668x_appl_set_reference_clock
notes: Change from APPL_Set_ReferenceClock in tef668x_sourcecode
*/
static int tef668x_appl_set_reference_clock(struct tef668x_data *tef668x, uint32_t frequency,
					    bool is_ext_clk)
{
	int ret = -1;
	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter\n", __func__);

	ret = tef668x_set_cmd(tef668x, TEF668x_MODULE_APPL, TEF668x_Cmd_Set_ReferenceClock, 9,
			      (uint16_t)(frequency >> 16), (uint16_t)frequency, is_ext_clk);
	if (ret > 0)
		return 1;

	return ret;
}

/*
Function: tef668x_idle_state
notes: Change from DevTEF668x_Idle_state in tef668x_sourcecode

[ w 40 80 01 [ r 0001 ]
Alternatively Wait 50 ms.
Set reference frequency
*/
static int tef668x_idle_state(struct tef668x_data *tef668x)
{
	int ret = -1;

	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter tef668x_state= %d\n", __func__, tef668x_state);

	if (tef668x_state != eDevTEF668x_Boot_state) {
		//Set reference frequency
		if (SUCCESS == tef668x_appl_set_reference_clock(tef668x, TEF668x_REF_CLK,
								TEF668x_IS_EXT_CLK)) {
			//Activate : tef668x_appl_activate mode = 1.[ w 40 05 01 0001 ]
			ret = tef668x_appl_activate(tef668x, 1);
			if (ret > 0) {
				usleep_range(TEF668x_TIMER_WAIT_IDLE,
					     TEF668x_TIMER_WAIT_IDLE + 100); //50 ms
			}
		}
	}
	return ret;
}

static int tef668x_patch_load(struct tef668x_data *tef668x, const uint8_t *p_lut_bytes, size_t size)
{
	uint8_t buf[TEF668x_SPLIT_SIZE + 1] = { 0 };
	size_t len = 0, sum = 0;
	int ret = -1;
	buf[0] = 0x1b;

	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter\n", __func__);

	while (size) {
		len = (size > TEF668x_SPLIT_SIZE) ? TEF668x_SPLIT_SIZE : size;
		size -= len;

		memcpy(buf + 1, p_lut_bytes, len);
		p_lut_bytes += len;
		sum += len; //try

		ret = i2c_master_send(tef668x->client, buf, len + 1);
		usleep_range(1000, 1000 + 100); //1 ms
		if (ret < 0) {
			rdev_err(tef668x->dev, "tef668x_patch_load error\n");
			break;
		}
		if (len == 24)
			rdev_reg_dbg(tef668x->dev,
				     "tef668x_patch_load addr = %x reg_addr: 0x%x, reg_data :%x,"
				     "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,"
				     "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n,",
				     tef668x->client->addr, buf[0], buf[1], buf[2], buf[3], buf[4],
				     buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
				     buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18],
				     buf[19], buf[20], buf[21], buf[22], buf[23], buf[24]);
		else
			rdev_reg_dbg(tef668x->dev,
				     "tef668x_patch_load addr = %x reg_addr: 0x%x, reg_data :%x,"
				     "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,"
				     "%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n,",
				     tef668x->client->addr, buf[0], buf[1], buf[2], buf[3], buf[4],
				     buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
				     buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18],
				     buf[19], buf[20], buf[21], buf[22]);
		//delay for debug output
		if ((sum % 128) == 0) {
			//usleep_range(TEF668x_TIMER_WAIT_IDLE, TEF668x_TIMER_WAIT_IDLE + 100);//50 ms
		}
	}

	rdev_info(tef668x->dev, "%s leave, ret = %d\n", __func__, ret);
	return ret;
}

static int tef668x_patch_init(struct tef668x_data *tef668x)
{
	int ret = -1, i = 0;
	if (!tef668x)
		return -EPERM;

	rdev_info(tef668x->dev, "%s enter\n", __func__);
	//[ w 1C 0000 ]
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab1);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab1 error\n");
		return ret;
	}
	//[ w 1C 0074 ]
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab2);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab2 error\n");
		return ret;
	}
	//p_patch_bytes use PATCH.c
	rdev_info(tef668x->dev, "tef668x_patch_load p_patch_bytes\n");
	ret = tef668x_patch_load(tef668x, p_patch_bytes, patch_size);
	if (ret < 0) {
		while (i < TEF_I2C_RETRIES) {
			rdev_err(tef668x->dev, "int p_patch_bytes error,wait 10ms,retry five\n");
			//every + 10 ms.
			usleep_range(TEF668x_TIMER_POWER, TEF668x_TIMER_POWER + 100);
			ret = tef668x_patch_load(tef668x, p_patch_bytes, patch_size);
			if (ret < 0) {
				rdev_err(tef668x->dev, "int p_patch_bytes error\n");
				i++;
				if (i == 5)
					return ret;
			} else
				break;
		}
	}
	//[ w 1C 0000 ]
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab1);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab1 error\n");
		return ret;
	}
	//[ w 1C 0075 ]
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab3);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab3 error\n");
		return ret;
	}
	//p_patch_bytes use PATCH.c
	rdev_info(tef668x->dev, "tef668x_patch_load  p_lut_bytes \n");
	ret = tef668x_patch_load(tef668x, p_lut_bytes, lut_size);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int p_lut_bytes error,wati 10ms,retry again\n");
		//every + 10 ms.
		usleep_range(TEF668x_TIMER_POWER, TEF668x_TIMER_POWER + 100);
		ret = tef668x_patch_load(tef668x, p_lut_bytes, lut_size);
		if (ret < 0) {
			rdev_err(tef668x->dev, "int p_lut_bytes two error\n");
			return ret;
		}
	}
	//[ w 1C 0000 ]
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab1);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab1 error\n");
		return ret;
	}
	//[ w 14 0001 ]
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab4);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab error\n");
		return ret;
	}
	rdev_info(tef668x->dev, "%s leave, ret = %d\n", __func__, ret);
	return ret;
}

static int tef668x_boot_state(struct tef668x_data *tef668x)
{
	int ret = -1;
	if (!tef668x)
		return -EPERM;

	rdev_info(tef668x->dev, "%s enter\n", __func__);
	ret = tef668x_patch_init(tef668x);

	//every + 50 ms.
	usleep_range(TEF668x_TIMER_WAIT_IDLE, TEF668x_TIMER_WAIT_IDLE + 100);
	rdev_info(tef668x->dev, "%s leave, ret = %d\n", __func__, ret);
	return ret;
}

static int tef668x_start_init(struct tef668x_data *tef668x)
{
	int ret = -1;

	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter\n", __func__);

	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab0);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab0 error\n");
		return ret;
	}
	//every + 10 ms.
	usleep_range(TEF668x_TIMER_POWER, TEF668x_TIMER_POWER + 100);

	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab5);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab5 error\n");
		return ret;
	}
	//every + 10 ms.
	usleep_range(TEF668x_TIMER_POWER, TEF668x_TIMER_POWER + 100);
	ret = tef668x_boot_state(tef668x);
	if (ret > 0) {
		tef668x_state = eDevTEF668x_Idle_state;
	} else {
		rdev_err(tef668x->dev, "boot state error\n");
		return ret;
	}
	usleep_range(TEF668x_TIMER_WAIT_IDLE,
		     TEF668x_TIMER_WAIT_IDLE + 100); //50 ms

	//APPL_Get_Operation_Status [ w 40 80 01 [ r 0001 ]
	ret = tef668x_idle_state(tef668x);
	if (ret > 0) {
		tef668x_state = eDevTEF668x_Wait_Active;
	} else {
		rdev_err(tef668x->dev, "tef668x_idle_state error\n");
		return ret;
	}

	usleep_range(TEF668x_TIMER_WAIT_IDLE,
		     TEF668x_TIMER_WAIT_IDLE + 100); //50 ms
	ret = tef668x_wait_active(tef668x);
	if (ret > 0) {
		tef668x_state = eDevTEF668x_Active_state;
	} else {
		rdev_err(tef668x->dev, "tef668x_wait_active error\n");
		return ret;
	}

	tef668x_active_state(tef668x);

	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab6);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab6 error\n");
		return ret;
	}

	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab7);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab7 error\n");
		return ret;
	}
	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab8);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab8 error\n");
		return ret;
	}

	rdev_info(tef668x->dev, "%s leave, ret = %d\n", __func__, ret);

	return 0;
}

static int tef668x_stop_deinit(struct tef668x_data *tef668x)
{
	int ret = -1;

	if (!tef668x)
		return -EPERM;
	rdev_info(tef668x->dev, "%s enter\n", __func__);

	ret = tef668x_i2c_writes(tef668x, devTEF668x_Patch_CmdTab0);
	if (ret < 0) {
		rdev_err(tef668x->dev, "int tab0 error\n");
		return ret;
	}

	return 0;
}

/*
Function: tef668x_audio_set_mute
notes: Change from devTEF668x_Audio_Set_Mute in tef668x_sourcecode
*/
static int tef668x_audio_set_mute(struct tef668x_data *tef668x, uint8_t mode)
{
	int ret = -1;

	if (!tef668x)
		return -EPERM;

	rdev_debug(tef668x->dev, "%s : mute mode = %hhu\n", __func__, mode);

	ret = tef668x_set_cmd(tef668x, TEF668x_MODULE_AUDIO, TEF668x_Cmd_Set_Mute, 5,
			      (uint16_t)mode);
	if (ret < 0) {
		rdev_err(tef668x->dev, "%s fail\n", __func__);
		goto fail;
	}

	if (mode)
		tef668x->mute_status = 1;
	else
		tef668x->mute_status = 0;
fail:
	return ret;
}

static uint8_t tef668x_audio_get_mute(struct tef668x_data *tef668x)
{
	if (!tef668x) {
		return -EPERM;
	}

	rdev_debug(tef668x->dev, "%s : mute = %hhu\n", __func__, tef668x->mute_status);

	return tef668x->mute_status;
}

/*
Function: tef668x_audio_set_volume
notes: Change from devTEF668x_Audio_Set_Volume in tef668x_sourcecode

module 48 AUDIO
cmd 10 Set_Volume volume

index
1 volume
	[ 15:0 ] (signed)
	audio volume
	-599 ... +240 = -60 ... +24 dB volume
	0 = 0 dB (default)
*/
static int tef668x_audio_set_volume(struct tef668x_data *tef668x, int16_t volume)
{
	int ret = -1;

	if (!tef668x)
		return -EPERM;

	if ((volume < AUDIO_VOLUME_MIN || volume > AUDIO_VOLUME_MAX)) {
		rdev_err(tef668x->dev, "%s volume invalid : volume = %hd\n", __func__, volume);
		return -EINVAL;
	}

	rdev_debug(tef668x->dev, "%s, volume = %hd\n", __func__, volume);

	ret = tef668x_set_cmd(tef668x, TEF668x_MODULE_AUDIO, TEF668x_Cmd_Set_Volume, 5,
			      volume * 10);
	if (ret < 0) {
		rdev_err(tef668x->dev, "%s fail\n", __func__);
		goto fail;
	}

	tef668x->cur_volume = volume;
fail:
	return ret;
}

static int16_t tef668x_audio_get_volume(struct tef668x_data *tef668x)
{
	if (!tef668x)
		return -EPERM; // EPERM: -1, need check, but almost never comes to this line

	rdev_debug(tef668x->dev, "%s, volume = %hd\n", __func__, tef668x->cur_volume);

	return tef668x->cur_volume;
}

/*-----------------------------------------------------------------------
Function: tef668x_radio_set_band_and_freq
notes: Change from Radio_SetFreq in tef668x_sourcecode

Input:		mode:
                              TEF663X_PRESETMODE.,TEF663X_SEARCHMODE
                              TEF663X_AFUPDATEMODE,TEF663X_JUMPMODE...
                        Freq:
Output:	
Description:
------------------------------------------------------------------------*/
static int tef668x_radio_set_band_and_freq(struct tef668x_data *tef668x, uint8_t mode, uint8_t Band,
					   uint16_t Freq)
{
	int ret = -EINVAL;
	AR_TuningAction_t A2Mode = eAR_TuningAction_Preset;
	uint8_t band_back = Band;

	if (!tef668x)
		return -EPERM;

	rdev_debug(tef668x->dev, "%s: mode = %hhu, band = %hhu, freq = %hu\n", __func__, mode, Band,
		   Freq);

	/*frequency baundary check*/
	if ((Freq > FreqBaundConfig[tef668x->cur_band].MaxFreq) ||
	    (Freq < FreqBaundConfig[tef668x->cur_band].MinFreq)) {
		Freq = FreqBaundConfig[tef668x->cur_band].MinFreq;
	}
	if (Band >= MaxBandNum) {
		rdev_err(tef668x->dev, "%s: invalid band: 0x%x\n", __func__, Band);
		return -EINVAL;
	}

	tef668x->cur_band = Band;
	tef668x->cur_freq = Freq;
	station_record[tef668x->cur_band].Freq[0] = tef668x->cur_freq;

	switch (Band) {
	case FM1_BAND:
	case FM2_BAND:
	case FM3_BAND:
		Band = 0;
		break;
	case LW_BAND:
		Band = 1;
		break;
	case MW_BAND:
		Band = 2;
		break;
	case SW_BAND:
		Band = 3;
		break;
	}

	if (Is_Radio_Atomic2) {
		switch (mode) {
		case Radio_PRESETMODE:
			A2Mode = eAR_TuningAction_Preset;
			break;
		case Radio_SEARCHMODE:
			A2Mode = eAR_TuningAction_Search;
			break;
		case Radio_AFUPDATEMODE:
			A2Mode = eAR_TuningAction_AF_Update;
			break;
		case Radio_JUMPMODE:
			A2Mode = eAR_TuningAction_Jump;
			break;
		case Radio_CHECKMODE:
			A2Mode = eAR_TuningAction_Check;
			break;
		}
		ret = tef668x_radio_tune_to(tef668x, TEF668x_Is_FM_Freq(Freq), A2Mode, Freq);
		if (ret < 0) {
			rdev_err(tef668x->dev, "%s fail, freq = %hu\n", __func__, Freq);
			goto fail;
		}
	} else {
		/*update pll*/
		//RadioDrv_UpdatePll(mode,Band,Freq); //xiaomi delete and add log
		rdev_err(tef668x->dev, "%s: not Atomic2 dev type, please check the radio chip\n",
			 __func__);
		goto fail;
	}
	return 0;
fail:
	tef668x->cur_band = 0;
	tef668x->cur_freq = 0;
	station_record[band_back].Freq[0] = tef668x->cur_freq;
	return ret;
}

static uint16_t tef668x_radio_get_freq(struct tef668x_data *tef668x)
{
	if (!tef668x)
		return -EPERM;

	rdev_debug(tef668x->dev, "%s freq = %hu\n", __func__, tef668x->cur_freq);

	return tef668x->cur_freq;
}

static int tef668x_radio_set_freq(struct tef668x_data *tef668x, uint16_t freq)
{
	int ret = -1;
	uint8_t band = FM1_BAND;

	rdev_debug(tef668x->dev, "%s: freq = %hu\n", __func__, freq);
	if ((freq >= TEF668x_FM_FREQUENCY_MIN) && (freq <= TEF668x_FM_FREQUENCY_MAX)) {
		band = FM1_BAND;
	} else if ((freq >= TEF668x_MW_FREQUENCY_MIN) && (freq <= TEF668x_MW_FREQUENCY_MAX)) {
		band = MW_BAND;
	} else {
		rdev_err(tef668x->dev, "out of freq range  = %d\n", freq);
		return -EINVAL;
	}

	ret = tef668x_radio_set_band_and_freq(tef668x, Radio_PRESETMODE, band, freq);
	if (ret < 0) {
		rdev_err(tef668x->dev, "tef668x_radio_set_band_and_freq failed(%d)\n", ret);
	}

	return ret;
}

/*-----------------------------------------------------------------------
Function name:	tef668x_radio_get_band
notes: Change from Radio_GetCurrentBand in tef668x_sourcecode
Input:
Output:
Description:	 return current band
------------------------------------------------------------------------*/
static uint8_t tef668x_radio_get_band(struct tef668x_data *tef668x)
{
	return tef668x->cur_band;
}

static int tef668x_radio_set_band(struct tef668x_data *tef668x, uint8_t band)
{
	int ret = -1;
	uint16_t freq = 0;
	if (band >= MaxBandNum) {
		rdev_err(tef668x->dev, "%s: invalid band: 0x%x\n", __func__, band);
		return -EINVAL;
	}

	if (station_record[band].Freq[0] != 0)
		freq = station_record[band].Freq[0];
	else
		freq = FreqBaundConfig[band].MinFreq;
	ret = tef668x_radio_set_band_and_freq(tef668x, Radio_PRESETMODE, band, freq);
	if (ret < 0) {
		rdev_err(tef668x->dev, "%s: tef668x_radio_set_band_and_freq failed(%d)\n", __func__,
			 ret);
	}
	//tef668x->cur_band = band;
	//tef668x->cur_freq = freq;

	return ret;
}

/*-----------------------------------------------------------------------
Function name: tef668x_radio_get_stereo
notes: Change from Radio_CheckStereo in tef668x_sourcecode
Input:	
Output:
Description:	 check stereo indicator
------------------------------------------------------------------------*/
int tef668x_radio_get_stereo_status(struct tef668x_data *tef668x, uint16_t *stereo)
{
	uint16_t status;
	uint8_t buf[2] = { 0 };
	bool stereo_status = FALSE;
	bool fm = (tef668x->cur_band <= FM3_BAND);

	if (Is_Radio_Atomic2) {
		if (SUCCESS == tef668x_get_cmd(tef668x, fm ? TEF668x_MODULE_FM : TEF668x_MODULE_AM,
					       TEF668x_Cmd_Get_Signal_Status, buf, sizeof(buf))) {
			status = Convert8bto16b(buf);
			stereo_status = (status & bit15) ? TRUE : FALSE;
		}
	}

	if (stereo_status)
		*stereo = 1;
	else
		*stereo = 0;

	return 0;
}

/*-----------------------------------------------------------------------
Function name:	tef668x_clear_current_station
notes: Change form Radio_ClearCurrentStation in tef668x_sourcecode
Input:
Output:
Description:	 set current station
------------------------------------------------------------------------*/
static void tef668x_clear_current_station(struct tef668x_data *tef668x)
{
	tef668x->cur_station = 0;
}

/*-----------------------------------------------------------------------
Function name:	tef668x_get_freq_step
notes: Change form Radio_GetFreqStep in tef668x_sourcecode
Input:
Output:
Description:	 Get current band freq step
------------------------------------------------------------------------*/
static uint32_t tef668x_get_freq_step(uint8_t band)
{
	return (band <= FM3_BAND) ? Radio_AreaConfig.FM_ManualSeekStep :
				    Radio_AreaConfig.AM_ManualSeekStep;
}

/*====================================================
 Function:tef668x_change_freq_one_step
 notes: Change form Radio_ChangeFreqOneStep in tef668x_sourcecode
 Input:
      UP/DOWN
 OutPut:
      Null
 Desp:
     change curren freq on step
=========================================================*/
static void tef668x_change_freq_one_step(struct tef668x_data *tef668x, uint8_t UpDown)
{
	uint32_t step;

	if (!tef668x)
		return;

	step = tef668x_get_freq_step(tef668x->cur_band);
	rdev_debug(tef668x->dev, "%s, updown = %hhu, step = %u\n", __func__, UpDown, step);

	if (UpDown == 1) { /*increase one step*/
		tef668x->cur_freq += step;
		/*frequency baundary check	*/
		if (tef668x->cur_freq > FreqBaundConfig[tef668x->cur_band].MaxFreq) {
			tef668x->cur_freq = FreqBaundConfig[tef668x->cur_band].MinFreq;
		}
	} else { /*decrease one step*/
		tef668x->cur_freq -= step;
		/*frequency baundary check*/
		if (tef668x->cur_freq < FreqBaundConfig[tef668x->cur_band].MinFreq) {
			tef668x->cur_freq = FreqBaundConfig[tef668x->cur_band].MaxFreq;
		}
	}
}

/*
Function: tef668x_get_radio_qrs
notes: Change from Radio_Get_QRS in tef668x_sourcecode

return dector time in ms
return = ms
*/
static uint8_t tef668x_get_radio_qrs(struct tef668x_data *tef668x, bool fm)
{
	uint8_t status = 0;

	if (!tef668x)
		return -EPERM;

	if (Is_Radio_Atomic2) {
		if (SUCCESS == tef668x_radio_get_quality_status(tef668x, fm, &status)) {
			return status;
		}
	}

	rdev_debug(tef668x->dev, "%s, status = %hhu\n", __func__, status);

	return status;
}

/*
Function: tef668x_get_radio_level
notes: Change from Radio_Get_Level in tef668x_sourcecode

level detector result
output: -200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
return =  dBuV
*/
static int tef668x_get_radio_level(struct tef668x_data *tef668x, bool fm)
{
	int16_t ret = -255, level = -255;
	uint8_t status = 0;

	if (!tef668x) {
		return -EPERM;
	}

	if (Is_Radio_Atomic2) {
		if (SUCCESS == tef668x_radio_get_quality_level(tef668x, fm, &status, &level)) {
			rdev_debug(tef668x->dev, "%s, level = %hd, status = %hhu\n", __func__,
				   level, status);
			return level;
		}
	}

	rdev_err(tef668x->dev, "%s fail, level return = %hd\n", __func__, level);

	return ret;
}

/*
Function: tef668x_get_radio_data
notes: Change from Radio_Get_Data in tef668x_sourcecode

status =  0.1 ... 32 ms
level = -20 ... 120 dBuV
usn = 0 ... 100%
wam = 0 ... 100%
offset = -1200 ... 1200 (*0.1 kHz) = -120 kHz ... 120 kHz
bandwidth = FM, 560 ... 3110 [*0.1 kHz]  AM 30 ... 80 [*0.1 kHz]
modulation = 0 ... 100%
*/
static int tef668x_get_radio_data(struct tef668x_data *tef668x, bool fm, uint8_t *status,
				  int16_t *level, uint8_t *usn, uint8_t *wam, int16_t *offset,
				  int16_t *bandwidth, uint8_t *modulation)
{
	if (!tef668x) {
		return -EPERM;
	}

	rdev_debug(tef668x->dev, "%s\n", __func__);

	if (Is_Radio_Atomic2) {
		if (SUCCESS == tef668x_radio_get_quality_data(tef668x, fm, status, level, usn, wam,
							      offset, bandwidth, modulation)) {
			return SUCCESS;
		}
	}
	rdev_err(tef668x->dev, "%s fail\n", __func__);
	return !SUCCESS;
}

/*--------------------------------------------------------------------
 Function:tef668x_check_radio_station
 notes: Change from Radio_CheckStation in Tuner_Proc.c

 Input: 
      Null
 OutPut:
      Null
 Desp:
     check station if aviable
---------------------------------------------------------------------*/
static int tef668x_check_radio_station(struct tef668x_data *tef668x)
{
	int checkifstep = NO_STATION;
	uint8_t i = 0, status = 0, threshold = 0;
	uint8_t usn = 0, wam = 0, modulation = 0;
	int16_t level = 0, offset = 0, bandwidth = 0, time = 0;

	bool fm = (tef668x->cur_band <= FM3_BAND);

	if (!tef668x) {
		return -EPERM;
	}

	rdev_debug(tef668x->dev, "%s\n", __func__);

	threshold = fm ? radio_para[PARAM_FM_LEVEL_AVAILABLE_TIME].value :
			 radio_para[PARAM_AM_LEVEL_AVAILABLE_TIME].value; // fm at least delay 5ms
	//xiaomi add 10 ms wait
	usleep_range(TEF668x_TIMER_WAIT_SEEK, TEF668x_TIMER_WAIT_SEEK + 100);
	if (tef668x_get_radio_qrs(tef668x, fm) < threshold) {
		checkifstep = NO_STATION;
		rdev_debug(tef668x->dev, "tef668x_get_radio_qrs  fail, freq = %d\n",
			   tef668x->cur_freq);
		goto end;
	}

	for (i = 0; i < 3; i++) { /*Check 3 times*/
		threshold =
			fm ? ((SeekSenLevel == HIGH) ? radio_para[PARAM_FM_SCAN_LEVEL_HI].value :
						       radio_para[PARAM_FM_SCAN_LEVEL].value) :
			     ((SeekSenLevel == HIGH) ? radio_para[PARAM_AM_SCAN_LEVEL_HI].value :
						       radio_para[PARAM_AM_SCAN_LEVEL].value);
		//xiaomi add 10 ms * 3 = 30.  10ms min --30ms max
		usleep_range(TEF668x_TIMER_WAIT_SEEK, TEF668x_TIMER_WAIT_SEEK + 100);
		level = tef668x_get_radio_level(tef668x, fm);
		if (threshold > level) { //exit check
			checkifstep = NO_STATION;
			rdev_debug(tef668x->dev,
				   "func:%s,line:%d, fm:%d, level:%d , threshold:%d, "
				   "freq:%d, tef668x_get_radio_level  fail\n",
				   __func__, __LINE__, fm, level, threshold, tef668x->cur_freq);
			goto end;
		}
	}
	// xiaomi add: wait RADIO_USN_AVAILABLE_TIME for max
	while (time++ < radio_para[PARAM_RADIO_USN_AVAILABLE_TIME].value) {
		rdev_debug(tef668x->dev, "func:%s,line:%d, freq:%d\n", __func__, __LINE__,
			   tef668x->cur_freq);
		//xiaomi add wait 1ms once
		usleep_range(TEF668x_TIMER_WAIT_USN, TEF668x_TIMER_WAIT_USN + 100);
		//wait usn... available (at least 34ms)
		if (tef668x_get_radio_qrs(tef668x, fm) >= RADIO_USN_AVAILABLE_TIME) {
			break;
		}
	}
	if (SUCCESS == tef668x_get_radio_data(tef668x, fm, &status, &level, &usn, &wam, &offset,
					      &bandwidth, &modulation)) {
		checkifstep = NO_STATION;
		//xiaomi add  FM_BAND_WIDTH check for test
		if (fm ? ((usn < radio_para[PARAM_FM_USN_DISTURBANCE].value) &&
			  (wam < radio_para[PARAM_FM_WAM_DISTURBANCE].value) &&
			  (offset < radio_para[PARAM_FM_FREQ_OFFSET].value) &&
			  (bandwidth > radio_para[PARAM_FM_BAND_WIDTH].value)) :
			 (offset < radio_para[PARAM_AM_FREQ_OFFSET].value)) {
			checkifstep = PRESENT_STATION;
			rdev_para_dbg(tef668x->dev,
				      "func:%s,line:%d, fm:%d, level:%d, threshold:%d, usn:%d,"
				      "wam:%d, offset:%d, bandwidth:%d, freq:%d is good station \n",
				      __func__, __LINE__, fm, level, threshold, usn, wam, offset,
				      bandwidth, tef668x->cur_freq);
			goto end;
		}
	}

	checkifstep = NO_STATION;
	rdev_para_dbg(tef668x->dev,
		      "func:%s,line:%d, fm:%d, level:%d , threshold:%d, usn:%d,"
		      "wam:%d,offset:%d, bandwidth:%d ,freq:%d is bad station \n",
		      __func__, __LINE__, fm, level, threshold, usn, wam, offset, bandwidth,
		      tef668x->cur_freq);
end:
	return checkifstep;
}

/*====================================================
 Function:tef668x_radio_update_prog_cnt
 notes: Change form Tuner_Update_ProgCnt in tef668x_sourcecode
 Input:
      Null
 OutPut:
      Null
 Desp:
     update tuner pll
=========================================================*/
static int tef668x_radio_update_prog_cnt(struct tef668x_data *tef668x, AR_TuningAction_t mode)
{
	int ret = -1;
	uint16_t freq = 0;

	if (!tef668x)
		return -EPERM;

	rdev_debug(tef668x->dev, "%s\n", __func__);

	freq = tef668x->cur_freq;

	ret = tef668x_radio_set_band_and_freq(tef668x, mode, tef668x_radio_get_band(tef668x), freq);
	if (ret < 0) {
		rdev_err(tef668x->dev, "tef668x_radio_set_band_and_freq failed(%d)\n", ret);
	}

	return ret;
}

/*====================================================
 Function:tef668x_radio_step_seek
 notes: Change from Tuner_Manual_Sub in Tuner_Proc.c

 Input:
      Null
 OutPut:
      Null
 Desp:
       manual set tuner sub
=========================================================*/
static void tef668x_radio_step_seek(struct tef668x_data *tef668x, bool direction)
{
	if (!tef668x)
		return;

	rdev_debug(tef668x->dev, "%s, direction = %d\n", __func__, (int)direction);

	/*change one step of tuning frequency*/
	if (direction) {
		F_Tune_UP = 1;
	} else {
		F_Tune_UP = 0;
	}
	tef668x_change_freq_one_step(tef668x, F_Tune_UP);
	/*write "new programmble value" into PLL*/
	tef668x_radio_update_prog_cnt(tef668x, Radio_PRESETMODE);
	/*clear station number*/
	tef668x_clear_current_station(tef668x);
}

/*====================================================
 Function: tef668x_save_radio_station
 notes: Change from Radio_Save_Station in tef668x_sourcecode

 Input:
      Null
 OutPut:
      Null
 Desp:
     Save one station to mem
=========================================================*/
static void tef668x_save_radio_station(struct tef668x_data *tef668x, uint8_t StationNumber,
				       uint16_t StationFreq)
{
	rdev_debug(tef668x->dev, "%s\n", __func__);

	station_record[tef668x->cur_band].Freq[StationNumber] = StationFreq;

	//xiaomi delete, eeprom not found
	//EEPROM_Write(EEPRom_station_recordDefaultAddr + tef668x->cur_band *sizeof(station_record[0]) ,
	//	(uint8_t *)(&station_record[tef668x->cur_band].Freq[StationNumber]),
	//	2);
}

/*====================================================
 Function:tef668x_radio_event_end
 notes: Change from Tuner_Event_End in tef668x_sourcecode

 Input:
      Null
 OutPut:
      Null
 Desp:
       exit current tuner work mode
=========================================================*/
static void tef668x_radio_event_end(struct tef668x_data *tef668x)
{
	rdev_debug(tef668x->dev, "%s\n", __func__);
	/*set tuner in idle status*/
	//TunerWorkMode=0; //xiaomi delete
	//TunerSubMode=0;   //xiaomi delete
	//F_Store_Station_Ok=FALSE;   //xiaomi delete

	/*stop TunerWaitTimer*/
	//TimerStop(&TunerWaitTimer); //xiaomi delete
	/*stop StereoDetTimer*/
	//TimerStop(&StereoDetTimer); //xiaomi delete
	//Save current station
	tef668x_save_radio_station(tef668x, 0, tef668x->cur_freq);
}

/*====================================================
 Function:tef668x_radio_auto_seek
 notes: Change from Tuner_Seek_Sub in Tuner_Proc.c
=========================================================*/
static int tef668x_radio_auto_seek(struct tef668x_data *tef668x, bool direction)
{
	int ret = -1;
	uint16_t start_freq = 0, init_freq = 0;

	if (!tef668x)
		return -EPERM;

	rdev_debug(tef668x->dev, "%s\n", __func__);

	if (tef668x->cur_band >= MaxBandNum) {
		tef668x->cur_band = FM1_BAND;
	}
	if ((tef668x->cur_freq > FreqBaundConfig[tef668x->cur_band].MaxFreq) ||
	    (tef668x->cur_freq < FreqBaundConfig[tef668x->cur_band].MinFreq)) {
		init_freq = FreqBaundConfig[tef668x->cur_band].MinFreq;
		tef668x_radio_set_freq(tef668x, init_freq);
	}

	start_freq = tef668x->cur_freq; //save current freq
	/*increase or decreas one step*/
	if (direction) {
		F_Tune_UP = 1;
	} else {
		F_Tune_UP = 0;
	}
	/*to process seek event step by step*/
	do {
		tef668x_change_freq_one_step(tef668x, F_Tune_UP);
		/*write "new programmble value" into PLL*/
		ret = tef668x_radio_update_prog_cnt(tef668x, Radio_SEARCHMODE);
		if (ret < 0) {
			rdev_err(tef668x->dev, "auto seek execute fail\n");
			goto end;
		}
		tef668x_clear_current_station(tef668x); //clear station number
		/*station check*/
		if (PRESENT_STATION == tef668x_check_radio_station(tef668x)) { /*get good station*/
			rdev_debug(tef668x->dev, "auto seek get good station, freq = %d\n",
				   tef668x->cur_freq);
			break;
		} else { /*no station*/
			continue; // loop seek
		}
	} while (start_freq != tef668x->cur_freq);

	ret = tef668x_radio_update_prog_cnt(tef668x, Radio_PRESETMODE);
	if (ret < 0) {
		rdev_err(tef668x->dev, "auto seek freq set fail, freq = %d\n", tef668x->cur_freq);
	}
end:
	tef668x_radio_event_end(tef668x);
	return ret;
}

static ssize_t tef668x_reg_store(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct tef668x_data *tef668x = dev_get_drvdata(dev);
	uint8_t databuf[3] = { 0 };

	if (2 == sscanf(buf, "%x %x %x", &databuf[0], &databuf[1], &databuf[2])) {
		rdev_info(tef668x->dev, "buf %s databuf[0] %#x, databuf[1] %#x,databuf[2] %#x", buf,
			  databuf[0], databuf[1], databuf[2]);
		tef668x_write_register(tef668x, databuf[0], databuf[1], databuf[2]);
	}

	return count;
}

static ssize_t tef668x_radio_para_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct tef668x_data *tef668x = dev_get_drvdata(dev);
	uint16_t databuf[3] = { 0 };
	uint16_t i = 0;

	if (2 == sscanf(buf, "%hu %hu", &databuf[0], &databuf[1])) {
		rdev_info(tef668x->dev, "buf %s, databuf[0] = %hu, databuf[1] = %hu", buf,
			  databuf[0], databuf[1]);
		i = databuf[0];
		if (i < sizeof(radio_para) / sizeof(Radio_CheckPara)) {
			radio_para[i].value = databuf[1];
			rdev_info(tef668x->dev, " para%hu, value = %hu set success", databuf[0],
				  databuf[1]);
		}
	}

	return count;
}

static ssize_t tef668x_suspend_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct tef668x_data *tef668x = dev_get_drvdata(dev);
	uint8_t databuf = 0;

	if (1 == sscanf(buf, "%x", &databuf)) {
		rdev_info(tef668x->dev, "databuf =  %x", databuf);
		if (!!databuf) {
			if (!tef668x->suspend)
				tef668x_suspend(tef668x->dev);
		} else {
			if (tef668x->suspend)
				tef668x_resume(tef668x->dev);
		}
	}

	return count;
}

static ssize_t tef668x_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tef668x_data *tef668x = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 1;
	uint32_t reg_val = 0;
	int reg_num = TEF_ADJUT_REG;

	for (i = 1; i <= reg_num; i++) {
		tef668x_read_register(tef668x, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t tef668x_radio_para_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i = 0, max_len = 0;
	int para_count = sizeof(radio_para) / sizeof(Radio_CheckPara);
	int line_count = 4;
	for (i = 0; i < para_count; i++) {
		max_len = max_t(int, max_len, strlen(radio_para[i].name));
	}

	for (i = 0; i < para_count; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%d-%-*s : %hu        ", i, max_len,
				radio_para[i].name, radio_para[i].value);
		if ((i + 1) % line_count == 0)
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static ssize_t tef668x_suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tef668x_data *tef668x = dev_get_drvdata(dev);
	ssize_t len = 0;

	len = snprintf(buf, PAGE_SIZE - len, "%hhu\n", tef668x->suspend);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, tef668x_reg_show, tef668x_reg_store);
static DEVICE_ATTR(radio_para, S_IWUSR | S_IRUGO, tef668x_radio_para_show,
		   tef668x_radio_para_store);
static DEVICE_ATTR(suspend, S_IWUSR | S_IRUGO, tef668x_suspend_show, tef668x_suspend_store);

static struct attribute *tef668x_attributes[] = { &dev_attr_reg.attr, &dev_attr_radio_para.attr,
						  &dev_attr_suspend.attr, NULL };

static struct attribute_group tef668x_attribute_group = {
	.attrs = tef668x_attributes,
};

static int radio_set_freq(struct radio_device *rdev, uint16_t freq)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	rdev_debug(tef668x->dev, "%s: freq = %hu\n", __func__, freq);
	ret = tef668x_radio_set_freq(tef668x, freq);

	return ret;
}

/*-----------------------------------------------------------------------
Function name:	radio_get_freq
notes: Change from Radio_GetCurrentFreq in tef668x_sourcecode
Input:
Output:
Description:	return current freq
------------------------------------------------------------------------*/
static int radio_get_freq(struct radio_device *rdev, uint16_t *freq)
{
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	*freq = tef668x_radio_get_freq(tef668x);
	rdev_debug(tef668x->dev, "%s: freq = %hu\n", __func__, *freq);

	return 0;
}

static int radio_set_volume(struct radio_device *rdev, int16_t vol)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	rdev_debug(tef668x->dev, "%s: volume = %hd\n", __func__, vol);
	ret = tef668x_audio_set_volume(tef668x, vol);

	return ret;
}

static int radio_get_volume(struct radio_device *rdev, int16_t *vol)
{
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	*vol = tef668x_audio_get_volume(tef668x);
	rdev_debug(tef668x->dev, "%s: volume = %hd\n", __func__, *vol);

	return 0;
}

static int radio_set_mute(struct radio_device *rdev, uint8_t mute)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	rdev_debug(tef668x->dev, "%s: mute = %hhu\n", __func__, mute);
	ret = tef668x_audio_set_mute(tef668x, mute);

	return ret;
}

static int radio_get_mute(struct radio_device *rdev, uint8_t *mute)
{
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	*mute = tef668x_audio_get_mute(tef668x);
	rdev_debug(tef668x->dev, "%s: mute = %hhu\n", __func__, *mute);

	return 0;
}

static int radio_set_band(struct radio_device *rdev, uint8_t band)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);

	ret = tef668x_radio_set_band(tef668x, band);

	return ret;
}

static int radio_get_band(struct radio_device *rdev, uint8_t *band)
{
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	*band = tef668x_radio_get_band(tef668x);
	rdev_debug(tef668x->dev, "%s: mute = %hu\n", __func__, *band);

	return 0;
}

static int radio_auto_seek(struct radio_device *rdev, bool direction)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	ret = tef668x_radio_auto_seek(tef668x, direction);

	return ret;
}

static int radio_step_seek(struct radio_device *rdev, bool direction)
{
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	tef668x_radio_step_seek(tef668x, direction);

	return 0;
}

static int radio_get_freq_range(struct radio_device *rdev, uint16_t *bottom_freq,
				uint16_t *top_freq)
{
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;
	tef668x = container_of(rdev, struct tef668x_data, rdev);
	if (tef668x->cur_band >= MaxBandNum) {
		rdev_err(tef668x->dev, "%s: invalid band: 0x%x\n", __func__, tef668x->cur_band);
		return -EINVAL;
	}
	*bottom_freq = FreqBaundConfig[tef668x->cur_band].MinFreq;
	*top_freq = FreqBaundConfig[tef668x->cur_band].MaxFreq;
	return 0;
}

static int radio_get_stereo_mono(struct radio_device *rdev, uint16_t *mode)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;

	if (!rdev)
		return -EPERM;
	tef668x = container_of(rdev, struct tef668x_data, rdev);

	ret = tef668x_radio_get_stereo_status(tef668x, mode);
	return ret;
}

static int radio_set_reset(struct radio_device *rdev)
{
	int ret = -1;
	struct tef668x_data *tef668x = NULL;
	int16_t pre_volume =  0;

	if (!rdev)
		return -EPERM;

	tef668x = container_of(rdev, struct tef668x_data, rdev);
	rdev_info(tef668x->dev, "%s\n", __func__);
	pre_volume = tef668x->cur_volume;
	ret = tef668x_stop_deinit(tef668x);
	if (ret < 0) {
		rdev_err(tef668x->dev, "%s fail: tef668x_stop_deinit fail\n", __func__);
		return ret;
	}
	msleep(10);
	ret = tef668x_start_init(tef668x);
	if (ret < 0) {
		rdev_err(tef668x->dev, "%s fail: tef668x_start_init fail\n", __func__);
		return ret;
	}
	tef668x_audio_set_mute(tef668x, tef668x->mute_status);
	tef668x_audio_set_volume(tef668x, pre_volume);
	tef668x_radio_set_freq(tef668x, tef668x->cur_freq);
	rdev_info(tef668x->dev, "%s success\n", __func__);
	return 0;
}

static void tef668x_radiodev_controller_init(struct radio_device *rdev)
{
	rdev->control.set_freq = radio_set_freq;
	rdev->control.get_freq = radio_get_freq;
	rdev->control.set_mute = radio_set_mute;
	rdev->control.get_mute = radio_get_mute;
	rdev->control.set_volume = radio_set_volume;
	rdev->control.get_volume = radio_get_volume;
	rdev->control.set_band = radio_set_band;
	rdev->control.get_band = radio_get_band;
	rdev->control.auto_seek = radio_auto_seek;
	rdev->control.step_seek = radio_step_seek;
	rdev->control.get_freq_range = radio_get_freq_range;
	rdev->control.get_stereo_mono = radio_get_stereo_mono;
	rdev->control.reset = radio_set_reset;
}

#ifdef CONFIG_PM_SLEEP
static int tef668x_suspend(struct device *dev)
{
	int ret = 0;
	struct tef668x_data *tef668x = NULL;

	if (!dev)
		return -ENODEV;

	tef668x = dev_get_drvdata(dev);
	if (!tef668x)
		return -EINVAL;

	rdev_info(tef668x->dev, "%s enter\n", __func__);

	ret = tef668x_stop_deinit(tef668x);
	if (ret < 0) {
		rdev_err(tef668x->dev, "tef668x_suspend fail\n");
		return ret;
	}
	tef668x->suspend = 1;
	rdev_info(tef668x->dev, "%s success\n", __func__);

	return 0;
}

static int tef668x_resume(struct device *dev)
{
	int ret = 0;
	struct tef668x_data *tef668x = NULL;
	int16_t pre_volume = 0;
	if (!dev)
		return -ENODEV;

	tef668x = dev_get_drvdata(dev);
	if (!tef668x)
		return -EINVAL;

	rdev_info(tef668x->dev, "%s enter\n", __func__);

	pre_volume = tef668x->cur_volume;
	ret = tef668x_start_init(tef668x);
	if (ret < 0) {
		rdev_err(tef668x->dev, "%s fail\n", __func__);
		return ret;
	}

	tef668x_audio_set_mute(tef668x, tef668x->mute_status);
	tef668x_audio_set_volume(tef668x, pre_volume);
	tef668x_radio_set_freq(tef668x, tef668x->cur_freq);
	tef668x->suspend = 0;
	rdev_info(tef668x->dev, "%s success\n", __func__);

	return 0;
}

static const struct dev_pm_ops tef668x_dev_pm_ops = {
	.suspend = tef668x_suspend,
	.resume = tef668x_resume,
};
#endif

static int tef668x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
	struct tef668x_data *tef668x;

	rdev_info(&client->dev, "%s: tef668x i2c probe E, addr = %x\n", __func__, client->addr);

	/* Allocate driver data */
	tef668x = devm_kzalloc(&client->dev, sizeof(struct tef668x_data), GFP_KERNEL);
	if (!tef668x)
		return -ENOMEM;

	tef668x_priv = tef668x;
	/* Initialize client driver structure */
	tef668x->client = client;
	tef668x->dev = &client->dev;
	/* regmap config */
	i2c_set_clientdata(client, tef668x);

	ret = tef668x_start_init(tef668x);
	if (ret < 0)
		goto out0;

	tef668x->regmap = devm_regmap_init_i2c(client, &tef668x_regmap);
	rdev_info(tef668x->dev, "allocate register map done\n");
	if (IS_ERR(tef668x->regmap)) {
		ret = PTR_ERR(tef668x->regmap);
		rdev_err(tef668x->dev, "Failed to allocate register map: %d\n", ret);
		goto out0;
	}

	/* create attr */
	ret = sysfs_create_group(&client->dev.kobj, &tef668x_attribute_group);
	if (ret < 0) {
		rdev_err(tef668x->dev, "error creating sysfs attr files");
		goto out0;
	}

	tef668x->rdev.dev = &client->dev;
	tef668x_radiodev_controller_init(&tef668x->rdev);
	ret = radiodev_controller_register(&tef668x->rdev);
	if (ret < 0) {
		rdev_err(tef668x->dev, "radiodev_controller_register fail");
		goto out1;
	}

	rdev_info(&client->dev, "%s: tef668x i2c probe done, ret %d\n", __func__, ret);
	return 0;
out1:
	sysfs_remove_group(&client->dev.kobj, &tef668x_attribute_group);
out0:
	tef668x_priv =NULL;
	return ret;
}

static void tef668x_i2c_remove(struct i2c_client *client)
{
	struct tef668x_data *tef668x = dev_get_drvdata(&client->dev);
	rdev_info(&client->dev, "%s: tef668x i2c remove.\n", __func__);
	/*rm attr node*/
	sysfs_remove_group(&client->dev.kobj, &tef668x_attribute_group);
	radiodev_controller_unregister(&tef668x->rdev);
	tef668x_priv =NULL;
	return;
}

/* TBD */
static const struct i2c_device_id tef668x_i2c_id[] = { { "tef668x", 0 }, {} };

MODULE_DEVICE_TABLE(i2c, tef668x_i2c_id);

/* match dtsi */
static struct of_device_id tef668x_dt_match[] = {
	{ .compatible = "nxp,tef668x" },
	{},
};
MODULE_DEVICE_TABLE(of, tef668x_dt_match);

static struct i2c_driver tef668x_i2c_driver = {
	.driver = {
		.name		= "tef668x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tef668x_dt_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef CONFIG_PM_SLEEP
		.pm = &tef668x_dev_pm_ops,
#endif
	},
	.id_table	= tef668x_i2c_id,
	.probe		= tef668x_i2c_probe,
	.remove		= tef668x_i2c_remove,
};

module_i2c_driver(tef668x_i2c_driver);

MODULE_DESCRIPTION("tef668x driver Register R/W & GPIO control");
MODULE_AUTHOR("N/A");
MODULE_LICENSE("GPL");
