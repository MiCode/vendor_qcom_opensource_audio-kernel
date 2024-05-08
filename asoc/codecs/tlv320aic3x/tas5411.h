#ifndef _TAS5411_H_
#define _TAS5411_H_

/* AMP Current Adjustment Register */
#define AMP_ADJUT_REG			3
#define AMP_I2C_RETRIES			5
#define AMP_RETRY_WAIT_TIME  3000000

#define TAS5411_REG_FALTE_STATUS	0x01
#define TAS5411_REG_DIAGNOSTIC		0x02
#define TAS5411_REG_CONTROL		0x03

struct tas5411_data {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	/* AMP mute io controller */
	int enable_mute_pin_control;
	struct pinctrl *tas5411_mute_pinctrl;
	struct pinctrl_state *pinctrl_state[2];
	/* reserved */
	int enable_falut_pin_control;
	int enable_standby_pin_control;
};

int tas5411_i2c_writes(struct tas5411_data *tas5411,
	unsigned char reg_addr, unsigned char *buf, unsigned int len);
int tas5411_i2c_reads(struct tas5411_data *tas5411,
	unsigned char reg_addr, unsigned char *data_buf, unsigned int data_len);
int tas5411_pa_mute_switch_init(struct tas5411_data *tas5411);
int tas5411_pinctrl_controller(bool amp_mute_status);

#endif
