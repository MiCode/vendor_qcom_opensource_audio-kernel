#ifndef __OCA72XXX_H__
#define __OCA72XXX_H__
#include <linux/version.h>
#include <linux/kernel.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "ocspa_device.h"
#include "ocspa_monitor.h"
#include "ocspa_fw.h"

#define OCA_CFG_UPDATE_DELAY
#define OCA_CFG_UPDATE_DELAY_TIMER	(3000)

#define OCA72XXX_NO_OFF_BIN		(0)
#define OCA72XXX_OFF_BIN_OK		(1)

#define OCA72XXX_PRIVATE_KCONTROL_NUM	(4)
#define OCA72XXX_PUBLIC_KCONTROL_NUM	(1)

#define OCA_I2C_RETRIES			(5)
#define OCA_I2C_RETRY_DELAY		(2)
#define OCA_I2C_READ_MSG_NUM		(2)

#define OCA72XXX_FW_NAME_MAX		(64)
#define OCA_NAME_BUF_MAX			(64)
#define OCA_LOAD_FW_RETRIES		(3)

#define OCA_DEV_REG_RD_ACCESS		(1 << 0)
#define OCA_DEV_REG_WR_ACCESS		(1 << 1)

#define OCARW_ADDR_BYTES			(1)
#define OCARW_DATA_BYTES			(1)
#define OCARW_HDR_LEN			(24)

/***********************************************************
 *
 * oca72xxx codec control compatible with kernel 4.19
 *
 ***********************************************************/
#if KERNEL_VERSION(4, 19, 1) <= LINUX_VERSION_CODE
#define OCA_KERNEL_VER_OVER_4_19_1
#endif

#if KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE
#define OCA_KERNEL_VER_OVER_5_4_0
#endif

#if KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE
#define OCA_KERNEL_VER_OVER_5_10_0
#endif
#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
#define OCA_KERNEL_VER_OVER_6_1_0
#endif

#if KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE
#define OCA_KERNEL_VER_OVER_6_6_0
#endif

#ifdef OCA_KERNEL_VER_OVER_4_19_1
typedef struct snd_soc_component oca_snd_soc_codec_t;
#else
typedef struct snd_soc_codec oca_snd_soc_codec_t;
#endif

struct oca_componet_codec_ops {
	int (*add_codec_controls)(oca_snd_soc_codec_t *codec,
		const struct snd_kcontrol_new *controls, unsigned int num_controls);
	void (*unregister_codec)(struct device *dev);
};


/********************************************
 *
 * oca72xxx devices attributes
 *
 *******************************************/
enum {
	OCARW_FLAG_WRITE = 0,
	OCARW_FLAG_READ,
};

enum {
	OCARW_I2C_ST_NONE = 0,
	OCARW_I2C_ST_READ,
	OCARW_I2C_ST_WRITE,
};

enum {
	OCARW_HDR_WR_FLAG = 0,
	OCARW_HDR_ADDR_BYTES,
	OCARW_HDR_DATA_BYTES,
	OCARW_HDR_REG_NUM,
	OCARW_HDR_REG_ADDR,
	OCARW_HDR_MAX,
};

struct oca_i2c_packet {
	char status;
	unsigned int reg_num;
	unsigned int reg_addr;
	char *reg_data;
};


/********************************************
 *
 * oca72xxx device struct
 *
 *******************************************/
struct oca72xxx {
	int32_t dev_index;
	char fw_name[OCA72XXX_FW_NAME_MAX];
	char *current_profile;
	char *last_profile;
	char *kctl_profile;
	char prof_off_name[OCA_PROFILE_STR_MAX];
	uint32_t pa_status;
	uint32_t off_bin_status;
	struct device *dev;

	struct mutex reg_lock;
	struct oca_device oca_dev;
	struct oca_i2c_packet i2c_packet;

	struct delayed_work fw_load_work;
	struct acf_bin_info acf_info;

	oca_snd_soc_codec_t *codec;

	struct list_head list;

	struct oca_monitor monitor;
};

int oca72xxx_update_profile(struct oca72xxx *oca72xxx, char *profile);
int mievent_report(unsigned int eventid,const char *value,struct device *dev);

#endif
