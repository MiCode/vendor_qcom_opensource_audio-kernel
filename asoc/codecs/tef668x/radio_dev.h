#ifndef _RADIO_DEV_H_
#define _RADIO_DEV_H_
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>

#define RADIO_DRV_NAME "MI Radio Driver"
#define RADIO_DRV_CARD_NAME "MI Radio"
#define RADIO_DEV_MAX_NUM 10
#define RADIO_DEV_MAX_BUF 20

#define RADIO_LOG_CLOSE 0
#define RADIO_LOG_DEBUG 1
#define RADIO_LOG_REG 2
#define RADIO_LOG_CMD 4
#define RADIO_LOG_PARAMS 8
#define FREQ_MUL 160

/* Stereo/Mono mode */
#define RADIO_MONO_MODE 0
#define RADIO_STEREO_MODE 1

/* RDS modes */
#define RADIO_RDS_DISABLE 0
#define RADIO_RDS_ENABLE 1

typedef enum {
	RADIO_MODE_OFF,
	RADIO_MODE_ON,
	RADIO_MODE_BOOT,
	RADIO_MODE_IDLE,
	RADIO_MODE_WAIT,
	RADIO_MODE_ACTIVE,
	RADIO_MODE_MAX
} RADIO_MODE_STATE;

extern uint32_t log_debug;

#define rdev_debug(dev, fmt, ...)                                                      \
	do {                                                                           \
		if (log_debug & RADIO_LOG_DEBUG)                                       \
			dev_printk_index_wrap(_dev_info, KERN_INFO, dev, dev_fmt(fmt), \
					      ##__VA_ARGS__);                          \
	} while (0)

#define rdev_reg_dbg(dev, fmt, ...)                                                    \
	do {                                                                           \
		if (log_debug & RADIO_LOG_REG)                                         \
			dev_printk_index_wrap(_dev_info, KERN_INFO, dev, dev_fmt(fmt), \
					      ##__VA_ARGS__);                          \
	} while (0)

#define rdev_cmd_dbg(dev, fmt, ...)                                                    \
	do {                                                                           \
		if (log_debug & RADIO_LOG_CMD)                                         \
			dev_printk_index_wrap(_dev_info, KERN_INFO, dev, dev_fmt(fmt), \
					      ##__VA_ARGS__);                          \
	} while (0)

#define rdev_para_dbg(dev, fmt, ...)                                                   \
	do {                                                                           \
		if (log_debug & (RADIO_LOG_PARAMS | RADIO_LOG_DEBUG))                  \
			dev_printk_index_wrap(_dev_info, KERN_INFO, dev, dev_fmt(fmt), \
					      ##__VA_ARGS__);                          \
	} while (0)

#define rdev_info(dev, fmt, ...) \
	dev_printk_index_wrap(_dev_info, KERN_INFO, dev, dev_fmt(fmt), ##__VA_ARGS__)
#define rdev_err(dev, fmt, ...) \
	dev_printk_index_wrap(_dev_err, KERN_ERR, dev, dev_fmt(fmt), ##__VA_ARGS__)

struct radio_device;

struct radio_controller {
	int (*set_freq)(struct radio_device *rdev, uint16_t freq);
	int (*get_freq)(struct radio_device *rdev, uint16_t *freq);
	int (*set_band)(struct radio_device *rdev, uint8_t band);
	int (*get_band)(struct radio_device *rdev, uint8_t *band);
	int (*set_volume)(struct radio_device *rdev, int16_t volume);
	int (*get_volume)(struct radio_device *rdev, int16_t *volume);
	int (*set_mute)(struct radio_device *rdev, uint8_t mode);
	int (*get_mute)(struct radio_device *rdev, uint8_t *mode);
	int (*auto_seek)(struct radio_device *rdev, bool direction);
	int (*step_seek)(struct radio_device *rdev, bool direction);
	int (*get_freq_range)(struct radio_device *rdev, uint16_t *bottom_freq, uint16_t *top_freq);
	int (*get_stereo_mono)(struct radio_device *rdev, uint16_t *mode);
	int (*set_stereo_mono)(struct radio_device *rdev, uint16_t mode); // TODO
	int (*set_radio_mode)(struct radio_device *rdev, uint16_t mode); // TODO
	int (*set_rds_mode)(struct radio_device *rdev, uint16_t mode); // TODO
	int (*reset)(struct radio_device *rdev);
};

struct radio_device {
	struct device *dev;
	struct video_device *video_dev;
	struct v4l2_device v4l2_device;
	struct radio_controller control;
	struct kobject *kobj;

	struct mutex rdev_lock;
	unsigned int rd_index; // RDS read index
	unsigned int wr_index; // RDS write index
	wait_queue_head_t read_queue; // // RDS read queue
};

int radiodev_controller_register(struct radio_device *rdev);
void radiodev_controller_unregister(struct radio_device *rdev);

#endif
