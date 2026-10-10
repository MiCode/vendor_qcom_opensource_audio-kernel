#ifndef __OCA72XXX_MONITOR_H__
#define __OCA72XXX_MONITOR_H__

#define OCA_WAIT_DSP_OPEN_TIME			(3000)
#define OCA_VBAT_CAPACITY_MIN			(0)
#define OCA_VBAT_CAPACITY_MAX			(100)
#define OCA_VMAX_INIT_VAL			(0xFFFFFFFF)
#define OCA_VBAT_MAX				(100)
#define OCA_VMAX_MAX				(0)
#define OCA_DEFAULT_MONITOR_TIME			(3000)
#define OCA_WAIT_TIME				(3000)
#define REG_STATUS_CHECK_MAX			(10)

#define OCA_MONITOR_TIME_MIN			(0)
#define OCA_MONITOR_TIME_MAX			(50000)

enum oca_monitor_init {
	OCA_MONITOR_CFG_WAIT = 0,
	OCA_MONITOR_CFG_OK = 1,
};

enum oca_monitor_hdr_info {
	OCA_MONITOR_HDR_DATA_SIZE = 0x00000004,
	OCA_MONITOR_HDR_DATA_BYTE_LEN = 0x00000004,
};

enum oca_monitor_data_ver {
	OCA_MONITOR_DATA_VER = 0x00000001,
	OCA_MONITOR_DATA_VER_MAX,
};

enum oca_monitor_first_enter {
	OCA_FIRST_ENTRY = 0,
	OCA_NOT_FIRST_ENTRY = 1,
};

struct oca_bin_header {
	uint32_t check_sum;
	uint32_t header_ver;
	uint32_t bin_data_type;
	uint32_t bin_data_ver;
	uint32_t bin_data_size;
	uint32_t ui_ver;
	char product[8];
	uint32_t addr_byte_len;
	uint32_t data_byte_len;
	uint32_t device_addr;
	uint32_t reserve[4];
};

struct oca_monitor_header {
	uint32_t monitor_switch;
	uint32_t monitor_time;
	uint32_t monitor_count;
	uint32_t step_count;
	uint32_t reserve[4];
};

struct vmax_step_config {
	uint32_t vbat_min;
	uint32_t vbat_max;
	int vmax_vol;
};

struct oca_monitor {
	int32_t dev_index;
	uint8_t first_entry;
	uint8_t timer_cnt;
	uint32_t vbat_sum;
	int32_t custom_capacity;
	uint32_t pre_vmax;

	int bin_status;
	struct oca_monitor_header monitor_hdr;
	struct vmax_step_config *vmax_cfg;
};

void oca72xxx_monitor_cfg_free(struct oca_monitor *monitor);
int oca72xxx_monitor_bin_parse(struct device *dev,
			char *monitor_data, uint32_t data_len);
int oca72xxx_monitor_no_dsp_get_vmax(struct oca_monitor *monitor,
					int32_t *vmax);
void oca72xxx_monitor_init(struct device *dev, struct oca_monitor *monitor,
				struct device_node *dev_node);
void oca72xxx_monitor_exit(struct oca_monitor *monitor);
int oca72xxx_dev_monitor_switch_set(struct oca_monitor *monitor, uint32_t enable);

#endif
