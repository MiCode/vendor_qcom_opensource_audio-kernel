#ifndef __OCA72XXX_ACF_BIN_H__
#define __OCA72XXX_ACF_BIN_H__

#include "ocspa_device.h"

#define OCA_PROJECT_NAME_MAX		(24)
#define OCA_CUSTOMER_NAME_MAX		(16)
#define OCA_CFG_VERSION_MAX		(4)
#define OCA_TBL_VERSION_MAX		(4)
#define OCA_DDE_DEVICE_TYPE		(0)
#define OCA_DDE_SKT_TYPE			(1)
#define OCA_DDE_DEFAULT_TYPE		(2)

#define OCA_REG_ADDR_BYTE		(1)
#define OCA_REG_DATA_BYTE		(1)

#define OCA_PROFILE_STR_MAX 		(32)
#define OCA_POWER_OFF_NAME_SUPPORT_COUNT	(5)

enum oca_cfg_hdr_version {
	OCA_ACF_HDR_VER_0_0_0_1 = 0x00000001,
	OCA_ACF_HDR_VER_1_0_0_0 = 0x01000000,
};

enum oca_acf_dde_type_id {
	OCA_DEV_NONE_TYPE_ID = 0xFFFFFFFF,
	OCA_DDE_DEV_TYPE_ID = 0x00000000,
	OCA_DDE_SKT_TYPE_ID = 0x00000001,
	OCA_DDE_DEV_DEFAULT_TYPE_ID = 0x00000002,
	OCA_DDE_TYPE_MAX,
};

enum oca_roca_data_type_id {
	OCA_BIN_TYPE_REG = 0x00000000,
	OCA_BIN_TYPE_DSP,
	OCA_BIN_TYPE_DSP_CFG,
	OCA_BIN_TYPE_DSP_FW,
	OCA_BIN_TYPE_HDR_REG,
	OCA_BIN_TYPE_HDR_DSP_CFG,
	OCA_BIN_TYPE_HDR_DSP_FW,
	OCA_BIN_TYPE_MUTLBIN,
	OCA_SKT_UI_PROJECT,
	OCA_DSP_CFG,
	OCA_MONITOR,
	OCA_BIN_TYPE_MAX,
};

enum {
	OCA_DEV_TYPE_OK = 0,
	OCA_DEV_TYPE_NONE = 1,
};

enum oca_profile_status {
	OCA_PROFILE_WAIT = 0,
	OCA_PROFILE_OK,
};

enum oca_acf_load_status {
	OCA_ACF_WAIT = 0,
	OCA_ACF_UPDATE,
};

enum oca_bin_dev_profile_id {
	OCA_PROFILE_MUSIC = 0x0000,
	OCA_PROFILE_VOICE,
	OCA_PROFILE_VOIP,
	OCA_PROFILE_RINGTONE,
	OCA_PROFILE_RINGTONE_HS,
	OCA_PROFILE_LOWPOWER,
	OCA_PROFILE_BYPASS,
	OCA_PROFILE_MMI,
	OCA_PROFILE_FM,
	OCA_PROFILE_NOTIFICATION,
	OCA_PROFILE_RECEIVER,
	OCA_PROFILE_OFF,
	OCA_PROFILE_MAX,
};

struct oca_acf_hdr {
	int32_t a_id;				/* acf file ID 0xa15f908 */
	char project[OCA_PROJECT_NAME_MAX];	/* project name */
	char custom[OCA_CUSTOMER_NAME_MAX];	/* custom name :huawei xiaomi vivo oppo */
	uint8_t version[OCA_CFG_VERSION_MAX];	/* author update version */
	int32_t author_id;			/* author id */
	int32_t ddt_size;			/* sub section table entry size */
	int32_t dde_num;			/* sub section table entry num */
	int32_t ddt_offset;			/* sub section table offset in file */
	int32_t hdr_version;			/* sub section table version */
	int32_t reserve[3];			/* Reserved Bits */
};

struct oca_acf_dde {
	int32_t type;				/* dde type id */
	char dev_name[OCA_CUSTOMER_NAME_MAX];	/* customer dev name */
	int16_t dev_index;			/* dev id */
	int16_t dev_bus;			/* dev bus id */
	int16_t dev_addr;			/* dev addr id */
	int16_t dev_profile;			/* dev profile id */
	int32_t data_type;			/* data type id */
	int32_t data_size;			/* dde data size in block */
	int32_t data_offset;			/* dde data offset in block */
	int32_t data_crc;			/* dde data crc checkout */
	int32_t reserve[5];			/* Reserved Bits */
};

struct oca_acf_dde_v_1_0_0_0 {
	uint32_t type;				/* DDE type id */
	char dev_name[OCA_CUSTOMER_NAME_MAX];	/* customer dev name */
	uint16_t dev_index;			/* dev id */
	uint16_t dev_bus;			/* dev bus id */
	uint16_t dev_addr;			/* dev addr id */
	uint16_t dev_profile;			/* dev profile id*/
	uint32_t data_type;			/* data type id */
	uint32_t data_size;			/* dde data size in block */
	uint32_t data_offset;			/* dde data offset in block */
	uint32_t data_crc;			/* dde data crc checkout */
	char dev_profile_str[OCA_PROFILE_STR_MAX];	/* dde custom profile name */
	uint32_t chip_id;			/* dde custom product chip id */
	uint32_t reserve[4];
};

struct oca_data_with_header {
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

struct oca_data_container {
	uint32_t len;
	uint8_t *data;
};

struct oca_prof_desc {
	uint32_t prof_st;
	char *prof_name;
	char dev_name[OCA_CUSTOMER_NAME_MAX];
	struct oca_data_container data_container;
};

struct oca_all_prof_info {
	struct oca_prof_desc prof_desc[OCA_PROFILE_MAX];
};

struct oca_prof_info {
	int count;
	int status;
	int prof_type;
	char (*prof_name_list)[OCA_PROFILE_STR_MAX];
	struct oca_prof_desc *prof_desc;
};

struct acf_bin_info {
	int load_count;
	int fw_size;
	int16_t dev_index;
	char *fw_data;
	int product_cnt;
	const char **product_tab;
	struct oca_device *oca_dev;

	struct oca_acf_hdr acf_hdr;
	struct oca_prof_info prof_info;
};

char *oca72xxx_ctos_get_prof_name(int profile_id);
void oca72xxx_acf_profile_free(struct device *dev,
		struct acf_bin_info *acf_info);
int oca72xxx_acf_parse(struct device *dev, struct acf_bin_info *acf_info);
struct oca_prof_desc *oca72xxx_acf_get_prof_desc_form_name(struct device *dev,
			struct acf_bin_info *acf_info, char *profile_name);
int oca72xxx_acf_get_prof_index_form_name(struct device *dev,
			struct acf_bin_info *acf_info, char *profile_name);
char *oca72xxx_acf_get_prof_name_form_index(struct device *dev,
			struct acf_bin_info *acf_info, int index);
int oca72xxx_acf_get_profile_count(struct device *dev,
			struct acf_bin_info *acf_info);
char *oca72xxx_acf_get_prof_off_name(struct device *dev,
			struct acf_bin_info *acf_info);
void oca72xxx_acf_init(struct oca_device *oca_dev, struct acf_bin_info *acf_info, int index);


#endif
