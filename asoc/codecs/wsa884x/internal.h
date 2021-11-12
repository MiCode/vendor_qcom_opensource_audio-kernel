/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 */

#ifndef WSA884X_INTERNAL_H
#define WSA884X_INTERNAL_H

#include <asoc/wcd-irq.h>
#include "wsa884x.h"
#include "wsa884x-registers.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#define SWR_SLV_MAX_REG_ADDR	0x2009
#define SWR_SLV_START_REG_ADDR	0x40
#define SWR_SLV_MAX_BUF_LEN	20
#define BYTES_PER_LINE		12
#define SWR_SLV_RD_BUF_LEN	8
#define SWR_SLV_WR_BUF_LEN	32
#define SWR_SLV_MAX_DEVICES	2
#endif /* CONFIG_DEBUG_FS */

#define WSA884X_DRV_NAME "wsa884x-codec"
#define WSA884X_NUM_RETRY	5

#define WSA884X_VERSION_ENTRY_SIZE 32
#define WSA884X_VARIANT_ENTRY_SIZE 32

#define WSA884X_VERSION_1_0 0

enum {
	G_21_DB = 0,
	G_19P5_DB,
	G_18_DB,
	G_16P5_DB,
	G_15_DB,
	G_13P5_DB,
	G_12_DB,
	G_10P5_DB,
	G_9_DB,
	G_7P5_DB,
	G_6_DB,
	G_4P5_DB,
	G_3_DB,
	G_1P5_DB,
	G_0_DB,
	G_M1P5_DB,
	G_M3_DB,
	G_M4P5_DB,
	G_M6_DB,
	G_MAX_DB,
};

enum {
	PA_AUX_DISABLE = 0,
	PA_AUX_M6_DB =   3,
	PA_AUX_M4P5_DB = 4,
	PA_AUX_M3_DB =   5,
	PA_AUX_M1P5_DB = 6,
	PA_AUX_0_DB =    7,
	PA_AUX_7P5_DB =  8,
	PA_AUX_12_DB =   9,
	PA_AUX_18_DB = 0xA,
};

enum {
	ISENSE_6_DB = 0,
	ISENSE_12_DB,
	ISENSE_15_DB,
	ISENSE_18_DB
};

enum {
	VSENSE_M12_DB = 0,
	VSENSE_M15_DB,
	VSENSE_M18_DB,
	VSENSE_M21_DB,
	VSENSE_M24_DB
};

enum {
	DISABLE = 0,
	ENABLE,
};

enum {
	SWR_DAC_PORT = 0,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_PBR_PORT,
	SWR_VISENSE_PORT,
	SWR_CPS_PORT
};

struct wsa_ctrl_platform_data {
	void *handle;
	int (*update_wsa_event)(void *handle, u16 event, u32 data);
	int (*register_notifier)(void *handle, struct notifier_block *nblock,
				bool enable);
};

struct swr_port {
	u8 port_id;
	u8 ch_mask;
	u32 ch_rate;
	u8 num_ch;
	u8 port_type;
};

extern struct regmap_config wsa884x_regmap_config;

/*
 * Private data Structure for wsa884x. All parameters related to
 * WSA884X codec needs to be defined here.
 */
struct wsa884x_priv {
	struct regmap *regmap;
	struct device *dev;
	struct swr_device *swr_slave;
	struct snd_soc_component *component;
	bool comp_enable;
	bool visense_enable;
	bool cps_enable;
	bool pbr_enable;
	bool ext_vdd_spk;
	bool dapm_bias_off;
	struct swr_port port[WSA884X_MAX_SWR_PORTS];
	int global_pa_cnt;
	int dev_mode;
	int comp_offset;
	struct mutex res_lock;
	struct snd_info_entry *entry;
	struct snd_info_entry *version_entry;
	struct snd_info_entry *variant_entry;
	struct device_node *wsa_rst_np;
	int pa_mute;
	int curr_temp;
	int variant;
	int version;
	u8 pa_gain;
	u8 bat_cfg;
	u8 rload;
	u8 system_gain;
	int min_gain;
	int pa_aux_gain;
	struct irq_domain *virq;
	struct wcd_irq_info irq_info;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dent;
	struct dentry *debugfs_peek;
	struct dentry *debugfs_poke;
	struct dentry *debugfs_reg_dump;
	unsigned int read_data;
#endif
	struct device_node *parent_np;
	struct platform_device *parent_dev;
	struct notifier_block parent_nblock;
	void *handle;
	int (*register_notifier)(void *handle,
				struct notifier_block *nblock, bool enable);
	struct cdc_regulator *regulator;
	int num_supplies;
	struct regulator_bulk_data *supplies;
	unsigned long status_mask;
	struct snd_soc_dai_driver *dai_driver;
	struct snd_soc_component_driver *driver;
};

#endif /* WSA884X_INTERNAL_H */
