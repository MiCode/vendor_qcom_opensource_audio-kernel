/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _BESBEV_INTERNAL_H
#define _BESBEV_INTERNAL_H

#include <asoc/wcd-clsh.h>
#include <asoc/wcd-irq.h>
#include "besbev.h"

#define BESBEV_MAX_MICBIAS 2

/* Convert from vout ctl to micbias voltage in mV */
#define  WCD_VOUT_CTL_TO_MICB(v)  (1600 + v * 50)
#define MAX_PORT 8
#define MAX_CH_PER_PORT 8

extern struct regmap_config besbev_regmap_config;

struct codec_port_info {
	u32 slave_port_type;
	u32 master_port_type;
	u32 ch_mask;
	u32 num_ch;
	u32 ch_rate;
};

enum {
	DISABLE = 0,
	ENABLE,
};

enum {
	SWR_DAC_PORT,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_VISENSE_PORT,
};

struct besbev_ctrl_platform_data {
	void *handle;
	int (*update_besbev_event)(void *handle, u16 event, u32 data);
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

struct besbev_priv {
	struct device *dev;

	int variant;
	int pa_mute;
	int curr_temp;
	u8 pa_gain;
	struct swr_device *swr_slave;
	struct snd_soc_component *component;
	bool comp_enable;
	bool speaker_present;
	bool visense_enable;
	bool visense_support;
	bool ext_vdd_spk;
	struct device_node *spmi_np;
	struct swr_port port[BESBEV_MAX_SWR_PORTS];
	int global_pa_cnt;
	int dev_mode;
	struct regmap *regmap;

	struct swr_device *swr_dev;

	s32 micb_ref[BESBEV_MAX_MICBIAS];
	s32 pullup_ref[BESBEV_MAX_MICBIAS];

	struct fw_info *fw_data;

	struct mutex micb_lock;

	bool dapm_bias_off;

	struct irq_domain *virq;
	struct wcd_irq_info irq_info;
	u32 rx_clk_cnt;
	int num_irq_regs;
	/* to track the status */
	unsigned long status_mask;

	u8 num_tx_ports;
	u8 num_rx_ports;
	struct codec_port_info
			tx_port_mapping[MAX_PORT][MAX_CH_PER_PORT];
	struct codec_port_info
			rx_port_mapping[MAX_PORT][MAX_CH_PER_PORT];
	struct regulator_bulk_data *supplies;
	struct notifier_block nblock;
	/* wcd callback to bolero */
	void *handle;
	int (*update_wcd_event)(void *handle, u16 event, u32 data);
	int (*register_notifier)(void *handle,
				struct notifier_block *nblock,
				bool enable);
	int (*wakeup)(void *handle, bool enable);
	u32 version;
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
	struct cdc_regulator *regulator;
	int num_supplies;
	char *besbev_name_prefix;
	struct snd_soc_dai_driver *dai_driver;
	struct snd_soc_component_driver *driver;
	/* Entry for version info */
	struct mutex res_lock;
	struct snd_info_entry *entry;
	struct snd_info_entry *version_entry;
	struct snd_info_entry *variant_entry;
	struct device_node *besbev_rst_np;
	struct device *spmi_dev;
	int reset_reg;
	int mbias_cnt;
	struct mutex rx_clk_lock;
	struct mutex main_bias_lock;
	bool dev_up;
	bool usbc_hs_status;
	struct notifier_block psy_nb;
	struct work_struct soc_eval_work;
	bool low_soc;
	int foundry_id_reg;
	int foundry_id;
	bool comp_support;
};

struct besbev_micbias_setting {
	u32 micb1_mv;
	u32 micb2_mv;
};

struct besbev_pdata {
	struct device_node *spmi_np;
	struct device_node *besbev_slave;
	struct besbev_micbias_setting micbias;

	struct cdc_regulator *regulator;
	int num_supplies;
	int reset_reg;
	int foundry_id_reg;
	bool speaker_present;
};

struct wcd_ctrl_platform_data {
	void *handle;
	int (*update_wcd_event)(void *handle, u16 event, u32 data);
	int (*register_notifier)(void *handle,
				 struct notifier_block *nblock,
				 bool enable);
};

enum {
	WCD_RX1,
	WCD_RX2,
	WCD_RX3
};

enum {
	/* INTR_CTRL_INT_MASK_1 */
	BESBEV_IRQ_RESERVED_0,

	/* INTR_CTRL_INT_MASK_2 */
	BESBEV_IRQ_RESERVED_1,
	BESBEV_IRQ_RESERVED_2,
};

extern void besbev_disable_bcs_before_slow_insert(
				struct snd_soc_component *component,
				bool bcs_disable);
extern int besbev_get_micb_vout_ctl_val(u32 micb_mv);
extern int besbev_micbias_control(struct snd_soc_component *component,
			int micb_num, int req, bool is_dapm);
extern int besbev_global_mbias_enable(struct snd_soc_component *component);
extern int besbev_global_mbias_disable(struct snd_soc_component *component);

#endif  /* _BESBEV_INTERNAL_H */
