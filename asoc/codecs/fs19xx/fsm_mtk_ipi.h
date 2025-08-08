/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2020-08-20 File created.
 */

#ifndef __FSM_MTK_H__
#define __FSM_MTK_H__

#include "fsm-dev.h"
#include <linux/firmware.h>

#define FSM_RUNIN_TEST

// FSADSP MODULE ID
#define AFE_MODULE_ID_FSADSP_TX (0x10001110)
#define AFE_MODULE_ID_FSADSP_RX (0x10001111)

// FSADSP PARAM ID
#define CAPI_V2_PARAM_FSADSP_TX_ENABLE     0x11111601
#define CAPI_V2_PARAM_FSADSP_RX_ENABLE     0x11111611
#define CAPI_V2_PARAM_FSADSP_BASE          0x10001FA0
#define CAPI_V2_PARAM_FSADSP_MODULE_ENABLE 0x10001FA1 // enable rx or tx module
#define CAPI_V2_PARAM_FSADSP_MODULE_PARAM  0x10001FA2 // send preset to adsp
#define CAPI_V2_PARAM_FSADSP_APP_MODE      0x10001FA3 // set rx effect mode(scene)
#define CAPI_V2_PARAM_FSADSP_CLIPPING_EST  0x10001FA4 // 
#define CAPI_V2_PARAM_FSADSP_TX_FMT        0x10001FA5 // tx format
#define CAPI_V2_PARAM_FSADSP_LIVEDATA      0x10001FA6 // livedata
#define CAPI_V2_PARAM_FSADSP_RE25          0x10001FA7 // re25
#define CAPI_V2_PARAM_FSADSP_CFG           0x10001FA8 // configuration
#define CAPI_V2_PARAM_FSADSP_VBAT          0x10001FA9 // Vbat
#define CAPI_V2_PARAM_FSADSP_VER           0x10001FAA // Version
#define CAPI_V2_PARAM_FSADSP_CALIB         0x10001FAB // Calibration
#define CAPI_V2_PARAM_FSADSP_ADPMODE       0x10001FAC // adp mode for spc1915
#define CAPI_V2_PARAM_FSADSP_SETTING       0x10001FAF
#define CAPI_V2_PARAM_FSADSP_PARAM_ACDB    0x10001FB0

#define APR_CHUNK_SIZE (256)
#define FSM_CALIB_PAYLOAD_SIZE (32)

enum fsm_testcase {
	FSM_TC_ENABLE_ALL = 0,
	FSM_TC_DISABLE_ALL = 1,
	FSM_TC_DISABLE_EQ = 2,
	FSM_TC_DISABLE_PROT = 3,
	FSM_TC_MAX,
};

#define FSADSP_RE25_CMD_VERSION_V1 0xA001
struct fsadsp_cal_data {
	uint16_t rstrim;
	uint16_t channel;
	int re25;
} __packed;

struct fsadsp_cmd_re25 {
	uint16_t version;
	uint16_t ndev;
	struct fsadsp_cal_data cal_data[FSM_DEV_MAX];
} __packed;

struct fsm_afe {
	int module_id;
	int port_id;
	int param_id;
	bool op_set;
	int param_size;
};
typedef struct fsm_afe fsm_afe_t;

static inline int fsm_afe_get_rx_port(void)
{
	return 0;
}

static inline int fsm_afe_get_tx_port(void)
{
	return 0;
}

int fsm_afe_mod_ctrl(bool enable);
void fsm_reset_re25_data(void);
int fsm_set_re25_data(struct fsm_re25_data *data);
int fsm_afe_read_re25(uint32_t *re25, int count);
int fsm_afe_save_re25(struct fsadsp_cmd_re25 *cmd_re25);
int fsm_afe_get_livedata(void *ldata, int size);
int fsm_afe_send_apr(struct fsm_afe *afe, void *buf, uint32_t length);
#ifdef CONFIG_FSM_CROSSFADE
int fsm_afe_swap_channels(bool force);
int fsm_afe_handle_direction(bool is_set, int *dir);
#endif

#endif // __FSM_MTK_H__
