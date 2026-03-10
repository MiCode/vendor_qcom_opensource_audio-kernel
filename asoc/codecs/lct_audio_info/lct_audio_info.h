// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 XiaoMi Inc.
 */
#ifndef __LCT_AUDIO_INFO_H__
#define __LCT_AUDIO_INFO_H__

#define AUDIO_INFO_MAX_LEN     (64)
#define CN_HW_NAME     "androidboot.hwname=CN_device"
#define GL_HW_NAME     "androidboot.hwname=GL_device"
enum audio_pa_chip_type {
	PA_CHIP_TYPE_UNKNOWN = 0,
	PA_CHIP_TYPE_SIPA = 1,
	PA_CHIP_TYPE_FSM = 2,
	PA_CHIP_TYPE_AW = 3,
};

struct lct_audio_info {
    char codec_name[AUDIO_INFO_MAX_LEN];
    char pa_name[AUDIO_INFO_MAX_LEN];
    char audio_switch_name[AUDIO_INFO_MAX_LEN];
};

#endif //__LCT_AUDIO_INFO_H__
