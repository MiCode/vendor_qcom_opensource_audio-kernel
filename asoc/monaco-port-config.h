/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MONACO_PORT_CONFIG
#define _MONACO_PORT_CONFIG

#include <soc/swr-common.h>

/*
 * Add port configuration in the format
 *{ si, off1, off2, hstart, hstop, wd_len, bp_mode, bgp_ctrl, lane_ctrl, dir, stream_type}
 */

static struct port_params rx_frame_params_default[SWR_MSTR_PORT_LEN] = {
	{3,  0,  0,  0xFF, 0xFF, 1,    0xFF, 0xFF, 1},
	{31, 0,  0,  3,    6,    7,    0,    0xFF, 0},
	{31, 11, 11, 0xFF, 0xFF, 4,    1,    0xFF, 0},
	{7,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0},
	{0,  0,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0,    0},
};

static struct port_params rx_frame_params_dsd[SWR_MSTR_PORT_LEN] = {
	{3,  0,  0,  0xFF, 0xFF, 1,    0xFF, 0xFF, 1},
	{31, 0,  0,  3,    6,    7,    0,    0xFF, 0},
	{31, 11, 11, 0xFF, 0xFF, 4,    1,    0xFF, 0},
	{7,  9,  0,  0xFF, 0xFF, 0xFF, 0xFF, 1,    0},
	{3,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 3,    0},
};

/* TX UC1: TX1: 1ch, TX2: 2chs, TX3: 1ch(MBHC) */
static struct port_params tx_frame_params_default[SWR_MSTR_PORT_LEN] = {
	{3,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0},  /* TX1 */
	{3,  2,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0},  /* TX2 */
	{3,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0},  /* TX3 */
};

/* TX UC1: TX1: 1ch, TX2: 2chs */
static struct port_params tx_frame_params_besbev[SWR_MSTR_PORT_LEN] = {
	{3,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0},  /* TX1 */
	{3,  2,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0},  /* TX2 */
};

static struct port_params rx_frame_params_besbev[SWR_MSTR_PORT_LEN] = {
	{3,  0,  0,  0xFF, 0xFF, 1,    0xFF, 0xFF, 1, 0, 0},
	{31, 0,  0,  3,    6,    7,    0,    0xFF, 0, 0, 0},
	{31, 1,  0,  0xFF, 0xFF, 4,    1,    0xFF, 0, 0, 0},
	{7,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0}, /* AUX OUT */
	{0,  0,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0,    0, 0, 0},
	{0,  0,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0,    0, 0, 0},
	{3,  3,  0,  0xFF, 0xFF, 0xFF, 0x1, 0xFF,  0, 1, 1}, /* AMIC PCM IN */
};

static struct port_params rx_frame_params_visense[SWR_MSTR_PORT_LEN] = {
	{3,  0,  0,  0xFF, 0xFF, 1,    0xFF, 0xFF, 1, 0, 0},
	{31, 0,  0,  3,    6,    7,    0,    0xFF, 0, 0, 0},
	{31, 1,  0,  0xFF, 0xFF, 4,    1,    0xFF, 0, 0, 0},
	{7,  1,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0}, /* AUX OUT */
	{0,  0,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0,    0, 0, 0},
	{0,  0,  0,  0xFF, 0xFF, 0xFF, 0xFF, 0,    0, 0, 0},
	{7,  3,  0,  0xFF, 0xFF, 0xFF, 0x1,  0xFF, 0xFF, 1, 1}, /* VISENSE PCM IN */
};

static struct swr_mstr_port_map sm_port_map[] = {
	{VA_MACRO, SWR_UC0, tx_frame_params_default},
	{RX_MACRO, SWR_UC0, rx_frame_params_default},
	{RX_MACRO, SWR_UC1, rx_frame_params_dsd},
};

static struct swr_mstr_port_map sm_port_map_besbev[] = {
	{VA_MACRO, SWR_UC0, tx_frame_params_besbev},
	{RX_MACRO, SWR_UC0, rx_frame_params_visense},
};

static struct swr_mstr_port_map sm_port_map_besbev_amic[] = {
	{RX_MACRO, SWR_UC0, rx_frame_params_besbev},
};
#endif /* _MONACO_PORT_CONFIG */
