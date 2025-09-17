/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024. Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ROULEUR_H
#define _ROULEUR_H

#include <bindings/audio-codec-port-types.h>


#define ROULEUR_MAX_SLAVE_CH_TYPES 10
#define ZERO 0

struct rouleur_swr_slave_ch_map {
        u8 ch_type;
        u8 index;
};

/*
 * #define MBHC 29
#define ADC1 30
#define ADC2 31
#define ADC3 32
#define ADC4 33
#define DMIC0 34
#define DMIC1 35
#define DMIC2 36
#define DMIC3 37
#define DMIC4 38
#define DMIC5 39
#define DMIC6 40
#define DMIC7 41
#define DMIC8 42
#define DMIC9 43
#define DMIC10 44
*/

static const struct rouleur_swr_slave_ch_map rouleur_swr_slv_tx_ch_idx[] = {
        {ADC1, 0},
        {ADC2, 1},
        {ADC3, 2},
        {DMIC0, 3},
        {DMIC1, 4},
        {MBHC, 5},
        {DMIC2, 6},
        {DMIC3, 7},
        {DMIC4, 8},
        {DMIC5, 9},
};

/* SWRM_TX1_CH1 46
   SWRM_TX1_CH2 47
   SWRM_TX1_CH3 48
   SWRM_TX1_CH4 49
   SWRM_TX2_CH1 50
   SWRM_TX2_CH2 51
   SWRM_TX2_CH3 52
   SWRM_TX2_CH4 53
   SWRM_TX3_CH1 54
   SWRM_TX3_CH2 55
   SWRM_TX3_CH3 56
   SWRM_TX3_CH4 57
   SWRM_TX_PCM_IN 58
   SWRM_RX_PCM_IN 59
*/

static int rouleur_swr_master_ch_map[] = {
        ZERO,
        SWRM_TX1_CH1,
        SWRM_TX1_CH2,
        SWRM_TX1_CH3,
        SWRM_TX1_CH4,
        SWRM_TX2_CH1,
        SWRM_TX2_CH2,
        SWRM_TX2_CH3,
        SWRM_TX2_CH4,
        SWRM_TX3_CH1,
        SWRM_TX3_CH2,
        SWRM_TX3_CH3,
        SWRM_TX3_CH4,
        SWRM_TX_PCM_IN,
};


#ifdef CONFIG_SND_SOC_ROULEUR
static inline int rouleur_slave_get_master_ch_val(int ch)
{
        int i;

        for (i = 0; i < ROULEUR_MAX_SLAVE_CH_TYPES; i++)
                if (ch == rouleur_swr_master_ch_map[i])
                        return i;
        return 0;
}

static inline int rouleur_slave_get_master_ch(int idx)
{
        return rouleur_swr_master_ch_map[idx];
}

static inline int rouleur_slave_get_slave_ch_val(int ch)
{
        int i;

        for (i = 0; i < ROULEUR_MAX_SLAVE_CH_TYPES; i++)
                if (ch == rouleur_swr_slv_tx_ch_idx[i].ch_type)
                        return rouleur_swr_slv_tx_ch_idx[i].index;

        return -EINVAL;
}

extern int rouleur_info_create_codec_entry(struct snd_info_entry *codec_root,
				    struct snd_soc_component *component);
#else
extern int rouleur_info_create_codec_entry(struct snd_info_entry *codec_root,
				    struct snd_soc_component *component)
{
	return 0;
}
#endif /* CONFIG_SND_SOC_ROULEUR */

#endif
