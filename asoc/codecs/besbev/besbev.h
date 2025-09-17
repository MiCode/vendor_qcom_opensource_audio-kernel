/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*/

#ifndef _BESBEV_H
#define _BESBEV_H

#define BESBEV_MAX_SWR_PORTS 4
#define BESBEV_DRV_NAME "besbev_codec"
#define CONFIG_BESBEV_COMP_SUPPORT 0

#ifdef CONFIG_SND_SOC_BESBEV

struct besbev_temp_register {
	u8 d1_msb;
	u8 d1_lsb;
	u8 d2_msb;
	u8 d2_lsb;
	u8 dmeas_msb;
	u8 dmeas_lsb;
};

extern int besbev_info_create_codec_entry(struct snd_info_entry *codec_root,
				    struct snd_soc_component *component);

extern int besbev_disable_visense(struct snd_soc_component *component);

extern int besbev_amic_init(struct snd_soc_component *component);
#else
extern int besbev_info_create_codec_entry(struct snd_info_entry *codec_root,
				    struct snd_soc_component *component)
{
	return 0;
}

extern int besbev_disable_visense(struct snd_soc_component *component)
{
	return 0;
}

extern int besbev_amic_init(struct snd_soc_component *component)
{
	return 0;
}
#endif /* CONFIG_SND_SOC_BESBEV */

#endif /* _BESBEV_H */
