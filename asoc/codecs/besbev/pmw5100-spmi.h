/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _PMW5100_SPMI_H
#define _PMW5100_SPMI_H

#ifdef CONFIG_PMW5100_SPMI
int pmw5100_spmi_write(struct device *dev, int reg, int value);
int pmw5100_spmi_read(struct device *dev, int reg, int *value);
#else
int pmw5100_spmi_write(struct device *dev, int reg, int value)
{
	return 0;
}
int pmw5100_spmi_read(struct device *dev, int reg, int *value);
{
	return 0;
}
#endif	/* CONFIG_PMW5100_SPMI */

#endif  /* _PMW5100_SPMI_H */
