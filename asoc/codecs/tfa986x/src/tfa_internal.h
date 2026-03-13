/* 
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 * Copyright 2020 GOODIX 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
	internal functions for TFA layer (not shared with SRV and HAL layer!)
*/

#ifndef __TFA_INTERNAL_H__
#define __TFA_INTERNAL_H__

#include "tfa_dsp_fw.h"
#include "tfa_ext.h"

#if __GNUC__ >= 4
  #define TFA_INTERNAL __attribute__ ((visibility ("hidden")))
#else
  #define TFA_INTERNAL
#endif

#define TFA98XX_GENERIC_SLAVE_ADDRESS 0x1C

TFA_INTERNAL enum Tfa98xx_Error tfa98xx_check_rpc_status(struct tfa_device *tfa, int *pRpcStatus);
TFA_INTERNAL enum Tfa98xx_Error tfa98xx_wait_result(struct tfa_device *tfa, int waitRetryCount);

/* bitfield extraction macros */
#define TFA_BF_REG(bf) ((bf & (0xff00)) >> 8)
#define TFA_BF_POS(bf) ((bf & (0xf0)) >> 4)
#define TFA_BF_WIDTH(bf) ((bf & (0x0f)) + 1)
#define TFA_BF_MSK(bf) (((1 << TFA_BF_WIDTH(bf)) - 1) << TFA_BF_POS(bf))

#endif /* __TFA_INTERNAL_H__ */

