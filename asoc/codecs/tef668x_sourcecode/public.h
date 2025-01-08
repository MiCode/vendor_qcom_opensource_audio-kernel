/***********************************************************************//**
 * @file		public.h
 * @purpose		This example used to test NXP HERO function
 * @version		0.1
 * @date		1. Oct. 2010
 * @author		NXP BU Automotive Car Entertainment Team & VITEC ELECTRONICS(SHENZHEN) AE Team
 *---------------------------------------------------------------------
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors and VITEC Electronics assume no responsibility or liability for the
 * use of the software, convey no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors and VITEC Electronics
 * reserve the right to make changes in the software without
 * notification. NXP Semiconductors and VITEC Electronics also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#include <LPC17xx.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "Typedef.h"

#include "I2C.h"
#include "AR.h"	
#ifdef SYSTEM_INCLUDES_TEF668X
#include "tef668x/Lithio_Drv.h"
#include "tef668x/Lithio_Boot.h"
#endif
#ifdef SYSTEM_INCLUDES_TEF665X
#include "tef665x/Atomic2_Drv.h"
#include "tef665x/Atomic2_Boot.h"
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
#include "tef664x/Helio_Drv.h"
#include "tef664x/Helio_Boot.h"
#endif

#inlude "Radio_Func.h"
#include "Tuner_Proc.h"





#include <math.h>





