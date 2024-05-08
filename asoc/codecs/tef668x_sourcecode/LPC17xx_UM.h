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

#ifndef LPC_17XX_UM_H
#define LPC_17XX_UM_H

#include "Typedef.h"

//refrence to lpc17xx_um.pdf page64

#define PCTIM0	bit1
#define PCTIM1	bit2
#define PCUART0	bit3
#define PCUART1	bit4

#define PCPWM1	bit6
#define PCI2C0	bit7
#define PCSPI	bit8
#define PCRTC	bit9
#define PCSSP1	bit10

#define PCADC	bit12
#define PCCAN1	bit13
#define PCCAN2	bit14
#define PCRIT	bit16
#define PCMCPWM	bit17
#define PCQEI	bit18
#define PCI2C1	bit19

#define PCSSP0	bit21
#define PCTIM2	bit22
#define PCTIM3	bit23
#define PCUART2	bit24
#define PCUART3	bit25
#define PCI2C2	bit26
#define PCI2S	bit27

#define PCGPDMA	bit29
#define PCENET	bit30
#define PCUSB	bit31

//I2C0CONSET and I2C1CONSET used to configure Master mode
#define I2EN 		bit6		//I2C interface enable
#define STA 		bit5		//START flag
#define STO 		bit4		//STOP flag
#define SI 		bit3		//I2C interrupt flag
#define AA		bit2		//Assert acknowledge flag

//I2CONCLR
#define I2ENC	bit6		//I2C interface Disable bit
#define STAC 	bit5		//START flag Clear bit

#define SIC 		bit3		//I2C interrupt Clear bit.
#define AAC		bit2		//Assert acknowledge Clear bit.

#endif

