/***********************************************************************//**
 * @file		main.c
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

#define ROOT
#include "public.h"



/*-----------------------------------------------------------------------
Function name:	main
Input:			N/A
Output:			N/A
Description:	
------------------------------------------------------------------------*/
int main(void)
{
	Init();
	Dbg_printf(" init finish\n");

	while(1)
	{
	
         //add this to project
          Tuner_Mode_Proc();
          NXP_Device_Boot_Proc();
	
	}
}







