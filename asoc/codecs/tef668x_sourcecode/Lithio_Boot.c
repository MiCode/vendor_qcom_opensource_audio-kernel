

#include "../public.h"


/* -----------------------------------------------------------------------------
 * Internal define:
 *-----------------------------------------------------------------------------*/
 #ifndef Lithio_Printf
 //#define Lithio_Printf(...)
 #define Lithio_Printf	Dbg_printf
 #endif


/* -----------------------------------------------------------------------------
 * Macro Define
 *-----------------------------------------------------------------------------*/
#define TEF668x_REF_CLK		9216000	//reference clock frequency
#define TEF668x_IS_EXT_CLK	0	//external clock input

#define TEF668x_TIMER_POWER   10//10ms
#define TEF668x_TIMER_WAIT_IDLE   50
#define TEF668x_TIMER_WAIT_ACTIVE   100


/* -----------------------------------------------------------------------------
 * Internal Prototypes:
 *-----------------------------------------------------------------------------*/

static TEF668x_STATE DevTEF668x_State;

static U32 DevTEF668x_Timer;
/* -----------------------------------------------------------------------------
* Exported functions:
* -----------------------------------------------------------------------------
*/

void DevTEF668x_Init(void)
{
	DevTEF668x_State = eDevTEF668x_Power_on;
	TimerSet(&DevTEF668x_Timer,TEF668x_TIMER_POWER);	//power supply voltage settling + 5 ms.
}

//return 0 if in busy
//return 1 if i2c active
//return 2 if device not exist
static int DevTEF668x_Power_on(void)
{
	static  int PowerOn_Counter=0;
	TEF668x_STATE status;
	int r;

	if(TimerHasExpired(&DevTEF668x_Timer))//Alternatively Wait radio supply power settling time + 5 ms.
	{
//Wait until device is found present on the I2C bus (‘boot state’) :
//Repeat APPL_Get_Operation_Status read until I2C acknowledge from device;
//status = 0 (‘boot state’) is found.
//[ w 40 80 01 [ r 0000 ]
		if(SUCCESS == devTEF668x_APPL_Get_Operation_Status(&status))
		{
			Lithio_Printf("TEF668x found\n");
			PowerOn_Counter = 0;
			r = 1;	//return with OK
		}
		else if(++PowerOn_Counter > 50){
			PowerOn_Counter = 0;
			r = 2;	//return with not exist
		}
		else
		{
			TimerSet(&DevTEF668x_Timer,TEF668x_TIMER_POWER);	//every + 5 ms.
			r = 0;	//return with busy

			Lithio_Printf("TEF668x not active\n");
		}
	}

	return r;
}

static int DevTEF668x_Boot_state(void)
{
	if(devTEF668x_Patch_Init())
	{
		//Command ‘Start’ will bring the device into ‘idle state’: [ w 14 0001 ]
		U8 buf[30];

		buf[0] = 0x14;
		buf[1] = 0;
		buf[2] = 1;

		if(SUCCESS ==I2C1_WriteData(TEF668x_SlaveAddr,buf, 3))
		{
			TimerSet(&DevTEF668x_Timer,TEF668x_TIMER_WAIT_IDLE);	//every + 50 ms.
			return 1;
		}
		else
		{
			Lithio_Printf("DevTEF668x_Boot_state error\n");
		}
	}
	
	return 0;
}

/*
Repeat devTEF668x_APPL_Get_Operation_Status read until status = 1 (‘idle state’) is found.
[ w 40 80 01 [ r 0001 ]
Alternatively Wait 50 ms.
Set reference frequency
*/
static int DevTEF668x_Idle_state(void)
{
	TEF668x_STATE status;
	
	if(TimerHasExpired(&DevTEF668x_Timer))//Alternatively Wait 50 ms.
	{
		if(SUCCESS == devTEF668x_APPL_Get_Operation_Status(&status)){
			if(status != eDevTEF668x_Boot_state){
				//Set reference frequency
				//try if(SUCCESS == APPL_Set_ReferenceClock(TEF668x_REF_CLK, TEF668x_IS_EXT_CLK))
				{
				//Activate : APPL_Activate mode = 1.[ w 40 05 01 0001 ]
					if(SUCCESS == APPL_Activate())
					{
						TimerSet(&DevTEF668x_Timer,TEF668x_TIMER_WAIT_ACTIVE);	

						return 1;
					}
				}
			}
		}
	}

	return 0;
}

static int DevTEF668x_Wait_Active(void)
{
	TEF668x_STATE status;
	
	if(TimerHasExpired(&DevTEF668x_Timer))//Alternatively Wait 50 ms.
	{
		if(SUCCESS == devTEF668x_APPL_Get_Operation_Status(&status)){
			if((status != eDevTEF668x_Boot_state) &&(status != eDevTEF668x_Idle_state))
			{
			     Radio_Tune_To(eAR_TuningAction_Search,8980);//try to tune GUI
				devTEF668x_Para_Load(1,1);
				
				Tuner_Update_ProgCnt(eAR_TuningAction_Preset);//set tuner to current Freq with search mode
				

				return 1;
			}
		}
	}

	return 0;
}

static int DevTEF668x_Active_state(void)
{
	return 1;
}

void DevTEF668x_Boot_Process(void)
{
	int r;
	
	switch(DevTEF668x_State)
	{
		case eDevTEF668x_Power_on:
			r= DevTEF668x_Power_on();

			if(r==1)
				DevTEF668x_State = eDevTEF668x_Boot_state;
			else if(r==2)
				DevTEF668x_State = eDevTEF668x_Not_Exist;
			break;
			
		case eDevTEF668x_Boot_state:
			if(DevTEF668x_Boot_state())
				DevTEF668x_State = eDevTEF668x_Idle_state;
			break;
			
		case eDevTEF668x_Idle_state:
			if(DevTEF668x_Idle_state())
				DevTEF668x_State = eDevTEF668x_Wait_Active;
			
			break;
			
		case eDevTEF668x_Wait_Active:
			if(DevTEF668x_Wait_Active())
				DevTEF668x_State = eDevTEF668x_Active_state;
			
			break;
			
		case eDevTEF668x_Active_state:
			DevTEF668x_Active_state();
			break;

		case eDevTEF668x_Not_Exist://do nothing
		default:
			break;
	}
}


/****************************************************************/
/* Function Calls for Control Commands and Information Requests */
/****************************************************************/


