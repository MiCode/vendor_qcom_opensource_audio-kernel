
#ifndef _LITHIO_BOOT_H_
#define _LITHIO_BOOT_H_


/*
TEF668x state transition times will fall within the following limits :
	Power-on  -> Boot state : power supply voltage settling + 5 ms.
	Boot state ->   Idle state : 50 ms.
	Idle state  ->  Active state : 100 ms
*/
typedef enum{
	
	eDevTEF668x_Boot_state,
	eDevTEF668x_Idle_state,
	eDevTEF668x_Wait_Active,
	eDevTEF668x_Active_state,
	
	eDevTEF668x_Not_Exist,	
	eDevTEF668x_Power_on,
	eDevTEF668x_Last
}TEF668x_STATE;

//void DevTEF668x_Process(void);
void DevTEF668x_Boot_Process(void);

#endif


