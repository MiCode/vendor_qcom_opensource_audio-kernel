/***********************************************************************//**
 * @file		tune_proc.c
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
#include "public.h"





/*rewrite freqency waite timer*/
static U32 TunerWaitTimer; 
/*fm stereo detection timer*/
static U32 StereoDetTimer; 
/*scan start freq bakup*/
 static U16 Radio_ScanStartFreq;



 /*extern variable declaration*/
extern const FreqBaundDef FreqBaundConfig[MaxBandNum];
extern const Radio_AreaConfigDef Radio_AreaConfig;
extern StationMemType StationRecord[MaxStationNum]; 

/*Tuner function declaration  */
void Tuner_Event_End(void);
void Tuner_Turn_ON(void);
void Tuner_KeyFilter_Process(void);
void Tuner_Key_Process(void);
void Tuner_Set_Mode(void);
void Tuner_ModeExecute_Process(void);

void Tuner_Update_ProgCnt(U8 mode);
void Tuner_Check_Stereo(void);
void Tuner_Tuning_Loop_Interrupt(void);
void Tuner_Scan_Sub(void);
void Tuner_Seek_Sub(void);
void Tuner_BandChange_Sub(void);
void Tuner_PresetRead_Sub(void);
void Tuner_PresetWrite_Sub(void);
void Tuner_Manual_Sub(void);

void Para_Align_Process(void);
void Tuner_SetSeekSens_Sub(void);




/*====================================================
 Function:Tuner_Mode_Proc
 Input: 
      Null
 OutPut:
      Null
 Desp:
       Tuner process,all tuner funtion run in this function
=========================================================*/
void Tuner_Mode_Proc(void)
{
        /*check system mode*/
	if((F_Sys_Reset)||(F_Hero_InitOK==0)||(Curr_Work_Mode==GUI))
		return;
	
	if(Last_Work_Mode==Tuner)	{/*exit tuner mode*/
		Last_Work_Mode=0xff;
		Tuner_Event_End();
		return;
	}
	if(Next_Work_Mode==Tuner)/*enter into tuner moder*/
	{
		Next_Work_Mode=0xff;
		Curr_Work_Mode=Tuner;	  
		/*turn on*/
		Tuner_Turn_ON();
		/*audio input select*/
		Audio_SetPrimaryInputSource(INPUT_SOURCE_RADIO);		//Sin(0);		
		PowerONTimer = 0;
		/*select audio channel*/
		Audio_Set();		
	}
	if(Curr_Work_Mode==Tuner)
	{       /*to make sure get the correct tuner key*/
		Tuner_KeyFilter_Process();
		/*to judge turner main work mode*/
		Tuner_Key_Process();
		/*to run tuner according to work mod*/
		Tuner_ModeExecute_Process();
	}
}

/*====================================================
 Function:Tuner_Event_End
 Input: 
      Null
 OutPut:
      Null
 Desp:
       exit current tuner work mode
=========================================================*/
void Tuner_Event_End(void)
{      
	/*set tuner in idle status*/
	TunerWorkMode=0;
	TunerSubMode=0;
	F_Store_Station_Ok=FALSE;

	/*stop TunerWaitTimer*/
	TimerStop(&TunerWaitTimer);
	/*stop StereoDetTimer*/
	TimerStop(&StereoDetTimer);

	//Save current station
	Radio_Save_Station(0,Radio_GetCurrentFreq());
}

/*====================================================
 Function:Tuner_StatusResume
 Input: 
      Null
 OutPut:
      Null
 Desp:
=========================================================*/
void Tuner_StatusResume(void)
{
	Tuner_Update_ProgCnt(eAR_TuningAction_Preset);
}
/*====================================================
 Function:Tuner_Turn_ON
 Input: 
      Null
 OutPut:
      Null
 Desp:
      tuner  power on
=========================================================*/
void Tuner_Turn_ON(void)
{
	Tuner_StatusResume();	 
	Tuner_Event_End();
	return;
}
/*====================================================
 Function:Tuner_Update_ProgCnt(U8 mode )
 Input: 
      Null
 OutPut:
      Null
 Desp:
     update tuner pll 
=========================================================*/
void Tuner_Update_ProgCnt(AR_TuningAction_t mode )
{
	Radio_SetFreq(mode,Radio_GetCurrentBand(),Radio_GetCurrentFreq());   
}

/*====================================================
 Function:Tuner_KeyFilter_Process
 Input: 
      Null
 OutPut:
      Null
 Desp:
      get actival keycode
=========================================================*/
void Tuner_KeyFilter_Process(void)
{
	//to judge wether it`s valid key of current work mode
	TunerKeyCode=KeyCode;
}
/*====================================================
 Function:Tuner_Key_Process
 Input: 
      Null
 OutPut:
      Null
 Desp:
      enter tuner work mode
=========================================================*/
void Tuner_Key_Process(void)
{
	switch(TunerKeyCode)
	{
		case 0:
			switch(TunerWorkMode)
			{
				case K_TUNE_UP:
				case K_TUNE_DN:	
				case K_LM1://Save preset station
				case K_LM2:	
				case K_LM3:
				case K_LM4:
				case K_LM5:
				case K_LM6:
					Tuner_Event_End();
					break;
				default:
					break;
			}
			break;
		case K_TUNE_UP:
		case K_TUNE_DN:/*manually  tune*/
			Display_DisableTimeout();
			Tuner_Set_Mode();
			F_Tune_UP=0;
			if(TunerKeyCode==K_TUNE_UP)
			    	F_Tune_UP=1;
			Tuner_Manual_Sub();
			break;
		case K_SCAN:/* scan*/
	
			Display_DisableTimeout();
			if(TunerWorkMode==TunerKeyCode)/*stop scan*/
			{
				/*interrupt loop_tuning and recall last station then end!*/
				Tuner_Tuning_Loop_Interrupt();	
				break;
			}
		case K_AS:	
		case K_SEEK_UP:
		case K_SEEK_DN:/*seek*/
			Display_DisableTimeout();
			if(TunerWorkMode!=TunerKeyCode)
			{
				Tuner_Set_Mode();
			}	
			break;
		case K_BAND:/*next band*/
			Display_DisableTimeout();
			Tuner_Set_Mode();
			break;
		case K_LM1:
		case K_LM2:	
		case K_LM3:
		case K_LM4:
		case K_LM5:
		case K_LM6:/*store station*/
	//		Display_DisableTimeout();
			if(TunerWorkMode!=0){
				break;
			}	
			if(TunerWorkMode!=TunerKeyCode)
			   	Tuner_Set_Mode();				
			break;
		case K_SM1:
		case K_SM2:	
		case K_SM3:
		case K_SM4:
		case K_SM5:
	//	case K_SM6:/*preset station*/
			Display_DisableTimeout();
			if(TunerWorkMode!=TunerKeyCode)
			   	Tuner_Set_Mode();			
			break;
		case K_RDS:
			Tuner_SetSeekSens_Sub();	
			break;	
		default:
			break;
	}
}
/*====================================================
 Function:Tuner_Set_Mode
 Input: 
      Null
 OutPut:
      Null
 Desp:
      set tuner work mode
=========================================================*/
void Tuner_Set_Mode(void)
{
	TunerWorkMode=TunerKeyCode;
	TunerSubMode=10;
	return;
}

/*====================================================
 Function:Tuner_Manual_Sub
 Input: 
      Null
 OutPut:
      Null
 Desp:
       manual set tuner sub
=========================================================*/
void Tuner_Manual_Sub(void)
{
	/*change one step of tuning frequency*/
	Radio_ChangeFreqOneStep(F_Tune_UP);

	/*write "new programmble value" into PLL*/
	Tuner_Update_ProgCnt(eAR_TuningAction_Preset);
	/*clear station number*/
	Radio_ClearCurrentStation();
}

/*====================================================
 Function:Tuner_Tuning_Loop_Interrupt
 Input: 
      Null
 OutPut:
      Null
 Desp:
       exit tuning loop
=========================================================*/
void Tuner_Tuning_Loop_Interrupt(void)
{
	if(TunerWorkMode==K_SCAN)/*if Scan mode set to pre station*/
	{

	}
	/*Rewrite freqency*/	
	Tuner_Update_ProgCnt(eAR_TuningAction_Preset);		
	Tuner_Event_End();
}
/*====================================================
 Function:Tuner_Seek_Sub
 Input: 
      Null
 OutPut:
      Null
 Desp:
       auto seek sub
=========================================================*/
void Tuner_Seek_Sub(void)
{
	/*to process seek event step by step*/
	switch(TunerSubMode)
	{
		case 10:/*Seek Tuning start
			display off 'M*','STERO', if it has*/
			F_Stereo_Disp=0;
			Radio_ScanStepCount=0;//reset scan step count
			Radio_ScanStartFreq=Radio_GetCurrentFreq();//save current freq
			TunerSubMode=20;
			break;
		case 20:/*Loop start*/
		        Radio_ScanStepCount++;
			/*increase or decreas one step*/
			Radio_ChangeFreqOneStep(F_Tune_UP);

			/*write "new programmble value" into PLL*/
			Tuner_Update_ProgCnt(eAR_TuningAction_Search);
			
			/*set wait time for PLL lock*/
			TimerSet(&TunerWaitTimer,TUNER_SCAN_WAITE_TIME);

			TunerSubMode=30;//set tunersubmode
			Radio_CheckStationInit();//set check SD and IF start step
			Radio_ClearCurrentStation();//clear station number
			break;
			
		/*check Signel level and IF step after wait timer */
		case 30:
			if(!TimerHasExpired(&TunerWaitTimer))  /*time out check*/
			break;
			/*station check*/
			Radio_CheckStation( );   
			if( Radio_CheckStationStatus() >= NO_STATION)
				TunerSubMode = 40; 
			break;

		case 40:
			if(Radio_CheckStationStatus()== NO_STATION)  {/*no station*/			 	
				if(Radio_ScanStartFreq==Radio_GetCurrentFreq())  
					TunerSubMode = 50;	
				else 
					TunerSubMode = 20;	/*判断是否扫描完一周，如果是则停止。*/
			}
			else if(Radio_CheckStationStatus()== PRESENT_STATION){/*已经收到台*/         
				TunerSubMode = 50;
			}
			break;
			
		case 50:/*stop Seek Tuning*/
			/*Tuner finish current work mode*/
			Tuner_Update_ProgCnt(eAR_TuningAction_Preset);
			Tuner_Event_End();
			break;
	}
	return;
}
/*====================================================
 Function:Tuner_Scan_Sub
 Input: 
      Null
 OutPut:
      Null
 Desp:
       scan key action
=========================================================*/
void Tuner_Scan_Sub(void)
{
    	switch(TunerSubMode)
	{
		case 10:/*Seek Tuning start init*/
			F_Stereo_Disp=0;
			Radio_ScanStepCount=0;//reset scan step count
		        Radio_ScanStartFreq=Radio_GetCurrentFreq();//save current freq
			TunerSubMode=20;
			break;
		case 20:/*Loop start*/
			/*increase or decreas one step*/
			Radio_ScanStepCount++;			
			Radio_ChangeFreqOneStep(F_Tune_UP);

			/*write "new programmble value" into PLL*/
			Tuner_Update_ProgCnt(eAR_TuningAction_Search);
			/*set wait time for PLL lock*/	
			TimerSet(&TunerWaitTimer,TUNER_SCAN_WAITE_TIME);

			TunerSubMode=30;//set tunersubmode
			Radio_CheckStationInit();//set check  and IF start step
			Radio_ClearCurrentStation();//Clear station number
			break;
			
		
		case 30:/*check Signel level and IF step after wait timer */
			/*check stereo*/
		       Tuner_Check_Stereo();
			
			/*time out check*/
			if(!TimerHasExpired(&TunerWaitTimer))  
				break;
					   
			/*Check station*/		   
			Radio_CheckStation( );   

			if( Radio_CheckStationStatus() >=NO_STATION)
				TunerSubMode = 40; 
			break;

		case 40:
			if(Radio_CheckStationStatus()== NO_STATION)  {    /*no station*/   
				//TunerSubMode = 20;	//set next station
				if(Radio_ScanStartFreq==Radio_GetCurrentFreq())  
				{
				        Tuner_Update_ProgCnt(eAR_TuningAction_Preset);
				         Tuner_Event_End();	
				}
				else 
					TunerSubMode = 20;	/*判断是否扫描完一周，如果是则停止。*/
			}
			else if(Radio_CheckStationStatus()== PRESENT_STATION){
				TunerSubMode = 50;
				TimerSet(&TunerWaitTimer,TUNER_SCAN_PLAY_TIME);//play 5s
				Tuner_Update_ProgCnt(eAR_TuningAction_Preset);
			}
			break;
			
		case 50:
			/*check stereo */
			 Tuner_Check_Stereo();
			/*check if time out*/
			 if(!TimerHasExpired(&TunerWaitTimer)) 
			  	break;
			  /*play time end*/
			 if(Radio_ScanStartFreq==Radio_GetCurrentFreq())  
			 {
			         Tuner_Event_End();	
			 }
			else 
				TunerSubMode = 20;	
			 break;
	}
}

unsigned char TunerSmeter[7];//store 6 smeter values used for auto store stations. [0] not used!!
unsigned char  TunerMemNum;
#define AutoStore_TotalNum	6
/*=============================================
Name:		Tuner_Smeter_Store
Description:	store strongest 6 stations according to smeter value
==============================================*/
void Tuner_Smeter_Store(void)
{

	unsigned char i,j=0;
	unsigned char TempValue;
	unsigned char  CurrentBand=Radio_GetCurrentBand();
	TempValue=0;

	if(TunerMemNum>AutoStore_TotalNum)
	{	//already store buffer is full
		TunerMemNum=AutoStore_TotalNum;

		//to judge which smeter is the lowest??
		TempValue=RadioDrv_GetData(Radio_LEVEL);
		for(i=1;i<(AutoStore_TotalNum+1);i++)
		{
			if(TunerSmeter[i]<TempValue)
			{
				TempValue=TunerSmeter[i];
				j=i;
			}
		}

		if(j==0)	return;//means current smeter value is the lowest
		
		//rearrange smeter and station buffer
		for(i=j;i<AutoStore_TotalNum;i++){
			TunerSmeter[i]=TunerSmeter[i+1];
                        StationRecord[CurrentBand].Freq[i]=StationRecord[CurrentBand].Freq[i+1];
		        Radio_StoreStation( CurrentBand, i);
			
		}

		TunerSmeter[AutoStore_TotalNum]=RadioDrv_GetData(Radio_LEVEL);
	        StationRecord[CurrentBand].Freq[AutoStore_TotalNum]=Radio_GetCurrentFreq();
		Radio_StoreStation( CurrentBand, i);
	}
	else
	{	//store stations less than 6 station
		TunerSmeter[TunerMemNum]=RadioDrv_GetData(Radio_LEVEL);;//store semeter value
		Radio_SaveCurrentFreq2Preset(TunerMemNum);
	}
	return;

}
/*=============================================
Name:		Tuner_AutoStore_Sub
Description:	Auto Store Function
==============================================*/
void Tuner_AutoStore_Sub(void)
{
	unsigned char i;

	switch(TunerSubMode)
	{
		case 10://auto store start
			if(Curr_Work_Mode == Tuner)
			    F_Stereo_Disp=0;
			
		        TunerMemNum=0;
			//initialize semeter buffer
			for(i=0;i<7;i++)
				TunerSmeter[i]=0;	
			
			Radio_ScanStepCount=0;//reset scan step count
		        Radio_ScanStartFreq=Radio_GetCurrentFreq();//save current freq
			TunerSubMode=20;
			F_Tune_UP=1;
			F_Find_Station=0;

		case 20://loop tuning start
			//increase or decreas one step
			Radio_ScanStepCount++;			
			Radio_ChangeFreqOneStep(F_Tune_UP);

			/*write "new programmble value" into PLL*/
			Tuner_Update_ProgCnt(eAR_TuningAction_Search);
			/*set wait time for PLL lock*/	
			TimerSet(&TunerWaitTimer,TUNER_SCAN_WAITE_TIME);

			TunerSubMode=30;//set tunersubmode
			Radio_CheckStationInit();//set check  and IF start step
			Radio_ClearCurrentStation();//Clear station number
			break;
			
		case 30:
	        	/*check stereo*/
		//       Tuner_Check_Stereo();			
			/*time out check*/
			if(!TimerHasExpired(&TunerWaitTimer))  
				break;
					   
			/*Check station*/		   
			Radio_CheckStation( );   

			if( Radio_CheckStationStatus() >=NO_STATION)
				TunerSubMode = 40; 
			break;
		case 40:
                     if(Radio_CheckStationStatus()== PRESENT_STATION)
                     {
            			F_Find_Station=1;//find a station
            			TunerMemNum++;

            			if(Curr_Work_Mode == Tuner)
            				F_Refresh = 1;
            			//store strongest six stations!
            			Tuner_Smeter_Store();           			     		
                     } 
                    
                    if(Radio_ScanStartFreq==Radio_GetCurrentFreq())  //scan one loop finished?
			       TunerSubMode = 50;	
                    else 
			        TunerSubMode = 20;	
                    break;
                    
			
		case 50:
			if(F_Find_Station)//if find station listen  mem  1
			{
				F_Find_Station=0;
				
				if(Curr_Work_Mode == Tuner)
					F_Refresh = 1;
				Radio_SelectPreset(1);
			//	Radio_StoreStationRecord(Radio_GetCurrentBand());//Write to EEPROM
			}
			else //no find station listen last station
			{
                               Tuner_Update_ProgCnt(eAR_TuningAction_Preset);
			}
			//Tuner finish current work mode
			Tuner_Event_End();
			break;
	}
	return;
}

/*====================================================
 Function:Tuner_BandChange_Sub
 Input: 
      Null
 OutPut:
      Null
 Desp:
      band key action
=========================================================*/
void Tuner_BandChange_Sub(void)
{
	/*set to next band*/
	Radio_NextBand();
	/*rewrite parameters*/
	Radio_Para_Update();
	Tuner_Event_End();
}

/*====================================================
 Function:Tuner_PresetWrite_Sub
 Input: 
      Null
 OutPut:
      Null
 Desp:
      preset store sub
=========================================================*/
void Tuner_PresetWrite_Sub(void)
{
	unsigned char station=0;
	if(F_Store_Station_Ok==TRUE)
	   	return;
       switch(TunerWorkMode)
  	{
		case K_LM1:
			station=1;
			break;
		case K_LM2:	
			station=2;
			break;
		case K_LM3:
			station=3;
			break;
		case K_LM4:
			station=4;
			break;
		case K_LM5:
			station=5;
			break;
		case K_LM6:
			station=6;
			break;
	}  
     	if(station)   	
	{
		/*set to current station*/
		Radio_SaveCurrentFreq2Preset(station);
		F_Store_Station_Ok=TRUE;
		Beep(1);//beep
		/*select station */
		Radio_SelectPreset(station);
     	}
	/*diplay store ok*/
	Display_SetTimeout(DISPLAY_TIMEOUT_RadioStoreOK,1000);
}
/*====================================================
 Function:Tuner_PresetRead_Sub
 Input: 
      Null
 OutPut:
      Null
 Desp:
      preset read sub
=========================================================*/
void Tuner_PresetRead_Sub(void)
{
      	unsigned char station=0;
       switch(TunerWorkMode)
  	{
		case K_SM1:
			station=1;
			break;
		case K_SM2:	
			station=2;
			break;
		case K_SM3:
			station=3;
			break;
		case K_SM4:
			station=4;
			break;
		case K_SM5:
			station=5;
			break;
		case K_SM6:
			station=6;
			break;
	}  
    	if(station)/*set to selected station*/
		Radio_SelectPreset(station);
	Tuner_Event_End();
}
/*====================================================
 Function:Tuner_Check_Stereo
 Input: 
      Null
 OutPut:
      Null
 Desp:
      check fm stereo signal
=========================================================*/
void Tuner_Check_Stereo(void)
{
      bool st_indicator;
	 int16_t lev;
	/*check if current band is FM Band and time out*/
	if(TimerHasExpired(&StereoDetTimer)&& (Radio_IsFMBand()))
	{    /*reset timer*/
            TimerSet(&StereoDetTimer, FM_CHECK_STEREO_TIME);
	    /*read status data*/
	    // RadioDrv_ReadData(2);	 /*display on 'STEREO'*/ 
	    st_indicator=Radio_CheckStereo();
	    Get_Level_Data(1,&lev);
	    F_Stereo_Disp=0;
            if((st_indicator==TRUE)&&(lev>=StereoSeparation_Level_H))  {          
                    F_Stereo_Disp=1; 
            }
          else  if((st_indicator==FALSE)||(lev<=StereoSeparation_Level_L)){   /*display off 'STEREO'    */          
                   F_Stereo_Disp=0;           
           }
       }	
	return;
}


void Tuner_SetSeekSens_Sub(void)
{
    if(SeekSensLevel==HIGH)
	{
		SeekSensLevel=LOW;	
		}
	else
	{
		SeekSensLevel=HIGH;
		}
	Radio_SetSeekSenLevel(SeekSensLevel);
	//Display_SetTimeout(DISPLAY_TIMEOUT_SeekSensLev,1000);
}
/*====================================================
 Function:Tuner_ModeExecute_Process
 Input: 
      Null
 OutPut:
      Null
 Desp:
       tuner work mode execute
=========================================================*/
void Tuner_ModeExecute_Process(void)
{
	switch(TunerWorkMode)
	{
	       case 0:/*detect ST signal*/
		   	Tuner_Check_Stereo();
		   	break;
		case K_SEEK_UP:/*seek up or seek down*/
		case K_SEEK_DN:
			F_Tune_UP=0;
			if(TunerWorkMode==K_SEEK_UP)
				F_Tune_UP=1;
	//		Display_DisableTimeout();
			Tuner_Seek_Sub();
			break;
		case K_SCAN:/*do scan*/
			F_Tune_UP=1;
//			Display_DisableTimeout();
			Tuner_Scan_Sub();
			break;
		case K_BAND:/*do change band */
		//	Display_DisableTimeout();
			Tuner_BandChange_Sub();
			break;
		case K_LM1:/*Save preset station*/
		case K_LM2:	
		case K_LM3:
		case K_LM4:
		case K_LM5:
		case K_LM6:
			Tuner_PresetWrite_Sub();	
			break;
		case K_SM1:/*select preset station*/
		case K_SM2:	
		case K_SM3:
		case K_SM4:
		case K_SM5:
//		case K_SM6:
//			Display_DisableTimeout();
			Tuner_PresetRead_Sub();		
			break;
		case K_AS:
			Tuner_AutoStore_Sub();
			break;
		default:
			break;
	}
}





/*-----------------------------------------------------------------------
Function name:	Tuner_Para_Init
Input:			
Output:			
Description: read radio parameters from eeprom,and power on init	 
------------------------------------------------------------------------*/
void Tuner_Para_Init(void)
{       
	Radio_Func_Init();
}


S8 Para_Index;//parameter index
U8 Para_DefaultValue;//defaut value
U8 Para_CurrValue;//parameter current value
U8 F_Set_High ;//set para high 4bit flag
/*-----------------------------------------------------------------------
Function name:	Tuner_Para_Align_Mode_Proc
Input:			
Output:			
Description:	 This function enter align  mode ,in this mode,you can change hero's radio parameters,
and save changed parameters into eeprom
------------------------------------------------------------------------*/
void Tuner_Para_Align_Mode_Proc(void)
{       
        /*check system mode*/
	if((F_Sys_Reset)||(F_Hero_InitOK==0)||(Curr_Work_Mode==GUI))
		return;
	if(Last_Work_Mode==Align)/*exit align mode*/
	{
		Last_Work_Mode=0xff;
		Radio_Para_Update();
		Radio_StoreConfig();
		
	}
	if(Next_Work_Mode==Align)/*enter into align mode*/
	{
		Next_Work_Mode=0xff;
		Curr_Work_Mode=Align;
		Para_Index=0x00;
		F_Set_High=1;
		/*turn on*/
		Tuner_Turn_ON();
		/*audio input select*/
		Audio_SetPrimaryInputSource(INPUT_SOURCE_RADIO);
	}
	if(Curr_Work_Mode==Align)/*align mode process*/
	{
		Para_Align_Process();
	}
}
/*-----------------------------------------------------------------------
Function name:	Para_Align_Process
Input:			
Output:			
Description:	 operation of changing parameters
------------------------------------------------------------------------*/
void Para_Align_Process(void)
{
#if 0
	S8 WheelKey=GetWheelKey();
	U8 tempH,tempL;
	switch(KeyCode)
     	{
          	case Enter:/*high /low half byte set exchange*/
			if(F_Set_High)
				F_Set_High=0;
			else
				F_Set_High=1;
			break;
		case KeyUp:/*next parameters setup*/
			Para_Index++;//what's parameters ?please see Hero_Radio_DefParameters define
			if(Para_Index>=CONFIG_PARA_LEN)
				Para_Index=0;
			F_Set_High=1;
			break;
		case KeyDown:/*previous parameters setup*/
			Para_Index--;
			if(Para_Index<0)
				Para_Index=CONFIG_PARA_LEN-1;
			F_Set_High=1;
			break;
	}
	Para_DefaultValue =Hero_Radio_DefParameters[Para_Index];
	Para_CurrValue=RadioConfig[Para_Index];

	if(F_Set_High){  //set parameter high 4bit
		tempH=(Para_CurrValue>>4)+WheelKey;
		tempL=Para_CurrValue&0X0F;
		Para_CurrValue=(tempH&0x0F)*16+tempL;
	}
	else{  /*set parameter low 4bit*/  	
		tempH=(Para_CurrValue>>4);
		tempL=(Para_CurrValue&0X0F)+WheelKey;
		Para_CurrValue=tempH*16+(tempL&0x0F);
	}
	if(RadioConfig[Para_Index]!=Para_CurrValue)//new value set
	{
           RadioConfig[Para_Index]=Para_CurrValue;
	       Radio_Para_Update();
	}
	#endif
}



 



