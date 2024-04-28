/***********************************************************************//**
 * @file		Hero.c
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

/* station */
StationMemType StationRecord[MaxStationNum]; 

/* area config*/
#define AreaSelect Radio_CHN

#if (AreaSelect==Radio_CHN)   /*China */          
const Radio_AreaConfigDef Radio_AreaConfig={
    //FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
    10800,        8750,      1620,       522,         FM_Step_100k,        FM_Step_100k,                 9,                  9};
	
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		
   /*0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW*/
    {8750,10800},{8750,10800},{8750,10800},{522,1710},{144,288},{2300,27000}};    
    
#elif (AreaSelect==Radio_EUR)  /*Europe */   	                        
const Radio_AreaConfigDef Radio_AreaConfig={
//FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
10800,        8750,      1602,       531,         FM_Step_100k,        FM_Step_100k,               9,                  9};
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		 //0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW
    {6500,10800},{6500,10800},{6500,10800},{144,288},{522,1710},{2300,27000}};    // 
    
#elif (AreaSelect==Radio_USA)      /*USA */                
const Radio_AreaConfigDef Radio_AreaConfig={
//FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
10790,        8750,      1710,       530,        FM_Step_100k,        FM_Step_100k,         10,                  10};
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		 //0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW
    {6500,10800},{6500,10800},{6500,10800},{144,288},{522,1710},{2300,27000}};    // 
    
#elif (AreaSelect==Radio_JPN)      /*Japan */                
const Radio_AreaConfigDef Radio_AreaConfig={
//FM_MaxFreq  FM_MinFreq  AM_MaxFreq  AM_MinFreq  FM_AutoSeekStep/k  FM_ManualSeekStep/k  AM_AutoSeekStep/k  AM_ManualSeekStep/k
9000,        7600,      1629,       522,          FM_Step_100k,        FM_Step_100k,               9,                   9};
const FreqBaundDef FreqBaundConfig[MaxBandNum]={		 //0-FM1,1-FM2,2-FM3,3-LW,4-MW,5-SW
    {6500,10800},{6500,10800},{6500,10800},{144,288},{522,1710},{2300,27000}};    // 
    
#endif


/*Hero radio default parameters */
const U8 Hero_Radio_DefParameters[CONFIG_PARA_LEN]=
{
  0,	  //FM TUNER(AGC&Bandwidth)  //,0  
  0x00/*0x06*/,	  //AM TUNER(AGC&Bandwidth)  // 1	modify	nick 20110707
  0,     //TUNER_OPT (Antenna type&Antenna atteuation) no atteuation (only am )fm=0x00 //2
  0x4a,// FM RADIO (EMS,CNS,CE,NBSA,NBSB,//3
  0x4a,//AM  RADIO (EMS,CNS,CE,NBSA,NBSB,//4
  0x01,	//fm_Signal//5
  0x06/*0x0a*//* 0x05*/,	//am Signal//6					  modify	nick 20110707
  0x05,	//fm_BWCTRL(only fm),am=0x05//7
  0x00,	//Special (only fm) am=0x00//8
  0x30,	//fm Level offset	//9
  0x30,	//am Level offset	//10

   //softmute
   0x65,	//fm softmute time	 //11
   0x0e,	//am softmute time	//12
   0x3f/*0x10*//*0x00*/,	   //fm           softmute det    //13			 modify	nick 20110707
   0xd0,	   // am         softmute det    //14
   0x4a/*0x3A*/,		// fm  softmue lev  //15
   0x40,		//  am softmue lev  //16
   0x77/*0x33*/,		//fm softmute N_M	(fm only am:0x33)//17
   0x0a/*0x0C*/,	//fm softmute limit	//18		/*-20db*/				    modify	nick 20110707
   0x04,	//am softmute limit	//19

   //highcut
   0x6E, //fm_Highcut time//20
   0x12, //am Highcut time	//21
   0x35/*0x00*/,	//	fm  Highcut det//22			 modify	nick 20110707
   0xb5,	//	am  Highcut det   //23
   0x23/*0x33*/,	//	fm  Highcut level   //24	 modify	nick 20110707
   0x5b,	//	am  Highcut level   //25
   0x33,	//	Highcut N_M	(fm only am:0x33)//26
   0x04/*0x02*/,	//	fm   Highcut lim  //27			 modify	nick 20110707
   0x02,	//	am   Highcut lim  //28		
   0x04,   //fm Highcut low cut	//29
   0x05,   //am Highcut low cut	//30

 //stereo
   0x75,	 //fm stereo time(only fm ,am=0x75)	 	//31
  0x3f/* 0x00*/,	//	 stereo det(only fm ,am=0x10)	 	//32 modify	nick 20110707	
   0x4A,			//  stereo level	 (only fm ,am=0x4a)	 	//33
   0x33,	//	stereo N_M	   (only fm ,am=0x33)	 //34
   0x00,	//	stereo lim     (only fm ,am=0x0)	 //35
   0x3f/*0x00*/,	//	  stereo HB DET    (only fm ,am=0x10)	//36    modify	nick 20110707
   0x13,          // stereo HB LNM   (only fm ,am=0x4a)	 //37
};


/* radio default station */
const StationMemType StationDefaultRecord[MaxStationNum]=			 //now config: MaxBandNum=6, MaxStationNum=7 
{
	{10000,10000,10000,10000,10000,10000,10000},     //fm1				  
	{10000,10000,10000,10000,10000,10000,10000},		//fm2
	{10000,10000,10000,10000,10000,10000,10000},     //fm3
	{522,522,522,522,522,522,522},		//mw
	{144,144,144,144,144,144,144},		//lw
	{2300,2300,2300,2300,2300,2300,2300}		//sw
};

/*radio parameters */
U8 RadioConfig[CONFIG_PARA_LEN];
/*check station step*/
static U8 CheckIfStep;

/*current radio  band*/
U8 Radio_CurrentBand;
/*current radio  freqency*/
U16 Radio_CurrentFreq;
/*current radio  station*/
U8 Radio_CurrentStation;

eDev_Type RadioDev = Radio_Atomic2;
eDev_Type AudioDev = Radio_Atomic2;

/*-----------------------------------------------------------------------
Function name:	Radio_SetFreq
Input:		mode:
                              TEF663X_PRESETMODE.,TEF663X_SEARCHMODE
                              TEF663X_AFUPDATEMODE,TEF663X_JUMPMODE...
                        Freq:
Output:			
Description:	 
------------------------------------------------------------------------*/
void Radio_SetFreq(U8 mode,U8 Band,U16 Freq)
{
       /*frequency baundary check*/
	if((Freq>FreqBaundConfig[Radio_CurrentBand].MaxFreq)||(Freq<FreqBaundConfig[Radio_CurrentBand].MinFreq)){
		Freq = FreqBaundConfig[Radio_CurrentBand].MinFreq;
	}
	if(Band>=MaxBandNum){
		return;
	}
	Radio_CurrentBand=Band;
	switch(Band)
	{
	   case FM1_BAND:
	   case FM2_BAND:
	   case FM3_BAND:
	        Band=0;
	 		break;
		case LW_BAND:
		    Band=1;
			break;
		case MW_BAND:
		    Band=2;
			break;
		case SW_BAND:
		    Band=3;
			break;
	
	}

	Radio_CurrentFreq=Freq;
	StationRecord[Radio_CurrentBand].Freq[0]=Radio_CurrentFreq;


	if(Is_Radio_Atomic2){
		AR_TuningAction_t A2Mode;

		switch(mode){
			case Radio_PRESETMODE:
				A2Mode = eAR_TuningAction_Preset;
				break;
			case Radio_SEARCHMODE:
				A2Mode = eAR_TuningAction_Search;
				break;
			case Radio_AFUPDATEMODE:
				A2Mode = eAR_TuningAction_AF_Update;
				break;
			case Radio_JUMPMODE:
				A2Mode = eAR_TuningAction_Jump;
				break;
			case Radio_CHECKMODE:
				A2Mode = eAR_TuningAction_Check;
				break;
		}
		Radio_Tune_To(A2Mode,Freq);
	}
	else if(Is_Radio_Hero){
		/*update pll*/
		RadioDrv_UpdatePll(mode,Band,Freq);
	}
}
/*====================================================
 Function:Radio_ChangeFreqOneStep
 Input: 
      UP/DOWN
 OutPut:
      Null
 Desp:
     change curren freq on step
=========================================================*/
void Radio_ChangeFreqOneStep(U8 UpDown )
{
	int step;

	step=Radio_GetFreqStep(Radio_CurrentBand);
	if(UpDown==1)
	{	/*increase one step*/
		Radio_CurrentFreq+=step;
		/*frequency baundary check	*/
		if(Radio_CurrentFreq>FreqBaundConfig[Radio_CurrentBand].MaxFreq)	
		{  
			Radio_CurrentFreq=FreqBaundConfig[Radio_CurrentBand].MinFreq;
		}
	}
	else	
	{      /*decrease one step*/
		Radio_CurrentFreq-=step;  
		/*frequency baundary check*/	
		if(Radio_CurrentFreq<FreqBaundConfig[Radio_CurrentBand].MinFreq)	
		{	
		    	Radio_CurrentFreq=FreqBaundConfig[Radio_CurrentBand].MaxFreq;
		}		
	}

}
/*-----------------------------------------------------------------------
Function name:	Radio_GetCurrentFreq
Input:			
Output:			
Description:	return current freq
------------------------------------------------------------------------*/
U16 Radio_GetCurrentFreq(void)
{
	 return Radio_CurrentFreq;
}
/*-----------------------------------------------------------------------
Function name:	Radio_GetCurrentBand
Input:			
Output:			
Description:	 return current band 
------------------------------------------------------------------------*/
U8 Radio_GetCurrentBand(void)
{
	 return Radio_CurrentBand;
}

/*-----------------------------------------------------------------------
Function name:	Radio_GetCurrentStation
Input:			
Output:			
Description:	 return current station 
------------------------------------------------------------------------*/
U8 Radio_GetCurrentStation(void)
{
	 return Radio_CurrentStation;
}
/*-----------------------------------------------------------------------
Function name:	Radio_ClearCurrentStation
Input:			
Output:			
Description:	 set current station 
------------------------------------------------------------------------*/
void Radio_ClearCurrentStation(void)
{
	  Radio_CurrentStation=0;
}
/*-----------------------------------------------------------------------
Function name:	Radio_GetFreqStep
Input:			
Output:			
Description:	 Get current band freq step
------------------------------------------------------------------------*/
U32 Radio_GetFreqStep(U8 band)
{
	return (band<=FM3_BAND)? Radio_AreaConfig.FM_ManualSeekStep : Radio_AreaConfig.AM_ManualSeekStep;
}
/*-----------------------------------------------------------------------
Function name:	Radio_IsFMBand
Input:			
Output:			
Description:	 check current band is fm band
------------------------------------------------------------------------*/
U8 Radio_IsFMBand(void)
{ 
      return ((Radio_CurrentBand<=FM3_BAND)?1:0 )	;
}

/*-----------------------------------------------------------------------
Function name:	Radio_SetBand
Input:			Band
Output:			
Description:	 set band
------------------------------------------------------------------------*/
void Radio_SetBand(U8 Band)
{
	Radio_SetFreq(Radio_PRESETMODE,Band,StationRecord[Band].Freq[0]);
}
/*-----------------------------------------------------------------------
Function name:	Radio_NextBand
Input:			NULL
Output:			
Description:	 set to next band
------------------------------------------------------------------------*/
void Radio_NextBand(void)
{
	Radio_CurrentBand++;
	/*check band avilable*/
	if(Radio_CurrentBand>=MaxBandNum)	
	{
		Radio_CurrentBand=0;
	}
	/*set to current band*/
	Radio_SetBand(Radio_CurrentBand);
	Radio_CurrentFreq=StationRecord[Radio_CurrentBand].Freq[0];
	Radio_CurrentStation=0;
}
/*-----------------------------------------------------------------------
Function name:	Radio_ValidFreq
Input:			band and freqencey pointer
Output:			valid freqency
Description:	 	min<= freq <= max, aligned by step
------------------------------------------------------------------------*/
static void Radio_ValidFreq(U8 band,U16 *pFreq)
{
	U16 step,min,max;

	step = Radio_GetFreqStep(band);
	max = FreqBaundConfig[band].MaxFreq;
	min = FreqBaundConfig[band].MinFreq;

	if((*pFreq > max)||(*pFreq < min))	//frequency baundary check
		*pFreq = min;
	else {  //step check
		int num  = (*pFreq - min)/step;	//align freq by step
		*pFreq = min + num * step;
	}
}
/*-----------------------------------------------------------------------
Function name:	Radio_ReadStationRecord
Input:			
Output:			
Description:	 write  one station to eeprom
------------------------------------------------------------------------*/
void Radio_StoreStation(U8 Band,U8 Station)
{
    EEPROM_Write(EEPRom_StationRecordDefaultAddr + Band *sizeof(StationRecord[0]) +2*Station,
	(U8 *)(&StationRecord[Band].Freq[Station]),2);

}
/*-----------------------------------------------------------------------
Function name:	Radio_ReadStationRecord
Input:			
Output:			
Description:	 read  one station from eeprom
------------------------------------------------------------------------*/
void Radio_ReadStation(U8 Band,U8 Station)
{
    EEPROM_Read(EEPRom_StationRecordDefaultAddr + Band *sizeof(StationRecord[0]) +2*Station,
	(U8 *)(&StationRecord[Band].Freq[Station]),2);

}
/*-----------------------------------------------------------------------
Function name:	Radio_ReadStationRecord
Input:			
Output:			
Description:	 read all station from eeprom
------------------------------------------------------------------------*/
void Radio_ReadStationRecord(void)
{
	U16 band,station;
	U16 *pFreq;
	
	//EEPROM_Read(EEPRom_StationRecordDefaultAddr,(U8 *)(&StationRecord[0].Freq[0]),sizeof(StationRecord));
        /* for(band=0;band<MaxBandNum;band++)
	     for(station=0;station<MaxStationNum;station++)
	     	Radio_ReadStation( band, station);*/
		 
	/*set all frequency to valid value*/
	for(band=0;band<MaxBandNum;band++)
	{	
	        EEPROM_Read(EEPRom_StationRecordDefaultAddr+ band *sizeof(StationRecord[0]),(U8 *)(&StationRecord[band].Freq[0]),sizeof(StationRecord[0]));
		pFreq = &StationRecord[band].Freq[0];	
		for(station=0;station<MaxStationNum;station++,pFreq++) 
		{
			Radio_ValidFreq(band,pFreq);
		}
	}
}
/*-----------------------------------------------------------------------
Function name:	Radio_StoreStationRecord
Input:			
Output:			
Description:	 store one band of station to eeprom
------------------------------------------------------------------------*/
void Radio_StoreStationRecord(U8 Band)
{
/*
	EEPROM_Write(EEPRom_StationRecordDefaultAddr + Band *sizeof(StationRecord[0]) ,
	(U8 *)(&StationRecord[Band].Freq[0]),
	sizeof(StationRecord[0]));	 */
	int i;
	for(i=0;i<MaxStationNum;i++)
	{
		 Radio_StoreStation( Band, i);
	}
	      
}

/*-----------------------------------------------------------------------
Function name:	Radio_ReadCurrentBand
Input:			
Output:			
Description:	read station from eeprom 
------------------------------------------------------------------------*/
U8 Radio_ReadCurrentBand(void)
{
	U8 band;
	/*read current band from eeprom*/
	EEPROM_Read(EEPRom_CurrentBandAddr,&band, 1);
	/*if band not aviliable*/
	if(band>=MaxBandNum)
		band=FM1_BAND;
	return band;
}
/*-----------------------------------------------------------------------
Function name:	Radio_StoreCurrentBand
Input:			
Output:			
Description:	 store current band  to eeprom
------------------------------------------------------------------------*/
void Radio_StoreCurrentBand(void)
{
	EEPROM_Write(EEPRom_CurrentBandAddr,&Radio_CurrentBand,1);
}
/*-----------------------------------------------------------------------
Function name:	Radio_ReadConfig
Input:			
Output:			
Description:	read parameters from eeprom 
------------------------------------------------------------------------*/
void Radio_ReadConfig(void)
{
    int i;
	for(i=0;i<CONFIG_PARA_LEN;i=i+2)
   	EEPROM_Read(EEPRom_ConfigAddr+i,RadioConfig+i,2);
//	    EEPROM_Read(EEPRom_ConfigAddr,RadioConfig,CONFIG_PARA_LEN);	
}
/*-----------------------------------------------------------------------
Function name:	Radio_StoreConfig
Input:			
Output:			
Description:	 store parameters into eeprom
------------------------------------------------------------------------*/
void Radio_StoreConfig(void)
{     
    int i;
	for(i=0;i<CONFIG_PARA_LEN;i=i+2)
    	EEPROM_Write(EEPRom_ConfigAddr+i,RadioConfig+i,2);
   //     EEPROM_Write(EEPRom_ConfigAddr,RadioConfig,CONFIG_PARA_LEN);	
}
/*-----------------------------------------------------------------------
Function name:	Radio_StoreAll
Input:			
Output:			
Description:	 store all the radio para,station to eeprom
------------------------------------------------------------------------*/
void Radio_StoreAll(void)
{
        U8 i;
	/*store radio parameters*/
	Radio_StoreConfig();			
	/*write eeprom available flag*/
	i=AvailableDataFlag;
	EEPROM_Write(EEPRom_CheckAddr,&i,1);
	/*store current band*/
	Radio_StoreCurrentBand();
	/*store station*/
	for(i=0;i<MaxBandNum;i++)	
	{
	    Radio_StoreStationRecord(i);
	}
}

/*-----------------------------------------------------------------------
Function name:	
Input:			
Output:			
Description:	 check stereo indicator
------------------------------------------------------------------------*/
bool Radio_CheckStereo(void)
{
	U16 Status;
	bool stereo = FALSE;

	if(Is_Radio_Atomic2)
	{
		if(SUCCESS==devTEF665x_Radio_Get_Signal_Status(1,&Status))
			stereo = ( Status & bit15)? TRUE :FALSE;
	}
	else if(Is_Radio_Hero)
	{
		/*read status data*/	   
		RadioDrv_ReadData(1);
		/*Get status data*/
		Status=RadioDrv_GetData(Radio_STATUS);
		stereo = ( Status & bit3)? TRUE :FALSE;
	}

	return stereo;
}

bool SeekSenLevel = LOW;
void Radio_SetSeekSenLevel(U8 Lev)
{
	SeekSenLevel = Lev;
}

//return dector time in ms
//return = ms
int Radio_Get_QRS(bool fm)
{
	U8 status;

	if(Is_Radio_Atomic2)
	{
		if(SUCCESS == devTEF665x_Radio_Get_Quality_Status(fm,&status))
		{
			return status;
		}
	}
	else if(Is_Radio_Hero)
	{
			RadioDrv_ReadData(1);
			status=RadioDrv_GetData(Radio_STATUS);   //get status
			status &=0xE0;

			switch(status){
				case 0x00:
					return 0;	//detector time 0 

				case 0x20:
					return 1;	//detector time 0 -- 1 ms

				case 0x40:
					return 2;	//detector time 1 -- 2 ms
				
				case 0x60:
					return 8;	//detector time 2 -- 8 ms
				
				case 0x80:
					return 32;	//detector time 8 -- 32 ms
				
				case 0xa0:
					return 100;	//detector time > 32 ms
			}
	}

	return 0;
}

//level detector result
//output: -200 ... 1200 (0.1 * dBuV) = -20 ... 120 dBuV RF input level
//return =  dBuV
int Radio_Get_Level(bool fm)
{
	int16_t level;
	U8 status;
	
	if(Is_Radio_Atomic2)
	{
		if(SUCCESS == devTEF665x_Radio_Get_Quality_Level(fm,&status,&level))
		{
			return level;
		}
	}
	else if(Is_Radio_Hero)
	{
		/*read status data*/
		RadioDrv_ReadData(2);
			/*get level*/	 
		//level = 1 * (((int8_t)RadioDrv_GetData(Radio_LEVEL))/2 - 8);   
		level = (((int8_t)RadioDrv_GetData(Radio_LEVEL)) -16 )/2 ;   

		return level;
	}

	return -255;
}

// status =  0.1 … 32 ms
// level = -20 ... 120 dBuV
// usn = 0 … 100%
// wam = 0 … 100%
//offset = -1200 … 1200 (*0.1 kHz) = -120 kHz … 120 kHz
//bandwidth = FM, 560 … 3110 [*0.1 kHz]  AM 30 … 80 [*0.1 kHz]
// modulation = 0 … 100%
int Radio_Get_Data(bool fm,U8 *status,int16_t *level,U8 *usn,U8 *wam,int16_t *offset,int16_t *bandwidth,U8 *modulation)
{
	if(Is_Radio_Atomic2)
	{
		if(SUCCESS == devTEF665x_Radio_Get_Quality_Data (fm,status,level,usn,wam,offset,bandwidth,modulation))
		{
			return SUCCESS;
		}
	}
	else if(Is_Radio_Hero)
	{
		/*read status data*/
		RadioDrv_ReadData(5);

			/*get level*/	 
		//*level = 1 * (((int8_t)RadioDrv_GetData(Radio_LEVEL))/2 - 8);   
		*level =  (((int8_t)RadioDrv_GetData(Radio_LEVEL)) -16 )/2 ;   ;   
			/*get noise*/	 
		*usn = ((int16_t)(100 * (U16)RadioDrv_GetData(Radio_USN))>>8); 
			/*get multpath*/	
		*wam = ((int16_t)(100 * (U16)RadioDrv_GetData(Radio_WAM))>>8); 
			/*get freqency offset*/
		if(fm)
			*offset = 10 * (int16_t)(0x7f & RadioDrv_GetData(Radio_FOF)); 
		else
			*offset = 1 * (int16_t)(0x7f & RadioDrv_GetData(Radio_FOF)); 

		return SUCCESS;
	}

	return !SUCCESS;
}
/*--------------------------------------------------------------------
 Function:Radio_CheckStation
 Input: 
      Null
 OutPut:
      Null
 Desp:
     check station if aviable
---------------------------------------------------------------------*/
void Radio_CheckStation(void)
{
	unsigned char threshold;
	unsigned char i;

	U8 status,usn, wam, modulation;
	int16_t level, offset,bandwidth;
	
	bool fm = (Radio_CurrentBand<=FM3_BAND);
	
	switch(CheckIfStep)
	{
		case 10:/*start check init*/
			CheckIfStep = 20;
			break;

		case 20:/*Check QRS(quality of read status)	 */
			CheckIfStep=NO_STATION;

			threshold = fm? RADIO_FM_LEVEL_AVAILABLE_TIME : RADIO_AM_LEVEL_AVAILABLE_TIME;

			if(Radio_Get_QRS(fm) >= threshold)
				CheckIfStep=30;  /*Set to next step*/	

			break;   

		case 30:/*check level */
			CheckIfStep=40;

			for(i=0;i<3;i++)
			{/*Check 3 times*/
				threshold = fm ? ((SeekSenLevel ==HIGH) ? FM_SCAN_LEVEL_HI : FM_SCAN_LEVEL) 
							: ((SeekSenLevel ==HIGH) ? AM_SCAN_LEVEL_HI : AM_SCAN_LEVEL);

				if(threshold > Radio_Get_Level(fm))
				{//exit check
					CheckIfStep=NO_STATION;
					break;
				}
			}
			break;
		case 40:
			if(Radio_Get_QRS(fm) < RADIO_USN_AVAILABLE_TIME)	//wait usn... available
				break;
			
			CheckIfStep = NO_STATION;

			if(SUCCESS == Radio_Get_Data(fm,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation))
			{
				if( fm? ((usn<FM_USN_DISTURBANCE)&&(wam < FM_WAM_DISTURBANCE)&&(offset < FM_FREQ_OFFSET))
									: (offset < AM_FREQ_OFFSET))

				CheckIfStep = PRESENT_STATION ;
			}
			break;
            case NO_STATION:
			break; 
            case PRESENT_STATION:
			break; 
            default:
			CheckIfStep = NO_STATION;
			break;        
        }
}
/*--------------------------------------------------------------------
 Function:Radio_CheckStationStatus
 Input: 
      Null
 OutPut:
      CheckIfStep
 Desp:
     get check station step
---------------------------------------------------------------------*/
U8 Radio_CheckStationStatus(void)
{
	return CheckIfStep;
}

/*--------------------------------------------------------------------
 Function:Radio_CheckStationInit
 Input: 
      Null
 OutPut:
      NULL
 Desp:
      check station init
---------------------------------------------------------------------*/
void Radio_CheckStationInit(void)
{
	CheckIfStep=10;
}
/*-----------------------------------------------------------------------
Function name:	Radio_SelectPreset
Input:		Station :1~6	
Output:		
Description:	 select preset station
------------------------------------------------------------------------*/
void Radio_SelectPreset(U8 Station)
{
	if(StationRecord[Radio_CurrentBand].Freq[Station]==0) 
	{		//no station record
		return;
	}
	/*set radio freqency*/	
	Radio_SetFreq(Radio_PRESETMODE,Radio_CurrentBand,StationRecord[Radio_CurrentBand].Freq[Station]);
	/*reset station*/
	StationRecord[Radio_CurrentBand].Freq[0]=StationRecord[Radio_CurrentBand].Freq[Station];
	Radio_CurrentStation=Station;
	Radio_CurrentFreq=StationRecord[Radio_CurrentBand].Freq[0];
}
/*-----------------------------------------------------------------------
Function name:	Radio_SaveCurrentFreq2Preset
Input:			
Output:		
Description:	preset station save
------------------------------------------------------------------------*/
void Radio_SaveCurrentFreq2Preset(U8 Station)
{
	if(StationRecord[Radio_CurrentBand].Freq[Station]==Radio_CurrentFreq)
	{
		return;
	}
	StationRecord[Radio_CurrentBand].Freq[Station]=StationRecord[Radio_CurrentBand].Freq[0];
	Radio_CurrentStation=Station;
	Radio_CurrentFreq=StationRecord[Radio_CurrentBand].Freq[0];
	/*store  station*/
	//Radio_StoreStationRecord(Radio_CurrentBand);
	Radio_StoreStation(Radio_CurrentBand,Radio_CurrentStation);
}
/*-----------------------------------------------------------------------
Function name:	
Input:			
Output:			
Description:	 
------------------------------------------------------------------------*/
void Radio_Para_Init(void)
{
	U8 Check_eeprom;
	/*get the tef663x last config stored in eeprom*/
	Radio_ReadConfig();				
	EEPROM_Read(EEPRom_CheckAddr,&Check_eeprom,1);
	/*not store the config in eeprom*/
	if(Check_eeprom!=AvailableDataFlag)	
	{       
		/*load default parameters*/
		memcpy(RadioConfig,Hero_Radio_DefParameters,CONFIG_PARA_LEN);
		memcpy(StationRecord,StationDefaultRecord,sizeof(StationRecord));	
		/*store config to eeprom*/
		Radio_StoreAll();
	}
	else
	{	    /*load station from eeprom*/    
     	Radio_ReadStationRecord();		
	} 
	  /*init band*/
	Radio_CurrentStation=1;
	Radio_CurrentBand=FM1_BAND;
	Radio_CurrentFreq=StationRecord[Radio_CurrentBand].Freq[Radio_CurrentStation];
}
/*-----------------------------------------------------------------------
Function name:	Radio_Para_WriteAll
Input:			
Output:			
Description:	 write all the radio parameters to hereo
------------------------------------------------------------------------*/
void Radio_Para_WriteAll(void)
{
	U8 IIcDataBuff[29];
	IIcDataBuff[0] = 0x03;
	IIcDataBuff[1] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_TUNER:rPara_AM_TUNER;//TUNER//
	IIcDataBuff[2] = (Radio_CurrentBand<=FM3_BAND)?0x00:rPara_AM_TUNER_OPT;//TUNER_OPT
	IIcDataBuff[3] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_RADIO:rPara_AM_RADIO;//RADIO
	IIcDataBuff[4] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SIGNAL:rPara_AM_SIGNAL;//SIGNAL

	IIcDataBuff[5] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_BWCTRL:0X05;//BWCTRL
	IIcDataBuff[6] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SPECIAL:0X00;//SPECIAL
	IIcDataBuff[7] = 0X41;//RDS
	IIcDataBuff[8]=0;
	IIcDataBuff[9]=0;
	IIcDataBuff[10] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_LEVEL:rPara_AM_LEVEL;//LEVEL_OFFSET

	//Softmute 
	IIcDataBuff[11] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SOFTMUTE_TIME:rPara_AM_SOFTMUTE_TIME;//
	IIcDataBuff[12] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SOFTMUTE_DET:rPara_AM_SOFTMUTE_DET;//
	IIcDataBuff[13] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SOFTMUTE_LEV:rPara_AM_SOFTMUTE_LEV;//
	IIcDataBuff[14] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SOFTMUTE_NM:0x33;//
	IIcDataBuff[15] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_SOFTMUTE_LIM:rPara_AM_SOFTMUTE_LIM;//

	//high cut
	IIcDataBuff[16] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_HIGHCUT_TIME:rPara_AM_HIGHCUT_TIME;//
	IIcDataBuff[17] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_HIGHCUT_DET:rPara_AM_HIGHCUT_DET;//
	IIcDataBuff[18] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_HIGHCUT_LEV:rPara_AM_HIGHCUT_LEV;//
	IIcDataBuff[19] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_HIGHCUT_NM:0x33;//
	IIcDataBuff[20] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_HIGHCUT_LIM:rPara_AM_HIGHCUT_LIM;//
	IIcDataBuff[21] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_HIGHCUT_LOC:rPara_AM_HIGHCUT_LOC;//

	//stereo
	IIcDataBuff[22] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREO_TIME:0x75;//
	IIcDataBuff[23] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREO_DET:/*0x10*/0x3F;//nick modify 2011-06-28
	IIcDataBuff[24] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREO_LEV:0x4A;//
	IIcDataBuff[25] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREO_NM:0x33;//
	IIcDataBuff[26] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREO_LIM:0x00;//
	IIcDataBuff[27] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREOHB_DET:/*0x10*/0x3F;////nick modify 2011-06-28
	IIcDataBuff[28] = (Radio_CurrentBand<=FM3_BAND)?rPara_FM_STEREOHB_NM:/*0x4A*/0x5b;//
	I2C1_WriteData(TEF663x_SlaveAddr,IIcDataBuff,29);
        
}

/*====================================================
 Function:Radio_Save_Station
 Input: 
      Null
 OutPut:
      Null
 Desp:
     Save one station to mem
=========================================================*/
void Radio_Save_Station(U8 StationNumber ,U16 StationFreq)
{	
	StationRecord[Radio_CurrentBand].Freq[StationNumber]=StationFreq;

	EEPROM_Write(EEPRom_StationRecordDefaultAddr + Radio_CurrentBand *sizeof(StationRecord[0]) ,
		(U8 *)(&StationRecord[Radio_CurrentBand].Freq[StationNumber]),
		2);

}


//--------------------------------------------------------------------------------
