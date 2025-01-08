/***********************************************************************//**
 * @file		Hero.h
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
#ifndef __RADIO_FUNC_H
#define __RADIO_FUNC_H

typedef enum{
	Radio_Hero,
	Radio_Atomic,
	Radio_Atomic2,
	Radio_Helio,

	Radio_Last
}eDev_Type;
extern eDev_Type RadioDev;
extern eDev_Type AudioDev;

#define Is_Radio_Atomic2		(RadioDev == Radio_Atomic2)
#define Is_Radio_Hero		(RadioDev == Radio_Hero)

#define Is_Audio_Atomic2		(AudioDev == Radio_Atomic2)
#define Is_Audio_Hero		(AudioDev == Radio_Hero)

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW  0;
#endif

#define TUNER_LEVEL_14dB	14
#define TUNER_LEVEL_20dB	20
#define TUNER_LEVEL_25dB	25
#define TUNER_LEVEL_35dB	35
#define TUNER_LEVEL_45dB	45
#define TUNER_LEVEL_55dB	55

#define TUNER_PERCENT_27		27
#define TUNER_PERCENT_23		23

#define TUNER_OFFSET_10KHz		100
#define TUNER_OFFSET_1500Hz	15


#define RADIO_FM_LEVEL_AVAILABLE_TIME	4	//5 ms after tuning
#define RADIO_AM_LEVEL_AVAILABLE_TIME	36	//36 ms after tuning
#define RADIO_USN_AVAILABLE_TIME	34	//34ms

#define FM_SCAN_LEVEL			TUNER_LEVEL_25dB	//dB, 0.5dB/step -8dB~99.5dB
#define AM_SCAN_LEVEL			TUNER_LEVEL_35dB	//dB, 0.5dB/step -8dB~99.5dB
#define FM_SCAN_LEVEL_HI		TUNER_LEVEL_45dB	//dB, 0.5dB/step -8dB~99.5dB
#define AM_SCAN_LEVEL_HI		TUNER_LEVEL_55dB	//dB, 0.5dB/step -8dB~99.5dB

#define FM_USN_DISTURBANCE	TUNER_PERCENT_27	//disturbance = 27%
#define FM_WAM_DISTURBANCE	TUNER_PERCENT_23	//disturbance = 23%

#define FM_FREQ_OFFSET			TUNER_OFFSET_10KHz	//offset = 10k, step=100Hz
#define AM_FREQ_OFFSET			TUNER_OFFSET_1500Hz	//offset = 1.5k, step=100Hz

#define STEREO_Separation_Level_H  TUNER_LEVEL_20dB		//Stereo indication by LEVEL
#define STEREO_Separation_Level_L  TUNER_LEVEL_14dB		//Stereo indication by LEVEL

/*radio band define*/
#define MaxBandNum 6		 
#define FM1_BAND 0
#define FM2_BAND 1
#define FM3_BAND 2
#define MW_BAND 3
#define LW_BAND 4
#define SW_BAND 5

/*one step define
NOTE:FM Freqency uint is 10KHz,AM Freqency uint is 1KHz*/
#define AM_Step_9k 9  //am step 9khz
#define AM_Step_10k 10 //am step 10khz
#define FM_Step_50k 5 //fm step 50khz
#define FM_Step_100k 10 //fm step 10 0khz
#define FM_Step_200k 20 //fm step 200khz

/*tuner scan/seek condition define*/
#define Radio_ScanLevel 		TUNER_LEVEL_25dB  //dB, 0.5dB/step -8dB~99.5dB
#define Radio_FMScanSen ((U8)((Radio_ScanLevel+8)*2))//DX
#define Radio_FMScanSen_Loc ((U8)((Radio_ScanLevel+TUNER_LEVEL_20dB+8)*2))//   Local Att=DX+20
#define Radio_FMScanUNoise 70
#define Radio_FMScanMNoise 60
#define Radio_FMScanOffset 10

#define Radio_FM_Status 0x20
#define Radio_AMScanSen 		((8+TUNER_LEVEL_35dB)*2)//DX
#define Radio_AMScanSen_Loc 	((8+TUNER_LEVEL_35dB+20)*2)  //   Local Att=DX+20
#define Radio_AMScanOffset 		TUNER_OFFSET_1500Hz
#define Radio_AM_Status 0xA0

#define StereoSeparation_Level_H  ((U8)((TUNER_LEVEL_20dB+8)*2))		//Stereo indication by LEVEL
#define StereoSeparation_Level_L  ((U8)((TUNER_LEVEL_14dB+8)*2))		//Stereo indication by LEVEL
/*check station result */
#define NO_STATION   90
#define PRESENT_STATION        100

/*Radio parameters define*/
#define CONFIG_PARA_LEN  38
#define rPara_FM_TUNER  RadioConfig[0]  //FM TUNER(AGC&Bandwidth)  //,0  
#define rPara_AM_TUNER  RadioConfig[1]  //AM TUNER(AGC&Bandwidth)  // 1
#define rPara_AM_TUNER_OPT  RadioConfig[2]    //TUNER_OPT (Antenna type&Antenna atteuation) no atteuation (only am )fm=0x00 //2
#define rPara_FM_RADIO  RadioConfig[3] //FM RADIO (EMS,CNS,CE,NBSA,NBSB,//3
#define rPara_AM_RADIO  RadioConfig[4]//AM  RADIO (EMS,CNS,CE,NBSA,NBSB,//4
#define rPara_FM_SIGNAL  RadioConfig[5]//fm_Signal//5
#define rPara_AM_SIGNAL  RadioConfig[6]//Am_Signal//6
#define rPara_FM_BWCTRL RadioConfig[7]//fm_BWCTRL(only fm),am=0x05//7
#define rPara_FM_SPECIAL  RadioConfig[8]//FM Special (only fm) am=0x00//8
#define rPara_FM_LEVEL  RadioConfig[9] // fm Level offset	//9
#define rPara_AM_LEVEL  RadioConfig[10] // fm Level offset	//10

 #define rPara_FM_SOFTMUTE_TIME  RadioConfig[11]//fm softmute time	 //11
 #define rPara_AM_SOFTMUTE_TIME  RadioConfig[12]//am softmute time	//12
 #define rPara_FM_SOFTMUTE_DET  RadioConfig[13]  //fm           softmute det    //13
 #define rPara_AM_SOFTMUTE_DET  RadioConfig[14]   // am         softmute det    //14
 #define rPara_FM_SOFTMUTE_LEV  RadioConfig[15]		// fm  softmue lev  //15
 #define rPara_AM_SOFTMUTE_LEV  RadioConfig[16]		//  am softmue lev  //16
 #define rPara_FM_SOFTMUTE_NM  RadioConfig[17]		//fm softmute N_M	(fm only am:0x33)//17
 #define rPara_FM_SOFTMUTE_LIM  RadioConfig[18]	//fm softmute limit	//18
 #define rPara_AM_SOFTMUTE_LIM  RadioConfig[19]	//am softmute limit	//19

   #define rPara_FM_HIGHCUT_TIME  RadioConfig[20]//fm_Highcut time//20
   #define rPara_AM_HIGHCUT_TIME  RadioConfig[21]//am Highcut time	//21
   #define rPara_FM_HIGHCUT_DET  RadioConfig[22]	//	fm  Highcut det//22
   #define rPara_AM_HIGHCUT_DET  RadioConfig[23]	//	am  Highcut det   //23
   #define rPara_FM_HIGHCUT_LEV  RadioConfig[24]	//	fm  Highcut level   //24
   #define rPara_AM_HIGHCUT_LEV  RadioConfig[25]	//	am  Highcut level   //25
   #define rPara_FM_HIGHCUT_NM  RadioConfig[26]	//	Highcut N_M	(fm only am:0x33)//26
   #define rPara_FM_HIGHCUT_LIM  RadioConfig[27]	//	fm   Highcut lim  //27
   #define rPara_AM_HIGHCUT_LIM RadioConfig[28]	//	am   Highcut lim  //28
   #define rPara_FM_HIGHCUT_LOC RadioConfig[29]  //fm Highcut low cut	//29
   #define rPara_AM_HIGHCUT_LOC RadioConfig[30]   //am Highcut low cut	//30

   #define rPara_FM_STEREO_TIME  RadioConfig[31]	 //fm stereo time(only fm ,am=0x75)	 	//31
   #define rPara_FM_STEREO_DET  RadioConfig[32]	//	 stereo det(only fm ,am=0x10)	 	//32	
   #define rPara_FM_STEREO_LEV  RadioConfig[33]			//  stereo level	 (only fm ,am=0x4a)	 	//33
   #define rPara_FM_STEREO_NM RadioConfig[34]	//	stereo N_M	   (only fm ,am=0x33)	 //34
   #define rPara_FM_STEREO_LIM  RadioConfig[35]	//	stereo lim     (only fm ,am=0x0)	 //35
   #define rPara_FM_STEREOHB_DET RadioConfig[36]	//	  stereo HB DET    (only fm ,am=0x10)	//36 
   #define rPara_FM_STEREOHB_NM RadioConfig[37]        // stereo HB LNM   (only fm ,am=0x4a)	 //37
   

/*station max*/
#define MaxStationNum 7/*Station count(6)+current statuin(1)1*/



#define F_UP   1
#define F_DOWN  0
#define StationRecordLen (MaxBandNum*MaxStationNum*2)

/*are config parameters struct*/
typedef struct{         //area radio parameter 
	U32 FM_MaxFreq;			 //fm max freq
	U32 FM_MinFreq;                        //fm min freq
	U32 AM_MaxFreq;                       // am max freq
	U32 AM_MinFreq;                        //am min freq
	U32 FM_AutoSeekStep;               // fm step
	U32 FM_ManualSeekStep;           
	U32 AM_AutoSeekStep;              //am step
	U32 AM_ManualSeekStep;
}Radio_AreaConfigDef;

/*area define*/
enum Radio_AreaCode{  
    Radio_CHN=0,
    Radio_EUR,
    Radio_USA,
    Radio_JPN
};

 /*band freq range*/
typedef struct     
{
    U16 MinFreq;
    U16 MaxFreq;
}FreqBaundDef;
 
/*tation freq*/
typedef struct 
{
    U16 Freq[MaxStationNum];   //0-current work freq,or backup freq;
}StationMemType;


extern U8 RadioConfig[CONFIG_PARA_LEN];
extern const U8 Hero_Radio_DefParameters[CONFIG_PARA_LEN];
/*---------------------------------------------------------------------------------
function define start
----------------------------------------------------------------------------------*/

/*band function*/
extern void Radio_SetBand(U8 Band);
extern void Radio_NextBand(void);
extern U8 Radio_IsFMBand(void);
extern U8 Radio_GetCurrentBand(void);

/*preset function*/
extern void Radio_SaveCurrentFreq2Preset(U8 Station);
extern void Radio_SelectPreset(U8 Station);
extern void Radio_ClearCurrentStation(void);
extern U8 Radio_GetCurrentStation(void);

/*radio freqency function*/
extern void Radio_SetFreq(U8 mode,U8 Band,U16 Freq);
extern U32 Radio_GetFreqStep(U8 band);
extern void Radio_ChangeFreqOneStep(U8 UpDown );
extern U16 Radio_GetCurrentFreq(void);

/*parameters*/
extern void Radio_Para_Init(void);
extern void Radio_Para_WriteAll(void);


/*check station */
extern void Radio_CheckStation(void);
extern void Radio_CheckStationInit(void);
extern U8 Radio_CheckStationStatus(void);

/*read/save  radio setup parameters from/ to eeprom*/
void Radio_ReadStationRecord(void);
void Radio_StoreCurrentBand(void);
void Radio_StoreAll(void);
void Radio_ReadConfig(void);
void Radio_Save_Station(U8 StationNumber ,U16 StationFreq);
void Radio_StoreConfig(void);
void Radio_StoreStationRecord(U8 Band);
void Radio_SetSeekSenLevel(U8 Lev);
void Radio_StoreStation(U8 Band,U8 Station);
bool Radio_CheckStereo(void);
int Radio_Get_Level(bool fm);

#endif

