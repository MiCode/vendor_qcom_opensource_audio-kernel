

#ifndef _LITHIO_DRV_H_
#define _LITHIO_DRV_H_

#define TEF668x_SlaveAddr 0xC8

#define TEF668x_FM_FREQUENCY_MIN	6500		//step 10kHz
#define TEF668x_FM_FREQUENCY_MAX	10800
#define TEF668x_FM_FREQUENCY_STEP	10

#define TEF668x_LW_FREQUENCY_MIN	144			//step 1kHz
#define TEF668x_LW_FREQUENCY_MAX	288
#define TEF668x_LW_FREQUENCY_STEP	1

#define TEF668x_MW_FREQUENCY_MIN	522			//step 1kHz
#define TEF668x_MW_FREQUENCY_MAX	1710
#define TEF668x_MW_FREQUENCY_STEP	1

#define TEF668x_SW_FREQUENCY_MIN	2300		//step 1kHz
#define TEF668x_SW_FREQUENCY_MAX	27000
#define TEF668x_SW_FREQUENCY_STEP	1

#define TEF668x_Is_FM_Freq(a)	((a>=TEF668x_FM_FREQUENCY_MIN)&&(a<=TEF668x_FM_FREQUENCY_MAX))
#define TEF668x_Is_LW_Freq(a)	((a>=TEF668x_LW_FREQUENCY_MIN)&&(a<=TEF668x_LW_FREQUENCY_MAX))
#define TEF668x_Is_MW_Freq(a)	((a>=TEF668x_MW_FREQUENCY_MIN)&&(a<=TEF668x_MW_FREQUENCY_MAX))
#define TEF668x_Is_SW_Freq(a)	((a>=TEF668x_SW_FREQUENCY_MIN)&&(a<=TEF668x_SW_FREQUENCY_MAX))
#define TEF668x_Is_AM_Freq(a)	((TEF668x_Is_LW_Freq(a))||(TEF668x_Is_MW_Freq(a))||(TEF668x_Is_SW_Freq(a)))

typedef enum
{
	TEF668x_MODULE_FM = 32,
	TEF668x_MODULE_AM = 33,
	TEF668x_MODULE_AUDIO   = 48,
	TEF668x_MODULE_APPL  = 64
} TEF668x_MODULE;

typedef enum
{
	TEF668x_Cmd_Tune_To  =  1,
	TEF668x_Cmd_Set_Tune_Options   =2,
	TEF668x_Cmd_Set_Bandwidth      =10,
	TEF668x_Cmd_Set_RFAGC            =11,
	TEF668x_Cmd_Set_Antenna =12,

	TEF668x_Cmd_Set_MphSuppression = 20,
	TEF668x_Cmd_Set_ChannelEqualizer=22,
	TEF668x_Cmd_Set_NoiseBlanker = 23,
	TEF668x_Cmd_Set_NoiseBlanker_Audio = 24,

	TEF668x_Cmd_Set_DigitalRadio = 30,
	TEF668x_Cmd_Set_Deemphasis = 31,

	TEF668x_Cmd_Set_LevelStep = 38,
	TEF668x_Cmd_Set_LevelOffset = 39,

	TEF668x_Cmd_Set_Softmute_Time = 40,
	TEF668x_Cmd_Set_Softmute_Mod = 41,
	TEF668x_Cmd_Set_Softmute_Level = 42,
	TEF668x_Cmd_Set_Softmute_Noise = 43,
	TEF668x_Cmd_Set_Softmute_Mph = 44,
	TEF668x_Cmd_Set_Softmute_Max = 45,

	TEF668x_Cmd_Set_Highcut_Time = 50,
	TEF668x_Cmd_Set_Highcut_Mod = 51,
	TEF668x_Cmd_Set_Highcut_Level = 52,
	TEF668x_Cmd_Set_Highcut_Noise = 53,
	TEF668x_Cmd_Set_Highcut_Mph = 54,
	TEF668x_Cmd_Set_Highcut_Max = 55,
	TEF668x_Cmd_Set_Highcut_Min = 56,
	TEF668x_Cmd_Set_Lowcut_Min = 58,
	
	TEF668x_Cmd_Set_Stereo_Time = 60,
	TEF668x_Cmd_Set_Stereo_Mod = 61,
	TEF668x_Cmd_Set_Stereo_Level = 62,
	TEF668x_Cmd_Set_Stereo_Noise = 63,
	TEF668x_Cmd_Set_Stereo_Mph = 64,
	TEF668x_Cmd_Set_Stereo_Max = 65,
	TEF668x_Cmd_Set_Stereo_Min = 66,

	TEF668x_Cmd_Set_StHiBlend_Time = 70,
	TEF668x_Cmd_Set_StHiBlend_Mod = 71,
	TEF668x_Cmd_Set_StHiBlend_Level = 72,
	TEF668x_Cmd_Set_StHiBlend_Noise = 73,
	TEF668x_Cmd_Set_StHiBlend_Mph= 74,
	TEF668x_Cmd_Set_StHiBlend_Max = 75,
     TEF668x_Cmd_Set_StHiBlend_Min = 76,
	
	TEF668x_Cmd_Set_Scaler = 80,
	TEF668x_Cmd_Set_RDS = 81,
	TEF668x_Cmd_Set_QualityStatus = 82,
	TEF668x_Cmd_Set_DR_Blend = 83,
	TEF668x_Cmd_Set_DR_Options = 84,
	TEF668x_Cmd_Set_Specials = 85,

	TEF668x_Cmd_Get_Quality_Status = 128,
	TEF668x_Cmd_Get_Quality_Data = 129,
	TEF668x_Cmd_Get_RDS_Status = 130,
	TEF668x_Cmd_Get_RDS_Data = 131,
	TEF668x_Cmd_Get_AGC = 132,
	TEF668x_Cmd_Get_Signal_Status = 133,
	TEF668x_Cmd_Get_Processing_Status = 134,
	TEF668x_Cmd_Get_Interface_Status = 135,
} TEF668x_RADIO_COMMAND;

typedef enum
{
	TEF668x_Cmd_Set_Volume = 10,
	TEF668x_Cmd_Set_Mute = 11,
	TEF668x_Cmd_Set_Input = 12,
	TEF668x_Cmd_Set_Output_Source = 13,
	
	TEF668x_Cmd_Set_Ana_Out = 21,
	TEF668x_Cmd_Set_Dig_IO = 22,
	TEF668x_Cmd_Set_Input_Scaler = 23,
	TEF668x_Cmd_Set_WaveGen = 24
} TEF668x_AUDIO_COMMAND;

typedef enum
{
	TEF668x_Cmd_Set_OperationMode = 1,
	TEF668x_Cmd_Set_GPIO = 3,
	TEF668x_Cmd_Set_ReferenceClock = 4,
	TEF668x_Cmd_Activate = 5,

	TEF668x_Cmd_Get_Operation_Status = 128,
	TEF668x_Cmd_Get_GPIO_Status = 129,
	TEF668x_Cmd_Get_Identification = 130,
	TEF668x_Cmd_Get_LastWrite = 131
} TEF668x_APPL_COMMAND;


/***********************/
/* Write mode commands */
/***********************/

#define TEF668x_COMMAND_TUNETO                  0x01U
#define TEF668x_COMMAND_SETTUNEOPTIONS          0x02U
#define TEF668x_COMMAND_SETBANDWIDTH            0x0AU
#define TEF668x_COMMAND_SETRFAGC                0x0BU
#define TEF668x_COMMAND_SETANTENNA              0x0CU

#define TEF668x_COMMAND_SETMPHSUPPRESSION       0x14U
#define TEF668x_COMMAND_SETCHANNELEQUALIZER     0x16U
#define TEF668x_COMMAND_SETNOISEBLANKER         0x17U
#define TEF668x_COMMAND_SETNOISEBLANKERAUDIO    0x18U

#define TEF668x_COMMAND_SETDIGITALRADIO         0x1EU
#define TEF668x_COMMAND_SETDEEMPHASIS           0x1FU

#define TEF668x_COMMAND_SETLEVELOFFSET          0x27U

#define TEF668x_COMMAND_SETSOFTMUTETIME         0x28U
#define TEF668x_COMMAND_SETSOFTMUTEMOD          0x29U
#define TEF668x_COMMAND_SETSOFTMUTELEVEL        0x2AU
#define TEF668x_COMMAND_SETSOFTMUTENOISE        0x2BU
#define TEF668x_COMMAND_SETSOFTMUTEMPH          0x2CU
#define TEF668x_COMMAND_SETSOFTMUTEMAX          0x2DU

#define TEF668x_COMMAND_SETHIGHCUTTIME          0x32U
#define TEF668x_COMMAND_SETHIGHCUTMOD           0x33U
#define TEF668x_COMMAND_SETHIGHCUTLEVEL         0x34U
#define TEF668x_COMMAND_SETHIGHCUTNOISE         0x35U
#define TEF668x_COMMAND_SETHIGHCUTMPH           0x36U
#define TEF668x_COMMAND_SETHIGHCUTMAX           0x37U
#define TEF668x_COMMAND_SETHIGHCUTMIN           0x38U
#define TEF668x_COMMAND_SETLOWCUTMAX            0x39U
#define TEF668x_COMMAND_SETLOWCUTMIN            0x3AU

#define TEF668x_COMMAND_SETSTEREOTIME           0x3CU
#define TEF668x_COMMAND_SETSTEREOMOD            0x3DU
#define TEF668x_COMMAND_SETSTEREOLEVEL          0x3EU
#define TEF668x_COMMAND_SETSTEREONOISE          0x3FU
#define TEF668x_COMMAND_SETSTEREOMPH            0x40U
#define TEF668x_COMMAND_SETSTEREOMAX            0x42U

#define TEF668x_COMMAND_SETSTHIBLENDTIME        0x46U
#define TEF668x_COMMAND_SETSTHIBLENDMOD         0x47U
#define TEF668x_COMMAND_SETSTHIBLENDLEVEL       0x48U
#define TEF668x_COMMAND_SETSTHIBLENDNOISE       0x49U
#define TEF668x_COMMAND_SETSTHIBLENDMPH         0x4AU
#define TEF668x_COMMAND_SETSTHIBLENDMAX         0x4BU
#define TEF668x_COMMAND_SETSTHIBLENDMIN         0x4CU

#define TEF668x_COMMAND_SETSCALER               0x50U
#define TEF668x_COMMAND_SETRDS                  0x51U
#define TEF668x_COMMAND_SETQUALITYSTATUS        0x52U
#define TEF668x_COMMAND_SETDRBLEND              0x53U
#define TEF668x_COMMAND_SETDROPTIONS            0x54U
#define TEF668x_COMMAND_SETSPECIALS             0x55U

#define TEF668x_COMMAND_SETVOLUME               0x0AU
#define TEF668x_COMMAND_SETMUTE                 0x0BU
#define TEF668x_COMMAND_SETINPUT                0x0CU

#define TEF668x_COMMAND_SETANAOUT               0x15U
#define TEF668x_COMMAND_SETDIGIO                0x16U
#define TEF668x_COMMAND_SETINPUTSCALER          0x17U
#define TEF668x_COMMAND_SETWAVEGEN              0x18U

#define TEF668x_COMMAND_SETOPERATIONMODE        0x01U
#define TEF668x_COMMAND_SETKEYCODE              0x02U
#define TEF668x_COMMAND_SETGPIO                 0x03U
#define TEF668x_COMMAND_SETREFERENCECLOCK       0x04U
#define TEF668x_COMMAND_ACTIVATE                0x05U

/**********************/
/* Read mode commands */
/**********************/

#define TEF668x_API_READ_LENWR                  3
#define TEF668x_API_READ_QSTATUS_LEN            2
#define TEF668x_API_READ_QDATA_LEN              14
#define TEF668x_API_READ_AGC_LEN                4
#define TEF668x_API_READ_RDSSTATUS_LEN          2
#define TEF668x_API_READ_RDSDATA_LEN            10
#define TEF668x_API_READ_BLOCKERR_LEN           2

#define TEF668x_COMMAND_GETQUALITYSTATUS        0x80U
#define TEF668x_COMMAND_GETQUALITYDATA          0x81U
#define TEF668x_COMMAND_GETRDSSTATUS            0x82U
#define TEF668x_COMMAND_GETRDSDATA              0x83U
#define TEF668x_COMMAND_GETAGC                  0x84U
#define TEF668x_COMMAND_GETSIGNALSTATUS         0x85U
#define TEF668x_COMMAND_GETPROCESSINGSTATUS     0x86U
#define TEF668x_COMMAND_GETINTERFACESTATUS      0x87U

#define TEF668x_COMMAND_GETOPERATIONSTATUS      0x80U
#define TEF668x_COMMAND_GETGPIOSTATUS           0x81U
#define TEF668x_COMMAND_GETIDENTIFICATION       0x82U
#define TEF668x_COMMAND_GETLASTWRITE            0x83U

/****************/
/* Byte offsets */
/****************/

#define TEF668x_INDEX_TUNETO_MODE               1
#define TEF668x_INDEX_TUNETO_FREQUENCY          2

/**************/
/* Byte masks */
/**************/

/* Get_Quality */
#define TEF668x_GETQUALITY_STATUS_TIMESTAMP_MSK 0x01FF
#define TEF668x_GETQUALITY_STATUS_TIMESTAMP_POS 0

#define TEF668x_GETQUALITY_STATUS_AFUPDATE_MSK  0x8000
#define TEF668x_GETQUALITY_STATUS_AFUPDATE_POS  15


int devTEF668x_Write_Test(int len,...);

int devTEF668x_Set_Cmd(TEF668x_MODULE module, U8 cmd, int len,...);

int devTEF668x_Radio_Tune_To (bool fm,U16 mode,U16 frequency );
int devTEF668x_Radio_Get_Quality_Status (bool fm,U8 *status);
int devTEF668x_Radio_Get_Quality_Level (bool fm,U8 *status,int16_t *level);
int devTEF668x_Radio_Get_Quality_Data (bool fm,U8 *status,int16_t *level,U8 *usn,U8 *wam,int16_t *offset,int16_t *bandwidth,U8 *modulation);
int devTEF668x_APPL_Get_Operation_Status(U8 *status);
int devTEF668x_APPL_Set_ReferenceClock(U16 frequency_high,U16 frequency_low,U16 type);
int devTEF668x_APPL_Activate(U16 mode);
int devTEF668x_Patch_Init(void);
int devTEF668x_Audio_Set_Mute(U16 mode);
int devTEF668x_Audio_Set_Volume(int16_t volume);
int devTEF668x_Radio_Get_Signal_Status(bool fm,U16 *status);
int devTEF668x_Exist(void);
int devTEF668x_Para_Load(bool fm,bool update_all);
int devTEF668x_Audio_Set_Dig_IO(U16 signal,U16 mode,U16 format,U16 operation,U16 samplerate);
int devTEF668x_Audio_Set_Output_Source(eAudio_Output out,eAudio_Source source);
int devTEF668x_Audio_Set_OperationMode(U16 mode);


#endif 

