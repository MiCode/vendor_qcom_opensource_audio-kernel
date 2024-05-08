#ifndef _tef668x_H_
#define _tef668x_H_

/* AMP Current Adjustment Register */

 #define TEF_ADJUT_REG			3
 #define TEF_I2C_RETRIES			5
#define TEF_RETRY_WAIT_TIME  3000000
#define TEF668x_SPLIT_SIZE		24

#define TEF668x_REG_START                     0x14
#define TEF668x_REG_INIT                         0x1B
#define TEF668x_REG_CONTROL                0x1C
#define TEF668x_REG_FM_TUNE_TO          0x20
#define TEF668x_REG_AM_TUNE_TO          0x21
#define TEF668x_REG_AUDIO_SET_MUTE  0x30
#define TEF668x_REG_MODULE_INIT          0x40
#define TEF668x_REG_GET_PROPERTY     0xFA
#define TEF668x_REG_MAX                        0xFF

#define TEF668x_REF_CLK		9216000	//reference clock frequency
//#define TEF668x_REF_CLK		55466670	//reference clock frequency
#define TEF668x_IS_EXT_CLK	0	//external clock input

#define TEF668x_TIMER  50000   //5ms
#define TEF668x_TIMER_POWER   100000   //10ms
#define TEF668x_TIMER_WAIT_IDLE   500000  //50ms
#define TEF668x_TIMER_WAIT_ACTIVE   1000000  //100ms

#define TEF668x_CMD_LEN_MAX	20

#define High_16bto8b(a)	((uint8_t)((a) >> 8))
#define Low_16bto8b(a) 	((uint8_t)(a )) 

#define Convert8bto16b(a)	((uint16_t)(((uint16_t)(*(a))) << 8 |((uint16_t)(*(a+1)))))

#define TAB_NUM(tab)	(sizeof(tab)/sizeof(tab[0]))
#define TAB_NEXT_ADDR(tab)	(&(tab[TAB_NUM(tab)]))

#define SUCCESS 1


#define TEF668x_FM_FREQUENCY_MIN	6500		//step 10kHz
#define TEF668x_FM_FREQUENCY_MAX	10800
#define TEF668x_FM_FREQUENCY_STEP	10

#define TEF668x_LW_FREQUENCY_MIN	144		//step 1kHz
#define TEF668x_LW_FREQUENCY_MAX	288
#define TEF668x_LW_FREQUENCY_STEP	1

#define TEF668x_MW_FREQUENCY_MIN	522		//step 1kHz
#define TEF668x_MW_FREQUENCY_MAX	1710
#define TEF668x_MW_FREQUENCY_STEP	1

#define TEF668x_SW_FREQUENCY_MIN	2300		//step 1kHz
#define TEF668x_SW_FREQUENCY_MAX	27000
#define TEF668x_SW_FREQUENCY_STEP	1

struct tef668x_data {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct tef668x_device *tef668x_dev;
};

/*
TEF668x state transition times will fall within the following limits :
	Power-on  -> Boot state : power supply voltage settling + 5 ms.
	Boot state ->   Idle state : 50 ms.
	Idle state  ->  Active state : 100 ms
*/
typedef enum{
	eDevTEF668x_Boot_state = 0,
	eDevTEF668x_Idle_state,
	eDevTEF668x_Wait_Active,
	eDevTEF668x_Active_state,

	eDevTEF668x_Not_Exist,
	eDevTEF668x_Power_on,
	eDevTEF668x_Last
}TEF668x_STATE;

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

typedef enum
{
    eAR_TuningAction_Preset       = 1, /*!< Tune to new program with short mute time */
    eAR_TuningAction_Search       = 2, /*!< Tune to new program and stay muted */
    eAR_TuningAction_AF_Update    = 3, /*!< Tune to alternative frequency, store quality and tune back with inaudible mute */
    eAR_TuningAction_Jump         = 4, /*!< Tune to alternative frequency with short inaudible mute  */
    eAR_TuningAction_Check        = 5, /*!< Tune to alternative frequency and stay muted */
    eAR_TuningAction_End          = 7  /*!< Release the mute of a Search/Check action (frequency is ignored) */
} AR_TuningAction_t, *pAR_TuningAction_t;

extern int Radio_Tune_To(struct tef668x_data *tef668x, AR_TuningAction_t mode, U16 frequency );

#endif
