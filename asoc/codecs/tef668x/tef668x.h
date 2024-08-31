#ifndef _tef668x_H_
#define _tef668x_H_

/* AMP Current Adjustment Register */

#define TEF_ADJUT_REG 3
#define TEF_I2C_RETRIES 5
#define TEF_RETRY_WAIT_TIME 3000000
#define TEF668x_SPLIT_SIZE 24

#define TEF668x_REG_START 0x14
#define TEF668x_REG_INIT 0x1B
#define TEF668x_REG_CONTROL 0x1C
#define TEF668x_REG_FM_TUNE_TO 0x20
#define TEF668x_REG_AM_TUNE_TO 0x21
#define TEF668x_REG_AUDIO_SET_MUTE 0x30
#define TEF668x_REG_MODULE_INIT 0x40
#define TEF668x_REG_GET_PROPERTY 0xFA
#define TEF668x_REG_MAX 0xFF

#define TEF668x_REF_CLK 9216000 //reference clock frequency
//#define TEF668x_REF_CLK		55466670	//reference clock frequency
#define TEF668x_IS_EXT_CLK 0 //external clock input

#define TEF668x_TIMER_WAIT_SEEK 10000 //xiaomi add 10ms wait
#define TEF668x_TIMER_WAIT_USN 1000 //xiaomi add 1ms wait
#define TEF668x_TIMER 5000 //5ms
#define TEF668x_TIMER_POWER 10000 //10ms
#define TEF668x_TIMER_WAIT_IDLE 50000 //50ms
#define TEF668x_TIMER_WAIT_ACTIVE 100000 //100ms

#define TEF668x_CMD_LEN_MAX 20

/*station max*/
#define MaxStationNum 7 /*Station count(6)+current statuin(1)1*/

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW 0;
#endif

/*area define*/
#define Radio_CHN 0
#define Radio_EUR 1
#define Radio_USA 2
#define Radio_JPN 3

/* area config*/
#define AreaSelect Radio_CHN

/*wait time define*/
#define TUNER_SCAN_WAITE_TIME 50
/*Scan play time*/
#define TUNER_SCAN_PLAY_TIME 5000
/*Fm stereo check time*/
#define FM_CHECK_STEREO_TIME 50

#define TUNER_LEVEL_14dB 14
#define TUNER_LEVEL_20dB 20
#define TUNER_LEVEL_25dB 25
#define TUNER_LEVEL_35dB 35
#define TUNER_LEVEL_45dB 45
#define TUNER_LEVEL_55dB 55

#define TUNER_PERCENT_27 27
#define TUNER_PERCENT_23 23

#define TUNER_OFFSET_10KHz 100
#define TUNER_OFFSET_1500Hz 15

#define TUNER_FM_BANDWITDTH 900 //xiaomi add

/*radio band define*/
#define MaxBandNum 6
#define FM1_BAND 0
#define FM2_BAND 1
#define FM3_BAND 2
#define MW_BAND 3
#define LW_BAND 4
#define SW_BAND 5

/* tuner mode */
#define Radio_PRESETMODE 0x10
#define Radio_SEARCHMODE 0x20
#define Radio_AFUPDATEMODE 0x30
#define Radio_JUMPMODE 0x40
#define Radio_CHECKMODE 0x50
#define Radio_ENDMODE 0x70

/*one step define
NOTE:FM Freqency uint is 10KHz,AM Freqency uint is 1KHz*/
#define AM_Step_9k 9 //am step 9khz
#define AM_Step_10k 10 //am step 10khz
#define FM_Step_50k 5 //fm step 50khz
#define FM_Step_100k 10 //fm step 10 0khz
#define FM_Step_200k 20 //fm step 200khz

/*check station result */
#define NO_STATION 90
#define PRESENT_STATION 100

#define RADIO_FM_LEVEL_AVAILABLE_TIME 4 //5 ms after tuning
#define RADIO_AM_LEVEL_AVAILABLE_TIME 36 //36 ms after tuning
#define RADIO_USN_AVAILABLE_TIME 34 //34ms

#define FM_SCAN_LEVEL TUNER_LEVEL_25dB //dB, 0.5dB/step -8dB~99.5dB
#define AM_SCAN_LEVEL TUNER_LEVEL_35dB //dB, 0.5dB/step -8dB~99.5dB
#define FM_SCAN_LEVEL_HI TUNER_LEVEL_45dB //dB, 0.5dB/step -8dB~99.5dB
#define AM_SCAN_LEVEL_HI TUNER_LEVEL_55dB //dB, 0.5dB/step -8dB~99.5dB

#define FM_USN_DISTURBANCE TUNER_PERCENT_27 //disturbance = 27%
#define FM_WAM_DISTURBANCE TUNER_PERCENT_23 //disturbance = 23%

#define FM_FREQ_OFFSET TUNER_OFFSET_10KHz //offset = 10k, step=100Hz
#define AM_FREQ_OFFSET TUNER_OFFSET_1500Hz //offset = 1.5k, step=100Hz

#define FM_BAND_WIDTH TUNER_FM_BANDWITDTH //xiaomi add for Radio_CheckStation

#define High_16bto8b(a) ((uint8_t)((a) >> 8))
#define Low_16bto8b(a) ((uint8_t)(a))

#define Convert8bto16b(a) \
	((uint16_t)(((uint16_t)(*(a))) << 8 | ((uint16_t)(*(a + 1)))))

#define TAB_NUM(tab) (sizeof(tab) / sizeof(tab[0]))
#define TAB_NEXT_ADDR(tab) (&(tab[TAB_NUM(tab)]))

#define SUCCESS 1

#define TEF668x_FM_FREQUENCY_MIN 6500 //step 10kHz
#define TEF668x_FM_FREQUENCY_MAX 10800
#define TEF668x_FM_FREQUENCY_STEP 10

#define TEF668x_LW_FREQUENCY_MIN 144 //step 1kHz
#define TEF668x_LW_FREQUENCY_MAX 288
#define TEF668x_LW_FREQUENCY_STEP 1

#define TEF668x_MW_FREQUENCY_MIN 522 //step 1kHz
#define TEF668x_MW_FREQUENCY_MAX 1710
#define TEF668x_MW_FREQUENCY_STEP 1

#define TEF668x_SW_FREQUENCY_MIN 2300 //step 1kHz
#define TEF668x_SW_FREQUENCY_MAX 27000
#define TEF668x_SW_FREQUENCY_STEP 1

struct tef668x_data {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct tef668x_device *tef668x_dev;
};

/*radio chip device type*/
typedef enum {
	Radio_Hero,
	Radio_Atomic,
	Radio_Atomic2, // tef668x is Atomic2
	Radio_Helio,
	Radio_Last
} eDev_Type;

/*are config parameters struct*/
typedef struct { //area radio parameter
	U32 FM_MaxFreq; //fm max freq
	U32 FM_MinFreq; //fm min freq
	U32 AM_MaxFreq; // am max freq
	U32 AM_MinFreq; //am min freq
	U32 FM_AutoSeekStep; // fm step
	U32 FM_ManualSeekStep;
	U32 AM_AutoSeekStep; //am step
	U32 AM_ManualSeekStep;
} Radio_AreaConfigDef;

/*band freq range*/
typedef struct {
	U16 MinFreq;
	U16 MaxFreq;
} FreqBaundDef;

/*tation freq*/
typedef struct {
	U16 Freq[MaxStationNum]; //0-current work freq,or backup freq;
} StationMemType;

/*
TEF668x state transition times will fall within the following limits :
	Power-on  -> Boot state : power supply voltage settling + 5 ms.
	Boot state ->   Idle state : 50 ms.
	Idle state  ->  Active state : 100 ms
*/
typedef enum {
	eDevTEF668x_Boot_state = 0,
	eDevTEF668x_Idle_state,
	eDevTEF668x_Wait_Active,
	eDevTEF668x_Active_state,

	eDevTEF668x_Not_Exist,
	eDevTEF668x_Power_on,
	eDevTEF668x_Last
} TEF668x_STATE;

typedef enum {
	TEF668x_Cmd_Tune_To = 1,
	TEF668x_Cmd_Set_Tune_Options = 2,
	TEF668x_Cmd_Set_Bandwidth = 10,
	TEF668x_Cmd_Set_RFAGC = 11,
	TEF668x_Cmd_Set_Antenna = 12,

	TEF668x_Cmd_Set_MphSuppression = 20,
	TEF668x_Cmd_Set_ChannelEqualizer = 22,
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
	TEF668x_Cmd_Set_StHiBlend_Mph = 74,
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

typedef enum {
	TEF668x_Cmd_Set_Volume = 10,
	TEF668x_Cmd_Set_Mute = 11,
	TEF668x_Cmd_Set_Input = 12,
	TEF668x_Cmd_Set_Output_Source = 13,

	TEF668x_Cmd_Set_Ana_Out = 21,
	TEF668x_Cmd_Set_Dig_IO = 22,
	TEF668x_Cmd_Set_Input_Scaler = 23,
	TEF668x_Cmd_Set_WaveGen = 24
} TEF668x_AUDIO_COMMAND;

typedef enum {
	TEF668x_Cmd_Set_OperationMode = 1,
	TEF668x_Cmd_Set_GPIO = 3,
	TEF668x_Cmd_Set_ReferenceClock = 4,
	TEF668x_Cmd_Activate = 5,

	TEF668x_Cmd_Get_Operation_Status = 128,
	TEF668x_Cmd_Get_GPIO_Status = 129,
	TEF668x_Cmd_Get_Identification = 130,
	TEF668x_Cmd_Get_LastWrite = 131
} TEF668x_APPL_COMMAND;

typedef enum {
	eAR_TuningAction_Preset =
		1, /*!< Tune to new program with short mute time */
	eAR_TuningAction_Search = 2, /*!< Tune to new program and stay muted */
	eAR_TuningAction_AF_Update =
		3, /*!< Tune to alternative frequency, store quality and tune back with inaudible mute */
	eAR_TuningAction_Jump =
		4, /*!< Tune to alternative frequency with short inaudible mute  */
	eAR_TuningAction_Check =
		5, /*!< Tune to alternative frequency and stay muted */
	eAR_TuningAction_End =
		7 /*!< Release the mute of a Search/Check action (frequency is ignored) */
} AR_TuningAction_t,
	*pAR_TuningAction_t;
#endif
