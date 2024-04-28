

/* ---------------------------------------------------------------------------
 * Standard include files:
 * --------------------------------------------------------------------------- */

/*----------------------------------------------------------------------------
 * Project specific include files:
 * --------------------------------------------------------------------------- */



#include "public.h"

/* -----------------------------------------------------------------------------
 * Internal define:
 *-----------------------------------------------------------------------------*/
 //#define AR_Printf(...)
 #define AR_Printf	Dbg_printf
 
/* ---------------------------------------------------------------------------
 * Types and defines:
 * --------------------------------------------------------------------------- */

#define Is_FM	Radio_IsFMBand()

/* ---------------------------------------------------------------------------
 * Internal Prototypes (protected) for Driver initialization:
 * --------------------------------------------------------------------------- */


/* ---------------------------------------------------------------------------
 * Exported functions register abstraction layer:
 * --------------------------------------------------------------------------- */

int Radio_Tune_To(AR_TuningAction_t mode,   U16 frequency )
{
	int status;
#ifdef SYSTEM_INCLUDES_TEF665X
	status = devTEF665x_Radio_Tune_To(TEF665X_Is_FM_Freq(frequency), (U16)mode, frequency);
#endif
#ifdef SYSTEM_INCLUDES_TEF668X
	status = devTEF668x_Radio_Tune_To(TEF668x_Is_FM_Freq(frequency), (U16)mode, frequency);
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
     status = devTEF664x_Radio_Tune_To(mode, frequency);
#endif
	return status;
}


int Radio_Para_ReLoad(bool fm)
{
	int status;
#ifdef SYSTEM_INCLUDES_TEF665X
	status = devTEF665x_Para_Load(fm, 0);
#endif
#ifdef SYSTEM_INCLUDES_TEF668X
	status = devTEF668x_Para_Load(fm, 0);
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
     status = devTEF664x_Radio_Para_Write(fm);
#endif

	return status;
}


int Get_Quality_Status (bool fm,
    U8 *status )
{ 
#ifdef SYSTEM_INCLUDES_TEF668X
	return devTEF668x_Radio_Get_Quality_Status (fm,status);
#endif
#ifdef SYSTEM_INCLUDES_TEF665X
	return devTEF665x_Radio_Get_Quality_Status (fm,status);
#endif 
#ifdef SYSTEM_INCLUDES_TEF664X
     return  devTEF664x_Radio_Get_Quality_Status(status);
#endif
}

int AM_Get_Quality_Data (
    AM_QData_t *pAMQ )
{
     int r;
	S16 level,offset,bandwidth;
	U8 	status,modulation,wam,usn;
     #ifdef SYSTEM_INCLUDES_TEF668X
     r = devTEF668x_Radio_Get_Quality_Data(0,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
	#endif
	#ifdef SYSTEM_INCLUDES_TEF665X
     r = devTEF665x_Radio_Get_Quality_Data(0,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
	#endif
	#ifdef SYSTEM_INCLUDES_TEF664X
     r = devTEF664x_Radio_Get_Quality_Data(&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
     #endif
	if(r==SUCCESS)
		{
	       pAMQ->FreqOffset=offset;
		  pAMQ->IFBandwidth=bandwidth;
		  pAMQ->Level=level;
		  pAMQ->Status=status;
		}
		return r;
}
int FM_Get_Quality_Data (
    FM_QData_t *pFMQ )
{
     int r;
	S16 level,offset,bandwidth;
	U8 	status,modulation,wam,usn;
     #ifdef SYSTEM_INCLUDES_TEF668X
     r = devTEF668x_Radio_Get_Quality_Data(1,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
	#endif
	#ifdef SYSTEM_INCLUDES_TEF665X
     r = devTEF665x_Radio_Get_Quality_Data(1,&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
	#endif
	#ifdef SYSTEM_INCLUDES_TEF664X
     r = devTEF664x_Radio_Get_Quality_Data(&status,&level,&usn,&wam,&offset,&bandwidth,&modulation);
     #endif

	if(r==SUCCESS)
		{
	       pFMQ->FreqOffset=offset;
		  pFMQ->IFBandwidth=bandwidth;
		  pFMQ->Level=level;
		  pFMQ->Status=status;
		  pFMQ->Modulation=modulation;
		  pFMQ->UltraSonicNoise=usn;
		  pFMQ->WidebandAMMultipath=wam;
		}
		return r;
}

int Get_Level_Data (
    bool fm,S16 *pLev )
{
     int r;
	U8 status;
	
#ifdef SYSTEM_INCLUDES_TEF668X
	r = devTEF668x_Radio_Get_Quality_Level(fm,&status,pLev);
	
	return r;
		
#endif
#ifdef SYSTEM_INCLUDES_TEF665X
	r = devTEF665x_Radio_Get_Quality_Level(fm,&status,pLev);
	
	return r;
		
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
	r = devTEF664x_Radio_Get_Quality_Level(&status,pLev);
	return r;
		
#endif
}
int Get_Signal_Status(bool fm,SignalSts_t *pST)
{
int r;

#ifdef SYSTEM_INCLUDES_TEF668X
    U16 status;
	r = devTEF668x_Radio_Get_Signal_Status(fm,&status);
	if(r==SUCCESS)
		{
     pST->Stereo=( status & bit15)? TRUE :FALSE;
	pST->DigSignal=( status & bit14)? TRUE :FALSE;
	}
	return r;
		
#endif
#ifdef SYSTEM_INCLUDES_TEF665X
    U16 status;
	r = devTEF665x_Radio_Get_Signal_Status(fm,&status);
	if(r==SUCCESS)
	{
      pST->Stereo=( status & bit15)? TRUE :FALSE ;
	 pST->DigSignal=( status & bit14)? TRUE :FALSE;
	}
	return r;
		
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
    U8 status;
	r = devTEF664x_Radio_Get_Quality_Status(&status);
	if(r==SUCCESS)
	{
      pST->Stereo=( status & bit3)? TRUE :FALSE ;
	 pST->DigSignal=FALSE;
	}
	return r;
		
#endif    
	
}
int APPL_Set_ReferenceClock(U32 frequency, bool is_ext_clk)
{
#ifdef SYSTEM_INCLUDES_TEF668X
	return devTEF668x_APPL_Set_ReferenceClock((U16)(frequency >> 16), (U16)frequency, is_ext_clk);
#endif
#ifdef SYSTEM_INCLUDES_TEF665X
	return devTEF665x_APPL_Set_ReferenceClock((U16)(frequency >> 16), (U16)frequency, is_ext_clk);
#endif
}

int APPL_Activate(void)
{
#ifdef SYSTEM_INCLUDES_TEF668X
	return devTEF668x_APPL_Activate(1);
#endif
#ifdef SYSTEM_INCLUDES_TEF665X
	return devTEF665x_APPL_Activate(1);
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
	
#endif
}


//mute=1, unmute=0
int AUDIO_Set_Mute(bool mute)
{
	#ifdef SYSTEM_INCLUDES_TEF665X
		return devTEF665x_Audio_Set_Mute(mute);//AUDIO_Set_Mute mode = 0 : disable mute
	#endif
	#ifdef SYSTEM_INCLUDES_TEF668X
		return devTEF668x_Audio_Set_Mute(mute);//AUDIO_Set_Mute mode = 0 : disable mute
	#endif
	#ifdef SYSTEM_INCLUDES_TEF664X
		return devTEF664x_Audio_Set_Mute(mute);//AUDIO_Set_Mute mode = 0 : disable mute
	#endif
}
//-60 … +24 dB volume
int AUDIO_Set_Volume(int vol)
{
#ifdef SYSTEM_INCLUDES_TEF665X
	return devTEF665x_Audio_Set_Volume((int16_t)vol);
#endif
#ifdef SYSTEM_INCLUDES_TEF668X
      return devTEF668x_Audio_Set_Volume((int16_t)vol);
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
	return devTEF664x_Audio_Set_Volume((int16_t)vol);
	#endif
}

int Set_Output_Source(eAudio_Output out,eAudio_Source source)
{
	
	#ifdef SYSTEM_INCLUDES_TEF665X
	return devTEF665x_Audio_Set_Output_Source(out,source);
	#endif
	#ifdef SYSTEM_INCLUDES_TEF668X
	return devTEF668x_Audio_Set_Output_Source(out,source);
	#endif
	#ifdef SYSTEM_INCLUDES_TEF664X
	return devTEF664x_Audio_Set_Output_Source(source);
	#endif
}

/*
AUDIO_Set_Dig_IO (1, 33, 2, 32, 256, 4800)
Dig. output 32 bit I²S, master, 48 kHz
*/
int AUDIO_Set_Dig_IO(eDIG_IO sig,eDIG_IO mod,eDIG_IO form,eDIG_IO oper, eDIG_IO sample)
{
	U16 signal,mode,format,operation,samplerate;

	switch(sig){
		case IIS_SD_0:
			signal = 32;
			break;
		case IIS_SD_1:
		default:
			signal = 33;
			break;
	}

	switch(mod){
		case IO_OFF:
			mode  =0;
			break;
		case IO_INPUT:
			mode  =1;
			break;
			
		case IO_OUTPUT:
		default:
			mode  =2;
			break;
	}

	switch(form){
		case IIS_16_BITS:
			format = 16;
			break;
		default:
		case IIS_32_BITS:
			format = 32;
			break;
		case LSB_16BITS:
			format = 272;
			break;
		case LSB_18BITS:
			format = 274;
			break;
		case LSB_20BITS:
			format = 276;
			break;
		case LSB_24BITS:
			format = 280;
			break;
	}

	switch(oper){
		case MODE_SLAVE:
			operation = 0;
			break;
		default:
			operation = 256;
			break;
	}

	switch(sample){
		case SAMPLE_RATE_3200:
			samplerate = 3200;
			break;
		case SAMPLE_RATE_4410:
			samplerate = 4410;
			break;
		case SAMPLE_RATE_4800:
			samplerate = 4800;
			break;
			
	}
#ifdef SYSTEM_INCLUDES_TEF665X	
	return devTEF665x_Audio_Set_Dig_IO(signal,mode, format,operation,samplerate);
#endif
#ifdef SYSTEM_INCLUDES_TEF668X	
	return devTEF668x_Audio_Set_Dig_IO(signal,mode, format,operation,samplerate);
#endif
}

//TRUE = ON;
//FALSE = OFF
void APPL_PowerSwitch(bool OnOff)
{
#ifdef SYSTEM_INCLUDES_TEF665X	
		devTEF665x_Audio_Set_OperationMode(OnOff? 0 : 1);//standby mode = 1
#endif
#ifdef SYSTEM_INCLUDES_TEF668X
		devTEF668x_Audio_Set_OperationMode(OnOff? 0 : 1);//standby mode = 1
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
	
#endif
}
void NXP_Device_Boot_Proc(void)
{
#ifdef SYSTEM_INCLUDES_TEF665X	
	DevTEF665x_Boot_Process();	
#endif
#ifdef SYSTEM_INCLUDES_TEF668X
     DevTEF668x_Boot_Process();	
#endif
#ifdef SYSTEM_INCLUDES_TEF664X
	Helio_Boot_Process();
#endif

}

