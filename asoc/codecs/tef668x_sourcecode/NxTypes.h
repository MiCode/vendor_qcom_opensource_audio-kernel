/******************************************************************************
   (c) NXP B.V. 2012. All rights reserved.

   This software is property of NXP Semiconductors. Unauthorized
   duplication and disclosure to third parties is prohibited.

   Disclaimer
   1. The NXP Software/Source Code is provided to Licensee "AS IS" without any
      warranties of any kind. NXP makes no warranties to Licensee and shall not
      indemnify Licensee or hold it harmless for any reason related to the NXP
      Software/Source Code or otherwise be liable to the NXP customer. The NXP
      customer acknowledges and agrees that the NXP Software/Source Code is
      provided AS-IS and accepts all risks of utilizing the NXP Software under
      the conditions set forth according to this disclaimer.

   2. NXP EXPRESSLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING,
      BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
      FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT OF INTELLECTUAL PROPERTY
      RIGHTS. NXP SHALL HAVE NO LIABILITY TO THE NXP CUSTOMER, OR ITS
      SUBSIDIARIES, AFFILIATES, OR ANY OTHER THIRD PARTY FOR ANY DAMAGES,
      INCLUDING WITHOUT LIMITATION, DAMAGES RESULTING OR ALLEGDED TO HAVE
      RESULTED FROM ANY DEFECT, ERROR OR OMMISSION IN THE NXP SOFTWARE/SOURCE
      CODE, THIRD PARTY APPLICATION SOFTWARE AND/OR DOCUMENTATION, OR AS A
      RESULT OF ANY INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHT OF ANY
      THIRD PARTY. IN NO EVENT SHALL NXP BE LIABLE FOR ANY INCIDENTAL,
      INDIRECT, SPECIAL, EXEMPLARY, PUNITIVE, OR CONSEQUENTIAL DAMAGES
      (INCLUDING LOST PROFITS) SUFFERED BY NXP CUSTOMER OR ITS SUBSIDIARIES,
      AFFILIATES, OR ANY OTHER THIRD PARTY ARISING OUT OF OR RELATED TO THE NXP
      SOFTWARE/SOURCE CODE EVEN IF NXP HAS BEEN ADVISED OF THE POSSIBILITY OF
      SUCH DAMAGES.

   3. NXP reserves the right to make changes to the NXP Software/Sourcecode any
      time, also without informing customer.

   4. Licensee agrees to indemnify and hold harmless NXP and its affiliated
      companies from and against any claims, suits, losses, damages,
      liabilities, costs and expenses (including reasonable attorney's fees)
      resulting from Licensee's and/or Licensee customer's/licensee's use of the
      NXP Software/Source Code.

*****************************************************************************/

#ifndef _NXTYPES_H_
#define _NXTYPES_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------
** Types and defines:
**-----------------------------------------------------------------------------
*/
#define OK           0
#define NOK          1

#define StatusOk        0
#define InitFailed      1

#ifdef __cplusplus
#define NULL            0
#else
#define NULL            ((void *) 0)
#endif

/**********************************
* Mask for data                   *
***********************************/
#define MASK_LSB_1BIT       0x01U
#define MASK_LSB_2BITS      0x03U
#define MASK_LSB_3BITS      0x07U
#define MASK_LSB_4BITS      0x0FU
#define MASK_LSB_5BITS      0x1FU
#define MASK_LSB_6BITS      0x3FU
#define MASK_LSB_7BITS      0x7FU
#define MASK_LSB_8BITS      0xFFU
#define SIGN_BIT_MASK_CHAR  0x80U
#define SIGN_BIT_PAD_24     0xFF000000U

#define SHIFT_3BYTE     24
#define SHIFT_2BYTE     16
#define SHIFT_1BYTE     8
#define SHIFT_0BYTE     0
#define BYTE3_MSK       0xFF000000
#define BYTE2_MSK       0x00FF0000
#define BYTE1_MSK       0x0000FF00
#define BYTE0_MSK       0x000000FF

#define GET_BYTE0(a) ((a) & BYTE0_MSK)
#define GET_BYTE1(a) (((a) & BYTE1_MSK) >> SHIFT_1BYTE)
#define GET_BYTE2(a) (((a) & BYTE2_MSK) >> SHIFT_2BYTE)
#define GET_BYTE3(a) (((a) & BYTE3_MSK) >> SHIFT_3BYTE)

/* read 4byte data from from 1byte memory area */
#define TM_UINT8_TO_UINT32(a,b,c,d) ((UInt32_t) ((UInt32_t) (a) << 24 | (UInt32_t) (b) << 16 | (UInt32_t) (c) << 8 | (UInt32_t) (d)))

/* read 2byte data from two 1byte memory area */
#define TM_UINT8_TO_UINT16(a,b) (((UInt16_t)(a)) << 8 | ((UInt16_t)(b)))

/* read 3byte data from three 1byte memory area */
#define TM_UINT8_TO_UINT24(a,b,c) ((((a) & SIGN_BIT_MASK_CHAR) == SIGN_BIT_MASK_CHAR) ? \
        (SIGN_BIT_PAD_24 | ((UInt32_t)(a)) << 16 | ((UInt32_t)(b) << 8) | (UInt32_t)(c)) : \
        (((UInt32_t)(a)) << 16 | ((UInt32_t)(b) << 8) | (UInt32_t)(c)))

/* Define return value for this implementation */
typedef uint32_t Status_t;
typedef uint32_t Handle_t;
typedef bool OnOff_t;

#define true    1
#define false   0

/********************************
* Structures for parameter
*********************************/

typedef struct
{
    uint16_t Status;
    int16_t Level;
    int16_t FreqOffset;
    uint8_t IFBandwidth;
    uint16_t Modulation;
} AM_QData_t;

typedef struct
{
    uint16_t Status;
    int16_t Level;
    uint16_t UltraSonicNoise;
    uint16_t WidebandAMMultipath;
    int16_t FreqOffset;
    uint8_t IFBandwidth;
    uint16_t Modulation;
} FM_QData_t;

typedef struct
{
    uint16_t Status;
    uint16_t groupA;
    uint16_t groupB;
    uint16_t groupC;
    uint16_t groupD;
    uint16_t BlockError;
} FM_RDSData_t;

typedef struct
{
    uint16_t InputAtt;
    uint16_t FeedbackAtt;
} AGC_Data_t;

typedef struct
{
    bool Stereo;
    bool DigSignal;
} SignalSts_t;

#define PRIMSYS         0
#define PRIMANARECV     0
#define SECANARECV      1
#define TERTANARECV     2
#define DIGRECVOFFSET   2
#define PRIMDIGRECV     0
#define SECDIGRECV      1
#define AUDRECVOFFSET   4
#define PRIMAUDRECV     0
#define SECAUDRECV      1
#define SECSECAUDRECV   2

#ifdef __cplusplus
}
#endif

#endif /* ifndef _NXTYPES_H_ */

