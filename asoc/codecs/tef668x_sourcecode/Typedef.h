/***********************************************************************//**
 * @file		Typedef.h
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

#ifndef TYPEDEF_H
#define TYPEDEF_H
 
#ifdef ROOT
/* This definition is for I/O variable and EXT definition */
/* So please do not use the name "IO_MEM".*/
#define EXT
#define ext
#else
#define EXT extern
#define ext extern
#endif

#define MCU32bit 1

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef signed char int8_t;
typedef signed short int16_t;
typedef unsigned int bool;
typedef signed char I8;
typedef unsigned char U8;
typedef signed char S8;
typedef unsigned short int U16;
typedef signed short int S16;
typedef unsigned int U32;
typedef signed int S32;
typedef unsigned long long int U64;
typedef signed long long int S64;


#define UNCHAR		unsigned char
#define UCHAR		unsigned char
#define UNINT		unsigned int
#define UCHARCONST	unsigned char const
#define ULONG		unsigned long
#define UINT		unsigned int

#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef ERROR
#define ERROR 0xFF
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define PARA_ERROR 0xFE

typedef struct Bit_Char{
	UCHAR B0:1;
	UCHAR B1:1;
	UCHAR B2:1;
	UCHAR B3:1;
	UCHAR B4:1;
	UCHAR B5:1;
	UCHAR B6:1;
	UCHAR B7:1;
}BitChar;

typedef struct Bit_Int {
	unsigned int b0:1;
	unsigned int b1:1;
	unsigned int b2:1;
	unsigned int b3:1;
	unsigned int b4:1;
	unsigned int b5:1;
	unsigned int b6:1;
	unsigned int b7:1;
	unsigned int b8:1;
	unsigned int b9:1;
	unsigned int b10:1;
	unsigned int b11:1;
	unsigned int b12:1;
	unsigned int b13:1;
	unsigned int b14:1;
	unsigned int b15:1;
} BitInt;

typedef union word_Union {
	unsigned int	data_Int;
	BitInt	data_Intbit;
} word_field;

typedef union  byte_format{
	unsigned char	data_byte;
	BitChar	data_Bbit;
} byte_field;

typedef union io_word 
{
	unsigned int	io_w;
	unsigned char	io_b[2];
}word_io;

typedef struct {
	unsigned int b0:1;
	unsigned int b1:1;
	unsigned int b2:1;
	unsigned int b3:1;
	unsigned int b4:1;
	unsigned int b5:1;
	unsigned int b6:1;
	unsigned int b7:1;
	unsigned int b8:1;
	unsigned int b9:1;
	unsigned int b10:1;
	unsigned int b11:1;
	unsigned int b12:1;
	unsigned int b13:1;
	unsigned int b14:1;
	unsigned int b15:1;
	unsigned int b16:1;
	unsigned int b17:1;
	unsigned int b18:1;
	unsigned int b19:1;
	unsigned int b20:1;
	unsigned int b21:1;
	unsigned int b22:1;
	unsigned int b23:1;
	unsigned int b24:1;
	unsigned int b25:1;
	unsigned int b26:1;
	unsigned int b27:1;
	unsigned int b28:1;
	unsigned int b29:1;
	unsigned int b30:1;
	unsigned int b31:1;
} Bit32;


typedef union
{
	U8 ByteData[2];
	U16 Data;
}U16ToU8;

typedef union
{
	S8 ByteData[2];
	S16 Data;
}S16ToS8;

typedef union
{
	S8 ByteData[4];
	S32 Data;
}S32ToS8;

typedef union
{
	U8 ByteData[4];
	U32 Data;
}U32ToU8;

#define bit0		(1<<0)
#define bit1		(1<<1)
#define bit2		(1<<2)
#define bit3		(1<<3)
#define bit4		(1<<4)
#define bit5		(1<<5)
#define bit6		(1<<6)
#define bit7		(1<<7)
#define bit8		(1<<8)
#define bit9		(1<<9)
#define bit10		(1<<10)
#define bit11		(1<<11)
#define bit12		(1<<12)
#define bit13		(1<<13)
#define bit14		(1<<14)
#define bit15		(1<<15)
#define bit16		(1<<16)
#define bit17		(1<<17)
#define bit18		(1<<18)
#define bit19		(1<<19)
#define bit20		(1<<20)
#define bit21		(1<<21)
#define bit22		(1<<22)
#define bit23		(1<<23)
#define bit24		(1<<24)
#define bit25		(1<<25)
#define bit26		(1<<26)
#define bit27		(1<<27)
#define bit28		(1<<28)
#define bit29		(1<<29)
#define bit30		(1<<30)
#define bit31		(1<<31)

#endif

