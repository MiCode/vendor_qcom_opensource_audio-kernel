#ifndef __OCA72XXX_PID_09_REG_H__
#define __OCA72XXX_PID_09_REG_H__

/* registers list */
#define OCA72XXX_PID_09_REG00		(0x00)
#define OCA72XXX_PID_09_REG01		(0x01)
#define OCA72XXX_PID_09_REG02		(0x02)
#define OCA72XXX_PID_09_REG03		(0x03)
#define OCA72XXX_PID_09_REG04		(0x04)
#define OCA72XXX_PID_09_REG05		(0x05)
#define OCA72XXX_PID_09_REG06		(0x06)
#define OCA72XXX_PID_09_REG07		(0x07)
#define OCA72XXX_PID_09_REG08		(0x08)
#define OCA72XXX_PID_09_REG09		(0x09)
#define OCA72XXX_PID_09_REG0A		(0x0A)
#define OCA72XXX_PID_09_REG0B		(0x0B)
#define OCA72XXX_PID_09_REG0C		(0x0C)
#define OCA72XXX_PID_09_REG0D		(0x0D)
#define OCA72XXX_PID_09_REG0E		(0x0E)
#define OCA72XXX_PID_09_REG0F		(0x0F)
#define OCA72XXX_PID_09_REG10		(0x10)
#define OCA72XXX_PID_09_REG11		(0x11)

/********************************************
 * soft control info
 * If you need to update this file, add this information manually
 *******************************************/
unsigned char oca72xxx_pid_09_softrst_access[2] = {0x00, 0xaa};

/********************************************
 * Register Access
 *******************************************/
#define OCA72XXX_PID_09_REG_MAX		(0x12)

#define REG_NONE_ACCESS					(0)
#define REG_RD_ACCESS					(1 << 0)
#define REG_WR_ACCESS					(1 << 1)

const unsigned char oca72xxx_pid_09_reg_access[OCA72XXX_PID_09_REG_MAX] = {
	[OCA72XXX_PID_09_REG00]	= (REG_RD_ACCESS),
	[OCA72XXX_PID_09_REG01]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG02]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG03]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG04]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG05]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG06]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG07]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG08]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG09]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG0A]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG0B]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG0C]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG0D]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG0E]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG0F]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG10]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[OCA72XXX_PID_09_REG11]	= (REG_RD_ACCESS | REG_WR_ACCESS),
};

#endif  /* #ifndef  __OCA72XXX_PID_09_REG_H__ */