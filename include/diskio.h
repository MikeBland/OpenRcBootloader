/*
 * Authors (alphabetical order)
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny <shadow@privy.de>
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * open9x is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*-----------------------------------------------------------------------
/  Low level disk interface modlue include file   (C)ChaN, 2010
/-----------------------------------------------------------------------*/

#ifndef DEF_DISKIO
#define DEF_DISKIO

#ifdef PCBSKY
#include "board.h"
#endif
#if ( defined(PCBX9D) || defined(PCB9XT) )
#include "stm32f2xx.h"
#endif

#ifndef PCBX12D
#if !defined(SIMU)
#include "core_cm3.h"
#endif
#else
#include "stm32f4xx.h"
#include "core_cm4.h"
#endif

#include "integer.h"

#define DN_MCI		0	/* Physical drive number for MCI */
#define DN_NAND		1	/* Physical drive number for NAND flash */


/* These functions are defined in asmfunc.S */
void Copy_al2un (BYTE *dst, const DWORD *src, int count);	/* Copy aligned to unaligned. */
void Copy_un2al (DWORD *dst, const BYTE *src, int count);	/* Copy unaligned to aligned. */


/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

DSTATUS disk_initialize (BYTE);
DSTATUS disk_status (BYTE);

#ifdef __cplusplus
extern "C" {
#endif
DRESULT disk_read (BYTE, BYTE*, DWORD, BYTE);
DRESULT disk_write (BYTE, const BYTE*, DWORD, BYTE);
DRESULT disk_ioctl (BYTE, BYTE, void*);
#ifdef __cplusplus
}
#endif

void disk_timerproc (void);


/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */


/* Command code for disk_ioctrl fucntion */

/* Generic ioctl command (defined for FatFs) */
#define CTRL_SYNC			0	/* Flush disk cache (for write functions) */
#define GET_SECTOR_COUNT	1	/* Get media size (for only f_mkfs()) */
#define GET_SECTOR_SIZE		2	/* Get sector size (for multiple sector size (_MAX_SS >= 1024)) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (for only f_mkfs()) */
#define CTRL_ERASE_SECTOR	4	/* Force erased a block of sectors (for only _USE_ERASE) */

/* Generic ioctl command */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */

/* NAND specific ioctl command */
#define NAND_FORMAT			30	/* Create physical format */



#ifdef PCBSKY
volatile extern BYTE Timer1, Timer2;	/* 100Hz decrement timer */
#endif

//#ifdef PCBX9D
//volatile extern DWORD Timer1, Timer2;	/* 100Hz decrement timer */
//#endif

//------------------------------------------------------------------------------
/// Detect if SD card is connected
//------------------------------------------------------------------------------
#define CardIsConnected() ( (PIOB->PIO_PDSR & PIO_PB7) == 0 )

extern uint32_t Card_ID[4] ;
extern uint32_t Card_SCR[2] ;
extern uint32_t Card_CSD[4] ;
extern uint32_t Sd_128_resp[4] ;
extern uint32_t Sd_rca ;
//extern uint32_t Cmd_55_resp ;

extern uint32_t SD_SetBusWidth( uint32_t busWidth) ;
extern void SD_EnableHsMode( uint8_t hsEnable) ;
extern uint32_t SD_SetSpeed( uint32_t mciSpeed ) ;
extern void SD_Reset( uint8_t keepSettings) ;
extern uint32_t sd_cmd55( void ) ;
extern uint32_t sd_acmd41( void ) ;
extern uint32_t sd_cmd2( void ) ;
extern uint32_t sd_cmd3( void ) ;
extern uint32_t sd_cmd7( void ) ;
extern uint32_t sd_cmd9( void ) ;
extern uint32_t sd_cmd16( void ) ;
extern uint32_t sd_acmd6( void ) ;
extern uint32_t sd_acmd51( uint32_t *presult ) ;
extern uint32_t sd_cmd13( uint32_t *status) ;
extern void sd_poll_10mS( void ) ;
#if (defined(PCBX9D) || defined(PCB9XT))
extern void sdPoll10ms( void ) ;
extern void sdInit( void ) ;
#endif
extern uint32_t sd_card_ready( void ) ;
extern uint32_t sd_read_block( uint32_t block_no, uint32_t *data ) ;

extern DWORD socket_is_empty( void ) ;

extern int8_t SD_ReadSectors(uint8_t *buff, uint32_t sector, uint32_t count) ;
extern int8_t SD_WriteSectors(uint8_t *buff, uint32_t sector, uint32_t count) ;

#endif
