/****************************************************************************
*  Copyright (c) 2018 by Michael Blandford. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
* Other Authors:
 * - Andre Bernet
 * - Bertrand Songis
 * - Bryan J. Rentoul (Gruvin)
 * - Cameron Weeks
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini
 * - Thomas Husterer
*
****************************************************************************/


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef PCBSKY
#include "AT91SAM3S4.h"
#include "core_cm3.h"
#endif

#include "radio.h"
#include "lcd.h"
#include "ff.h"
#include "diskio.h"
#include "drivers.h"
#include "logicio.h"

#include "menucontrol.h"

__attribute__ ((section(".version"), used))
// Temp edit to force a push
const uint8_t Version[] =
{
#ifdef PCBSKY
	'A', 'P', 'P', 'S', 'K', 'Y'
#endif
} ;

__attribute__ ((section(".text"), used))

#if defined(PCBSKY)
extern void init_rotary_encoder() ;
extern void checkRotaryEncoder() ;
#endif

#ifdef PCBSKY

#define RADIO_SKY	0
#define RADIO_PRO	1
#define RADIO_AR9X	2

void init_mtwi( void ) ;
#endif

volatile uint8_t  g_blinkTmr10ms;

#ifdef PCBSKY
uint32_t LastResult ;
#endif

// states
#define ST_MENU						0
#define ST_START					1
#define ST_DIR_CHECK			2
#define ST_OPEN_DIR				3
#define ST_FILE_LIST			4
#define ST_FLASH_CHECK		5
#define ST_FLASHING				6
#define ST_FLASH_DONE			7
#define ST_LOAD_APP				8
#define ST_USB						10
#define ST_REBOOT					11


#define MENU_NOTHING	0
#define MENU_EXIT			1
#define MENU_OPT1			2
#define MENU_OPT2			3

// Values in state
#define UPDATE_NO_FILES		0
#define UPDATE_FILE_LIST	1
#define UPDATE_CONFIRM		2
#define UPDATE_SELECTED		3
#define UPDATE_INVALID		4
#define UPDATE_ACTION			5
#define UPDATE_COMPLETE		6


struct fileControl
{
	uint8_t index ;
	uint32_t nameCount ;
	uint32_t vpos ;
	uint32_t hpos ;
	uint8_t ext[4] ;
} ;

uint32_t check_ready( void ) ;
uint32_t write_CoProc( uint32_t coInternalAddr, uint8_t *buf, uint32_t number ) ;
uint32_t coProcBoot( void ) ;
uint32_t read_status_CoProc( uint32_t bufaddr, uint32_t number ) ;

uint8_t Twi_rx_buf[26] ;
uint8_t CoProresult ;
uint8_t CoProcReady ;
uint8_t Addr_buffer[132] ;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

uint32_t FirmwareSize ;

uint32_t Master_frequency ;
volatile uint8_t  Tenms ;
//uint8_t EE_timer ;
//uint8_t USBcounter ;
//uint8_t SDcardDisabled ;
volatile uint16_t BlinkCounter ;
extern uint32_t sd_card_ready( void ) ;
//uint8_t UpdateItem ;

//TCHAR FlashFilename[60] ;
FATFS g_FATFS ;
//FIL FlashFile ;
DIR Dj ;
FILINFO Finfo ;

TCHAR Filenames[8][50] ;
uint32_t FileSize[20] ;
//uint32_t FnStartIndex ;
//uint32_t Valid ;
//uint8_t FIleExtension[4] ;

//uint32_t FlashSize ;


//uint32_t FlashBlocked = 1 ;
//uint32_t LockBits ;

//uint32_t Block_buffer[1024] ;
//UINT BlockCount ;

// Use a struct to force long word alignment
struct t_fileData
{
	uint8_t FileData[1024] ;
	uint8_t ExtraFileData[1024] ;
} Fdata ;

uint32_t BytesFlashed ;
uint32_t ByteEnd ;
uint32_t BlockOffset ;

struct fileControl FileControl = {	0,0,0,0, {0,0,0,0} } ;

struct t_maintenance
{
	TCHAR FlashFilename[60] ;
	FIL FlashFile ;
	UINT BlockCount ;
	UINT XblockCount ;
} Mdata ;

static uint32_t PowerUpDelay ;

uint8_t AllSubState ;
uint8_t XmegaSignature[4] ;

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

uint16_t getTmr2MHz()
{
#ifdef PCBSKY
	return TC1->TC_CHANNEL[0].TC_CV ;
#endif
}

#ifdef PCBSKY
// Start TIMER7 at 2000000Hz
void start_2Mhz_timer()
{
  register Tc *ptc ;
	register uint32_t timer ;

// Start Timer4 to provide 0.5uS clock for input capture

	timer = Master_frequency / (2*2000000) ;		// MCK/2 and 2MHz

	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x08000000L ;		// Enable peripheral clock to TC4

  ptc = TC1 ;		// Tc block 1 (TC3-5)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[1].TC_RC = timer ;
	ptc->TC_CHANNEL[1].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x0009C000 ;	// 0000 0000 0000 1001 1100 0000 0100 0000
																						// MCK/2, set @ RA, Clear @ RC waveform
	ptc->TC_CHANNEL[1].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)


	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x04000000L ;		// Enable peripheral clock to TC3

  ptc = TC1 ;		// Tc block 1 (TC3-5)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 2 ;
	ptc->TC_CHANNEL[0].TC_CMR = 0x00000000 ;	// Capture mode
	ptc->TC_CHANNEL[0].TC_CMR = 0x00090005 ;	// 0000 0000 0000 1001 0000 0000 0000 0101, XC0, A rise, B fall
	ptc->TC_CHANNEL[0].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)

//	configure_pins( PIO_PC23, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_B | PIN_PORTC | PIN_PULLUP ) ;
//	NVIC_SetPriority( TC3_IRQn, 14 ) ; // Low priority interrupt
//	NVIC_EnableIRQ(TC3_IRQn) ;
//	ptc->TC_CHANNEL[0].TC_IER = TC_IER0_LDRAS | TC_IER0_LDRBS ;
}

#endif


#ifdef PCBSKY
// Starts TIMER0 at full speed (MCK/2) for delay timing
// @ 36MHz this is 18MHz
// This was 6 MHz, we may need to slow it to TIMER_CLOCK2 (MCK/8=4.5 MHz)
void start_timer0()
{
  register Tc *ptc ;

  PMC->PMC_PCER0 |= 0x00800000L ;		// Enable peripheral clock to TC0

  ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 2 ;
	ptc->TC_CHANNEL[0].TC_CMR = 0x00008001 ;	// Waveform mode MCK/8 for 36MHz osc.(Upset be write below)
	ptc->TC_CHANNEL[0].TC_RC = 0xFFF0 ;
	ptc->TC_CHANNEL[0].TC_RA = 0 ;
	ptc->TC_CHANNEL[0].TC_CMR = 0x00008040 ;	// 0000 0000 0000 0000 1000 0000 0100 0000, stop at regC, 18MHz
	ptc->TC_CHANNEL[0].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
}

void stop_timer0( void )
{
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR0_CLKDIS ;		// Disable clock
}

void delay2ms()
{
	TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)
	while ( TC0->TC_CHANNEL[0].TC_CV < 36000 )		// 2mS, Value depends on MCK/2 (used 18MHz)
	{
		// Wait
	}
}

void delayNus( uint16_t time )
{
	time *= 18 ;				// Value depends on MCK/2 (used 18MHz)
	TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)
	while ( TC0->TC_CHANNEL[0].TC_CV < time )		// "time" uS, Value depends on MCK/2 (used 18MHz)
	{
		// Wait
	}
}
#endif

#if !defined(PCBTARANIS)
void lcd_putsnAtt0(uint8_t x,uint8_t y, const char * s,uint8_t len,uint8_t mode)
{
	register char c ;
  while(len!=0) {
    c = *s++ ;
		if ( c == 0 )
		{
			break ;			
		}
    x = lcd_putcAtt(x,y,c,mode);
    len--;
  }
}
#endif

void interrupt10ms()
{
	g_blinkTmr10ms += 1 ;
	BlinkCounter += 7 ;
	Tenms |= 1 ;			// 10 mS has passed
 	per10ms() ;
}

#ifdef PCBSKY
void init10msTimer()
{
  register Tc *ptc ;
	register uint32_t timer ;

  PMC->PMC_PCER0 |= 0x02000000L ;		// Enable peripheral clock to TC2

	timer = Master_frequency / 12800  ;		// MCK/128 and 100 Hz

  ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;
	ptc->TC_CHANNEL[2].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[2].TC_RC = timer ;			// 10 Hz
	ptc->TC_CHANNEL[2].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[2].TC_CMR = 0x0009C003 ;	// 0000 0000 0000 1001 1100 0000 0000 0011
																						// MCK/128, set @ RA, Clear @ RC waveform
	ptc->TC_CHANNEL[2].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
	
	NVIC_EnableIRQ(TC2_IRQn) ;
	TC0->TC_CHANNEL[2].TC_IER = TC_IER0_CPCS ;
}

extern "C" void TC2_IRQHandler()
{
  register uint32_t dummy;

  /* Clear status bit to acknowledge interrupt */
  dummy = TC0->TC_CHANNEL[2].TC_SR;
	(void) dummy ;		// Discard value - prevents compiler warning

	interrupt10ms() ;
	
}
#endif

uint8_t *cpystr( uint8_t *dest, uint8_t *source )
{
  while ( (*dest++ = *source++) )
    ;
  return dest - 1 ;
}

//FRESULT openFirmwareFile( uint32_t index )
//{
//	cpystr( cpystr( (uint8_t *)FlashFilename, (uint8_t *)"\\firmware\\" ), (uint8_t *)Filenames[index] ) ;
//	f_open( &FlashFile, FlashFilename, FA_READ ) ;
//	f_lseek ( &FlashFile, 32768 ) ;
//	return f_read( &FlashFile, (BYTE *)Block_buffer, 4096, &BlockCount ) ;
//}


extern Key keys[] ;

uint16_t statuses ;

FRESULT readBinDir( DIR *dj, FILINFO *fno, struct fileControl *fc )
{
	FRESULT fr ;
	uint32_t loop ;

	do
	{
		loop = 0 ;
		fr = f_readdir ( dj, fno ) ;		// First entry

		if ( fr != FR_OK || fno->fname[0] == 0 )
		{
			break ;
		}
		if ( *fno->lfname == 0 )
		{
			cpystr( (uint8_t *)fno->lfname, (uint8_t *)fno->fname ) ;		// Copy 8.3 name
		}
		if ( fc->ext[0] )
		{
			int32_t len = strlen(fno->lfname) - 4 ;
			if ( fc->ext[3] )
			{
				len -= 1 ;			
			}
			if ( len < 0 )
			{
				loop = 1 ;
			}
			if ( fno->lfname[len] != '.' )
			{
				loop = 1 ;
			}
			if ( ( fno->lfname[len+1] & ~0x20 ) != fc->ext[0] )
			{
				loop = 1 ;
			}
			if ( ( fno->lfname[len+2] & ~0x20 ) != fc->ext[1] )
			{
				loop = 1 ;
			}
			if ( ( fno->lfname[len+3] & ~0x20 ) != fc->ext[2] )
			{
				loop = 1 ;
			}
			if ( fc->ext[3] )
			{
				if ( ( fno->lfname[len+4] & ~0x20 ) != fc->ext[3] )
				{
					loop = 1 ;
				}
			}
		}
		else // looking for a Directory
		{
			if ( ( fno->fattrib & AM_DIR ) == 0 )
			{
				loop = 1 ;
			}
		}	
	} while ( loop ) ;
	return fr ;
}

DIR Djp ;
uint32_t fillNames( uint32_t index, struct fileControl *fc )
{
	uint32_t i ;
	FRESULT fr ;
	Finfo.lfname = Filenames[0] ;
	Finfo.lfsize = 48 ;
//  WatchdogTimeout = 200 ;
	DIR *pDj = &Dj ;	
	fr = f_readdir ( pDj, 0 ) ;					// rewind
	fr = f_readdir ( pDj, &Finfo ) ;		// Skip .
	fr = f_readdir ( pDj, &Finfo ) ;		// Skip ..
	i = 0 ;
	while ( i <= index )
	{
//  	WatchdogTimeout = 200 ;
		fr = readBinDir( pDj, &Finfo, fc ) ;		// First entry
		FileSize[0] = Finfo.fsize ;
		i += 1 ;
		if ( fr != FR_OK || Finfo.fname[0] == 0 )
		{
			return 0 ;
		}
	}
	for ( i = 1 ; i < 7 ; i += 1 )
	{
// 		WatchdogTimeout = 200 ;
		Finfo.lfname = Filenames[i] ;
		fr = readBinDir( pDj, &Finfo, fc ) ;		// First entry
		FileSize[i] = Finfo.fsize ;
		if ( fr != FR_OK || Finfo.fname[0] == 0 )
		{
			break ;
		}
	}
	return i ;
}



uint32_t fileList(uint8_t event, struct fileControl *fc )
{
	uint32_t limit ;
	uint32_t result = 0 ;
  uint8_t maxhsize ;
	uint32_t i ;
			 
	limit = 6 ;
	if ( fc->nameCount < limit )
	{
		limit = fc->nameCount ;						
	}
	maxhsize = 0 ;
	for ( i = 0 ; i < limit ; i += 1 )
	{
		uint32_t x ;
		uint32_t len ;
		len = x = strlen( Filenames[i] ) ;
		if ( x > maxhsize )
		{
			maxhsize = x ;							
		}
		if ( x > DISPLAY_CHAR_WIDTH )
		{
			if ( ( fc->hpos + DISPLAY_CHAR_WIDTH ) > x )
			{
				x = x - DISPLAY_CHAR_WIDTH ;
			}
			else
			{
				x = fc->hpos ;
			}
			len = DISPLAY_CHAR_WIDTH ;
		}
		else
		{
			x = 0 ;
		}
		lcd_putsn_P( 0, 16+FH*i, &Filenames[i][x], len ) ;
	}

	if ( ( event == EVT_KEY_REPT(BOOT_KEY_DOWN) ) || event == EVT_KEY_FIRST(BOOT_KEY_DOWN) )
	{
		if ( fc->vpos < limit-1 )
		{
			fc->vpos += 1 ;
		}
		else
		{
			if ( fc->nameCount > limit )
			{
				fc->index += 1 ;
				fc->nameCount = fillNames( fc->index, fc ) ;
			}
		}
	}
	if ( ( event == EVT_KEY_REPT(BOOT_KEY_UP)) || ( event == EVT_KEY_FIRST(BOOT_KEY_UP) ) )
	{
		if ( fc->vpos > 0 )
		{
			fc->vpos -= 1 ;
		}
		else
		{
			if ( fc->index )
			{
				fc->index -= 1 ;
				fc->nameCount = fillNames( fc->index, fc ) ;
			}
		}
	}
	if ( ( event == EVT_KEY_REPT(BOOT_KEY_RIGHT)) || ( event == EVT_KEY_FIRST(BOOT_KEY_RIGHT) ) )
	{
		if ( fc->hpos + DISPLAY_CHAR_WIDTH < maxhsize )	fc->hpos += 1 ;
	}
	if ( ( event == EVT_KEY_REPT(BOOT_KEY_LEFT)) || ( event == EVT_KEY_FIRST(BOOT_KEY_LEFT) ) )
	{
		if ( fc->hpos )	fc->hpos -= 1 ;
	}
	if ( ( event == EVT_KEY_LONG(BOOT_KEY_MENU) ) || ( event == EVT_KEY_BREAK(BTN_RE) ) )
	{
		// Select file to flash
		killEvents(event);
		result = 1 ;
	}
	if ( ( event == EVT_KEY_FIRST(BOOT_KEY_EXIT) ) || ( event == EVT_KEY_LONG(BTN_RE) ) )
	{
		// Select file to flash
		result = 2 ;
	}
	if ( event == EVT_KEY_BREAK(BOOT_KEY_MENU) )
	{
		// Tag file
		result = 3 ;
	}
#if defined(PCBTARANIS)
	lcd_filled_rect( 0, 2*FH+FH*fc->vpos, DISPLAY_CHAR_WIDTH*FW, 8, 0xFF, 0 ) ;
#else
	lcd_char_inverse( 0, 2*FH+FH*fc->vpos, DISPLAY_CHAR_WIDTH*FW, 0 ) ;
#endif
	return result ;
}


void menuUpCoPro(uint8_t event)
{
	static MState2 mstate2 ;
	
	FRESULT fr ;
	struct fileControl *fc = &FileControl ;
  static uint8_t mounted = 0 ;
	static uint32_t state ;
	static uint32_t firmwareAddress ;
	uint32_t i ;
	uint32_t width ;
	
	wdt_reset() ;
//	if ( WatchdogTimeout < 50 )
//	{
// 		WatchdogTimeout = 50 ;
//	}
	
	TITLE( "CoPro Update" ) ;
	mstate2.check_columns(event, 1 ) ;
	
	switch(event)
	{
    case EVT_ENTRY:
//  		WatchdogTimeout = 200 ;
			state = UPDATE_NO_FILES ;
			if ( mounted == 0 )
			{
  			fr = f_mount(0, &g_FATFS) ;
			}
			else
			{
				fr = FR_OK ;
			}
			if ( fr == FR_OK)
			{
				mounted = 1 ;
			}
			if ( mounted )
			{
				fr = f_chdir( (TCHAR *)"\\firmware" ) ;
				if ( fr == FR_OK )
				{
					state = UPDATE_NO_FILES ;
					fc->index = 0 ;
					fr = f_opendir( &Dj, (TCHAR *) "." ) ;
					if ( fr == FR_OK )
					{
						fc->ext[0] = 'B' ;
						fc->ext[1] = 'I' ;
						fc->ext[2] = 'N' ;
					}
					fc->ext[3] = 0 ;
					fc->index = 0 ;
					fc->nameCount = fillNames( 0, fc ) ;
					fc->hpos = 0 ;
					fc->vpos = 0 ;
					if ( fc->nameCount )
					{
						state = UPDATE_FILE_LIST ;
					}
				}
			}
    break ;
    
		case EVT_KEY_FIRST(BOOT_KEY_EXIT):
		case EVT_KEY_LONG(BTN_RE) :
			if ( state < UPDATE_ACTION )
			{
      	chainMenu(menuUpCoPro) ;
    		killEvents(event) ;
			}
    break ;
	}

	switch ( state )
	{
		case UPDATE_NO_FILES :
			lcd_puts_Pleft( 4*FH, "\005No Files" ) ;
//	    lcd_outdez( 21*FW, 4*FH, mounted ) ;
    break ;
		
		case UPDATE_FILE_LIST :
			if ( fileList( event, &FileControl ) == 1 )
			{
				state = UPDATE_CONFIRM ;
			}
    break ;
		case UPDATE_CONFIRM :
			lcd_puts_Pleft( 2*FH, "Flash Co-Proc. from" ) ;
			
			cpystr( cpystr( (uint8_t *)Mdata.FlashFilename, (uint8_t *)"\\firmware\\" ), (uint8_t *)Filenames[fc->vpos] ) ;
#if defined(PCBTARANIS)
			lcd_putsnAtt( 0, 4*FH, Filenames[fc->vpos], DISPLAY_CHAR_WIDTH, 0 ) ;
#else
			lcd_putsnAtt0( 0, 4*FH, Filenames[fc->vpos], DISPLAY_CHAR_WIDTH, 0 ) ;
#endif
			switch ( event )
			{
    		case EVT_KEY_LONG(BOOT_KEY_MENU):
				case EVT_KEY_BREAK(BTN_RE):
					state = UPDATE_SELECTED ;
    		break ;

				case EVT_KEY_LONG(BTN_RE):
    		case EVT_KEY_LONG(BOOT_KEY_EXIT):
					state = UPDATE_FILE_LIST ;		// Cancelled
    		break;
			}
    break ;
		
		case UPDATE_SELECTED :
			f_open( &Mdata.FlashFile, Mdata.FlashFilename, FA_READ ) ;
			f_read( &Mdata.FlashFile, (BYTE *)Fdata.FileData, 1024, &Mdata.BlockCount ) ;
			{
				firmwareAddress = 0x00000080 ;
				if ( check_ready() == 0 )
				{
					CoProcReady = 1 ;
				}
//				FirmwareSize = FileSize[fc->vpos] ;
				BytesFlashed = 0 ;
				BlockOffset = 0 ;
				ByteEnd = 1024 ;
				state = UPDATE_ACTION ;
			}
    break ;
	
		case UPDATE_ACTION :
			// Do the flashing
			lcd_puts_Pleft( 3*FH, "Flashing" ) ;
			{
				uint32_t size = FileSize[fc->vpos] ;
				width = BytesFlashed * 64 / size ;
				CoProresult = 0 ;
				if ( CoProcReady )
				{
					if ( BytesFlashed < ByteEnd )
					{
						uint32_t number ;

						number = ByteEnd - BytesFlashed ;
						if ( number > 128 )
						{
							number = 128 ;						
						}
						i = write_CoProc( (uint32_t)firmwareAddress, &Fdata.FileData[BlockOffset], number ) ;
						BlockOffset += 128 ;		// 8-bit bytes
						firmwareAddress += 128 ;
						BytesFlashed += 128 ;
					}
					else
					{
						if ( ByteEnd >= size )
						{
							state = UPDATE_COMPLETE ;
						}
						else
						{
							f_read( &Mdata.FlashFile, (BYTE *)Fdata.FileData, 1024, &Mdata.BlockCount ) ;
							ByteEnd += Mdata.BlockCount ;
							BlockOffset = 0 ;
						}
					}
				}
				else
				{
					CoProresult = 1 ;
					state = UPDATE_COMPLETE ;
				}
			}
			lcd_hline( 0, 5*FH-1, 65 ) ;
			lcd_hline( 0, 6*FH, 65 ) ;
			lcd_vline( 64, 5*FH, 8 ) ;
			for ( i = 0 ; i <= width ; i += 1 )
			{
				lcd_vline( i, 5*FH, 8 ) ;
			}
    break ;
			
		case UPDATE_COMPLETE :
			lcd_puts_Pleft( 3*FH, "Flashing Complete" ) ;
			if ( CoProresult )
			{
				lcd_puts_Pleft( 5*FH, "FAILED" ) ;
			}

			if ( ( event == EVT_KEY_FIRST(BOOT_KEY_EXIT) ) || ( event == EVT_KEY_LONG(BTN_RE) ) )
			{
				state = UPDATE_FILE_LIST ;
    		killEvents(event) ;
			}
    break ;
	}
}


int main()
{
	uint32_t displayTimer = 0 ;
	uint32_t state = ST_MENU ;
	
	init_soft_power() ;

#ifdef PCBSKY
  PMC->PMC_PCER0 = (1<<ID_PIOC)|(1<<ID_PIOB)|(1<<ID_PIOA) ;				// Enable clocks to PIOB and PIOA and PIOC
	MATRIX->CCFG_SYSIO |= 0x000000F0L ;		// Disable syspins, enable B4,5,6,7
#endif

#ifdef PCBSKY
	init_SDcard() ;
	start_timer0() ;
#endif

	lcd_init() ;

#ifdef PCBSKY
extern uint8_t OptrexDisplay ;
	OptrexDisplay = 1 ;
#endif
	lcd_clear() ;
	refreshDisplay() ;
#ifdef PCBSKY
	OptrexDisplay = 0 ;
	lcd_puts_Pleft( 0, "Update Co-Proc." ) ;
	refreshDisplay() ;
#endif

	__enable_irq() ;

	init10msTimer() ;
	start_2Mhz_timer() ;

#ifdef PCBSKY
	init_rotary_encoder() ;
//	PIOB->PUER = 0x40 ;
#endif
	

//#ifdef PCBSKY
//	if ( PIOB->PIO_PUSR & 0x40 )
//	{
//		PIOB->PIO_PUER = 0x40 ;
//	}
//#endif

	g_menuStack[0] = menuUpCoPro ;
//	g_menuStack[1] = menuUp1 ;	// this is so the first instance of [MENU LONG] doesn't freak out!
	EnterMenu = EVT_ENTRY ;

#ifdef PCBSKY
	init_mtwi() ;
#endif

	uint8_t event = getEvent() ;
	killEvents(event) ;

	for(;;)
	{
		wdt_reset() ;

//		maintenanceBackground() ;
#if defined(REV9E) || defined(PCBX7) || defined(PCBSKY) || defined(PCBX12D)
		checkRotaryEncoder() ;
#endif

		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog

			Tenms = 0 ;
		  event = getEvent() ;

	    lcd_clear() ;
 #ifndef PCBX9D
  #ifndef PCB9XT
			{
				int32_t x ;
//				if ( g_eeGeneral.rotaryDivisor == 1)
//				{
					x = Rotary_count >> 2 ;
//				}
//				else if ( g_eeGeneral.rotaryDivisor == 2)
//				{
//					x = Rotary_count >> 1 ;
//				}
//				else
//				{
//					x = Rotary_count ;
//				}
				Rotary_diff = x - LastRotaryValue ;
				LastRotaryValue = x ;
			}
  #endif
 #endif
#ifndef PCBX9D
 #ifndef PCB9XT
			if ( event == 0 )
			{
	extern int32_t Rotary_diff ;
				if ( Rotary_diff > 0 )
				{
					event = EVT_KEY_FIRST(BOOT_KEY_DOWN) ;
				}
				else if ( Rotary_diff < 0 )
				{
					event = EVT_KEY_FIRST(BOOT_KEY_UP) ;
				}
				Rotary_diff = 0 ;
			}
 #endif
#endif

			if ( PowerUpDelay < 200 )	// 2000 mS
			{
				PowerUpDelay += 1 ;
			}
			if ( PowerUpDelay >= 20 )	// 200 mS
			{
#ifdef PCBSKY
				sd_poll_10mS() ;
#endif			
			}

			if ( PowerUpDelay >= 70 )	// 700 mS
			{
				if ( EnterMenu )
				{
					event = EnterMenu ;
					EnterMenu = 0 ;
				}
				g_menuStack[g_menuStackPtr](event) ;
				// Only update display every 40mS, improves SPort update throughput
				if ( ++displayTimer >= 4 )
				{
					displayTimer = 0 ;
    			refreshDisplay() ;
				}
			}
			wdt_reset() ;

			if ( Tenms )
			{
				Tenms = 0 ;
			}
			
			if ( ( g_menuStack[0] == menuUpCoPro ) && ( g_menuStackPtr == 0 ) )
			{
	//			uint8_t event = getEvent() ;
				if ( ( event == EVT_KEY_LONG(BOOT_KEY_EXIT) ) || ( event == EVT_KEY_LONG(BTN_RE) ) )
				{
					state = ST_REBOOT ;
				}
			}
		}
		
		if ( ( state < ST_FLASH_CHECK ) || (state == ST_FLASH_DONE) )
		{
			if ( check_soft_power() == POWER_OFF )
			{
				soft_power_off() ;
				for(;;)
				{
					// Wait for power to go off
				}
			}
		}
		if ( state == ST_REBOOT )
		{
	 		wdt_reset() ;
			if ( (~read_keys() & 0x7E) == 0 )
			{
		  	NVIC_SystemReset() ;
			}
		}

	}
//	stop_timer0() ;

  return 0;
}


#ifdef PCBSKY

void init_mtwi()
{
	register Pio *pioptr ;
	register uint32_t timing ;
  
	PMC->PMC_PCER0 |= 0x00080000L ;		// Enable peripheral clock to TWI0
	
	TWI0->TWI_CR = TWI_CR_SWRST ;				// Reset in case we are restarting

	/* Configure PIO */
	pioptr = PIOA ;
  pioptr->PIO_ABCDSR[0] &= ~0x00000018 ;	// Peripheral A
  pioptr->PIO_ABCDSR[1] &= ~0x00000018 ;	// Peripheral A
  pioptr->PIO_PDR = 0x00000018 ;					// Assign to peripheral
	
	timing = 64000000 * 5 / 1500000 ;		// 5uS high and low 100kHz, trying 200 kHz
	timing += 15 - 4 ;
	timing /= 16 ;
	timing |= timing << 8 ;

	TWI0->TWI_CWGR = 0x00040000 | timing ;			// TWI clock set
	TWI0->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS ;		// Master mode enable
#ifdef REVX
	TWI0->TWI_MMR = 0x006F0100 ;		// Device 6F and master is writing, 1 byte addr
#else	
	TWI0->TWI_MMR = 0x00350000 ;		// Device 35 and master is writing
#endif
}

uint32_t read_status_CoProc( uint32_t bufaddr, uint32_t number )
{
	uint32_t i ;
	
	TWI0->TWI_MMR = 0x00351000 ;		// Device 35 and master is reading,
	TWI0->TWI_RPR = bufaddr ;
	TWI0->TWI_RCR = number-1 ;
	if ( TWI0->TWI_SR & TWI_SR_RXRDY )
	{
		(void) TWI0->TWI_RHR ;
	}
	TWI0->TWI_PTCR = TWI_PTCR_RXTEN ;	// Start transfers
	TWI0->TWI_CR = TWI_CR_START ;		// Start Rx
	
	// Now wait for completion
	
	for ( i = 0 ; i < 1000000 ; i += 1 )
	{
		if ( TWI0->TWI_SR & TWI_SR_RXBUFF )
		{
			break ;
		}
	}
	// Finished reading
	TWI0->TWI_PTCR = TWI_PTCR_RXTDIS ;	// Stop transfers
	TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Rx
	TWI0->TWI_RCR = 1 ;						// Last byte

	if ( i >= 1000000 )
	{
		return 0 ;
	}
	// Now wait for completion
	
	for ( i = 0 ; i < 100000 ; i += 1 )
	{
		if ( TWI0->TWI_SR & TWI_SR_TXCOMP )
		{
			break ;
		}	
	}
	if ( i >= 100000 )
	{
		return 0 ;
	}
	return 1 ;
}

#define TWI_CMD_REBOOT							0x55	// TWI Command to restart back in the bootloader

uint32_t coProcBoot()
{
	uint32_t i ;
	
	TWI0->TWI_MMR = 0x00350000 ;		// Device 35 and master is writing
	TWI0->TWI_THR = TWI_CMD_REBOOT ;	// Send reboot command
	TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx

	for ( i = 0 ; i < 100000 ; i += 1 )
	{
		if ( TWI0->TWI_SR & TWI_SR_TXCOMP )
		{
			break ;
		}	
	}
	
	if ( i >= 100000 )
	{
		return 0 ;
	}
	return 1 ;
}

uint32_t write_CoProc( uint32_t coInternalAddr, uint8_t *buf, uint32_t number )
{
	uint32_t i ;
	uint8_t *ptr ;

	Addr_buffer[0] = 0x01 ;		// Command, PROGRAM
	Addr_buffer[1] = coInternalAddr >> 8 ;
	Addr_buffer[2] = coInternalAddr ;
	ptr = buf ;
	// Copy data
	for ( i = 0 ; i < number ; i += 1 )
	{
		Addr_buffer[i+3] = *ptr++ ;		
	}
	// Pad to 128 bytes
	while ( i < 128 )
	{
		Addr_buffer[i+3] = 0xFF ;
		i += 1 ;	
	}
	// Now send TWI data using PDC
	TWI0->TWI_MMR = 0x00350000 ;		// Device 35 and master is writing
	// init PDC
	TWI0->TWI_TPR = (uint32_t)&Addr_buffer[1] ;
	TWI0->TWI_TCR = 130 ;
			
	TWI0->TWI_THR = Addr_buffer[0] ;		// Send data
	// kick off PDC, enable PDC end interrupt
	TWI0->TWI_PTCR = TWI_PTCR_TXTEN ;	// Start transfers
	
	// Now wait for completion
	
	for ( i = 0 ; i < 1000000 ; i += 1 )
	{
		if ( TWI0->TWI_SR & TWI_SR_TXBUFE )
		{
			break ;
		}
	}
	// Finished reading
	TWI0->TWI_PTCR = TWI_PTCR_TXTDIS ;	// Stop transfers
	TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
	if ( i >= 1000000 )
	{
		return 0 ;
	}
	
	for ( i = 0 ; i < 100000 ; i += 1 )
	{
		if ( TWI0->TWI_SR & TWI_SR_TXCOMP )
		{
			break ;
		}	
	}
	
	if ( i >= 100000 )
	{
		return 0 ;
	}

	return 1 ;
}

// returns 0 = OK, not 0 error
uint32_t check_ready()
{
	uint32_t result ;
	result = 0 ;				// OK
	init_mtwi() ;

	// initialise for updating
	if ( read_status_CoProc( (uint32_t)Twi_rx_buf, 22 ) )
	{
		if ( ( Twi_rx_buf[0] & 0x80 ) == 0 )
		{ // Not in bootloader
			if ( coProcBoot() == 0 )		// Make sure we are in the bootloader
			{
   		  result = 1 ;
			}
			else
			{
				uint32_t i ;

				for ( i = 0 ; i < 10000 ; i += 1 )
				{
					asm("nop") ;
				}
				if ( read_status_CoProc( (uint32_t)Twi_rx_buf, 22 ) )
				{
					if ( ( Twi_rx_buf[0] & 0x80 ) != 0x80 )
					{
		   		  result = 2 ;
					}
				}
				else
				{
   		  	result = 3 ;
				}	 
			}
		}
	}
	else
	{
   	result = 4 ;
	}

	read_status_CoProc( (uint32_t)Twi_rx_buf, 22 ) ;

	return result ;
}

#endif



