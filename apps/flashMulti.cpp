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

#ifdef PCBX9D
#include "stm32f2xx.h"
//#include "stm32f2xx_flash.h"
//#include "i2c_ee.h"
#include "hal.h"
#include "timers.h"

//extern "C" {
//#include "usb_dcd_int.h"
//#include "usb_bsp.h"
//#include "usbd_desc.h"
//#include "usbd_msc_core.h"
//#include "usbd_usr.h"
//}

#endif

#ifdef PCB9XT
//#define P9XT_DEBUG	1

#include "stm32f2xx.h"
#include "stm32f2xx_flash.h"
#include "hal.h"
#include "timers.h"
#include "mega64.h"

//extern "C" {
//#include "usb_dcd_int.h"
//#include "usb_bsp.h"
//#include "usbd_desc.h"
//#include "usbd_msc_core.h"
//#include "usbd_usr.h"
//}

#endif

#ifdef PCBX12D
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "timers.h"

#endif

#include "radio.h"
#include "lcd.h"
#include "ff.h"
#include "diskio.h"
#include "drivers.h"
#include "logicio.h"

#include "menucontrol.h"
#include "..\stamp-app.h"

__attribute__ ((section(".version"), used))
// Temp edit to force a push
const uint8_t Version[] =
{
#ifdef PCBSKY
	'A', 'P', 'P', 'S', 'K', 'Y'
#endif
#ifdef PCBX9D
 #ifdef REVPLUS
  #ifdef REV9E
	'A', 'P', 'P', 'X', '9', 'E'
  #else
	'A', 'P', 'P', 'X', '9', 'P'
	#endif
 #else
  #ifdef PCBX7
   #ifdef PCBT12
		'A', 'P', 'P', 'T', '1', '2'
	 #else
		'A', 'P', 'P', 'Q', 'X', '7'
	 #endif
  #else
   #ifdef PCBXLITE
	'A', 'P', 'P', 'X', 'L', 'T'
   #else
	'A', 'P', 'P', 'X', '9', 'D'
   #endif
  #endif
 #endif
#endif
#ifdef PCB9XT
	'A', 'P', 'P', '9', 'X', 'T'
#endif
} ;

__attribute__ ((section(".text"), used))

#if defined(REV9E) || defined(PCBX7) || defined(PCBSKY)
#ifndef PCBT12
extern void init_rotary_encoder() ;
extern void checkRotaryEncoder() ;
#endif
#endif

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D)
extern void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity ) ;
extern void disable_software_com1() ;
#endif

#ifdef PCBSKY
extern void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity ) ;
extern void init_software_com1(uint32_t baudrate, uint32_t invert, uint32_t parity) ;
extern void init_software_com2(uint32_t baudrate, uint32_t invert, uint32_t parity) ;
void disable_software_com1() ;
void disable_software_com2() ;
#endif

#ifdef PCB9XT
extern void com2_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity ) ;
volatile int32_t Rotary_count ;
int32_t LastRotaryValue ;
int32_t Rotary_diff ;
#endif

#ifdef PCBSKY

#define RADIO_SKY	0
#define RADIO_PRO	1
#define RADIO_AR9X	2

void init_mtwi( void ) ;
uint32_t clearMfp( void ) ;
uint8_t RadioType ;
uint16_t I2Cack ;
#endif


extern struct t_fifo128 Com1_fifo ;
extern struct t_fifo128 Com2_fifo ;

void menuUpMulti(uint8_t event) ;
uint32_t multiUpdate() ;
void stopMultiMode() ;

volatile uint8_t  g_blinkTmr10ms;

#ifdef PCBSKY
uint32_t LastResult ;
#endif

#ifdef PCBSKY
//extern void usbMassStorage( void ) ;
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

#ifdef PCBSKY
uint32_t ChipId ;
#endif

uint32_t BytesFlashed ;
uint32_t ByteEnd ;
uint32_t BlockOffset ;

uint16_t MultiPageSize ;
uint8_t MultiResult ;
uint8_t MultiType ;
uint8_t MultiPort ;
uint8_t MultiModule ;
uint8_t MultiInvert ;
uint8_t MultiStm ;

uint8_t MultiState ;
#define MULTI_IDLE				0
#define MULTI_START				1
#define MULTI_WAIT1				2
#define MULTI_WAIT2				3
#define MULTI_BEGIN				4
#define MULTI_FLASHING		5
#define MULTI_DONE				6

uint32_t HexFileIndex ;
uint32_t HexFileRead ;

//#if ( defined(PCBSKY) || defined(PCB9XT) )
//extern int32_t EblockAddress ;
//#endif
//extern uint32_t EepromBlocked ;

//extern void init_spi( void ) ;
//extern void writeBlock( void ) ;

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

#ifdef PCBSKY
#define CLEAR_TX_BIT_EXT() PIOA->PIO_CODR = PIO_PA17
#define SET_TX_BIT_EXT() PIOA->PIO_SODR = PIO_PA17
#define CLEAR_TX_BIT_INT() PIOC->PIO_CODR = PIO_PC15
#define SET_TX_BIT_INT() PIOC->PIO_SODR = PIO_PC15
#else
#ifdef PCB9XT
#define CLEAR_TX_BIT_EXT() GPIOA->BSRRH = 0x0080
#define SET_TX_BIT_EXT() GPIOA->BSRRL = 0x0080
#define CLEAR_TX_BIT_INT() GPIOA->BSRRH = 0x0400
#define SET_TX_BIT_INT() GPIOA->BSRRL = 0x0400
#else
#ifdef PCBXLITE
#define CLEAR_TX_BIT() GPIOC->BSRRL = 0x0040
#define SET_TX_BIT() GPIOC->BSRRH = 0x0040
#else
#define CLEAR_TX_BIT() GPIOA->BSRRL = 0x0080
#define SET_TX_BIT() GPIOA->BSRRH = 0x0080
#endif
#endif
#endif

#if defined(PCBX9D) || defined(PCB9XT)
#if !defined(PCBTARANIS)
#define INTERNAL_RF_ON()      GPIO_SetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define INTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#endif
#endif


/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

uint16_t getTmr2MHz()
{
#ifdef PCBSKY
	return TC1->TC_CHANNEL[0].TC_CV ;
#endif
#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D)
	return TIM7->CNT ;
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

#else
// Start TIMER7 at 2000000Hz
void start_2Mhz_timer()
{
	// Now for timer 7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN ;		// Enable clock
	
	TIM7->ARR = 0xFFFF ;
	TIM7->PSC = (Peri1_frequency*Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM7->CR2 = 0 ;
	TIM7->CR2 = 0x20 ;
	TIM7->CR1 = TIM_CR1_CEN ;
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

void initMultiMode()
{
#ifdef PCBSKY
//#ifdef REVX
//	init_mtwi() ;
//	clearMfp() ;
//#endif
	USART0->US_IDR = US_IDR_RXRDY ;
	UART0->UART_IDR = UART_IDR_RXRDY ;
	if ( MultiPort )
	{
		init_software_com2( 57600, MultiInvert ? SERIAL_NORM : SERIAL_INVERT, SERIAL_NO_PARITY ) ;
//		if ( PIOA->PIO_PDSR & PIO_PA9 )
//		{
//			init_software_com2( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ;
//		}
	}
	else
	{
		init_software_com1( 57600, MultiInvert ? SERIAL_NORM : SERIAL_INVERT, SERIAL_NO_PARITY ) ;
//		if ( PIOA->PIO_PDSR & PIO_PA5 )
//		{
//			init_software_com1( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ;
//		}
	}
	if ( MultiModule )
	{
		SET_TX_BIT_INT() ;
		configure_pins( PIO_PC15, PIN_ENABLE | PIN_OUTPUT | PIN_PORTC | PIN_HIGH ) ;
	}
	else
	{
		SET_TX_BIT_EXT() ;
		configure_pins( PIO_PA17, PIN_ENABLE | PIN_OUTPUT | PIN_PORTA | PIN_HIGH ) ;
	}
#endif
#ifdef PCB9XT
	if ( MultiPort )
	{
		com2_Configure( 57600, SERIAL_INVERT, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
	}
	else
	{
		com1_Configure( 57600, SERIAL_INVERT, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
	}
	if ( MultiModule )
	{
		configure_pins( PIN_INTPPM_OUT, PIN_OUTPUT | PIN_PORTA | PIN_HIGH ) ;
		INTERNAL_RF_ON() ;
	}
	else
	{
		configure_pins( PIN_EXTPPM_OUT, PIN_OUTPUT | PIN_PORTA | PIN_HIGH ) ;
		EXTERNAL_RF_ON() ;
	}
#endif
#ifdef PCBX9D
	com1_Configure( 57600, MultiInvert ? SERIAL_NORM : SERIAL_INVERT, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
	EXTERNAL_RF_ON() ;
	configure_pins( PIN_EXTPPM_OUT, PIN_OUTPUT | PORT_EXTPPM | PIN_LOW ) ;
#endif
}

void stopMultiMode()
{
	
#ifdef PCBSKY
	if ( MultiPort )
	{
		disable_software_com2() ;
	}
	else
	{
		disable_software_com1() ;
	}
	if ( MultiModule )
	{
		SET_TX_BIT_INT() ;
		configure_pins( PIO_PC15, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_PULLUP ) ;
	}
	else
	{
		configure_pins( PIO_PA17, PIN_ENABLE | PIN_INPUT | PIN_PORTA | PIN_PULLUP ) ;
	}
//	com1_Configure( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
#endif
#ifdef PCB9XT
	if ( MultiPort )
	{
		com2_Configure( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
	}
	else
	{
		com1_Configure( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
	}
	if ( MultiModule )
	{
		INTERNAL_RF_OFF() ;
		configure_pins( PIN_INTPPM_OUT, PIN_OUTPUT | PIN_PORTA | PIN_HIGH ) ;
	}
	else
	{
		EXTERNAL_RF_OFF() ;
		configure_pins( PIN_EXTPPM_OUT, PIN_OUTPUT | PIN_PORTA | PIN_HIGH ) ;
	}
#endif
#ifdef PCBX9D
	disable_software_com1() ;
//	com1_Configure( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud
	EXTERNAL_RF_OFF() ;
	configure_pins( PIN_EXTPPM_OUT, PIN_OUTPUT | PORT_EXTPPM | PIN_LOW ) ;
#endif
}

uint16_t getMultiFifo()
{
	if ( MultiPort )
	{
		return get_fifo128( &Com2_fifo ) ;
	}
	else
	{
		return get_fifo128( &Com1_fifo ) ;
	}
}

void sendMultiByte( uint8_t byte )
{
	uint16_t time ;
	uint32_t i ;
	
#ifdef PCBSKY
	Pio *pioptr ;
	uint32_t bit ;
	
	if ( MultiModule )
	{
		pioptr = PIOC ;
		bit = PIO_PC15 ;
	}
	else
	{
		pioptr = PIOA ;
		bit = PIO_PA17 ;
	}
#define CLEAR_TX_BIT() pioptr->PIO_CODR = bit
#define SET_TX_BIT() pioptr->PIO_SODR = bit
#endif

#ifdef PCB9XT
	uint32_t bit ;
	
	if ( MultiModule )
	{
		bit = 0x0400 ;
	}
	else
	{
		bit = 0x0080 ;
	}
#define CLEAR_TX_BIT() GPIOA->BSRRH = bit
#define SET_TX_BIT() GPIOA->BSRRL = bit
#endif

	__disable_irq() ;
	time = getTmr2MHz() ;
	CLEAR_TX_BIT() ;
	while ( (uint16_t) (getTmr2MHz() - time) < 34 )
	{
		// wait
	}
	time += 34 ;
	for ( i = 0 ; i < 8 ; i += 1 )
	{
		if ( byte & 1 )
		{
			SET_TX_BIT() ;
		}
		else
		{
			CLEAR_TX_BIT() ;
		}
		byte >>= 1 ;
		while ( (uint16_t) (getTmr2MHz() - time) < 35 )
		{
			// wait
		}
		time += 35 ;
	}
	SET_TX_BIT() ;
	__enable_irq() ;	// No need to wait for the stop bit to complete
	while ( (uint16_t) (getTmr2MHz() - time) < 34 )
	{
		// wait
	}
}


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

//uint32_t isFirmwareStart( uint32_t *block )
//{
//	if ( ( block[0] & 0xFFFC0000 ) != 0x20000000 )
//	{
//		if ( ( block[0] & 0xFFFC0000 ) != 0x10000000 )
//		{
//			return 0 ;
//		}
//	}
//	if ( ( block[1] & 0xFFF00000 ) != 0x08000000 )
//	{
//		return 0 ;
//	}
//	if ( ( block[2] & 0xFFF00000 ) != 0x08000000 )
//	{
//		return 0 ;
//	}
//	return 1 ;	
//}
//#endif

//#ifdef PCBX12D
//uint32_t isFirmwareStart( uint32_t *block )
//{
//	if ( ( block[0] & 0xFFFC0000 ) != 0x20000000 )
//	{
//		if ( ( block[0] & 0xFFFC0000 ) != 0x10000000 )
//		{
//			return 0 ;
//		}
//	}
//	if ( ( block[1] & 0xFFF00000 ) != 0x08000000 )
//	{
//		return 0 ;
//	}
//	if ( ( block[2] & 0xFFF00000 ) != 0x08000000 )
//	{
//		return 0 ;
//	}
//	return 1 ;	
//}
//#endif


//#ifdef PCBSKY
//uint32_t isFirmwareStart( uint32_t *block )
//{
//	if ( ChipId & 0x0080 )
//	{
//		if ( ( block[0] & 0xFFFC3000 ) != 0x20000000 )
//		{
//			return 0 ;
//		}
//	}
//	else
//	{
//		if ( ( block[0] & 0xFFFE3000 ) != 0x20000000 )
//		{
//			return 0 ;
//		}
//	}
//	if ( ( block[1] & 0xFFF80000 ) != 0x00400000 )
//	{
//		return 0 ;
//	}
//	if ( ( block[2] & 0xFFF80000 ) != 0x00400000 )
//	{
//		return 0 ;
//	}
//	return 1 ;	
//}
//#endif


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

#if ( defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) )
void init10msTimer()
{
	// Timer14
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN ;		// Enable clock
	TIM14->ARR = 9999 ;	// 10mS
	TIM14->PSC = (Peri1_frequency*Timer_mult1) / 1000000 - 1 ;		// 1uS from 12MHz
	TIM14->CCER = 0 ;	
	TIM14->CCMR1 = 0 ;
	TIM14->EGR = 0 ;
	TIM14->CR1 = 5 ;
	TIM14->DIER |= 1 ;
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn) ;
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
	TIM14->SR &= ~TIM_SR_UIF ;
	interrupt10ms() ;
}

void init_hw_timer()
{
	// Timer13
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN ;		// Enable clock
	TIM13->ARR = 65535 ;
	TIM13->PSC = (Peri1_frequency*Timer_mult1) / 10000000 - 1 ;		// 0.1uS from 12MHz
	TIM13->CCER = 0 ;	
	TIM13->CCMR1 = 0 ;
	TIM13->EGR = 0 ;
	TIM13->CR1 = 1 ;
}


// delay in units of 0.1 uS up to 6.5535 mS
void hw_delay( uint16_t time )
{
	TIM13->CNT = 0 ;
	TIM13->EGR = 1 ;		// Re-start counter
	while ( TIM13->CNT < time )
	{
		// wait
	}
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

//uint16_t WriteCounter ;
//extern uint32_t Breason ;

//#ifdef PCB9XT
//void console9xtInit()
//{
//	// Serial configure  
//	RCC->APB1ENR |= RCC_APB1ENR_UART4EN ;		// Enable clock
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
//	configure_pins( 0x00000001, PIN_PERIPHERAL | PIN_PUSHPULL | PIN_OS25 | PIN_PORTA | PIN_PER_8 ) ;
//	configure_pins( 0x00000002, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_8 | PIN_PULLUP ) ;
//	GPIOA->MODER = (GPIOA->MODER & 0xFFFFFFF0 ) | 0x0000000A ;	// Alternate func.
//	GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFFFFFF00 ) | 0x00000088 ;	// Alternate func.
//	UART4->BRR = Peri1_frequency / 115200 ;		// 97.625 divider => 19200 baud
////	UART4->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE ;
//	UART4->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE ;
//	UART4->CR2 = 0 ;
//	UART4->CR3 = 0 ;
////	NVIC_SetPriority( UART4_IRQn, 4 ) ; // Lower priority interrupt
////  NVIC_EnableIRQ(UART4_IRQn) ;
//}

////extern "C" void UART4_IRQHandler()
////{
////	if ( ( g_model.com2Function == COM2_FUNC_SBUSTRAIN ) || ( g_model.com2Function == COM2_FUNC_SBUS57600 ) )
////	{
////		put_fifo64( &Sbus_fifo, UART4->DR ) ;	
////	}
////	else
////	{
////		put_fifo64( &Console_fifo, UART4->DR ) ;	
////	}	 
////}

//void txmit( uint8_t c )
//{
//	/* Wait for the transmitter to be ready */
//  while ( (UART4->SR & USART_SR_TXE) == 0 ) ;

//  /* Send character */
//	UART4->DR = c ;
//}

//#endif

#ifdef PCB9XT
static void initWatchdog()
{
	IWDG->KR = 0x5555 ;		// Unlock registers
	IWDG->PR = 3 ;				// Divide by 32 => 1kHz clock
	IWDG->KR = 0x5555 ;		// Unlock registers
	IWDG->RLR = 1000 ;			// 1.0 seconds nominal
	IWDG->KR = 0xAAAA ;		// reload
	IWDG->KR = 0xCCCC ;		// start
}
#endif

#ifdef PCBX7
void ledOff()
{
  GPIO_ResetBits(LED_RED_GPIO, LED_RED_GPIO_PIN);
  GPIO_ResetBits(LED_BLUE_GPIO, LED_BLUE_GPIO_PIN);
  GPIO_ResetBits(LED_GREEN_GPIO, LED_GREEN_GPIO_PIN);
}

void ledRed()
{
  ledOff();
  GPIO_SetBits(LED_RED_GPIO, LED_RED_GPIO_PIN);
}

void ledGreen()
{
  ledOff();
  GPIO_SetBits(LED_GREEN_GPIO, LED_GREEN_GPIO_PIN);
}

void ledBlue()
{
  ledOff();
  GPIO_SetBits(LED_BLUE_GPIO, LED_BLUE_GPIO_PIN);
}
#endif // PCBX7

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

//#if !defined(PCBTARANIS) || defined(REV9E)
//	if ( event == 0 )
//	{
//extern int32_t Rotary_diff ;
//		if ( Rotary_diff > 0 )
//		{
//			event = EVT_KEY_FIRST(BOOT_KEY_DOWN) ;
//		}
//		else if ( Rotary_diff < 0 )
//		{
//			event = EVT_KEY_FIRST(BOOT_KEY_UP) ;
//		}
//		Rotary_diff = 0 ;
//	}
//#endif

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


void menuUp1(uint8_t event)
{
	static MState2 mstate2 ;
	
	FRESULT fr ;
	struct fileControl *fc = &FileControl ;
  static uint8_t mounted = 0 ;
	static uint32_t state ;
//	static uint32_t firmwareAddress ;
	uint32_t i ;
	uint32_t width ;
	
	wdt_reset() ;
//	if ( WatchdogTimeout < 50 )
//	{
// 		WatchdogTimeout = 50 ;
//	}
	
	TITLE( "Multi Update" ) ;
	mstate2.check_columns(event, 1 ) ;
	
	switch(event)
	{
    case EVT_ENTRY:
//  		WatchdogTimeout = 200 ;
			state = UPDATE_NO_FILES ;
			if ( mounted == 0 )
			{
#if defined(PCBTARANIS)
  			fr = f_mount(0, &g_FATFS_Obj) ;
#else				
  			fr = f_mount(0, &g_FATFS) ;
#endif
//#if defined(PCBX9D) || defined(PCB9XT)
//				unlockFlash() ;
//#endif
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
						if ( MultiType )
						{
							fc->ext[0] = 'H' ;
							fc->ext[1] = 'E' ;
							fc->ext[2] = 'X' ;
						}
						else
						{
							fc->ext[0] = 'B' ;
							fc->ext[1] = 'I' ;
							fc->ext[2] = 'N' ;
						}
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
      	chainMenu(menuUpMulti) ;
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
			lcd_puts_Pleft( 2*FH, "Flash Multi from" ) ;
			
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
				FirmwareSize = FileSize[fc->vpos] ;
//				firmwareAddress = 0x00000000 ;
				MultiState = MULTI_START ;
				MultiResult = 0 ;
				BytesFlashed = 0 ;
				BlockOffset = 0 ;
				ByteEnd = 1024 ;
				state = UPDATE_ACTION ;
			}
    break ;
	
		case UPDATE_ACTION :
			// Do the flashing
			lcd_puts_Pleft( 3*FH, "Flashing" ) ;
			if ( MultiState == MULTI_WAIT1	)
			{
				lcd_puts_Pleft( 4*FH, "Power on Delay" ) ;
			}
			width = multiUpdate() ;
			if ( width > FirmwareSize )
			{
				state = UPDATE_COMPLETE ;
				stopMultiMode() ;
			}
			width *= 64 ;
			width /= FirmwareSize ;
			lcd_outhex4( 0, 7*FH, (XmegaSignature[0] << 8) | XmegaSignature[1] ) ;
			lcd_outhex4( 25, 7*FH, (XmegaSignature[2] << 8) | XmegaSignature[3] ) ;
			
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
			if ( MultiResult )
			{
				lcd_puts_Pleft( 5*FH, "FAILED" ) ;
			}
			lcd_outhex4( 0, 7*FH, (XmegaSignature[0] << 8) | XmegaSignature[1] ) ;
			lcd_outhex4( 25, 7*FH, (XmegaSignature[2] << 8) | XmegaSignature[3] ) ;

			if ( ( event == EVT_KEY_FIRST(BOOT_KEY_EXIT) ) || ( event == EVT_KEY_LONG(BTN_RE) ) )
			{
#if defined(PCBX9D) || defined(PCB9XT)
				EXTERNAL_RF_OFF();
				INTERNAL_RF_OFF();
#endif
				state = UPDATE_FILE_LIST ;
    		killEvents(event) ;
			}
    break ;
	}
}

void displayDate( uint8_t y )
{
	uint8_t x ;
#ifdef PCBX9D
#if defined(PCBX7) || defined(PCBXLITE)
	x = FW*12+4 ;
 #else
	x = FW*20+4 ;
 #endif
#else
	x = FW*12+4 ;
#endif
	lcd_putc( x+11, y, '.' ) ;
	lcd_putc( x+32, y, '.' ) ;
	lcd_putsn_P( x, y, DATE_STR, 2 ) ;
	lcd_putsn_P( x+15, y, &DATE_STR[3], 3 ) ;
	lcd_putsn_P( x+36, y, &DATE_STR[7], 2 ) ;
}


void menuUpMulti(uint8_t event)
{
	TITLE( "Multi Options" ) ;
	displayDate( 7*FH ) ;

	static MState2 mstate2 ;
#if defined(PCBSKY) || defined(PCB9XT)
	mstate2.check_columns(event, 4 ) ;
#else
	mstate2.check_columns(event, 2 ) ;
#endif
	uint32_t sub = mstate2.m_posVert ;
	uint32_t subN = 0 ;

#ifdef PCBSKY
	if ( RadioType == RADIO_PRO )
	{
		lcd_puts_Pleft( 0, "\021Pro" ) ;
	}
#endif

	uint32_t y = FH ;
  lcd_putsAtt(0, y, "Update", (sub==subN) ? INVERS : 0 ) ;
  if(sub==subN)
	{
		if ( ( event == EVT_KEY_BREAK(BOOT_KEY_MENU) ) || ( event == EVT_KEY_BREAK(BTN_RE) ) )
		{
//			popMenu() ;
			chainMenu(menuUp1) ;
		}
	}
	y += FH ;
	subN += 1 ;
	lcd_puts_Pleft( y, "File Type" ) ;
	MultiType = checkIndexed( y, "\110\001\003BINHEX", MultiType, (sub==subN), event ) ;
	y += FH ;
	subN += 1 ;
#if defined(PCBSKY) || defined(PCB9XT)
	lcd_puts_Pleft( y, "Module" ) ;
	MultiModule = checkIndexed( y, "\110\001\010ExternalInternal", MultiModule, (sub==subN), event ) ;
	y += FH ;
	subN += 1 ;
	lcd_puts_Pleft( y, "COM Port" ) ;
	MultiPort = checkIndexed( y, "\150\001\00112", MultiPort, (sub==subN), event ) ;
	y += FH ;
	subN += 1 ;
#endif
#if defined(PCBSKY) || defined(PCB9XT) || defined(PCBX9D)
	lcd_puts_Pleft( y, "Invert Com Port" ) ;
	MultiInvert = checkIndexed( y, "\150\001\003 NOYES", MultiInvert, (sub==subN), event ) ;
#endif
}

//uint32_t menu()
//{
//	static uint32_t position = 3*FH ;
	
//	lcd_puts_Pleft( 3*FH, "  Option1" );
//	lcd_puts_Pleft( 4*FH, "  Option2" );
//	uint8_t event = getEvent() ;
  
//	switch(event)
//	{
//		case EVT_KEY_FIRST(BOOT_KEY_MENU):
//		case EVT_KEY_BREAK(BTN_RE):
//			return (position == 3*FH) ? MENU_OPT1 : MENU_OPT2 ;
//		break ;
		
//    case EVT_KEY_LONG(BOOT_KEY_EXIT):
//		case EVT_KEY_LONG(BTN_RE) :
//			return MENU_EXIT ;
//		break ;
//    case EVT_KEY_FIRST(BOOT_KEY_DOWN):
//			if ( position < 4*FH )
//			{
//				position += FH ;				
//			}
//		break ;
    
//		case EVT_KEY_FIRST(BOOT_KEY_UP):
//			if ( position > 3*FH )
//			{
//				position -= FH ;				
//			}
//		break ;
				 
//	}
//	lcd_char_inverse( 2*FW, position, 14*FW, 0 ) ;
//	return MENU_NOTHING ;
//}


int main()
{
#if defined(PCBX7) || defined(PCBXLITE)
	uint32_t i ;
#endif
#ifdef PCB9XT
	uint32_t i ;
#endif
	uint32_t displayTimer = 0 ;
//  uint8_t index = 0 ;

//#ifdef PCB9XT
//	console9xtInit() ;
//#endif

//#if ( defined(PCBX9D) || defined(PCB9XT) )
//  uint8_t TenCount = 2 ;
//#endif			
//  uint8_t maxhsize = DISPLAY_CHAR_WIDTH ;
//	FRESULT fr ;
	uint32_t state = ST_MENU ;
//	uint32_t nameCount = 0 ;
//	uint32_t vpos = 0 ;
//	uint32_t hpos = 0 ;
//#if ( defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) )
//	uint32_t firmwareAddress = 0x08000000 ;
//#endif			
//#ifdef PCBSKY
//	uint32_t firmwareAddress = 0x00400000 ;
//#endif			
//	uint32_t firmwareWritten = 0 ;

#if ( defined(PCBX9D) || defined(PCB9XT) )
	wdt_reset() ;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
#endif

	init_soft_power() ;

#ifdef PCB9XT
	initM64() ;
#endif

#ifdef PCBSKY
  PMC->PMC_PCER0 = (1<<ID_PIOC)|(1<<ID_PIOB)|(1<<ID_PIOA) ;				// Enable clocks to PIOB and PIOA and PIOC
	MATRIX->CCFG_SYSIO |= 0x000000F0L ;		// Disable syspins, enable B4,5,6,7
#endif

#ifdef PCBSKY
	ChipId = CHIPID->CHIPID_CIDR ;

//	uint32_t x = ChipId & 0x00000F00 ;

//	if ( ChipId & 0x0080 )
//	{
//		EFC->EEFC_FMR = (EFC->EEFC_FMR & 0x00000F00) | (6 << 8) ;
//	}

#endif

#ifdef PCBSKY
	init_SDcard() ;
//	PIOC->PIO_PER = PIO_PC25 ;		// Enable bit C25 (USB-detect)
	start_timer0() ;
#endif

#if defined(PCBX7) || defined(PCBXLITE)
	init_hw_timer()	;
	__enable_irq() ;
#endif // PCBX7

	lcd_init() ;

#ifdef PCBSKY
extern uint8_t OptrexDisplay ;
	OptrexDisplay = 1 ;
#endif
	lcd_clear() ;
#ifdef PCBX9D
#if defined(PCBX7) || defined(PCBXLITE)
	lcd_puts_Pleft( 0, "Update Multi" ) ;
 #else
	lcd_puts_Pleft( 0, "\006Update Multi" ) ;
 #endif
#endif
	refreshDisplay() ;
#ifdef PCBSKY
	OptrexDisplay = 0 ;
	refreshDisplay() ;
#endif

#ifdef PCBX9D	 
	init_keys() ;
	setup_switches() ;
//	I2C_EE_Init() ;
 #ifndef PCBX7
  #ifndef PCBXLITE
	init_hw_timer()	;
  #endif // PCBXLITE
 #endif // PCBX7
#endif

#ifdef PCB9XT
//	init_keys() ;
//	setup_switches() ;
	init_hw_timer()	;
#endif

 #ifndef PCBX7
  #ifndef PCBXLITE
	__enable_irq() ;
  #endif // PCBXLITE
 #endif // PCBX7


	init10msTimer() ;
	start_2Mhz_timer() ;

#ifdef PCB9XT
	BlSetColour( 60, 3 ) ;	
#endif

#if ( defined(PCBX9D) || defined(PCB9XT) )
	// SD card detect pin
#ifdef PCB9XT
	configure_pins( GPIO_Pin_CP, PIN_PORTC | PIN_INPUT | PIN_PULLUP ) ;
#else
	configure_pins( GPIO_Pin_CP, PIN_PORTD | PIN_INPUT | PIN_PULLUP ) ;
#endif
//#ifdef P9XT_DEBUG
//	i = 10 ;
//	do
//	{
//		if ( Tenms )
//		{
//	    wdt_reset() ;  // Retrigger hardware watchdog
//			Tenms = 0 ;
//			i -= 1 ;
//		}
//	} while ( i ) ;
//	BlSetColour( 50, 3 ) ;
//#endif
	
	disk_initialize( 0 ) ;
	sdInit() ;
//	unlockFlash() ;

//  usbInit() ;
//  usbStart() ;
//#ifdef P9XT_DEBUG
//	i = 10 ;
//	do
//	{
//		if ( Tenms )
//		{
//	    wdt_reset() ;  // Retrigger hardware watchdog
//			Tenms = 0 ;
//			i -= 1 ;
//		}
//	} while ( i ) ;
//	BlSetColour( 0, 2 ) ;
//#endif
#endif

#ifdef PCB9XT
	i = 40 ;
	do
	{
		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog
			Tenms = 0 ;
			i -= 1 ;
		}
	} while ( i ) ;
	initWatchdog() ;
#endif

#if defined(PCBX7) || defined(PCBXLITE)
	i = 40 ;
	do
	{
		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog
			Tenms = 0 ;
			i -= 1 ;
		}
	} while ( i ) ;
//	lcd_init() ;
#endif

#ifdef REV9E
	init_rotary_encoder() ;
#endif
#ifdef PCBX7
#ifndef PCBT12
	init_rotary_encoder() ;
#endif
#endif // PCBX7
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

	g_menuStack[0] = menuUpMulti ;
	g_menuStack[1] = menuUp1 ;	// this is so the first instance of [MENU LONG] doesn't freak out!
	EnterMenu = EVT_ENTRY ;

#ifdef PCBSKY
	init_mtwi() ;
	clearMfp() ;
	RadioType = ( I2Cack & TWI_SR_NACK ) == 0 ;
	if ( RadioType == RADIO_PRO )
	{
		lcdSetRefVolt( 20 ) ;
	}
#endif

	uint8_t event = getEvent() ;
	killEvents(event) ;

#ifdef PCB9XT
	BlSetColour( 70, 3 ) ;	
#endif

	for(;;)
	{
		wdt_reset() ;

#ifdef PCB9XT
	checkM64() ;
//	checkRotaryEncoder() ;
extern uint8_t M64EncoderPosition ;
	static uint8_t lastPosition = 0 ;
	if ( lastPosition != M64EncoderPosition )
	{
		int8_t diff = M64EncoderPosition - lastPosition ;
		if ( diff < 9 && diff > -9 )
		{
			Rotary_count += diff ;
		}
		lastPosition = M64EncoderPosition ;
	}
#endif	// PCB9XT

//		maintenanceBackground() ;
#if defined(REV9E) || defined(PCBX7) || defined(PCBSKY) || defined(PCBX12D)
#ifndef PCBT12
		checkRotaryEncoder() ;
#endif
#endif

		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog

			Tenms = 0 ;
		  event = getEvent() ;

	    lcd_clear() ;
#ifdef PCBX12D
			waitLcdClearDdone() ;
#endif
#ifdef PCBX12D
			{
				int32_t x ;
				x = Rotary_count >> 1 ;
				Rotary_diff = x - LastRotaryValue ;
				LastRotaryValue = x ;
			}
#else
 #ifndef PCBX9D
//  #ifndef PCB9XT
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
//  #endif
 #endif
#endif
#ifndef PCBX9D
// #ifndef PCB9XT
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
// #endif
#endif


			if ( EnterMenu )
			{
				event = EnterMenu ;
				EnterMenu = 0 ;
			}
			g_menuStack[g_menuStackPtr](event) ;
			// Only update display every 40mS, improves SPort update throughput
#ifdef PCB9XT
			if ( ++displayTimer >= 6 )
#else
			if ( ++displayTimer >= 4 )
#endif
			{
				displayTimer = 0 ;
    		refreshDisplay() ;
#if defined(PCBX12D)
				lcd_clearBackground() ;	// Start clearing other frame
#endif
			}

			wdt_reset() ;

			if ( Tenms )
			{
				Tenms = 0 ;
			}


			if ( PowerUpDelay < 200 )	// 2000 mS
			{
				PowerUpDelay += 1 ;
			}
			if ( PowerUpDelay >= 20 )	// 200 mS
			{
#ifdef PCBSKY
				sd_poll_10mS() ;
#endif			
#if ( defined(PCBX9D) || defined(PCB9XT) )
				sdPoll10ms() ;
#endif			
			}
			if ( ( g_menuStack[0] == menuUpMulti ) && ( g_menuStackPtr == 0 ) )
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
#ifdef PCBX7
extern void lcdOff() ;				
				lcdOff() ;
#endif // PCBX7
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

uint32_t eat( uint8_t byte )
{
	uint16_t time ;
	uint16_t rxchar ;

	time = getTmr2MHz() ;
	while ( (uint16_t) (getTmr2MHz() - time) < 25000 )	// 12.5mS
	{
		if ( ( rxchar = getMultiFifo() ) != 0xFFFF )
		{
			if ( rxchar == byte )
			{
				return 1 ;
			}
		}
	}
	return 0 ;
}

uint32_t hexFileNextByte()
{
	if ( HexFileIndex >= Mdata.XblockCount )
	{
		if ( Mdata.XblockCount < 1024 )
		{
			return 0 ;
		}
		f_read( &Mdata.FlashFile, (BYTE *)Fdata.ExtraFileData, 1024, &Mdata.XblockCount ) ;
		HexFileRead += Mdata.XblockCount ;
		HexFileIndex = 0 ;

	}
	return Fdata.ExtraFileData[HexFileIndex++] ;
}

uint32_t fromHex( uint32_t data )
{
	if ( data > 0x60 )
	{
		data -= 0x20 ;
	}
	data -= '0' ;
	if ( data > 9 )
	{
		data -= 7 ;
	}
	return data ;
}

uint8_t recordSize ;
uint8_t inRecord ;
uint8_t recordOffset ;
uint16_t recordAddress ;
uint8_t recordData[32] ;

#define RECORD_START			0
#define RECORD_LENGTH1	  1
#define RECORD_LENGTH2	  2
#define RECORD_ADD1			  3
#define RECORD_ADD2	  		4
#define RECORD_ADD3	  		5
#define RECORD_ADD4	  		6
#define RECORD_DATA1	  	7
#define RECORD_DATA2	  	8
#define RECORD_DONE		  	9

void hexFileReadRecord()
{
	uint32_t data ;
	uint32_t state = RECORD_START ;
	uint32_t length = 0 ;
	uint32_t byte = 0 ;
	recordOffset = 0 ;
	recordSize = 0 ;

	while ( state != RECORD_DONE )
	{
		data = hexFileNextByte() ;
		if ( data == 0 )
		{
			// End of File
			return ;
		}
		else
		{
			switch ( state )
			{
				case RECORD_START :
					if ( data == ':' )
					{
						state = RECORD_LENGTH1 ;
					}
				break ;
			
				case RECORD_LENGTH1 :
					length = fromHex( data ) ;
					state = RECORD_LENGTH2 ;
				break ;
				case RECORD_LENGTH2 :
					length = (length << 4) + fromHex( data ) ;
					if ( length == 0 )
					{
						return ;
					}
//					state = RECORD_ADD1 ;
//				break ;
//				case RECORD_ADD1 :
////					length = (length << 4) + fromHex( data ) ;
//					state = RECORD_ADD2 ;
//				break ;
//				case RECORD_ADD2 :
////					length = (length << 4) + fromHex( data ) ;
//					state = RECORD_ADD3 ;
//				break ;
//				case RECORD_ADD3 :
////					length = (length << 4) + fromHex( data ) ;
//					state = RECORD_ADD4 ;
//				break ;
//				case RECORD_ADD4 :
////					length = (length << 4) + fromHex( data ) ;
					hexFileNextByte() ;	// skip four characters
					hexFileNextByte() ;	// address
					hexFileNextByte() ;	// address
					hexFileNextByte() ;	// address
					hexFileNextByte() ;	// skip two characters
					hexFileNextByte() ;	// (record type)
					state = RECORD_DATA1 ;
				break ;
				case RECORD_DATA1 :
					byte = fromHex( data ) ;
					state = RECORD_DATA2 ;
				break ;
				case RECORD_DATA2 :
					byte = (byte << 4) + fromHex( data ) ;
					recordData[recordSize++] = byte ;
					length -= 1 ;
					if ( length == 0 )
					{
						return ;
					}
					if ( recordSize >= 32 )
					{
						return ;
					}
					state = RECORD_DATA1 ;
				break ;
			}
		}
	}
}

void hexFileStart()
{
	
	recordSize = 0 ;
	inRecord = 0 ;
	HexFileIndex = 0 ;
	memmove(Fdata.ExtraFileData, Fdata.FileData, 1024 ) ;	// Hex data to here
	Mdata.XblockCount = 1024 ;
	HexFileRead = 1024 ;
}

// Read file data to ExtraFileData, put binary into FileData
void hexFileRead1024( uint32_t address, UINT *blockCount )
{
	
	uint32_t i ;
	i = 0 ;
	while ( i < 1024 )
	{
		if ( inRecord )
		{
			Fdata.FileData[i++] = recordData[recordOffset++] ;
			if ( recordOffset >= recordSize )
			{
				inRecord = 0 ;
			}
		}
		else
		{
			hexFileReadRecord() ;
			if ( recordSize == 0 )
			{
				*blockCount = i ;
				return ;
			}
			inRecord = 1 ;
		}
	}
	*blockCount = i ;
	return ;
}


uint32_t multiUpdate()
{
	uint32_t i ;
	uint16_t time ;

	switch ( MultiState )
	{
		case MULTI_IDLE :
		break ;

		case MULTI_START :
			initMultiMode() ;
			MultiStm = 0 ;
			BytesFlashed = 0 ;
		  AllSubState = 0 ;
			if ( MultiType )	// Hex file
			{
				hexFileStart() ;
				hexFileRead1024( 0, &Mdata.BlockCount ) ;
			}
			MultiState = MULTI_WAIT1 ;
		break ;

		case MULTI_WAIT1 :
#if defined(PCBX9D) || defined(PCB9XT)
			if ( ++AllSubState > 120 )	// Allow for power on time
#else
			if ( ++AllSubState > 80 )
#endif
			{
				MultiState = MULTI_WAIT2 ;
				AllSubState = 0 ;
			}
		break ;

		case MULTI_WAIT2 :
		{	
			uint16_t rxchar ;
			while ( ( rxchar = getMultiFifo() ) != 0xFFFF )
			{
				// flush receive fifo
			}
			sendMultiByte( 0x30 ) ;
			sendMultiByte( 0x20 ) ;
			if ( eat(0x14) )
			{
				eat( 0x10 ) ;
				MultiState = MULTI_BEGIN ;
				AllSubState = 0 ;
			}
			else
			{
				if ( ++AllSubState == 80 )
				{
					MultiInvert ^= 1 ;
					initMultiMode() ;
				}
				if ( AllSubState > 160 )
				{
					MultiResult = 1 ;
					MultiState = MULTI_DONE ;
				}
			}
		}
		break ;

		case MULTI_BEGIN :
		{
			uint16_t rxchar ;
			while ( ( rxchar = getMultiFifo() ) != 0xFFFF )
			{
				// flush receive fifo
			}
			XmegaSignature[0] = 0 ;
			sendMultiByte( 0x75 ) ;
			sendMultiByte( 0x20 ) ;
			eat(0x14) ;
			time = getTmr2MHz() ;
			i = 0 ;
			while ( (uint16_t) (getTmr2MHz() - time) < 10000 )	// 5mS
			{
				if ( ( rxchar = getMultiFifo() ) != 0xFFFF )
				{
					XmegaSignature[i++] = rxchar ;
					if ( i > 3 )
					{
						break ;
					}
				}
			}
			if ( XmegaSignature[0] != 0x1E )
			{
				if ( ++AllSubState > 10 )
				{
					MultiResult = 1 ;
					MultiState = MULTI_DONE ;
				}
			}
			else
			{
				MultiState = MULTI_FLASHING ;
				MultiPageSize = 128 ;
				if ( XmegaSignature[2] == 0x42 )
				{
					MultiPageSize = 256 ;
				}
				if ( ( XmegaSignature[1] == 0x55 ) && ( XmegaSignature[2] == 0xAA ) )
				{
					MultiPageSize = 256 ;
					MultiStm = 1 ;
				}
			}
			BlockOffset = 0 ;
		}
		break ;

		case MULTI_FLASHING :
		{	
			uint16_t rxchar ;
			uint32_t addOffset ;
			sendMultiByte( 0x55 ) ;
			addOffset = BytesFlashed ;
			if ( MultiPageSize == 128 )
			{
				addOffset >>= 1 ;
			}
			if ( MultiStm )
			{
				addOffset >>= 1 ;
				addOffset += 0x1000 ;	// Word (16-bit) address offset
			}
			sendMultiByte( addOffset ) ;
			sendMultiByte( addOffset >> 8 ) ;
			sendMultiByte( 0x20 ) ;
			eat(0x14) ;
			eat(0x10) ;
			sendMultiByte( 0x64 ) ;
			sendMultiByte( MultiPageSize >> 8 ) ;
			sendMultiByte( MultiPageSize & 0x00FF ) ;
			sendMultiByte( 0 ) ;
			for ( i = 0 ; i < MultiPageSize ; i += 1 )
			{
				sendMultiByte( Fdata.FileData[BlockOffset+i] ) ;
			}
			sendMultiByte( 0x20 ) ;
			eat(0x14) ;

			time = getTmr2MHz() ;
			// For the STM module we need at least 30mS
			while ( (uint16_t) (getTmr2MHz() - time) < 40000 )	// 20mS
			{
				if ( ( rxchar = getMultiFifo() ) != 0xFFFF )
				{
					break ;
				}
			}
			if ( rxchar == 0xFFFF )
			{
				time = getTmr2MHz() ;
				// For the STM module we need at least 30mS
				while ( (uint16_t) (getTmr2MHz() - time) < 40000 )	// 20mS
				{
					if ( ( rxchar = getMultiFifo() ) != 0xFFFF )
					{
						break ;
					}
				}
			}

			if ( rxchar != 0x10 )
			{
				MultiResult = 1 ;
				MultiState = MULTI_DONE ;
				break ;
			}
		
			BytesFlashed += MultiPageSize ;
			BlockOffset += MultiPageSize ;
			if ( BytesFlashed >= ByteEnd )
			{
				if ( MultiType )	// Hex file
				{
					hexFileRead1024( BytesFlashed, &Mdata.BlockCount ) ;
				}
				else
				{
					f_read( &Mdata.FlashFile, (BYTE *)Fdata.FileData, 1024, &Mdata.BlockCount ) ;
				}
				BlockOffset = 0 ;
				ByteEnd += Mdata.BlockCount ;
		 		wdt_reset() ;
			}
			if ( MultiType )
			{
				if ( Mdata.BlockCount == 0 )
				{
					MultiState = MULTI_DONE ;
					BytesFlashed = FirmwareSize ;
				}
			}
			else
			{
				if ( BytesFlashed >= FirmwareSize )
				{
					MultiState = MULTI_DONE ;
					BytesFlashed = FirmwareSize ;
				}
			}
		}
		break ;

		case MULTI_DONE :
			sendMultiByte( 0x51 ) ;	// Exit bootloader
			sendMultiByte( 0x20 ) ;
			eat(0x14) ;
			eat(0x10) ;
			stopMultiMode() ;
			MultiState = MULTI_IDLE ;
			HexFileRead = BytesFlashed = FirmwareSize + 1 ;
			f_close( &Mdata.FlashFile ) ;
		break ;
	
	}
	return MultiType ? HexFileRead : BytesFlashed ;
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

uint32_t clearMfp()
{
	uint32_t i ;
	uint32_t j ;
	
	TWI0->TWI_MMR = 0x006F0100 ;		// Device 6F and master is writing, 1 byte addr
	TWI0->TWI_IADR = 7 ;
	TWI0->TWI_THR = 0 ;					// Value for clear
	TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
	for ( i = 0 ; i < 100000 ; i += 1 )
	{
		if ( ( j = TWI0->TWI_SR ) & TWI_SR_TXCOMP )
		{
			break ;
		}	
	}
	I2Cack = j ;
	if ( i >= 100000 )
	{
		return 0 ;
	}
	return 1 ;
}

#endif



