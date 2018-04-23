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

#define	NO_RECEIVE		0
#define WITH_RECEIVE	1

#ifdef PCBSKY
extern uint32_t txPdcUsart( uint8_t *buffer, uint32_t size, uint32_t receive ) ;
#endif

__attribute__ ((section(".version"), used))
// Temp edit to force a push
const uint8_t Version[] =
{
#ifdef PCBSKY
	'A', 'P', 'P', 'S', 'K', 'Y'
#endif
#ifdef PCBX9D
 #ifdef REVPLUS
	'A', 'P', 'P', 'X', '9', 'P'
 #else
  #ifdef PCBX7
	'A', 'P', 'P', 'Q', 'X', '7'
  #else
	'A', 'P', 'P', 'X', '9', 'D'
  #endif
 #endif
#endif
#ifdef PCB9XT
	'A', 'P', 'P', '9', 'X', 'T'
#endif
} ;

__attribute__ ((section(".text"), used))

#if defined(REV9E) || defined(PCBX7) || defined(PCBSKY)
extern void init_rotary_encoder() ;
extern void checkRotaryEncoder() ;
#endif

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D)
extern void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity ) ;
extern void x9dSPortTxStart( uint8_t *buffer, uint32_t count, uint32_t receive ) ;
#endif

#ifdef PCBSKY
extern void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity ) ;
#endif

#ifdef PCB9XT
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

void menuUpMulti(uint8_t event) ;
uint32_t multiUpdate() ;
void stopMultiMode() ;
extern void check_frsky() ;

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
#define CHANGE_SCANNING		0
#define CHANGE_ENTER_ID		1
#define CHANGE_SET_IDLE		2
#define CHANGE_SET_VALUE	3
#define CHANGE_FINISHED		4

//static uint8_t TxPacket[8] ;
static uint8_t TxPhyPacket[16] ;
//static uint8_t SportTimer ;

uint8_t SendCount ;
uint8_t RxCount ;
uint8_t RxLastCount ;
uint8_t IdIndex ;
uint8_t IdFound ;

uint8_t RxPacket[10] ;
uint8_t PhyId ;
uint8_t NewPhyId ;
uint16_t AppId ;

const uint8_t SportIds[28] = {0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45, 0xC6, 0x67,
				                      0x48, 0xE9, 0x6A, 0xCB, 0xAC, 0x0D, 0x8E, 0x2F,
															0xD0, 0x71, 0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
															0x98, 0x39, 0xBA, 0x1B/*, 0x7C, 0xDD, 0x5E, 0xFF*/ } ;


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

uint32_t Master_frequency ;
volatile uint8_t  Tenms ;
volatile uint16_t BlinkCounter ;


#ifdef PCBSKY
uint32_t ChipId ;
#endif

static uint32_t PowerUpDelay ;

uint8_t AllSubState ;

#if defined(PCBX9D) || defined(PCB9XT)
#if !defined(PCBTARANIS)
#define INTERNAL_RF_ON()      GPIO_SetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#define INTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR)
#ifdef PCBX7
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR);GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR);GPIO_ResetBits(GPIOPB, GPIO_Pin_2)
#else
#define EXTERNAL_RF_ON()      GPIO_SetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#define EXTERNAL_RF_OFF()     GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR)
#endif


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


#define PREC(n) ((n&0x20) ? ((n&0x10) ? 2 : 1) : 0)

void lcd_outdezNAtt( uint8_t x, uint8_t y, int32_t val, uint8_t mode, int8_t len )
{
  uint8_t fw = FWNUM;
  uint8_t prec = PREC(mode);
  int32_t tmp = abs(val);
  uint8_t xn = 0;
  uint8_t ln = 2;
  char c;
  uint8_t xinc ;
	int32_t i ;
	if ( len < 0 )
	{
		len = -len ;		
	}
  xinc = FWNUM ;
  Lcd_lastPos = FW;

  x -= xinc;
  Lcd_lastPos += x ;

  for ( i=1; i<=len; i++)
	{
    c = (tmp % 10) + '0';
		lcd_putcAtt(x, y, c, mode);
    if (prec==i)
		{
			{
        x -= 2;
        if (mode & INVERS)
          lcd_vline(x+1, y, 7);
        else
          lcd_plot(x+1, y+6);
      }
      if (tmp >= 10)
        prec = 0;
    }
    tmp /= 10;
    if (!tmp)
    {
      if (prec)
      {
        if ( prec == 2 )
        {
          if ( i > 1 )
          {
            prec = 0 ;
          }
        }
        else
        {
          prec = 0 ;
        }
      }
      break ;
    }
    x-=fw;
  }
  if (xn)
	{
    lcd_hline(xn, y+2*FH-4, ln);
    lcd_hline(xn, y+2*FH-3, ln);
  }
  if(val<0) lcd_putcAtt(x-fw,y,'-',mode);
}

void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint8_t mode )
{
  lcd_outdezNAtt( x,y,val,mode,5);
}

void lcd_outdez( uint8_t x, uint8_t y, int16_t val )
{
  lcd_outdezAtt(x,y,val,0);
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


void setCrc()
{
  uint16_t crc = 0 ;
  for ( uint8_t i=2; i<9; i++)
	{
  	crc += TxPhyPacket[i]; //0-1FF
  	crc += crc >> 8; //0-100
  	crc &= 0x00ff;
  }
	TxPhyPacket[9] = ~crc ;
}

void menuChangeId(uint8_t event)
{
	static uint32_t state ;
 	
	TITLE( "CHANGE SPort Id" ) ;

//	lcd_puts_Pleft( 2*FH, "Not Implemented(yet)" ) ;

	switch(event)
	{
    case EVT_ENTRY:
			RxPacket[1] = 0 ;			
			RxCount = 0 ;
			RxLastCount = 0 ;
			IdIndex = 0x1B ;
			IdFound = 0 ;
			state = CHANGE_SCANNING ;
			SendCount = 2 ;
#if defined(PCBX9D) || defined(PCB9XT)
			EXTERNAL_RF_ON() ;
#endif
    break ;
    
//		case EVT_KEY_FIRST(BOOT_KEY_EXIT):
//		case EVT_KEY_LONG(BTN_RE) :
//     	chainMenu(menuUpdate) ;
//   		killEvents(event) ;
//#if defined(PCBX9D) || defined(PCB9XT)
//			EXTERNAL_RF_OFF() ;
//#endif
//    break ;
		
#ifdef PCBX7
		case EVT_KEY_FIRST(BOOT_KEY_DOWN):
#else
		case EVT_KEY_FIRST(BOOT_KEY_UP):
#endif
			if ( state == CHANGE_ENTER_ID )
			{
				if ( ++NewPhyId > 0x1B )
				{
					NewPhyId = 0x1B ;
				}
			}
    break ;
    
#ifdef PCBX7
		case EVT_KEY_FIRST(BOOT_KEY_UP):
#else
		case EVT_KEY_FIRST(BOOT_KEY_DOWN):
#endif
			if ( state == CHANGE_ENTER_ID )
			{
				if ( NewPhyId  )
				{
					NewPhyId -= 1 ;
				}
			}
    break ;

		case EVT_KEY_LONG(BOOT_KEY_MENU):
			if ( ( state == CHANGE_SCANNING ) && IdFound )
			{
				state = CHANGE_ENTER_ID ;
			}
			if ( state == CHANGE_FINISHED )
			{
				state = CHANGE_SCANNING ;
				IdFound = 0 ;
				RxPacket[1] = 0 ;			
				RxCount = 0 ;
				RxLastCount = 0 ;
				SendCount = 2 ;
			}
    break ;
		
		case EVT_KEY_FIRST(BOOT_KEY_MENU):
			if ( state == CHANGE_ENTER_ID )
			{
				TxPhyPacket[0] = 0x7E ;
				TxPhyPacket[1] = 0x1C ;
				TxPhyPacket[2] = 0x21 ;
				TxPhyPacket[3] = 0xFF ;
				TxPhyPacket[4] = 0xFF ;
				TxPhyPacket[5] = 0 ;
				TxPhyPacket[6] = 0 ;
				TxPhyPacket[7] = 0 ;
				TxPhyPacket[8] = 0 ;

				setCrc() ;
#if defined(PCBX9D) || defined(PCB9XT)
				x9dSPortTxStart( TxPhyPacket, 10, NO_RECEIVE ) ;
#endif
#ifdef PCBSKY
				txPdcUsart( TxPhyPacket, 10, NO_RECEIVE ) ;
#endif
				state = CHANGE_SET_IDLE ;
				SendCount = 100 ;
			}
    break ;
	}

	switch ( state )
	{
		case CHANGE_SCANNING :
			lcd_puts_Pleft( 3*FH, "Scanning" ) ;
#ifdef PCBSKY
	if ( RadioType == RADIO_PRO )
	{
		lcd_puts_Pleft( 0, "\021Pro" ) ;
	}
#endif
			if ( --SendCount == 0 )
			{
				SendCount = 2 ;
				if ( ++IdIndex > 27 )
				{
					IdIndex = 0 ;
				}
				TxPhyPacket[0] = 0x7E ;
				TxPhyPacket[1] = SportIds[IdIndex] ;
#if defined(PCBX9D) || defined(PCB9XT)
				x9dSPortTxStart( TxPhyPacket, 2, WITH_RECEIVE ) ;
#endif
#ifdef PCBSKY
				txPdcUsart( TxPhyPacket, 2, WITH_RECEIVE ) ;
#endif
			}
		break ;
	
		case CHANGE_ENTER_ID :
			lcd_puts_Pleft( 3*FH, "New Id: " ) ;
	    lcd_outdez( 9*FW, 3*FH, NewPhyId ) ;
		break ;

		case CHANGE_SET_IDLE :
			lcd_puts_Pleft( 3*FH, "Set Idle state" ) ;
			if ( --SendCount == 0)
			{
				TxPhyPacket[0] = 0x7E ;
				TxPhyPacket[1] = 0x1C ;
				TxPhyPacket[2] = 0x31 ;
				TxPhyPacket[3] = AppId ;
				TxPhyPacket[4] = AppId >> 8 ;
				TxPhyPacket[5] = 1 ;
				TxPhyPacket[6] = NewPhyId ;
				TxPhyPacket[7] = 0 ;
				TxPhyPacket[8] = 0 ;

				setCrc() ;
#if defined(PCBX9D) || defined(PCB9XT)
				x9dSPortTxStart( TxPhyPacket, 10, NO_RECEIVE ) ;
#endif
#ifdef PCBSKY
				txPdcUsart( TxPhyPacket, 10, NO_RECEIVE ) ;
#endif
				state = CHANGE_SET_VALUE ;
				SendCount = 100 ;
			}
		break ;
		
		case CHANGE_SET_VALUE :
			lcd_puts_Pleft( 3*FH, "Set Active state" ) ;
			if ( --SendCount == 0)
			{
				TxPhyPacket[0] = 0x7E ;
				TxPhyPacket[1] = 0x1C ;
				TxPhyPacket[2] = 0x20 ;
				TxPhyPacket[3] = 0xFF ;
				TxPhyPacket[4] = 0xFF ;
				TxPhyPacket[5] = 0 ;
				TxPhyPacket[6] = 0 ;
				TxPhyPacket[7] = 0 ;
				TxPhyPacket[8] = 0 ;

				setCrc() ;
#if defined(PCBX9D) || defined(PCB9XT)
				x9dSPortTxStart( TxPhyPacket, 10, NO_RECEIVE ) ;
#endif
#ifdef PCBSKY
				txPdcUsart( TxPhyPacket, 10, NO_RECEIVE ) ;
#endif
				state = CHANGE_FINISHED ;
			}
		break ;
		
		case CHANGE_FINISHED :
			lcd_puts_Pleft( 3*FH, "Id Changed" ) ;
			lcd_puts_Pleft( 5*FH, "MENU LONG to restart" ) ;
			PhyId = NewPhyId ;
		break ;
	}

//	lcd_outhex4( 0, 7*FH, RxCount ) ;
	if ( RxPacket[1] == 0x10 )
	{
		if ( IdFound == 0 )
		{
			IdFound = 1 ;
			PhyId = RxPacket[0] ;
			NewPhyId = PhyId & 0x1F ;
			if ( NewPhyId > 0x1B )
			{
				NewPhyId = 0x1B ;
			}
		}
		AppId = RxPacket[2] | ( RxPacket[3] << 8 ) ;
//		lcd_outhex4( 0, 4*FH, RxPacket[0] ) ;
//		lcd_outhex4( 25, 4*FH, RxPacket[1] ) ;
//		lcd_outhex4( 50, 4*FH, RxPacket[2] ) ;
//		lcd_outhex4( 75, 4*FH, RxPacket[3] ) ;
//		lcd_outhex4( 100, 4*FH, RxPacket[4] ) ;
//		lcd_outhex4( 0, 5*FH, RxPacket[5] ) ;
//		lcd_outhex4( 25, 5*FH, RxPacket[6] ) ;
//		lcd_outhex4( 50, 5*FH, RxPacket[7] ) ;
//		lcd_outhex4( 75, 5*FH, RxPacket[8] ) ;
//		lcd_outhex4( 100, 5*FH, RxPacket[9] ) ;
	}
	if ( IdFound )
	{
		lcd_puts_Pleft( 6*FH, "Id    AppId" ) ;
		lcd_outdez( 5*FW, 6*FH, PhyId & 0x1F ) ;
		lcd_outhex4( 12*FW, 6*FH, AppId ) ;
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


int main()
{
#ifdef PCBX7
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
	start_timer0() ;
#endif

#ifdef PCBX7
	init_hw_timer()	;
	__enable_irq() ;
	configure_pins( GPIO_Pin_2, PIN_OUTPUT | PIN_PORTB | PIN_LOW ) ; // BOOT1/Sport power
#endif // PCBX7

	lcd_init() ;

#ifdef PCBSKY
extern uint8_t OptrexDisplay ;
	OptrexDisplay = 1 ;
#endif
	lcd_clear() ;
#ifdef PCBX9D
 #ifdef PCBX7
	lcd_puts_Pleft( 0, "Change Id" ) ;
 #else
	lcd_puts_Pleft( 0, "\006Change Id" ) ;
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
	init_hw_timer()	;
 #endif // PCBX7
#endif

#ifdef PCB9XT
//	init_keys() ;
//	setup_switches() ;
	init_hw_timer()	;
#endif

 #ifndef PCBX7
	__enable_irq() ;
 #endif // PCBX7


	init10msTimer() ;
	start_2Mhz_timer() ;

#ifdef PCB9XT
	BlSetColour( 60, 3 ) ;	
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

#ifdef PCBX7
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
	init_rotary_encoder() ;
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


	g_menuStack[0] = menuChangeId ;
//	g_menuStack[1] = menuUp1 ;	// this is so the first instance of [MENU LONG] doesn't freak out!
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

	com1_Configure( 57600, SERIAL_NORM, SERIAL_NO_PARITY ) ; // Kick off at 57600 baud

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
		checkRotaryEncoder() ;
#endif

		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog
			check_frsky() ;

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
			if ( ( g_menuStack[0] == menuChangeId ) && ( g_menuStackPtr == 0 ) )
			{
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
	TWI0->TWI_MMR = 0x006F0100 ;		// Device 6F and master is writing, 1 byte addr
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

// This is called from the receive processing.
// Packet has leading 0x7E stripped
void maintenance_receive_packet( uint8_t *packet, uint32_t check )
{
	RxCount += 1 ;
	RxPacket[0] = packet[0] ;
	RxPacket[1] = packet[1] ;
	RxPacket[2] = packet[2] ;
	RxPacket[3] = packet[3] ;
	RxPacket[4] = packet[4] ;
	RxPacket[5] = packet[5] ;
	RxPacket[6] = packet[6] ;
	RxPacket[7] = packet[7] ;
	RxPacket[8] = packet[8] ;
	RxPacket[9] = packet[9] ;
}




