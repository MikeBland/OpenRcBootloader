/*
 * Author - Mike Blandford
 *
 * Based on er9x by Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
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



#include <stdint.h>
#include <stdlib.h>

#ifdef PCBSKY
#include "radio.h"
#include "drivers.h"
#include "AT91SAM3S4.h"
#endif


#if ( defined(PCBX9D) || defined(PCB9XT) )
#include "radio.h"
#include "hal.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "drivers.h"
#endif

#ifdef PCBX12D
#include "radio.h"
#include "hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "drivers.h"
#endif


#include "logicio.h"

#ifdef EVT_KEY_MASK
#undef EVT_KEY_MASK
#endif

#define EVT_KEY_MASK             0x0f

volatile uint32_t Spi_complete ;

void b_putEvent( register uint8_t evt) ;
void per10ms( void ) ;
uint8_t getEvent( void ) ;
void pauseEvents(uint8_t event) ;
void killEvents(uint8_t event) ;

uint8_t s_evt;
void b_putEvent( uint8_t evt)
{
  s_evt = evt;
}

#ifdef PCBSKY
volatile int32_t Rotary_position ;
volatile int32_t Rotary_count ;
int32_t LastRotaryValue ;
int32_t Rotary_diff ;

void init_rotary_encoder()
{
//  register uint32_t dummy;

	configure_pins( PIO_PC19 | PIO_PC21, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_PULLUP ) ;	// 19 and 21 are rotary encoder
	configure_pins( PIO_PB6, PIN_ENABLE | PIN_INPUT | PIN_PORTB | PIN_PULLUP ) ;		// rotary encoder switch
//	PIOC->PIO_IER = PIO_PC19 | PIO_PC21 ;
//	dummy = PIOC->PIO_PDSR ;		// Read Rotary encoder (PC19, PC21)
//	dummy >>= 19 ;
//	dummy &= 0x05 ;			// pick out the three bits
//	Rotary_position &= ~0x45 ;
//	Rotary_position |= dummy ;
//	NVIC_SetPriority( PIOC_IRQn, 1 ) ; // Lower priority interrupt
//	NVIC_EnableIRQ(PIOC_IRQn) ;
//	LastRotaryValue = Rotary_count ;
}

void checkRotaryEncoder()
{
  register uint32_t dummy ;
	
	dummy = PIOC->PIO_PDSR ;		// Read Rotary encoder (PC19, PC21)
	dummy >>= 19 ;
	dummy &= 0x05 ;			// pick out the three bits
	if ( dummy != ( Rotary_position & 0x05 ) )
	{
		if ( ( Rotary_position & 0x01 ) ^ ( ( dummy & 0x04) >> 2 ) )
		{
			Rotary_count -= 1 ;
		}
		else
		{
			Rotary_count += 1 ;
		}
		Rotary_position &= ~0x45 ;
		Rotary_position |= dummy ;
	}
}

#endif

#if (defined(REV9E) || defined(PCBX7))

volatile int32_t Rotary_position ;
volatile int32_t Rotary_count ;
int32_t LastRotaryValue ;
int32_t Rotary_diff ;

#ifdef PCBX7
#ifndef PCBT12
void init_rotary_encoder()
{
	configure_pins( 0x0A00, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
//	g_eeGeneral.rotaryDivisor = 2 ;
}

void checkRotaryEncoder()
{
  register uint32_t dummy ;
	
	dummy = GPIOENCODER->IDR ;	// Read Rotary encoder ( PE11, PE9 )
	dummy >>= 9 ;
	dummy = (dummy & 1) | ( ( dummy >> 1 ) & 2 ) ;	// pick out the two bits
	if ( dummy != ( Rotary_position & 0x03 ) )
	{
		if ( ( Rotary_position & 0x01 ) ^ ( ( dummy & 0x02) >> 1 ) )
		{
			Rotary_count -= 1 ;
		}
		else
		{
			Rotary_count += 1 ;
		}
		Rotary_position &= ~0x03 ;
		Rotary_position |= dummy ;
	}
}
#endif
#else // PCBX7

void init_rotary_encoder()
{
//	configure_pins( 0x0060, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
	configure_pins( 0x3000, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
//	g_eeGeneral.rotaryDivisor = 2 ;
}

void checkRotaryEncoder()
{
  register uint32_t dummy ;
	
	dummy = GPIOENCODER->IDR ;	// Read Rotary encoder ( PE6, PE5 )
//	dummy >>= 5 ;
	dummy >>= 12 ;
	dummy &= 0x03 ;			// pick out the two bits
	if ( dummy != ( Rotary_position & 0x03 ) )
	{
		if ( ( Rotary_position & 0x01 ) ^ ( ( dummy & 0x02) >> 1 ) )
		{
			Rotary_count -= 1 ;
//			Rotary_count += 1 ;
		}
		else
		{
			Rotary_count += 1 ;
//			Rotary_count -= 1 ;
		}
		Rotary_position &= ~0x03 ;
		Rotary_position |= dummy ;
	}
}
#endif // PCBX7

uint8_t LastEvt ;

uint8_t getEvent()
{
  register uint8_t evt = s_evt;
  s_evt=0;
#ifndef PCBT12
	checkRotaryEncoder() ;
	{
		int32_t x ;
		x = Rotary_count >> 1 ;
		Rotary_diff = x - LastRotaryValue ;
		LastRotaryValue = x ;
	}
	if ( evt == 0 )
	{
extern int32_t Rotary_diff ;
		if ( Rotary_diff > 0 )
		{
			evt = EVT_KEY_FIRST(KEY_MINUS) ;
		}
		else if ( Rotary_diff < 0 )
		{
			evt = EVT_KEY_FIRST(KEY_PLUS) ;
		}
		Rotary_diff = 0 ;
	}
#endif
	if ( evt != 0 )
	{
		LastEvt = evt ;
	}
  return evt;
}

#else
uint8_t LastEvt ;
uint8_t getEvent()
{
  register uint8_t evt = s_evt;
  s_evt=0;
#ifdef PCBSKY
	{
		int32_t x ;
		x = Rotary_count >> 2 ;
		Rotary_diff = x - LastRotaryValue ;
		LastRotaryValue = x ;
	}
	if ( evt == 0 )
	{
extern int32_t Rotary_diff ;
		if ( Rotary_diff > 0 )
		{
			evt = EVT_KEY_FIRST(KEY_DOWN) ;
		}
		else if ( Rotary_diff < 0 )
		{
			evt = EVT_KEY_FIRST(KEY_UP) ;
		}
		Rotary_diff = 0 ;
	}
	if ( evt != 0 )
	{
		LastEvt = evt ;
	}
#endif
	return evt;
}
#endif

Key keys[NUM_KEYS] ;

void Key::input(bool val, EnumKeys enuk)
{
  //  uint8_t old=m_vals;
  m_vals <<= 1;  if(val) m_vals |= 1; //portbit einschieben
  m_cnt++;

  if(m_state && m_vals==0){  //gerade eben sprung auf 0
    if(m_state!=KSTATE_KILLED) {
      b_putEvent(EVT_KEY_BREAK(enuk));
    }
    m_cnt   = 0;
    m_state = KSTATE_OFF;
  }
  switch(m_state){
    case KSTATE_OFF:
      if(m_vals==FFVAL){ //gerade eben sprung auf ff
        m_state = KSTATE_START;
        m_cnt   = 0;
      }
      break;
      //fallthrough
    case KSTATE_START:
      b_putEvent(EVT_KEY_FIRST(enuk));
#ifdef KSTATE_RPTDELAY
      m_state   = KSTATE_RPTDELAY;
#else
      m_state   = 16;
#endif
      m_cnt     = 0;
      break;
#ifdef KSTATE_RPTDELAY
    case KSTATE_RPTDELAY: // gruvin: longer delay before first key repeat
      if(m_cnt == 32) b_putEvent(EVT_KEY_LONG(enuk)); // need to catch this inside RPTDELAY time
      if (m_cnt == 40) {
        m_state = 16;
        m_cnt = 0;
      }
      break;
#endif
    case 16:
#ifndef KSTATE_RPTDELAY
      if(m_cnt == 32) b_putEvent(EVT_KEY_LONG(enuk));
      //fallthrough
#endif
    case 8:
    case 4:
    case 2:
      if(m_cnt >= 48)  { //3 6 12 24 48 pulses in every 480ms
        m_state >>= 1;
        m_cnt     = 0;
      }
      //fallthrough
    case 1:
      if( (m_cnt & (m_state-1)) == 0)  b_putEvent(EVT_KEY_REPT(enuk));
      break;

    case KSTATE_PAUSE: //pause
      if(m_cnt >= 64)      {
        m_state = 8;
        m_cnt   = 0;
      }
      break;

    case KSTATE_KILLED: //killed
      break;
  }
}

#ifdef APP
void pauseEvents(uint8_t event)
#else
void b_pauseEvents(uint8_t event)
#endif
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  keys[event].pauseEvents();
}

void killEvents(uint8_t event)
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  keys[event].killEvents();
}

#ifdef APP
uint8_t menuPressed()
{
	if ( keys[KEY_MENU].isKilled() )
	{
		return 0 ;
	}
	return ( read_keys() & 2 ) == 0 ;
}

uint32_t keyState(EnumKeys enuk)
{
  if(enuk < (int)DIM(keys))  return keys[enuk].state() ? 1 : 0 ;
	return 0 ;
}
#endif

void per10ms()
{
	register uint32_t i ;

  uint8_t enuk = KEY_MENU;
  uint8_t    in = ~read_keys() & 0x7E ;
	// Bits 3-6 are down, up, right and left
	// Try to only allow one at a 
#ifdef REVX
	static uint8_t current ;
	uint8_t dir_keys ;
	uint8_t lcurrent ;

	dir_keys = in & 0x78 ;		// Mask to direction keys
	if ( ( lcurrent = current ) )
	{ // Something already pressed
		if ( ( lcurrent & dir_keys ) == 0 )
		{
			lcurrent = 0 ;	// No longer pressed
		}
		else
		{
			in &= lcurrent | 0x06 ;	// current or MENU or EXIT allowed
		}
	}
	if ( lcurrent == 0 )
	{ // look for a key
		if ( dir_keys & 0x20 )	// right
		{
			lcurrent = 0x60 ;		// Allow L and R for 9X
		}
		else if ( dir_keys & 0x40 )	// left
		{
			lcurrent = 0x60 ;		// Allow L and R for 9X
		}
		else if ( dir_keys & 0x08 )	// down
		{
			lcurrent = 0x08 ;
		}
		else if ( dir_keys & 0x10 )	// up
		{
			lcurrent = 0x10 ;
		}
		in &= lcurrent | 0x06 ;	// current or MENU or EXIT allowed
	}
	current = lcurrent ;
#endif

extern uint32_t readTrainerSwitch( void ) ;
	if ( readTrainerSwitch() )
	{
		in |= 0x80 ;
	}

  for( i=1; i<8; i++)
  {
		uint8_t value = in & (1<<i) ;
    //INP_B_KEY_MEN 1  .. INP_B_KEY_LFT 6
    keys[enuk].input(value,(EnumKeys)enuk);
    ++enuk;
  }

#ifdef REV9E
	uint8_t value = ~GPIOF->IDR & PIN_BUTTON_ENCODER ;
	keys[BTN_RE].input( value,(EnumKeys)BTN_RE); // Rotary Enc. Switch
#endif

#ifdef PCBSKY
//	checkRotaryEncoder() ;
	uint8_t value = ~PIOB->PIO_PDSR & 0x40 ;
	keys[BTN_RE].input( value,(EnumKeys)BTN_RE); // Rotary Enc. Switch
#endif


#ifdef APP
#ifdef PCB9XT
extern uint16_t M64Switches ;
	uint8_t value = (M64Switches & 0x0200) ? 1 : 0 ;
	keys[BTN_RE].input( value,(EnumKeys)BTN_RE); // Rotary Enc. Switch
#endif
#endif

}

#ifdef PCB9XT

int32_t get_fifo64( struct t_fifo64 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x3F ;
		return rxbyte ;
	}
	return -1 ;
}

void eeprom_write_enable() ;
uint32_t eeprom_write_one( uint8_t byte, uint8_t count ) ;
uint32_t spi_operation( register uint8_t *tx, register uint8_t *rx, register uint32_t count ) ;

void init_spi()
{
	register uint8_t *p ;
	uint8_t spi_buf[4] ;
//  SPI_InitTypeDef  SPI_InitStructure;
//  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock for CS */
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable GPIO clock for Signals */
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);
  /* Enable SPI clock, SPI1: APB2, SPI2: APB1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN ;    // Enable clock

	// APB1 clock / 2 = 133nS per clock
	SPI1->CR1 = 0 ;		// Clear any mode error
	SPI1->CR2 = 0 ;
	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA ;
	SPI1->CR1 |= SPI_CR1_MSTR ;	// Make sure in case SSM/SSI needed to be set first
	SPI1->CR1 |= SPI_CR1_SPE ;

	configure_pins( GPIO_Pin_SPI_EE_CS, PIN_PUSHPULL | PIN_OS25 | PIN_OUTPUT | PIN_PORTA ) ;
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
	configure_pins( GPIO_Pin_SPI_EE_SCK | GPIO_Pin_SPI_EE_MOSI, PIN_PUSHPULL | PIN_OS25 | PIN_PERIPHERAL | PIN_PORTB | PIN_PER_5 ) ;
	configure_pins( GPIO_Pin_SPI_EE_MISO, PIN_PERIPHERAL | PIN_PORTB | PIN_PULLUP | PIN_PER_5 ) ;

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN ;			// Enable DMA2 clock

	NVIC_SetPriority( DMA2_Stream0_IRQn, 1 ) ; // priority interrupt
  NVIC_EnableIRQ( DMA2_Stream0_IRQn ) ;
	NVIC_SetPriority( DMA2_Stream3_IRQn, 1 ) ; // priority interrupt
  NVIC_EnableIRQ( DMA2_Stream3_IRQn ) ;
        
	p = spi_buf ;
  
	eeprom_write_enable() ;
	
	*p = 1 ;		// Write status register command
	*(p+1) = 0 ;
	spi_operation( p, spi_buf, 2 ) ;
	      
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
//#ifdef STM32_SD_USE_DMA
//  /* enable DMA clock */
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//#endif
	
}

uint32_t spi_operation( register uint8_t *tx, register uint8_t *rx, register uint32_t count )
{
	register SPI_TypeDef *spiptr = SPI1 ;
	register uint32_t result ;
	
	GPIOA->BSRRH = GPIO_Pin_SPI_EE_CS ;		// output enable
	(void) spiptr->DR ;		// Dump any rx data
	while( count )
	{
		result = 0 ;
		while( ( spiptr->SR & SPI_SR_TXE ) == 0 )
		{
			// wait
			if ( ++result > 10000 )
			{
				result = 0xFFFF ;
				break ;				
			}
		}
		if ( result > 10000 )
		{
			break ;
		}
//		if ( count == 1 )
//		{
//			spiptr->SPI_CR = SPI_CR_LASTXFER ;		// LastXfer bit
//		}
		spiptr->DR = *tx++ ;
		result = 0 ;
		while( ( spiptr->SR & SPI_SR_RXNE ) == 0 )
		{
			// wait for received
			if ( ++result > 10000 )
			{
				result = 0x2FFFF ;
				break ;				
			}
		}
		if ( result > 10000 )
		{
			break ;
		}
		*rx++ = spiptr->DR ;
		count -= 1 ;
	}
	if ( result <= 10000 )
	{
		result = 0 ;
	}
	result = 0 ; 
	
	return result ;
}

void eeprom_write_enable()
{
	eeprom_write_one( 6, 0 ) ;
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
}

uint32_t eeprom_read_status()
{
	uint32_t result ;
	
	result = eeprom_write_one( 5, 1 ) ;
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
	return result ;
}

uint32_t  eeprom_write_one( uint8_t byte, uint8_t count )
{
	register SPI_TypeDef *spiptr = SPI1 ;
	register uint32_t result ;
	
	GPIOA->BSRRH = GPIO_Pin_SPI_EE_CS ;		// output enable
	(void) spiptr->DR ;		// Dump any rx data
	
	spiptr->DR = byte ;

	result = 0 ; 
	while( ( spiptr->SR & SPI_SR_RXNE ) == 0 )
	{
		// wait for received
		if ( ++result > 10000 )
		{
			break ;				
		}
	}
	if ( count == 0 )
	{
		return spiptr->DR ;
	}
	(void) spiptr->DR ;		// Dump the rx data
	spiptr->DR = 0 ;
	result = 0 ; 
	while( ( spiptr->SR & SPI_SR_RXNE ) == 0 )
	{
		// wait for received
		if ( ++result > 10000 )
		{
			break ;				
		}
	}
	return spiptr->DR ;
}

uint32_t spi_PDC_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count )
{
	static uint8_t discard_rx_command[4] ;

	Spi_complete = 0 ;
	if ( comlen > 4 )
	{
		Spi_complete = 1 ;
		return 0x4FFFF ;		
	}
	
	GPIOA->BSRRH = GPIO_Pin_SPI_EE_CS ;		// output enable
	DMA2_Stream0->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA2_Stream3->CR &= ~DMA_SxCR_EN ;		// Disable DMA

	spi_operation( command, discard_rx_command, comlen ) ;	// send command

	if ( rx )
	{
		DMA2_Stream0->CR = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_PL_0
											  | DMA_SxCR_MINC ;
		DMA2_Stream0->PAR = (uint32_t) &SPI1->DR ;
		DMA2_Stream0->M0AR = (uint32_t) rx ;
		DMA2_Stream0->NDTR = count ;
		DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 ; // Write ones to clear bits
		SPI1->CR2 |= SPI_CR2_RXDMAEN ;
		DMA2_Stream0->CR |= DMA_SxCR_EN ;		// Enable DMA
	}
	if ( tx )
	{
		DMA2_Stream3->M0AR = (uint32_t) tx ;
	}
	else
	{
		DMA2_Stream3->M0AR = (uint32_t) rx ;
	}
	DMA2_Stream3->NDTR = count ;
	DMA2_Stream3->CR = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_PL_0 
											  | DMA_SxCR_MINC | DMA_SxCR_DIR_0 ;
	DMA2_Stream3->PAR = (uint32_t) &SPI1->DR ;
	DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3 ; // Write ones to clear bits
	SPI1->CR2 |= SPI_CR2_TXDMAEN ;
	DMA2_Stream3->CR |= DMA_SxCR_EN ;		// Enable DMA
	if ( tx )
	{
		DMA2_Stream3->CR |= DMA_SxCR_TCIE ;
	}
	else
	{
		DMA2_Stream0->CR |= DMA_SxCR_TCIE ;
	}
	// Wait for things to get started, avoids early interrupt
//	for ( count = 0 ; count < 1000 ; count += 1 )
//	{
//		if ( ( spiptr->SPI_SR & SPI_SR_TXEMPTY ) == 0 )
//		{
//			break ;			
//		}
//	}
	
// For bootloader, wait for completion	
	count = 0 ;
	while( Spi_complete == 0 )
	{
		if ( ++count > 100000 )
		{
			break ;			
		}
	}
	
	DMA2_Stream3->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA2_Stream0->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA2_Stream0->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	DMA2_Stream3->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	SPI1->CR2 &= ~SPI_CR2_TXDMAEN & ~SPI_CR2_RXDMAEN ;
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
	
	if ( count > 1000000 )
	{
		return 1 ;
	}
	return 0 ;
}

extern "C" void DMA2_Stream0_IRQHandler()
{
	DMA2_Stream3->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA2_Stream0->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA2_Stream0->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	DMA2_Stream3->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	SPI1->CR2 &= ~SPI_CR2_TXDMAEN & ~SPI_CR2_RXDMAEN ;
	Spi_complete = 1 ;
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
}

extern "C" void DMA2_Stream3_IRQHandler()
{
	if ( DMA2_Stream3->NDTR )
	{
		DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CFEIF3 ; // Write ones to clear bits
		DMA2_Stream3->CR |= DMA_SxCR_EN ;		// Enable DMA
		return ;
	}
	// Wait for TXE=1
	uint32_t i ;
	i = 0 ;
	while ( ( SPI1->SR & SPI_SR_TXE ) == 0 )
	{
		if ( ++i > 10000 )
		{
			break ;		// Software timeout				
		}
	}
	// Then wait for BSY == 0
	i = 0 ;
	while ( SPI1->SR & SPI_SR_BSY )
	{
		if ( ++i > 10000 )
		{
			break ;		// Software timeout				
		}
	}
	DMA2_Stream3->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA2_Stream3->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	SPI1->CR2 &= ~SPI_CR2_TXDMAEN & ~SPI_CR2_RXDMAEN ;
	Spi_complete = 1 ;
	GPIOA->BSRRL = GPIO_Pin_SPI_EE_CS ;		// output disable
}

#endif // PCB9XT

// Backlight driver
// Reset/sync, >50uS low
// 0 bit  High - 0.5uS   Low - 2.0uS
// 1 bit  High - 1.2uS   Low - 1.3uS
// All +/-150nS
// Timer 4, channel 4, PortB bit 9
// Also needs Timer 4 channel 1 to cause DMA transfer
// Timer 4 counts up to 2.5uS then resets, channel 4 outputs signal
// Channel 1 requests DMA to channel 4 at 2uS
// At end of DMA, need to work out when to stop timer.
// Maybe set channel 4 compare to past 2.5uS
// DMA1, stream 0, channel 2, triggered by TIM4 CH1

// Testing on a X9D plus, on Ext PPM out PA7
// TIM2, ch2 and TIM2 Ch4 for DMA on CH3, Stream1

#ifdef PCB9XT

uint32_t BlLastLevel ;
uint16_t BlData[27] ;
uint8_t BlColours[3] ;
uint8_t BlChanged ;
uint8_t BlCount ;
uint8_t BlSending ;

void backlightSend() ;

// The value is 24 bits, 8 red, 8 green, 8 blue right justified
void backlightSet( uint32_t value )
{
	uint16_t *blptr = BlData ;
	uint32_t i ;
	value <<= 8 ;
	for ( i = 0 ; i < 24 ; i += 1 )
	{
//		*blptr++ = ( value & 0x80000000 ) ? 12 : 5 ;
		*blptr++ = ( value & 0x80000000 ) ? 6 : 3 ;
		value <<= 1 ;
	}
	*blptr++ = 40 ;
	*blptr++ = 40 ;
	*blptr = 40 ;
}


// Level is 0 to 100%
// colour is 0 red, 1 green, 2 blue
void BlSetColour( uint32_t level, uint32_t colour )
{
	if ( colour > 3 )
	{
		return ;
	}
	level *= 255 ;
	level /= 100 ;
	if ( level > 255 )
	{
		level = 255 ;
	}
	if ( colour == 3 )
	{
		BlColours[0] = level ;
		BlColours[1] = level ;
		BlColours[2] = level ;
	}
	else
	{
		BlColours[colour] = level ;
	}
	
	level = (BlColours[0] << 16 ) | (BlColours[1] << 8 ) | (BlColours[2] ) ;
	if ( level != BlLastLevel )
	{
		BlLastLevel = level ;
		backlightSet( level ) ;
		BlCount = 2 ;
		BlChanged = 1 ;
		if ( BlSending == 0 )
		{
			backlightSend() ;
		}		 
	}
}

extern uint32_t Peri1_frequency ;
extern uint32_t Timer_mult1 ;

void backlightSend()
{
	BlSending = 1 ;
	if ( BlChanged )
	{
		backlightSet( BlLastLevel ) ;
		BlChanged = 0 ;
		BlCount = 2 ;
	}
	
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ;           // Enable portB clock
  configure_pins( GPIO_Pin_9, PIN_PERIPHERAL | PIN_PORTB | PIN_PER_2 | PIN_OS25 | PIN_PUSHPULL ) ;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;		// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN ;		// Enable DMA1 clock

  TIM4->CR1 &= ~TIM_CR1_CEN ;
	TIM4->PSC = (Peri1_frequency*Timer_mult1) / 10000000 - 1 ;		// 0.1uS
  TIM4->ARR = 12 ;             // 1.3uS
  TIM4->CCR1 = 9 ;            // Update time
	TIM4->CCER = TIM_CCER_CC4E ;
	TIM4->CNT = 65536-710 ;
  TIM4->CCR4 = BlData[0] ;		// Past end
	TIM4->DIER |= TIM_DIER_CC1DE ;          // Enable DMA on CC1 match
	TIM4->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 ;                     // Force O/P high

  // Enable the DMA channel here, DMA1 stream 0, channel 2
  DMA1_Stream0->CR &= ~DMA_SxCR_EN ;              // Disable DMA
  DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 ; // Write ones to clear bits
  DMA1_Stream0->CR = DMA_SxCR_CHSEL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_0 
                 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 ;// | DMA_SxCR_PFCTRL ;
  DMA1_Stream0->PAR = (uint32_t)(&TIM4->CCR4);
  DMA1_Stream0->M0AR = (uint32_t)(&BlData[1]);
	DMA1_Stream0->NDTR = 26 ;
  DMA1_Stream0->CR |= DMA_SxCR_EN ;               // Enable DMA

  TIM4->SR &= ~TIM_SR_CC1IF ;                             // Clear flag
  TIM4->CR1 |= TIM_CR1_CEN ;
	DMA1_Stream0->CR |= DMA_SxCR_TCIE ;
	NVIC_SetPriority( DMA1_Stream0_IRQn, 5 ) ; // Lower priority interrupt
  NVIC_EnableIRQ( DMA1_Stream0_IRQn ) ;
}

extern "C" void DMA1_Stream0_IRQHandler()
{
	if ( --BlCount )
	{
		backlightSend() ;
		return ;
	}
	DMA1_Stream0->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA1_Stream0->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
  TIM4->CR1 &= ~TIM_CR1_CEN ;

	BlSending = 0 ;
	if ( BlChanged )
	{
		backlightSend() ;
	}
}


#endif // PCB9XT

