/****************************************************************************
*  Copyright (c) 2012 by Michael Blandford. All rights reserved.
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
*
****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


#ifdef PCBSKY
#include "AT91SAM3S4.h"
#include "radio.h"
#include "drivers.h"
#endif

#if ( defined(PCBX9D) || defined(PCB9XT) )
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "radio.h"
#include "hal.h"
#endif

#ifdef PCB9XT
#include "mega64.h"
#endif


#ifndef PCBX12D
#ifndef PCBX10
#ifndef SIMU
#include "core_cm3.h"
#endif
#endif
#endif

#if defined(PCBX12D) || defined(PCBX10)
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "stm32f4xx_gpio.h"
#include "radio.h"
#include "hal.h"
#endif

#include "logicio.h"

#ifdef PCBSKY
#include "lcd.h"
#endif


#ifndef SIMU
#ifdef PCBSKY
void configure_pins( uint32_t pins, uint16_t config )
{
	register Pio *pioptr ;
	
	pioptr = PIOA + ( ( config & PIN_PORT_MASK ) >> 6) ;
	if ( config & PIN_PULLUP )
	{
		pioptr->PIO_PPDDR = pins ;
		pioptr->PIO_PUER = pins ;
	}
	else
	{
		pioptr->PIO_PUDR = pins ;
	}

	if ( config & PIN_PULLDOWN )
	{
		pioptr->PIO_PUDR = pins ;
		pioptr->PIO_PPDER = pins ;
	}
	else
	{
		pioptr->PIO_PPDDR = pins ;
	}

	if ( config & PIN_HIGH )
	{
		pioptr->PIO_SODR = pins ;		
	}
	else
	{
		pioptr->PIO_CODR = pins ;		
	}

	if ( config & PIN_INPUT )
	{
		pioptr->PIO_ODR = pins ;
	}
	else
	{
		pioptr->PIO_OER = pins ;
	}

	if ( config & PIN_PERI_MASK_L )
	{
		pioptr->PIO_ABCDSR[0] |= pins ;
	}
	else
	{
		pioptr->PIO_ABCDSR[0] &= ~pins ;
	}
	if ( config & PIN_PERI_MASK_H )
	{
		pioptr->PIO_ABCDSR[1] |= pins ;
	}
	else
	{
		pioptr->PIO_ABCDSR[1] &= ~pins ;
	}

	if ( config & PIN_ENABLE )
	{
		pioptr->PIO_PER = pins ;		
	}
	else
	{
		pioptr->PIO_PDR = pins ;		
	}
}
#endif


#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)
void configure_pins( uint32_t pins, uint16_t config )
{
	uint32_t address ;
	GPIO_TypeDef *pgpio ;
	uint32_t thispin ;
  uint32_t pos ;

	address = ( config & PIN_PORT_MASK ) >> 8 ;
	address *= (GPIOB_BASE-GPIOA_BASE) ;
	address += GPIOA_BASE ;
	pgpio = (GPIO_TypeDef* ) address ;
	
  /* -------------------------Configure the port pins---------------- */
  /*-- GPIO Mode Configuration --*/
  for (thispin = 0x0001, pos = 0; thispin < 0x10000; thispin <<= 1, pos +=1 )
  {
    if ( pins & thispin)
    {
			uint32_t value ;
			value = pgpio->MODER ;
      value &= ~(GPIO_MODER_MODER0 << (pos * 2)) ;
      pgpio->MODER = value | (config & PIN_MODE_MASK) << (pos * 2) ;

      if ( ( (config & PIN_MODE_MASK ) == PIN_OUTPUT) || ( (config & PIN_MODE_MASK) == PIN_PERIPHERAL) )
      {
        /* Speed mode configuration */
				value = pgpio->OSPEEDR ;
        value &= ~GPIO_OSPEEDER_OSPEEDR0 << (pos * 2) ;
        pgpio->OSPEEDR = value | ((config & PIN_SPEED_MASK) >> 13 ) << (pos * 2) ;

        /* Output mode configuration*/
        pgpio->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pos)) ;
				if ( config & PIN_ODRAIN )
				{
        	pgpio->OTYPER |= (GPIO_OTYPER_OT_0) << pos ;
				}
      }
      /* Pull-up Pull down resistor configuration*/
			value = pgpio->PUPDR ;
      value &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pos * 2));
      pgpio->PUPDR = value | ((config & PIN_PULL_MASK) >> 2) << (pos * 2) ;

			value = pgpio->AFR[pos >> 3] ;
			value &= ~(0x000F << ((pos & 7)*4)) ;
			pgpio->AFR[pos >> 3] = value | ((config & PIN_PERI_MASK) >> 4) << ((pos & 7)*4) ;
    }
  }
}
		
#endif



#ifdef PCBSKY
	
void init_keys()
{
	register Pio *pioptr ;
	
	pioptr = PIOC ;
	// Next section configures the key inputs on the LCD data
	pioptr->PIO_PER = 0x0000003BL ;		// Enable bits 1,3,4,5, 0
	pioptr->PIO_OER = PIO_PC0 ;		// Set bit 0 output
	pioptr->PIO_ODR = 0x0000003AL ;		// Set bits 1, 3, 4, 5 input
	pioptr->PIO_PUER = 0x0000003AL ;		// Set bits 1, 3, 4, 5 with pullups

	pioptr = PIOB ;
	pioptr->PIO_PUER = PIO_PB5 ;					// Enable pullup on bit B5 (MENU)
	pioptr->PIO_PER = PIO_PB5 ;					// Enable bit B5
}

// Assumes PMC has already enabled clocks to ports
void setup_switches()
{
	configure_pins( 0x01808087, PIN_ENABLE | PIN_INPUT | PIN_PORTA | PIN_PULLUP ) ;
	configure_pins( 0x00000030, PIN_ENABLE | PIN_INPUT | PIN_PORTB | PIN_PULLUP ) ;
	configure_pins( 0x91114900, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_PULLUP ) ;
}


// Switch input pins
// Needs updating for REVB board ********
// AIL-DR  PA2
// TRIM_LH_DOWN PA7 (PA23)
// ELE_DR   PA8 (PC31)
// RUN_DR   PA15
// TRIM_LV_DOWN  PA27 (PA24)
// SW_TCUT     PA28 (PC20)
// TRIM_RH_DOWN    PA29 (PA0)
// TRIM_RV_UP    PA30 (PA1)
// TRIM_LH_UP    //PB4
// SW-TRAIN    PC8
// TRIM_RH_UP   PC9
// TRIM_RV_DOWN   PC10
// SW_IDL2     PC11
// SW_IDL1     PC14
// SW_GEAR     PC16
// TRIM_LV_UP   PC28

// KEY_MENU    PB6 (PB5)
// KEY_EXIT    PA31 (PC24)
// Shared with LCD data
// KEY_DOWN  LCD5  PC3
// KEY_UP    LCD6  PC2
// KEY_RIGHT LCD4  PC4
// KEY_LEFT  LCD3  PC5

// PORTA 1111 1000 0000 0000 1000 0001 1000 0100 = 0xF8008184 proto
// PORTA 0000 0001 1000 0000 1000 0000 0000 0111 = 0x01808087 REVB
// PORTB 0000 0000 0001 0000										 = 0x0010     proto
// PORTB 0000 0000 0010 0000										 = 0x0030     REVB
// PORTC 0001 0000 0000 0001 0100 1001 0000 0000 = 0x10014900 proto
// PORTC 1001 0001 0001 0001 0100 1001 0000 0000 = 0x91114900 REVB


// Prototype
// Free pins (PA16 is stock buzzer)
// PA23, PA24, PA25, PB7, PB13
// PC20, PC21(labelled 17), PC22, PC24
// REVB
// PA25, use for stock buzzer
// PB14, PB6
// PC21, PC19, PC15 (PPM2 output)
void config_free_pins()
{
	
	configure_pins( PIO_PB14, PIN_ENABLE | PIN_INPUT | PIN_PORTB | PIN_PULLUP ) ;

}

#endif

#endif // ndef BOOT

// keys:
// KEY_EXIT    PA31 (PC24)
// KEY_MENU    PB6 (PB5)
// KEY_DOWN  LCD5  PC3 (PC5)
// KEY_UP    LCD6  PC2 (PC1)
// KEY_RIGHT LCD4  PC4 (PC4)
// KEY_LEFT  LCD3  PC5 (PC3)
// Reqd. bit 6 LEFT, 5 RIGHT, 4 UP, 3 DOWN 2 EXIT 1 MENU
// LCD pins 5 DOWN, 4 RIGHT, 3 LEFT, 1 UP
#ifdef PCBSKY
uint32_t read_keys()
{
	register uint32_t x ;
	register uint32_t y ;

	x = LcdLock ? LcdInputs : PIOC->PIO_PDSR << 1 ; // 6 LEFT, 5 RIGHT, 4 DOWN, 3 UP ()
	
	y = x & 0x00000020 ;		// RIGHT
	if ( x & 0x00000004 )
	{
		y |= 0x00000010 ;			// UP
	}
	if ( x & 0x00000010 )
	{
		y |= 0x00000040 ;			// LEFT
	}
	if ( x & 0x00000040 )
	{
		y |= 0x00000008 ;			// DOWN
	}

	if ( PIOC->PIO_PDSR & 0x01000000 )
	{
		y |= 4 ;		// EXIT
	}
	if ( PIOB->PIO_PDSR & 0x000000020 )
	{
		y |= 2 ;		// MENU
	}
	return y ;
}
#endif

#ifdef PCBSKY
uint32_t read_trims()
{
	uint32_t trims ;
	uint32_t trima ;

	trims = 0 ;

	trima = PIOA->PIO_PDSR ;
// TRIM_LH_DOWN    PA7 (PA23)
#ifndef REVX
	if ( ( trima & 0x00800000 ) == 0 )
#else
	if ( ( PIOB->PIO_PDSR & 0x10 ) == 0 )
#endif
	{
		trims |= 1 ;
	}
    
// TRIM_LV_DOWN  PA27 (PA24)
	if ( ( trima & 0x01000000 ) == 0 )
	{
		trims |= 4 ;
	}

// TRIM_RV_UP    PA30 (PA1)
#ifndef REVX
	if ( ( trima & 0x00000002 ) == 0 )
#else
	if ( ( PIOC->PIO_PDSR & 0x00000400 ) == 0 )
#endif
	{
		trims |= 0x20 ;
	}

// TRIM_RH_DOWN    PA29 (PA0)
	if ( ( trima & 0x00000001 ) == 0 )
	{
		trims |= 0x40 ;
	}

// TRIM_LH_UP PB4
#ifndef REVX
	if ( ( PIOB->PIO_PDSR & 0x10 ) == 0 )
#else
	if ( ( trima & 0x00800000 ) == 0 )
#endif
	{
		trims |= 2 ;
	}

	trima = PIOC->PIO_PDSR ;
// TRIM_LV_UP   PC28
	if ( ( trima & 0x10000000 ) == 0 )
	{
		trims |= 8 ;
	}

// TRIM_RV_DOWN   PC10
#ifndef REVX
	if ( ( trima & 0x00000400 ) == 0 )
#else
	if ( ( PIOA->PIO_PDSR & 0x00000002 ) == 0 )
#endif
	{
		trims |= 0x10 ;
	}

// TRIM_RH_UP   PC9
	if ( ( trima & 0x00000200 ) == 0 )
	{
		trims |= 0x80 ;
	}

	return trims ;
}

#endif

#ifdef PCBSKY
extern "C" uint32_t initReadTrims( void ) ;

uint32_t initReadTrims( void )
{
  PMC->PMC_PCER0 = (1<<ID_PIOB)|(1<<ID_PIOA)|(1<<ID_PIOC) ;				// Enable clocks to PIOB and PIOA and PIOC
	config_free_pins() ;
	init_keys() ;
	setup_switches() ;
	uint32_t i ;
	for ( i = 0 ; i < 10000 ; i += 1 )
	{
		__asm("nop") ;
	}
	
	return read_trims() ;
}
#endif // PCBSKY

#ifdef PCBSKY

uint32_t readTrainerSwitch( void )
{
	if ( PIOC->PIO_PDSR & 0x00000100 )
	{
		return 1 ;
	}
	return 0 ;
}

#endif // PCBSKY

#ifdef PCBX9D
void init_keys()
{
#ifdef PCBX9LITE

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( PIN_BUTTON_MENU | PIN_BUTTON_EXIT | PIN_BUTTON_PAGE | PIN_BUTTON_ENCODER, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
#else // X3
#ifdef PCBX7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
 #ifdef PCBT12
	configure_pins( 0x008C, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
	configure_pins( 0x0E00, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
 #else
	configure_pins( 0x008C, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
	configure_pins( 0x0400, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
 #endif
#else // PCBX7
#ifdef REV9E	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN ; 		// Enable portF clock
	configure_pins( PIN_BUTTON_MENU | PIN_BUTTON_EXIT | PIN_BUTTON_PAGE, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
	configure_pins( PIN_BUTTON_ENCODER, PIN_INPUT | PIN_PULLUP | PIN_PORTF ) ;
#else
#ifdef PCBXLITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( 0x7D80, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
#else // PCBXLITE
// Buttons PE10, 11, 12, PD2, 3, 7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( 0x3C00, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
	configure_pins( 0x008C, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
#endif // PCBXLITE
#endif
#endif // PCBX7
#endif // PCBX9LITE
}

// Reqd. bit 6 LEFT, 5 RIGHT, 4 UP, 3 DOWN 2 EXIT 1 MENU
uint32_t read_keys()
{
	register uint32_t x ;
	register uint32_t y ;

#ifdef PCBXLITE
	x = GPIOE->IDR ; // 10 RIGHT(+), 11 LEFT(-), 12 ENT(DOWN)
	y = 0 ;
	
	// Still to handle the enter button
	if ( x & PIN_BUTTON_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & PIN_BUTTON_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	if ( x & PIN_BUTTON_LEFT )
	{
		y |= 0x02 << KEY_LEFT ;		// LEFT
	}
	if ( x & PIN_BUTTON_RIGHT )
	{
		y |= 0x02 << KEY_RIGHT ;		// RIGHT
	}
	if ( x & PIN_BUTTON_UP )
	{
		y |= 0x02 << KEY_UP ;			// up
	}
	if ( x & PIN_BUTTON_DOWN )
	{
		y |= 0x02 << KEY_DOWN ;		// DOWN
	}
#else // PCBXLITE
	x = GPIOD->IDR ; // 7 MENU, 3 PAGE(UP), 2 EXIT
	y = 0 ;
	
#ifdef PCBX7
 #ifdef PCBT12
	if ( x & PIN_BUTTON_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	if ( x & PIN_BUTTON_RIGHT )
	{
		y |= 0x02 << KEY_RIGHT ;		// RIGHT
	}
	if ( x & PIN_BUTTON_LEFT )
	{
		y |= 0x02 << KEY_LEFT ;		// RIGHT
	}
	x = GPIOE->IDR ;
	if ( x & PIN_BUTTON_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & PIN_BUTTON_UP )
	{
		y |= 0x02 << KEY_UP ;			// up
	}
	if ( x & PIN_BUTTON_DOWN )
	{
		y |= 0x02 << KEY_DOWN ;		// DOWN
	}
 #else // T12
	if ( x & PIN_BUTTON_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & PIN_BUTTON_PAGE )
	{
		y |= 0x02 << KEY_PAGE ;		// LEFT
	}
	if ( x & PIN_BUTTON_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	x = GPIOE->IDR ;
	if ( x & PIN_BUTTON_ENCODER )
	{
		y |= 0x02 << KEY_ENTER ;	// RIGHT
	}
	y |= (0x02 << KEY_PLUS) | (0x02 << KEY_MINUS) ;
 #endif // T12
#else // X7
	
#ifdef PCBX9LITE
	x = GPIOE->IDR ; // 10 RIGHT(+), 11 LEFT(-), 12 ENT(DOWN)
	y = 0 ;
	
	if ( x & PIN_BUTTON_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & PIN_BUTTON_PAGE )
	{
		y |= 0x02 << KEY_PAGE ;		// LEFT
	}
	if ( x & PIN_BUTTON_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	if ( x & PIN_BUTTON_ENCODER )
	{
		y |= 0x02 << KEY_ENTER ;	// RIGHT
	}
	y |= 0x02 << KEY_PLUS ;			// up
	y |= 0x02 << KEY_MINUS ;		// DOWN
#else // PCBX9LITE
	if ( x & PIN_BUTTON_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// up
//		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & PIN_BUTTON_PAGE )
	{
		y |= 0x02 << KEY_PAGE ;		// LEFT
	}
	if ( x & PIN_BUTTON_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;		// DOWN
//		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	
#ifdef REV9E	
	x = GPIOF->IDR ;
	if ( x & PIN_BUTTON_ENCODER )
	{
		y |= 0x02 << KEY_ENTER ;			// EXIT
	}
	y |= (0x02 << KEY_PLUS) | (0x02 << KEY_MINUS) ;
#else // 9E
	x = GPIOE->IDR ; // 10 RIGHT(+), 11 LEFT(-), 12 ENT(DOWN)
	if ( x & PIN_BUTTON_PLUS )
	{
		y |= 0x02 << KEY_PLUS ;			// MENU
//		y |= 0x02 << KEY_UP ;			// up
	}
	if ( x & PIN_BUTTON_MINUS )
	{
		y |= 0x02 << KEY_MINUS ;		// RIGHT
//		y |= 0x02 << KEY_DOWN ;		// DOWN
	}
	if ( x & PIN_BUTTON_ENTER )
	{
		y |= 0x02 << KEY_ENTER ;			// EXIT
//		y |= 0x02 << KEY_RIGHT ;		// RIGHT
	}
#endif // 9E
#endif // PCBX9LITE
#endif // PCBX7
#endif // PCBXLITE
	return y ;
}

uint32_t readTrainerSwitch( void )
{
#ifdef PCBX9LITE
	if ( GPIOA->IDR & PIN_SW_H )
#else // X3
#ifdef PCBX7
	if ( GPIOD->IDR & PIN_SW_H )
#else // PCBX7
#ifdef REVPLUS
 #ifdef REV9E
	if ( (GPIOF->IDR & GPIO_Pin_1) == 0 )
 #else
	if ( GPIOD->IDR & PIN_SW_H )
 #endif
#else
#ifdef PCBXLITE
	if ( GPIOE->IDR & PIN_BUTTON_SHIFT )
#else
	if ( GPIOE->IDR & PIN_SW_H )
#endif
#endif
#endif // PCBX7
#endif // PCBX9LITE
	{
		return 0 ;
	}
	return 1 ;
}

// 
void setup_switches()
{
#ifdef PCBX9LITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( PIN_SW_H, PIN_INPUT | PIN_PULLUP | PIN_PORTA ) ;
	configure_pins( PIN_SW_B_L | PIN_SW_B_H, PIN_INPUT | PIN_PULLUP | PIN_PORTB ) ;
	configure_pins( PIN_SW_F, PIN_INPUT | PIN_PULLUP | PIN_PORTC ) ;
	configure_pins( PIN_SW_A_L | PIN_SW_A_H | PIN_SW_C_L | PIN_SW_C_H, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
#else // X3
#ifdef PCBX7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( 0x0020, PIN_INPUT | PIN_PULLUP | PIN_PORTA ) ;
	configure_pins( PIN_SW_H | PIN_SW_C_L, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
	configure_pins( 0xE087, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
	 
#else // PCBX7
#ifdef PCBXLITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( 0x000F, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
	configure_pins( 0x0060, PIN_INPUT | PIN_PULLUP | PIN_PORTA ) ;
	configure_pins( 0x0030, PIN_INPUT | PIN_PULLUP | PIN_PORTB ) ;
#else // PCBXLITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
#ifdef REVPLUS
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portE clock
#endif
#ifdef REV9E
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN ; 		// Enable portF clock
#endif
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	configure_pins( 0x0020, PIN_INPUT | PIN_PULLUP | PIN_PORTA ) ;
#ifdef REVPLUS
	configure_pins( 0x003A, PIN_INPUT | PIN_PULLUP | PIN_PORTB ) ;
#else
	configure_pins( 0x0038, PIN_INPUT | PIN_PULLUP | PIN_PORTB ) ;
#endif
	configure_pins( 0xE307, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;
#ifdef REVPLUS
 #ifdef REV9E
 	configure_pins( GPIO_Pin_1, PIN_INPUT | PIN_PULLUP | PIN_PORTF) ;
 #else
 	configure_pins( PIN_SW_H, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
 #endif
#endif
#endif // PCBXLITE
#endif // PCBX7
#endif // X3
	
}

#endif  // PCBX9D

#ifdef PCB9XT
uint32_t read_keys()
{
	checkM64() ;
	return ~M64Buttons ;
}

uint32_t readTrainerSwitch( void )
{
	checkM64() ;
	if ( M64Switches & 0x0100 )
	{
		return 1 ;
	}
	return 0 ;
}


#endif // PCB9XT

#ifdef PCBX12D

void init_keys()
{
	configure_pins( KEYS_GPIO_PIN_MENU | KEYS_GPIO_PIN_RIGHT, PIN_INPUT | PIN_PULLUP | PIN_PORTC ) ;
	configure_pins( KEYS_GPIO_PIN_EXIT | KEYS_GPIO_PIN_LEFT | KEYS_GPIO_PIN_DOWN, PIN_INPUT | PIN_PULLUP | PIN_PORTI ) ;
	configure_pins( KEYS_GPIO_PIN_UP, PIN_INPUT | PIN_PULLUP | PIN_PORTG ) ;

// Switch H 
	configure_pins( SWITCHES_GPIO_PIN_H, PIN_INPUT | PIN_PULLUP | PIN_PORTG ) ;
}

// Reqd. bit 6 LEFT, 5 RIGHT, 4 UP, 3 DOWN 2 EXIT 1 MENU
uint32_t read_keys()
{
	register uint32_t x ;
	register uint32_t y ;

	x = GPIOC->IDR ;
	y = 0 ;
	if ( x & KEYS_GPIO_PIN_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & KEYS_GPIO_PIN_RIGHT )
	{
		y |= 0x02 << KEY_PAGE ;	// RIGHT
	}
	x = GPIOI->IDR ;
	if ( x & KEYS_GPIO_PIN_LEFT )
	{
		y |= 0x02 << KEY_ENTER ;		// LEFT
	}
	if ( x & KEYS_GPIO_PIN_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	if ( x & KEYS_GPIO_PIN_DOWN )
	{
		y |= 0x02 << KEY_MINUS ;		// DOWN
	}
	x = GPIOG->IDR ;
	if ( x & KEYS_GPIO_PIN_UP )
	{
		y |= 0x02 << KEY_PLUS ;			// up
	}
	return y ;
}

uint32_t readTrainerSwitch( void )
{
	if ( SWITCHES_GPIO_REG_H & SWITCHES_GPIO_PIN_H )
	{
		return 0 ;
	}
	return 1 ;
}


#endif


#ifdef PCBX10

void init_keys()
{
 #if defined(PCBTX16S)
 	configure_pins( KEYS_GPIO_PIN_MENU | KEYS_GPIO_PIN_UP | KEYS_GPIO_PIN_EXIT | KEYS_GPIO_PIN_PGDN | KEYS_GPIO_PIN_DOWN | KEYS_GPIO_PIN_PGUP, PIN_INPUT | PIN_PULLUP | PIN_PORTI ) ;
 	configure_pins( KEYS_GPIO_PIN_PGUP, PIN_INPUT | PIN_PULLUP | PIN_PORTC ) ;
 #else	
	configure_pins( KEYS_GPIO_PIN_MENU | KEYS_GPIO_PIN_UP | KEYS_GPIO_PIN_EXIT | KEYS_GPIO_PIN_PGDN | KEYS_GPIO_PIN_DOWN, PIN_INPUT | PIN_PULLUP | PIN_PORTI ) ;
 #endif

// Switch H 
	configure_pins( SWITCHES_GPIO_PIN_H, PIN_INPUT | PIN_PULLUP | PIN_PORTG ) ;
}

// Reqd. bit 6 LEFT, 5 RIGHT, 4 UP, 3 DOWN 2 EXIT 1 MENU
uint32_t read_keys()
{
	register uint32_t x ;
	register uint32_t y ;

 #if defined(PCBTX16S)
 	x = GPIOI->IDR ;
	if ( x & KEYS_GPIO_PIN_MENU )	// SYS
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & KEYS_GPIO_PIN_EXIT )	// TELE
	{
		y |= 0x02 << KEY_ENTER ;		// DOWN
	}
	if ( x & KEYS_GPIO_PIN_DOWN )	// RTN
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	if ( x & KEYS_GPIO_PIN_UP )	// MDL
	{
		y |= 0x02 << KEY_PLUS ;		// RIGHT
	}

	if ( x & KEYS_GPIO_PIN_PGDN )	// PAGE>
	{
		y |= 0x02 << KEY_PAGE ;			// up
	}
	if ( KEYS_GPIO_REG_PGUP & KEYS_GPIO_PIN_PGUP )	// PAGE<
	{
		y |= 0x02 << KEY_MINUS ;		// LEFT
	}
 #else
	x = GPIOI->IDR ;
	y = 0 ;
	if ( x & KEYS_GPIO_PIN_MENU )
	{
		y |= 0x02 << KEY_MENU ;			// MENU
	}
	if ( x & KEYS_GPIO_PIN_EXIT )
	{
		y |= 0x02 << KEY_EXIT ;			// EXIT
	}
	if ( x & KEYS_GPIO_PIN_DOWN )
	{
		y |= 0x02 << KEY_MINUS ;		// DOWN
	}
	if ( x & KEYS_GPIO_PIN_UP )
	{
		y |= 0x02 << KEY_PLUS ;			// up
	}
	if ( x & KEYS_GPIO_PIN_PGDN )
	{
		y |= 0x02 << KEY_PAGE ;		// LEFT
	}
	y |= 0x02 << KEY_ENTER ;	// RIGHT
 #endif
	return y ;
}

uint32_t readTrainerSwitch( void )
{
	if ( SWITCHES_GPIO_REG_H & SWITCHES_GPIO_PIN_H )
	{
		return 0 ;
	}
	return 1 ;
}


#endif

