/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
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

#include <stdint.h>
#ifdef PCBSKY 
#include "AT91SAM3S4.h"
#endif


#if ( defined(PCBX9D) || defined(PCB9XT) )
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "hal.h"
#endif

#include "radio.h"
#include "logicio.h"

#if (defined(REV9E) || defined(PCBX7) || defined(PCBXLITE))
#define POWER_STATE_OFF				0
#define POWER_STATE_START			1
#define POWER_STATE_RUNNING		2
#define POWER_STATE_STOPPING	3
#define POWER_STATE_STOPPED		4

uint8_t PowerState = POWER_STATE_OFF ;

#ifdef PCBT12
uint8_t PowerCount ;
#endif

#endif

#ifdef PCBSKY 


void init_soft_power()
{
	// Configure RF_power (PC17)
	configure_pins( PIO_PC17, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_NO_PULLUP | PIN_PULLDOWN ) ;
	
	configure_pins( PIO_PA8, PIN_ENABLE | PIN_INPUT | PIN_PORTA | PIN_PULLUP ) ;
}


// Returns zero if power is switched off
//  1 if power switch is on
//  2 if power switch off, trainer power on
uint32_t check_soft_power()
{
	if ( PIOC->PIO_PDSR & PIO_PC17 )		// Power on
	{
		return POWER_ON ;
	}
	return POWER_OFF ;	
}


// turn off soft power
void soft_power_off()
{
	configure_pins( PIO_PA8, PIN_ENABLE | PIN_OUTPUT | PIN_LOW | PIN_PORTA | PIN_NO_PULLUP ) ;
}

#endif

#ifdef PCBX9D

void soft_power_off()
{
  GPIO_ResetBits(GPIOPWR,PIN_MCU_PWR);
}

uint32_t check_soft_power()
{
#if (defined(REV9E) || defined(PCBX7) || defined(PCBXLITE))
	uint32_t switchValue ;
#endif
  
#if (defined(REV9E) || defined(PCBX7) || defined(PCBXLITE))
#if defined(PCBXLITE)
	switchValue = GPIO_ReadInputDataBit(GPIOPWRSENSE, PIN_PWR_STATUS) == Bit_RESET ;
#else
	switchValue = GPIO_ReadInputDataBit(GPIOPWR, PIN_PWR_STATUS) == Bit_RESET ;
#endif
	switch ( PowerState )
	{
		case POWER_STATE_OFF :
		default :
			PowerState = POWER_STATE_START ;
#ifdef PCBT12
			PowerCount = 0 ;
#endif
   		return POWER_ON ;
		break ;
			
		case POWER_STATE_START :
			if ( !switchValue )
			{
				PowerState = POWER_STATE_RUNNING ;
			}
   		return POWER_ON ;
		break ;

		case POWER_STATE_RUNNING :
			if ( switchValue )
			{
#ifdef PCBT12
				if ( ++PowerCount > 20 )
				{
					PowerState = POWER_STATE_STOPPING ;
   				return POWER_X9E_STOP ;
				}
				else
				{
   				return POWER_ON ;
				}
#endif
				PowerState = POWER_STATE_STOPPING ;
   			return POWER_X9E_STOP ;
			}
#ifdef PCBT12
			else
			{
				PowerCount = 0 ;
			}
#endif
   		return POWER_ON ;
		break ;

		case POWER_STATE_STOPPING :
			if ( !switchValue )
			{
				PowerState = POWER_STATE_STOPPED ;
			}
 			return POWER_OFF ;
		break ;

		case POWER_STATE_STOPPED :
 			return POWER_OFF ;
		break ;
	}

#else // REV9E
	if (GPIO_ReadInputDataBit(GPIOPWR, PIN_PWR_STATUS) == Bit_RESET)
    return POWER_ON;
  else
    return POWER_OFF;
#endif
}

void init_soft_power()
{
#if defined(PCBXLITE)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN ; 		// Enable portE clock
	
	GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR );
	GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR);
	GPIO_ResetBits(GPIOPWRSPORT, PIN_SPORT_PWR);

	configure_pins( PIN_INT_RF_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
	configure_pins( PIN_EXT_RF_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
	configure_pins( PIN_MCU_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTE ) ;

	configure_pins( PIN_PWR_STATUS, PIN_INPUT | PIN_PORTA ) ;

#else	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
#ifdef REVPLUS
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portD clock
#endif
#ifdef PCBX7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
#endif
	GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR );
	GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR);
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOPWR, ENABLE);

  /* GPIO  Configuration*/
#ifdef REVPLUS
	configure_pins( PIN_INT_RF_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTC ) ;
	configure_pins( PIN_EXT_RF_PWR | PIN_MCU_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
#else
 #ifdef PCBX7
	configure_pins( PIN_INT_RF_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTC ) ;
	configure_pins( PIN_EXT_RF_PWR | PIN_MCU_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
 #else
	configure_pins( PIN_INT_RF_PWR | PIN_EXT_RF_PWR | PIN_MCU_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
 #endif
#endif
	configure_pins( PIN_PWR_STATUS, PIN_INPUT | PIN_PULLUP | PIN_PORTD ) ;
	
	configure_pins( PIN_TRNDET, PIN_INPUT | PIN_PULLUP | PIN_PORTA ) ;
  
  // Soft power ON

#endif
	
	GPIO_SetBits(GPIOPWR,PIN_MCU_PWR);
#if (defined(REV9E) || defined(PCBX7) || defined(PCBXLITE))
	PowerState = POWER_STATE_START ;
#endif

}

#endif

#ifdef PCB9XT
void soft_power_off()
{
	configure_pins( PIN_MCU_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTC ) ;
  GPIO_ResetBits(GPIOPWR,PIN_MCU_PWR) ;
}


uint32_t check_soft_power()
{
	static uint32_t c1 = 0 ;
//	static uint32_t c2 = 0 ;
	uint16_t value = GPIOC->IDR ;
  if ( value & PIN_PWR_STATUS )
	{
		c1 = 0 ;
    return POWER_ON ;
	}
  else
	{
		c1 += 1 ;
		if ( c1 > 50 )
		{
    	return POWER_OFF;
		}
    return POWER_ON ;
	}
}

void init_soft_power()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	
	GPIO_ResetBits(GPIOPWRINT, PIN_INT_RF_PWR );
	GPIO_ResetBits(GPIOPWREXT, PIN_EXT_RF_PWR);

  /* GPIO  Configuration*/
	configure_pins( PIN_INT_RF_PWR | PIN_MCU_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTC ) ;
	configure_pins( PIN_EXT_RF_PWR, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;

	configure_pins( PIN_PWR_STATUS, PIN_INPUT | PIN_PORTC ) ;
	
	configure_pins( PIN_MCU_PWR, PIN_INPUT | PIN_PULLUP | PIN_PORTC ) ;
}

#endif

