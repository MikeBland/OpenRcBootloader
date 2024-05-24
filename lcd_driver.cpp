/**
  ******************************************************************************
  * @file    Project/lcd/lcd.c 
  * @author  X9D Application Team
  * @Hardware version V0.2
  * @date    11-July-2012
  * @brief   This file provides LCD Init and botom drivers.
  * *
  ******************************************************************************
*/

#include "radio.h"

#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "hal.h"
#include "aspi.h"

#include "timers.h"
#include "lcd.h"
#include "logicio.h"

#define	WriteData(x)	 AspiData(x)
#define	WriteCommand(x)	 AspiCmd(x)

#if defined(PCBX7) || defined(PCBXLITE) || defined(PCBX9LITE)

#define LCD_CONTRAST_OFFSET            20
#define RESET_WAIT_DELAY_MS            300 // Wait time after LCD reset before first command
#define WAIT_FOR_DMA_END()             do { } while (lcd_busy)


#define LCD_W	 128
#define IS_LCD_RESET_NEEDED()          true

bool lcdInitFinished = false;
void lcdInitFinish();

extern uint8_t DisplayBuf[] ;

void delay_ms(uint32_t ms)
{
  while (ms--) {
		hw_delay( 10000 ) ; // units of 0.1uS
  }
}

void lcdWriteCommand(uint8_t byte)
{
  LCD_A0_LOW();
  LCD_NCS_LOW();
  while ((SPI3->SR & SPI_SR_TXE) == 0) {
    // Wait
  }
  (void)SPI3->DR; // Clear receive
  LCD_SPI->DR = byte;
  while ((SPI3->SR & SPI_SR_RXNE) == 0) {
    // Wait
  }
  LCD_NCS_HIGH();
}

void lcdHardwareInit()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN ;
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN ;    // Enable clock
	
  GPIO_InitTypeDef GPIO_InitStructure;

  // APB1 clock / 2 = 133nS per clock
  LCD_SPI->CR1 = 0; // Clear any mode error
  LCD_SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA;
  LCD_SPI->CR2 = 0;
  LCD_SPI->CR1 |= SPI_CR1_MSTR;	// Make sure in case SSM/SSI needed to be set first
  LCD_SPI->CR1 |= SPI_CR1_SPE;
  
  LCD_NCS_HIGH();
  
  GPIO_InitStructure.GPIO_Pin = LCD_NCS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_NCS_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_RST_GPIO_PIN;
  GPIO_Init(LCD_RST_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_A0_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LCD_SPI_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_CLK_GPIO_PIN | LCD_MOSI_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(LCD_SPI_GPIO, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(LCD_SPI_GPIO, LCD_MOSI_GPIO_PinSource, LCD_GPIO_AF);
  GPIO_PinAFConfig(LCD_SPI_GPIO, LCD_CLK_GPIO_PinSource, LCD_GPIO_AF);

  LDC_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA
  LCD_DMA->HIFCR = LCD_DMA_FLAGS; // Write ones to clear bits
  LDC_DMA_Stream->CR =  DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  LDC_DMA_Stream->PAR = (uint32_t)&LCD_SPI->DR;

  LDC_DMA_Stream->NDTR = LCD_W;

  LDC_DMA_Stream->FCR = 0x05; // DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;

  NVIC_EnableIRQ(LCD_DMA_Stream_IRQn);
}

void lcdStart()
{
  lcdWriteCommand(0xe2); // (14) Soft reset
#ifdef PCBT12
	lcdWriteCommand(0xA0); // Set seg
  lcdWriteCommand(0xC8); // Set com
#else  
  lcdWriteCommand(0xa1); // Set seg
  lcdWriteCommand(0xc0); // Set com
#endif
  lcdWriteCommand(0xf8); // Set booster
  lcdWriteCommand(0x00); // 5x
  lcdWriteCommand(0xa3); // Set bias=1/6
  lcdWriteCommand(0x22); // Set internal rb/ra=5.0
  lcdWriteCommand(0x2f); // All built-in power circuits on
  lcdWriteCommand(0x81); // Set contrast
#ifdef PCBXLITE
  lcdWriteCommand(0x30); // Set Vop
#else
#ifdef PCBT12
  lcdWriteCommand(0x2A); // Set Vop
#else  
  lcdWriteCommand(0x36); // Set Vop
#endif
#endif
  lcdWriteCommand(0xa6); // Set display mode
}

volatile bool lcd_busy;

void refreshDisplay()
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }

  uint8_t * p = DisplayBuf;
  for (uint8_t y=0; y < 8; y++, p+=LCD_W) {
    lcdWriteCommand(0x10); // Column addr 0
    lcdWriteCommand(0xB0 | y); // Page addr y
#ifdef PCBT12
    lcdWriteCommand(0x0);
#else
    lcdWriteCommand(0x04);
#endif    
    
    LCD_NCS_LOW();
    LCD_A0_HIGH();

    lcd_busy = true;
    LDC_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA
    LCD_DMA->HIFCR = LCD_DMA_FLAGS; // Write ones to clear bits
    LDC_DMA_Stream->M0AR = (uint32_t)p;
    LDC_DMA_Stream->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; // Enable DMA & TC interrupts
    LCD_SPI->CR2 |= SPI_CR2_TXDMAEN;
  
    WAIT_FOR_DMA_END();

    LCD_NCS_HIGH();
    LCD_A0_HIGH();
  }
}

extern "C" void LCD_DMA_Stream_IRQHandler()
{
//  DEBUG_INTERRUPT(INT_LCD);

  LDC_DMA_Stream->CR &= ~DMA_SxCR_TCIE; // Stop interrupt
  LCD_DMA->HIFCR |= LCD_DMA_FLAG_INT; // Clear interrupt flag
  LCD_SPI->CR2 &= ~SPI_CR2_TXDMAEN;
  LDC_DMA_Stream->CR &= ~DMA_SxCR_EN; // Disable DMA

  while (LCD_SPI->SR & SPI_SR_BSY) {
    /* Wait for SPI to finish sending data
    The DMA TX End interrupt comes two bytes before the end of SPI transmission,
    therefore we have to wait here.
    */
  }
  LCD_NCS_HIGH();
  lcd_busy = false;
}

/*
  Proper method for turning of LCD module. It must be used,
  otherwise we might damage LCD crystals in the long run!
*/
void lcdOff()
{
  WAIT_FOR_DMA_END();

  /*
  LCD Sleep mode is also good for draining capacitors and enables us
  to re-init LCD without any delay
  */
  lcdWriteCommand(0xAE); // LCD sleep
  delay_ms(3); // Wait for caps to drain
}

void lcdReset()
{
  LCD_RST_LOW();
  delay_ms(150);
  LCD_RST_HIGH();
}


static void backlightInit()
{
#ifdef PCBXLITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;    // Enable clock
	configure_pins( BACKLIGHT_GPIO_PIN, PIN_PERIPHERAL | PIN_PER_1 | PIN_PORTA | PIN_PUSHPULL | PIN_OS2 | PIN_NO_PULLUP ) ;
  TIM1->ARR = 100;
  TIM1->PSC = (Peri1_frequency*Timer_mult1) / 10000 - 1 ;
  TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM
  TIM1->CCER = TIM_CCER_CC1E;
  TIM1->CCR1 = 100;
	TIM1->BDTR |= TIM_BDTR_MOE ;
  TIM1->EGR = 0;
  TIM1->CR1 = TIM_CR1_CEN; // Counter enable

#else	
 #ifdef PCBX9LITE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;    // Enable clock
	configure_pins( BACKLIGHT_GPIO_PIN, PIN_PERIPHERAL | PIN_PER_1 | PIN_PORTA | PIN_PUSHPULL | PIN_OS2 | PIN_NO_PULLUP ) ;
  TIM1->ARR = 100;
  TIM1->PSC = (Peri1_frequency*Timer_mult1) / 10000 - 1 ;
  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM
  TIM1->CCER = TIM_CCER_CC3E;
  TIM1->CCR1 = 100;
	TIM1->BDTR |= TIM_BDTR_MOE ;
  TIM1->EGR = 0;
  TIM1->CR1 = TIM_CR1_CEN; // Counter enable
 #else // X3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;    // Enable clock
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  TIM4->ARR = 100;
  TIM4->PSC = (Peri2_frequency*Timer_mult2) / 10000 - 1 ;
  TIM4->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM
  TIM4->CCER = TIM_CCER_CC2E;
  TIM4->CCR2 = 100;
  TIM4->EGR = 0;
  TIM4->CR1 = TIM_CR1_CEN; // Counter enable
 #endif // X3
#endif
}


/*
  Starts LCD initialization routine. It should be called as
  soon as possible after the reset because LCD takes a lot of
  time to properly power-on.

  Make sure that delay_ms() is functional before calling this function!
*/
void lcd_init()
{
  lcdHardwareInit();
	lcdInitFinished = false ;

  if (IS_LCD_RESET_NEEDED()) {
    lcdReset();
  }
	backlightInit() ;
	backlight_set( 50 ) ;
}

/*
  Finishes LCD initialization. It is called auto-magically when first LCD command is
  issued by the other parts of the code.
*/
void lcdInitFinish()
{
  lcdInitFinished = true;

  /*
    LCD needs longer time to initialize in low temperatures. The data-sheet
    mentions a time of at least 150 ms. The delay of 1300 ms was obtained
    experimentally. It was tested down to -10 deg Celsius.

    The longer initialization time seems to only be needed for regular Taranis,
    the Taranis Plus (9XE) has been tested to work without any problems at -18 deg Celsius.
    Therefore the delay for T+ is lower.
    
    If radio is reset by watchdog or boot-loader the wait is skipped, but the LCD
    is initialized in any case.

    This initialization is needed in case the user moved power switch to OFF and
    then immediately to ON position, because lcdOff() was called. In any case the LCD
    initialization (without reset) is also recommended by the data sheet.
  */

//  if (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
//#if !defined(BOOT)
//    while (g_tmr10ms < (RESET_WAIT_DELAY_MS/10)); // wait measured from the power-on
//#else
    delay_ms(RESET_WAIT_DELAY_MS);
//#endif
//  }
  
  lcdStart();
  lcdWriteCommand(0xAF); // dc2=1, IC into exit SLEEP MODE, dc3=1 gray=ON, dc4=1 Green Enhanc mode disabled
  delay_ms(20); // needed for internal DC-DC converter startup
}

void lcdSetRefVolt(uint8_t val)
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }

  lcdWriteCommand(0x81); // Set Vop
  lcdWriteCommand(val+LCD_CONTRAST_OFFSET); // 0-255
}


uint16_t BacklightBrightness ;



#ifdef PCBXLITE
void backlight_on()
{
	TIM1->CCR1 = 100 - BacklightBrightness ;
}

void backlight_off()
{
	TIM1->CCR1 = 0 ;
}

void backlight_set( uint16_t brightness )
{
	BacklightBrightness = brightness ;
	TIM1->CCR1 = 100 - BacklightBrightness ;
}
#else // PCBXLITE
 #ifdef PCBX9LITE
void backlight_on()
{
	TIM1->CCR3 = 100 - BacklightBrightness ;
}

void backlight_off()
{
	TIM1->CCR3 = 0 ;
}

void backlight_set( uint16_t brightness )
{
	BacklightBrightness = brightness ;
	TIM1->CCR3 = 100 - BacklightBrightness ;
}

 #else // X3

void backlight_on()
{
	TIM4->CCR2 = 100 - BacklightBrightness ;
}

void backlight_off()
{
	TIM4->CCR2 = 0 ;
}

void backlight_set( uint16_t brightness )
{
	BacklightBrightness = brightness ;
	TIM4->CCR2 = 100 - BacklightBrightness ;
}
 #endif // PCBX9LITE
#endif // PCBXLITE


#else // PCBX7

#ifdef REVPLUS
#define CONTRAST_OFS 120
#else
#define CONTRAST_OFS 12
#endif

#define __no_operation     __NOP


extern uint8_t DisplayBuf[] ;

void Set_Address(uint8_t x, uint8_t y)
{
  WriteCommand(x&0x0F);	//Set Column Address LSB CA[3:0]
  WriteCommand((x>>4)|0x10);	//Set Column Address MSB CA[7:4]
    
  WriteCommand((y&0x0F)|0x60);	//Set Row Address LSB RA [3:0]
  WriteCommand(((y>>4)&0x0F)|0x70);    //Set Row Address MSB RA [7:4]
}

void refreshDisplay()
{  
#ifdef REVPLUS
  for (uint32_t y=0; y<DISPLAY_H; y += 2)
#else
  for (uint32_t y=0; y<DISPLAY_H; y++)
 #endif
	{
    uint8_t *p = &DisplayBuf[(y>>3)*DISPLAY_W];
    uint8_t mask = (1 << (y%8));
		
#ifdef REVPLUS
		GPIO_TypeDef *gpiod = GPIOC ;
#else
		GPIO_TypeDef *gpiod = GPIOD ;
#endif
    
#ifdef REVPLUS
		Set_Address(0, y/2);
#else
		Set_Address(0, y);
#endif
    AspiCmd(0xAF);
    
		gpiod->BSRRL = PIN_LCD_CLK ;		// Clock high
		gpiod->BSRRL = PIN_LCD_A0 ;			// A0 high
#ifdef REVPLUS
    GPIOA->BSRRH = PIN_LCD_NCS ;		// CS low
#else
    gpiod->BSRRH = PIN_LCD_NCS ;		// CS low
#endif

#ifdef REVPLUS
		for (uint32_t x=0; x<DISPLAY_W; x += 1 )
#else
		for (uint32_t x=0; x<DISPLAY_W; x+=2)
#endif
		{
      uint32_t data ;
			data = 0 ;
#ifdef REVPLUS
			if ( p[x] & mask )
			{
				data = 0x0F ;
			}
			if (p[x] & (mask<<1) )
			{
				data += 0xF0 ;
			}	
#else
			if ( p[x] & mask )
			{
				data = 0xF0 ;
			}
			if (p[x+1] & mask )
			{
				data += 0x0F ;
			}	
#endif
			
        if(data&0x80)
        {
					gpiod->BSRRL = PIN_LCD_MOSI ;
        }
				else
				{
					gpiod->BSRRH = PIN_LCD_MOSI ;
				}
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x40)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x20)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x10)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x08)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x04)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x02)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x01)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*(uint32_t *)&gpiod->BSRRL = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low
				__no_operation() ;
				gpiod->BSRRL = PIN_LCD_CLK ;		// Clock high

		}
#ifdef REVPLUS
    GPIOA->BSRRL = PIN_LCD_NCS ;		// CS high
#else
    gpiod->BSRRL = PIN_LCD_NCS ;		// CS high
#endif
		gpiod->BSRRL = PIN_LCD_A0 ;
    WriteData(0);
  }
}

uint16_t BacklightBrightness ;

#ifdef REVPLUS

void backlight_w_on()
{
#ifdef REV9E
	TIM9->CCR1 = 100 - BacklightBrightness ;
#else
	TIM4->CCR2 = 100 - BacklightBrightness ;
#endif
}

void backlight_w_off()
{
#ifdef REV9E
	TIM9->CCR1 = 0 ;
#else
	TIM4->CCR2 = 0 ;
#endif
}

void backlight_on()
{
#ifdef REV9E
	TIM9->CCR2 = 100 - BacklightBrightness ;
#else
#ifdef PCBX7
	TIM4->CCR4 = 100 - BacklightBrightness ;
#else // PCBX7
	TIM4->CCR4 = 100 - BacklightBrightness ;
#endif // PCBX7
#endif
}

void backlight_off()
{
#ifdef REV9E
	TIM9->CCR2 = 0 ;
#else
	TIM4->CCR4 = 0 ;
#endif
}

void backlight_set( uint16_t brightness )
{
	BacklightBrightness = brightness ;
#ifdef REV9E
	TIM9->CCR1 = 100 - BacklightBrightness ;
	TIM9->CCR2 = 100 - BacklightBrightness ;
#else	
	TIM4->CCR2 = 100 - BacklightBrightness ;
	TIM4->CCR4 = 100 - BacklightBrightness ;
#endif
}

/**Init the Backlight GPIO */
static void LCD_BL_Config()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOBL, ENABLE);
  
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBL, &GPIO_InitStructure);

#ifdef REV9E
  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM9);
#else
  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM4);
#endif
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BLW;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBLW, &GPIO_InitStructure);

#ifdef REV9E
  GPIO_PinAFConfig(GPIOBLW, GPIO_PinSource_BLW ,GPIO_AF_TIM9);
#else
  GPIO_PinAFConfig(GPIOBLW, GPIO_PinSource_BLW ,GPIO_AF_TIM4);
#endif

#ifdef REV9E
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN ;    // Enable clock
	TIM9->ARR = 100 ;
	TIM9->PSC = (Peri1_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM9->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 ;	// PWM
	TIM9->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E ;
	
	BacklightBrightness = 40 ;
	TIM9->CCR1 = BacklightBrightness ;
	TIM9->CCR2 = BacklightBrightness ;
	TIM9->EGR = 0 ;
	TIM9->CR1 = TIM_CR1_CEN ;				// Counter enable
#else
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;    // Enable clock
	TIM4->ARR = 100 ;
	TIM4->PSC = (Peri1_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM4->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 ;	// PWM
	TIM4->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 ;	// PWM
	TIM4->CCER = TIM_CCER_CC4E | TIM_CCER_CC2E ;
	
	BacklightBrightness = 40 ;
	TIM4->CCR2 = BacklightBrightness ;
	TIM4->CCR4 = BacklightBrightness ;
	TIM4->EGR = 0 ;
	TIM4->CR1 = TIM_CR1_CEN ;				// Counter enable
#endif
}

/**Init the Backlight GPIO */
void Haptic_Config()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOHAPTIC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_HAPTIC;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOHAPTIC, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOHAPTIC, GPIO_PinSource_HAPTIC ,GPIO_AF_TIM10);

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN ;		// Enable clock
	TIM10->ARR = 100 ;
	TIM10->PSC = (Peri2_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM10->CCMR1 = 0x60 ;	// PWM
	TIM10->CCER = 1 ;	
	
	BacklightBrightness = 80 ;
	TIM10->CCR1 = BacklightBrightness ;
	TIM10->EGR = 0 ;
	TIM10->CR1 = 1 ;
}

#else

void backlight_on()
{
	TIM10->CCR1 = 100 - BacklightBrightness ;
}

void backlight_off()
{
	TIM10->CCR1 = 0 ;
}

void backlight_set( uint16_t brightness )
{
	BacklightBrightness = brightness ;
	TIM10->CCR1 = 100 - BacklightBrightness ;
}


/**Init the Backlight GPIO */
static void LCD_BL_Config()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOBL, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBL, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM10);

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN ;		// Enable clock
	TIM10->ARR = 100 ;
	TIM10->PSC = (Peri2_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM10->CCMR1 = 0x60 ;	// PWM
	TIM10->CCER = 1 ;	
	
	BacklightBrightness = 80 ;
	TIM10->CCR1 = BacklightBrightness ;
	TIM10->EGR = 0 ;
	TIM10->CR1 = 1 ;
}
#endif

/** Init the anolog spi gpio
*/
static void LCD_Hardware_Init()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD, ENABLE);
#ifdef REVPLUS
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_RST, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_NCS, ENABLE);
#endif  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /*!< Configure lcd CLK\ MOSI\ A0pin in output pushpull mode *************/
  GPIO_InitStructure.GPIO_Pin =PIN_LCD_MOSI | PIN_LCD_CLK | PIN_LCD_A0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_LCD, &GPIO_InitStructure);
  
  /*!< Configure lcd NCS pin in output pushpull mode ,PULLUP *************/
#ifdef REVPLUS
	GPIO_InitStructure.GPIO_Pin = PIN_LCD_NCS ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(GPIO_LCD_NCS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_LCD_RST ;
  GPIO_Init(GPIO_LCD_RST, &GPIO_InitStructure);

#else  
	GPIO_InitStructure.GPIO_Pin = PIN_LCD_NCS | PIN_LCD_RST ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(GPIO_LCD, &GPIO_InitStructure);
#endif  
}

#ifdef REVPLUS
static void LCD_Init()
{
  LCD_BL_Config() ;
  /*Hardware Reset need delay*/
  /*LCD_RST_LOW();
    Delay(50);    
    LCD_RST_HIGH();*/
    
  AspiCmd(0x25);   //(5) Temperature compensation curve definition: 0x25 = -0.05%/oC
	AspiCmd(0x2b);   //(6) Panel loading set ,Internal VLCD.
  AspiCmd(0xEA);	 //(27) set bias=1/10
  AspiCmd(0x81);	 //(11) Set Vop + next byte
  AspiCmd(25+CONTRAST_OFS);		//0--255
  AspiCmd(0xA6);	//inverse display off
	AspiCmd(0xA1);	//line rates,24 Klps
  AspiCmd(0x84);	//Disable Partial Display
  AspiCmd(0xC8);	//SET N-LINE INVERSION
  AspiCmd(0x00);	//Disable NIV
  AspiCmd(0xF1);	//Set CEN
  AspiCmd(0x3F);	// 1/64DUTY
  AspiCmd(0xC0);	//(21) Set mapping
  AspiCmd(0x04);	// MY=1, MX=0, MSF=0
  AspiCmd(0x89);	//(15) WA=1,column (CA) increment (+1) first until CA reaches CA boundary, then RA will increment by (+1).
  AspiCmd(0xF8);	//Set Window Program Enable  ,inside modle
  AspiCmd(0xD0);	 //(23) SET 4 bits/pixel, pattern 0
  AspiCmd(0xF4);   //starting column address of RAM program window.
  AspiCmd(0x00);
  AspiCmd(0xF5);   //starting row address of RAM program window.
  AspiCmd(0x00);
  AspiCmd(0xF6);   //ending column address of RAM program window.
  AspiCmd(0xD3);
  AspiCmd(0xF7);   //ending row address of RAM program window.
  AspiCmd(0x3F);
	AspiCmd(0xAF);	// Active and 16-grey scale
}
#else
static void LCD_Init()
{
  LCD_BL_Config() ;
  /*Hardware Reset need delay*/
  /*LCD_RST_LOW();
    Delay(50);    
    LCD_RST_HIGH();*/
    
  AspiCmd(0x25);   //Temperature compensation curve definition: 0x25 = -0.05%/oC
  AspiCmd(0x2b);   //Panel loading set ,Internal VLCD.
  AspiCmd(0xEA);	//set bias=1/10 :Command table NO.27
  AspiCmd(0x81);	//Set Vop
  AspiCmd(25+CONTRAST_OFS);		//0--255
  AspiCmd(0xA6);	//inverse display off
  AspiCmd(0xD1);	//SET RGB:Command table NO.21 .SET RGB or BGR.  D1=RGB
  AspiCmd(0xD5);	//set color mode 4K and 12bits  :Command table NO.22
  AspiCmd(0xA0);	//line rates,25.2 Klps
  AspiCmd(0xC8);	//SET N-LINE INVERSION
  AspiCmd(0x1D);	//Disable NIV
  AspiCmd(0xF1);	//Set CEN
  AspiCmd(0x3F);	// 1/64DUTY
  AspiCmd(0x84);	//Disable Partial Display
  AspiCmd(0xC4);	//MY=1,MX=0
  AspiCmd(0x89);	//WA=1,column (CA) increment (+1) first until CA reaches CA boundary, then RA will increment by (+1).

  AspiCmd(0xF8);	//Set Window Program Enable  ,inside modle
  AspiCmd(0xF4);   //starting column address of RAM program window.
  AspiCmd(0x00);
  AspiCmd(0xF5);   //starting row address of RAM program window.
  AspiCmd(0x60);
  AspiCmd(0xF6);   //ending column address of RAM program window.
  AspiCmd(0x47);
  AspiCmd(0xF7);   //ending row address of RAM program window.
  AspiCmd(0x9F);

  AspiCmd(0xAF);	//dc2=1,IC into exit SLEEP MODE,	 dc3=1  gray=ON 开灰阶	,dc4=1  Green Enhanc mode disabled	  绿色增强模式关
	
}
#endif

static void Delay(volatile unsigned int ms)
{
  volatile u8 i;
  while(ms != 0)
  {
    for(i=0;i<250;i++) {}
    for(i=0;i<75;i++) {}
    ms--;
  }
}

void lcd_init()
{
	GPIO_TypeDef *gpiod = GPIOD ;

  LCD_BL_Config();
  LCD_Hardware_Init();
  
	gpiod->BSRRL = PIN_LCD_RST ;		// RST high
  Delay(5);

  gpiod->BSRRH = PIN_LCD_RST ;		// RST low
  Delay(120); //11ms

	gpiod->BSRRL = PIN_LCD_RST ;		// RST high
  Delay(2500);
 
  AspiCmd(0xE2);
  Delay(2500);

  LCD_Init();
  Delay(120);
  AspiCmd(0xAF);	//dc2=1, IC into exit SLEEP MODE, dc3=1 gray=ON, dc4=1 Green Enhanc mode disabled
}

void lcdSetRefVolt(uint8_t val)
{
  AspiCmd(0x81);	//Set Vop
  AspiCmd(val+CONTRAST_OFS);		//0--255
}

#endif // PCBX7

