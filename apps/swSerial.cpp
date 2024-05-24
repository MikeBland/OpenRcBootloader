
#ifdef PCBSKY
#include "AT91SAM3S4.h"
#include "core_cm3.h"
#endif

#if ( defined(PCBX9D) || defined(PCB9XT) )
#include "hal.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"

#ifdef PCB9XT
#include "stm32f2xx_usart.h"
#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)
#endif

#endif

#if defined(PCBX12D) || defined(PCBX10)
#include "hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#endif

#include "radio.h"
#include "drivers.h"
#include "logicio.h"


//#if ( defined(PCBX9D) || defined(PCB9XT) )
//#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)
//#endif


#define BIT_TIME_9600		208
// 19200 baud, bit time 52.08
#define BIT_TIME_19200	104
// 38400 baud, bit time 26.04
#define BIT_TIME_38400	52
// 57600 baud, bit time 17.36uS
#define BIT_TIME_57600	35
// 100000 baud, bit time 10uS (SBUS)
#define BIT_TIME_100K		20
// 115200 baud, bit time 8.68uS
#define BIT_TIME_115K		17

// States in LineState
#define LINE_IDLE			0
#define LINE_ACTIVE		1

// States in BitState
#define BIT_IDLE			0
#define BIT_ACTIVE		1
#define BIT_FRAMING		2

struct t_softSerial
{
	uint32_t softwareComBit ;
	uint16_t bitTime ;
	uint16_t HtoLtime ;
	uint16_t LtoHtime ;
	uint16_t byte ;
	uint8_t lineState ;
	uint8_t captureMode ;
	uint8_t softSerInvert ;
	uint8_t bitState ;
	uint8_t bitCount ;
	uint8_t softSerialEvenParity ;
	uint8_t RxDisabled ;
	struct t_fifo128 *pfifo ;
} ;

struct t_fifo128 Com1_fifo ;
struct t_fifo128 Com2_fifo ;

struct t_softSerial SoftSerial1 ;
struct t_softSerial SoftSerial2 ;

void init_software_com1(uint32_t baudrate, uint32_t invert, uint32_t parity) ;
void disable_software_com1() ;

int32_t get_fifo128( struct t_fifo128 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x7F ;
		return rxbyte ;
	}
	return -1 ;
}

void put_fifo128( struct t_fifo128 *pfifo, uint8_t byte )
{
  uint32_t next = (pfifo->in + 1) & 0x7f;
	if ( next != pfifo->out )
	{
		pfifo->fifo[pfifo->in] = byte ;
		pfifo->in = next ;
	}
}

// time in units of 0.5uS, value is 1 or 0
void putCaptureTime( struct t_softSerial *pss, uint16_t time, uint32_t value )
{
	time += pss->bitTime/2 ;
	time /= pss->bitTime ;		// Now number of bits
	if ( value == 3 )
	{
		return ;
	}

	if ( pss->bitState == BIT_IDLE )
	{ // Starting, value should be 0
		pss->bitState = BIT_ACTIVE ;
		pss->bitCount = 0 ;
		if ( time > 1 )
		{
			pss->byte >>= time-1 ;
			pss->bitCount = time-1 ;
		}
	}
	else
	{
		if ( value )
		{
			uint32_t len = pss->softSerialEvenParity ? 9 : 8 ;
			while ( time )
			{
				if ( pss->bitCount >= len )
				{ // Got a byte
					if ( len == 9 )
					{
						// check parity (even)
						uint32_t parity = pss->byte ;
						parity ^= parity >> 4 ;
						parity ^= parity >> 2 ;
						parity ^= parity >> 1 ;
						parity ^= pss->byte >> 8 ;
						if ( ( parity & 1 ) == 0 )
						{
							if ( pss->pfifo )
							{
								put_fifo128( pss->pfifo, pss->byte ) ;
							}
						}
					}
					else
					{
						if ( pss->pfifo )
						{
							put_fifo128( pss->pfifo, pss->byte ) ;
						}
					}
					pss->bitState = BIT_IDLE ;
					time = 0 ;
					break ;
				}
				else
				{
					pss->byte >>= 1 ;
					pss->byte |= ( len == 9 ) ? 0x100 : 0x80 ;
					time -= 1 ;
					pss->bitCount += 1 ;
				}
			}
		}
		else
		{
			pss->byte >>= time ;
			pss->bitCount += time ;
		}
	}
}

#ifdef PCBSKY

void start_timer5()
{
#ifndef SIMU
  register Tc *ptc ;
//	register Pio *pioptr ;

	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x10000000L ;		// Enable peripheral clock to TC5

  ptc = TC1 ;		// Tc block 1 (TC3-5)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0x32 ;
	ptc->TC_CHANNEL[2].TC_CMR = 0x00000000 ;	// Capture mode
	ptc->TC_CHANNEL[2].TC_CMR = 0x00090007 ;	// 0000 0000 0000 1001 0000 0000 0000 0101, XC2, A rise, B fall
	ptc->TC_CHANNEL[2].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
	NVIC_SetPriority( TC5_IRQn, 14 ) ; // Low priority interrupt
	NVIC_EnableIRQ(TC5_IRQn) ;
#endif
}

void stop_timer5()
{
	TC1->TC_CHANNEL[2].TC_CCR = 2 ;		// Disable clock
	NVIC_DisableIRQ(TC5_IRQn) ;
}


void init_software_com(uint32_t baudrate, uint32_t invert, uint32_t parity, uint32_t port )
{
	struct t_softSerial *pss = &SoftSerial1 ;
	if ( port == 0 )
	{
		pss->softwareComBit = PIO_PA5 ;
		pss->pfifo = &Com1_fifo ;
	}
	else
	{
		pss->softwareComBit = PIO_PA9 ;
		pss->pfifo = &Com2_fifo ;
	}
	pss->bitTime = 2000000 / baudrate ;

	pss->softSerInvert = invert ? 0 : pss->softwareComBit ;
	pss->softSerialEvenParity = parity ? 1 : 0 ;

	TC1->TC_CHANNEL[0].TC_IDR = TC_IDR0_LDRAS ;		// No int on rising edge
	TC1->TC_CHANNEL[0].TC_IDR = TC_IDR0_LDRBS ;		// No int on falling edge
	TC1->TC_CHANNEL[0].TC_IDR = TC_IDR0_CPCS ;		// No compare interrupt
	pss->lineState = LINE_IDLE ;
	configure_pins( pss->softwareComBit, PIN_ENABLE | PIN_INPUT | PIN_PORTA ) ;
	PIOA->PIO_IER = pss->softwareComBit ;
	NVIC_SetPriority( PIOA_IRQn, 0 ) ; // Highest priority interrupt
	NVIC_EnableIRQ(PIOA_IRQn) ;
	start_timer5() ;
}

// Handle software serial on COM1 input (for non-inverted input)
void init_software_com1(uint32_t baudrate, uint32_t invert, uint32_t parity)
{
	init_software_com(baudrate, invert, parity, 0 ) ;
}

static void disable_software_com( uint32_t bit)
{
	stop_timer5() ;
	PIOA->PIO_IDR = bit ;
	NVIC_DisableIRQ(PIOA_IRQn) ;
}

void disable_software_com1()
{
	disable_software_com( PIO_PA5 ) ;
}

void disable_com1_Rx()
{
	struct t_softSerial *pss = &SoftSerial1 ;
	pss->RxDisabled = 1 ;
}

void enable_com1_Rx()
{
	struct t_softSerial *pss = &SoftSerial1 ;
	pss->RxDisabled = 0 ;
}


void init_software_com2(uint32_t baudrate, uint32_t invert, uint32_t parity)
{
	init_software_com(baudrate, invert, parity, 1 ) ;
}

void disable_software_com2()
{
	disable_software_com( PIO_PA9 ) ;
}

extern "C" void PIOA_IRQHandler()
{
  register uint32_t capture ;
  register uint32_t dummy ;
	
	capture =  TC1->TC_CHANNEL[0].TC_CV ;	// Capture time

	struct t_softSerial *pss = &SoftSerial1 ;
	dummy = PIOA->PIO_ISR ;			// Read and clear status register
	(void) dummy ;		// Discard value - prevents compiler warning

	dummy = PIOA->PIO_PDSR ;
	if ( pss->RxDisabled )
	{
		return ;
	}

	if ( ( dummy & pss->softwareComBit ) == pss->softSerInvert )
	{
		// L to H transition
		pss->LtoHtime = capture ;
		TC1->TC_CHANNEL[2].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
		TC1->TC_CHANNEL[2].TC_RC = pss->bitTime * 10 ;
		uint32_t time ;
		capture -= pss->HtoLtime ;
		time = capture ;
		putCaptureTime( pss, time, 0 ) ;
		TC1->TC_CHANNEL[2].TC_IER = TC_IER0_CPCS ;		// Compare interrupt
		(void) TC1->TC_CHANNEL[2].TC_SR ;
	}
	else
	{
		// H to L transition
		pss->HtoLtime = capture ;
		if ( pss->lineState == LINE_IDLE )
		{
			pss->lineState = LINE_ACTIVE ;
//			putCaptureTime( pss, 0, 3 ) ;
			TC1->TC_CHANNEL[2].TC_RC = capture + (pss->bitTime * 20) ;
			(void) TC1->TC_CHANNEL[2].TC_SR ;
		}
		else
		{
			uint32_t time ;
			capture -= pss->LtoHtime ;
			time = capture ;
			putCaptureTime( pss, time, 1 ) ;
		}
		TC1->TC_CHANNEL[2].TC_IDR = TC_IDR0_CPCS ;		// No compare interrupt
	}
}

extern "C" void TC5_IRQHandler()
{
	uint32_t status ;
	struct t_softSerial *pss = &SoftSerial1 ;

	status = TC1->TC_CHANNEL[2].TC_SR ;
	if ( status & TC_SR0_CPCS )
	{		
		uint32_t time ;
		time = TC1->TC_CHANNEL[0].TC_CV - pss->LtoHtime ;
		putCaptureTime( pss, time, 2 ) ;
		pss->lineState = LINE_IDLE ;
		TC1->TC_CHANNEL[2].TC_IDR = TC_IDR0_CPCS ;		// No compare interrupt
	}
}


#endif

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D) || defined(PCBX10)

void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity )
{
	// Serial configure  
//	if ( invert )
//	{
//	  NVIC_DisableIRQ(USART2_IRQn) ;
		init_software_com1( baudRate, invert, parity ) ;
//		return ;
//	}
//	disable_software_com1() ;

//	RCC->APB1ENR |= RCC_APB1ENR_USART2EN ;		// Enable clock
//#ifdef PCB9XT
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
//	GPIOB->BSRRH = 0x0004 ;		// output disable
//#else
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
//	GPIOD->BSRRH = PIN_SPORT_ON ;		// output disable
//#endif
//#ifdef PCB9XT
//// PB2 as SPort enable
//	configure_pins( 0x00000004, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTB ) ;
//	configure_pins( 0x00000008, PIN_PERIPHERAL | PIN_PUSHPULL | PIN_OS25 | PIN_PER_7 | PIN_PORTA ) ;
//	configure_pins( 0x00000004, PIN_PERIPHERAL | PIN_PER_7 | PIN_PORTA ) ;
//#else
//	configure_pins( 0x00000010, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
//	GPIOD->MODER = (GPIOD->MODER & 0xFFFFC0FF ) | 0x00002900 ;	// Alternate func.
//	GPIOD->AFR[0] = (GPIOD->AFR[0] & 0xF00FFFFF ) | 0x07700000 ;	// Alternate func.
//#endif
//	if ( baudRate > 10 )
//	{
//		USART2->BRR = Peri1_frequency / baudRate ;
//	}
//	else if ( baudRate == 0 )
//	{
//		USART2->BRR = Peri1_frequency / 57600 ;		// 16.25 divider => 57600 baud
//	}
//	else
//	{
//		USART2->BRR = Peri1_frequency / 9600 ;		// 97.625 divider => 9600 baud
//	}
//	USART2->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE ;
//	USART2->CR2 = 0 ;
//	USART2->CR3 = 0 ;
//	if ( parity )
//	{
//		USART2->CR1 |= USART_CR1_PCE | USART_CR1_M ;	// Need 9th bit for parity
//		USART2->CR2 |= 0x2000 ;	// 2 stop bits
//	}
//	if ( baudRate == 115200 )		// ASSAN DSM
//	{
//#ifdef PCB9XT
//		GPIOB->BSRRL = 0x0004 ;		// output disable
//#else
//		GPIOD->BSRRL = 0x0010 ;		// output enable
//#endif
//	}
//	NVIC_SetPriority( USART2_IRQn, 2 ) ; // Quite high priority interrupt
//  NVIC_EnableIRQ(USART2_IRQn);
}

//extern "C" void USART2_IRQHandler()
//{
//  uint32_t status;
//  uint8_t data;
	
//  status = USART2->SR ;

//  while (status & (USART_FLAG_RXNE | USART_FLAG_ERRORS))
//	{
//    data = USART2->DR;

//    if (!(status & USART_FLAG_ERRORS))
//		{
//			put_fifo128( &Com1_fifo, data ) ;
//		}
//		else
//		{
//			put_fifo128( &Com1_fifo, data ) ;
////			USART_ERRORS += 1 ;
////			if ( status & USART_FLAG_ORE )
////			{
////				USART_ORE += 1 ;
////			}
////			if ( status & USART_FLAG_NE )
////			{
////				USART_NE += 1 ;
////			}
////			if ( status & USART_FLAG_FE )
////			{
////				USART_FE += 1 ;
////			}
////			if ( status & USART_FLAG_PE )
////			{
////				USART_PE += 1 ;
////			}
//		}
//    status = USART2->SR ;
//  }
//}

void start_timer11()
{
#ifndef SIMU

	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN ;		// Enable clock
	TIM11->PSC = (Peri2_frequency*Timer_mult2) / 2000000 - 1 ;		// 0.5uS
	TIM11->CCER = 0 ;
	TIM11->DIER = 0 ;
	TIM11->CCMR1 = 0 ;
	 
	TIM11->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 ;

	TIM11->CR1 = TIM_CR1_CEN ;

	NVIC_SetPriority( TIM1_TRG_COM_TIM11_IRQn, 14 ) ; // Low priority interrupt
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn) ;
#endif
}

void stop_timer11()
{
	TIM11->CR1 = 0 ;
	NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn) ;
}

// Handle software serial on COM1 input (for non-inverted input)
void init_software_com1(uint32_t baudrate, uint32_t invert, uint32_t parity )
{
	struct t_softSerial *pss = &SoftSerial1 ;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN ;		// Enable clock
	
	pss->bitTime = 2000000 / baudrate ;
	pss->softSerialEvenParity = parity ? 1 : 0 ;

#ifdef PCB9XT
	pss->softSerInvert = invert ? 0 : GPIO_Pin_3 ;	// Port A3
#else
	pss->softSerInvert = invert ? 0 : GPIO_Pin_6 ;	// Port D6
#endif
	pss->pfifo = &Com1_fifo ;

	pss->lineState = LINE_IDLE ;

#ifdef PCB9XT
#define EXT_BIT_MASK	0x00000008
	SYSCFG->EXTICR[0] = 0 ;
#else
#define EXT_BIT_MASK	0x00000040
	SYSCFG->EXTICR[1] |= 0x0300 ;
#endif
	EXTI->IMR |= EXT_BIT_MASK ;
	EXTI->RTSR |= EXT_BIT_MASK ;
	EXTI->FTSR |= EXT_BIT_MASK ;

#ifdef PCB9XT
	configure_pins( GPIO_Pin_3, PIN_INPUT | PIN_PORTA ) ;
	NVIC_SetPriority( EXTI3_IRQn, 0 ) ; // Highest priority interrupt
	NVIC_EnableIRQ( EXTI3_IRQn) ;
#else
	configure_pins( GPIO_Pin_6, PIN_INPUT | PIN_PORTD ) ;
	NVIC_SetPriority( EXTI9_5_IRQn, 0 ) ; // Highest priority interrupt
	NVIC_EnableIRQ( EXTI9_5_IRQn) ;
#endif
	
	start_timer11() ;
}

void disable_software_com1()
{
	stop_timer11() ;
	EXTI->IMR &= ~EXT_BIT_MASK ;
#ifdef PCB9XT
	NVIC_DisableIRQ( EXTI3_IRQn ) ;
//#else
//	NVIC_DisableIRQ( EXTI9_5_IRQn ) ;
#endif
}

#ifdef PCB9XT
extern "C" void EXTI3_IRQHandler()
#else
extern "C" void EXTI9_5_IRQHandler()
#endif
{
  register uint32_t capture ;
  register uint32_t dummy ;
	struct t_softSerial *pss = &SoftSerial1 ;

	capture =  TIM7->CNT ;	// Capture time
	
#ifdef PCBX9D
	if ( ( EXTI->PR & EXT_BIT_MASK ) == 0 )
	{
		return ;
	}
#endif	
	EXTI->PR = EXT_BIT_MASK ;
	
#ifdef PCB9XT
	dummy = GPIOA->IDR ;
	if ( ( dummy & GPIO_Pin_3 ) == pss->softSerInvert )
#else
	dummy = GPIOD->IDR ;
	if ( ( dummy & GPIO_Pin_6 ) == pss->softSerInvert )
#endif
	{
		// L to H transition
		pss->LtoHtime = capture ;
		TIM11->CNT = 0 ;
		TIM11->CCR1 = pss->bitTime * 12 ;
		uint32_t time ;
		capture -= pss->HtoLtime ;
		time = capture ;
		putCaptureTime( pss, time, 0 ) ;
		TIM11->DIER = TIM_DIER_CC1IE ;
	}
	else
	{
		// H to L transition
		pss->HtoLtime = capture ;
		if ( pss->lineState == LINE_IDLE )
		{
			pss->lineState = LINE_ACTIVE ;
			putCaptureTime( pss, 0, 3 ) ;
		}
		else
		{
			uint32_t time ;
			capture -= pss->LtoHtime ;
			time = capture ;
			putCaptureTime( pss, time, 1 ) ;
		}
		TIM11->DIER = 0 ;
		TIM11->CCR1 = pss->bitTime * 20 ;
		TIM11->CNT = 0 ;
		TIM11->SR = 0 ;
	}
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler()
{
	uint32_t status ;
	struct t_softSerial *pss = &SoftSerial1 ;

	status = TIM11->SR ;
	if ( status & TIM_SR_CC1IF )
	{		
		uint32_t time ;
		time = TIM7->CNT - pss->LtoHtime ;
		putCaptureTime( pss, time, 2 ) ;
		pss->lineState = LINE_IDLE ;
		TIM11->DIER = 0 ;
		TIM11->SR = 0 ;
	}
	
}

//#endif // nX12D

#ifdef PCB9XT

#define CONSOLE_BAUDRATE 115200

static void consoleInit()
{
	// Serial configure  
	RCC->APB1ENR |= RCC_APB1ENR_UART4EN ;		// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	configure_pins( 0x00000001, PIN_PERIPHERAL | PIN_PUSHPULL | PIN_OS25 | PIN_PORTA | PIN_PER_8 ) ;
	configure_pins( 0x00000002, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_8 | PIN_PULLUP ) ;
	GPIOA->MODER = (GPIOA->MODER & 0xFFFFFFF0 ) | 0x0000000A ;	// Alternate func.
	GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFFFFFF00 ) | 0x00000088 ;	// Alternate func.
	UART4->BRR = Peri1_frequency / CONSOLE_BAUDRATE ;		// 97.625 divider => 19200 baud
	UART4->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE ;
	UART4->CR2 = 0 ;
	UART4->CR3 = 0 ;
	NVIC_SetPriority( UART4_IRQn, 2 ) ; // Changed to 3 for telemetry receive
  NVIC_EnableIRQ(UART4_IRQn) ;
}

static void UART4SetBaudrate ( uint32_t baudrate )
{
	UART4->BRR = Peri1_frequency / baudrate ;
}

static void com2Parity( uint32_t even )
{
	if ( even )
	{
		UART4->CR1 |= USART_CR1_PCE | USART_CR1_M ;
	}
	else
	{
		UART4->CR1 &= ~(USART_CR1_PCE | USART_CR1_M) ;
	}
}

void com2_Configure( uint32_t baudrate, uint32_t invert, uint32_t parity )
{
	consoleInit() ;
	UART4SetBaudrate( baudrate ) ;
	com2Parity( parity ) ;
}

extern "C" void UART4_IRQHandler()
{
  uint32_t status;
  uint8_t data;
	USART_TypeDef *puart = UART4 ;

  status = puart->SR ;
	
  while (status & (USART_FLAG_RXNE | USART_FLAG_ERRORS))
	{
    data = puart->DR;

    if (!(status & USART_FLAG_ERRORS))
		{
			put_fifo128( &Com2_fifo, data ) ;
		}
    status = puart->SR ;
	}
}


#endif

#endif

