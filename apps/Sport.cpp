

#ifdef PCBSKY
#include "AT91SAM3S4.h"
#include "core_cm3.h"
#endif

#if ( defined(PCBX9D) || defined(PCB9XT) )
#include "hal.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"

//#ifdef PCB9XT
#include "stm32f2xx_usart.h"
#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)
//#endif

#endif

#include "radio.h"
#include "drivers.h"
#include "logicio.h"

#define FRSKY_SPORT_PACKET_SIZE		9
#define SECOND_USART       USART0
#define SECOND_ID          ID_USART0


#define START_STOP      0x7e
#define BYTESTUFF       0x7d
#define STUFF_MASK      0x20

uint8_t numPktBytes = 0 ;
// Receive buffer state machine state defs
#define frskyDataIdle    0
#define frskyDataStart   1
#define frskyDataInFrame 2
#define frskyDataXOR     3
#define PRIVATE_COUNT		 4
#define PRIVATE_VALUE		 5

static uint8_t dataState = frskyDataIdle ;
uint8_t frskyRxBuffer[20] ;   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing
uint8_t frskyTxBuffer[20] ;   // Ditto for transmit buffer

struct t_fifo128 Com1_fifo ;

struct t_SportTx
{
	uint8_t *ptr ;
	uint16_t count ;
	uint8_t busy ;
	uint8_t index ;
	uint8_t data[16] ;
};// SportTx ;

struct t_XfireTx
{
	uint8_t *ptr ;
	uint16_t count ;
	uint8_t busy ;
	uint8_t index ;
	uint8_t data[16] ;
} XfireTx ;

union t_telemetryTx
{
	struct t_SportTx SportTx ;
	struct t_XfireTx XfireTx ;
} TelemetryTx ;

struct t_sendingSport
{
	uint8_t *buffer ;
	uint32_t count ;
} SendingSportPacket ;


extern void maintenance_receive_packet( uint8_t *packet, uint32_t check ) ;

#define RADIO_SKY	0
#define RADIO_PRO	1
#define RADIO_AR9X	2
extern uint8_t RadioType ;


static bool checkSportPacket()
{
	uint8_t *packet = frskyRxBuffer ;
  uint16_t crc = 0 ;
  for ( uint8_t i=1; i<FRSKY_SPORT_PACKET_SIZE; i++)
	{
    crc += packet[i]; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
  }
  return (crc == 0x00ff) ;
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


// Physical ID in packet[0] ;
void processSportPacket()
{
	uint8_t *packet = frskyRxBuffer ;

	maintenance_receive_packet( packet, checkSportPacket() ) ;
}
	 

void frsky_receive_byte( uint8_t data )
{
  uint8_t numbytes = numPktBytes ;
  
	switch (dataState) 
  {
    case frskyDataStart:
      if (data == START_STOP)
			{
        dataState = frskyDataInFrame ;
        numbytes = 0 ;
				break ; // Remain in userDataStart if possible 0x7e,0x7e doublet found.
			}
      dataState = frskyDataInFrame;
      if (numbytes < 19)
	      frskyRxBuffer[numbytes++] = data ;
      break;

    case frskyDataInFrame:
      if (data == BYTESTUFF)
      { 
          dataState = frskyDataXOR; // XOR next byte
          break; 
      }
      if (data == START_STOP) // end of frame detected
      {
       	dataState = frskyDataInFrame ;
        numbytes = 0;
        break;
      }
      if (numbytes < 19)
	      frskyRxBuffer[numbytes++] = data;
    break;

    case frskyDataXOR:
      dataState = frskyDataInFrame;
      if (numbytes < 19)
        frskyRxBuffer[numbytes++] = data ^ STUFF_MASK;
    break;

    case frskyDataIdle:
      if (data == START_STOP)
      {
        numbytes = 0;
        dataState = frskyDataStart;
      }
    break;
	    
  } // switch
 	if (numbytes >= FRSKY_SPORT_PACKET_SIZE)
	{
		processSportPacket() ;    	
	  numbytes = 0 ;
		dataState = frskyDataIdle;
	}
  numPktBytes = numbytes ;
}

void check_frsky()
{
	uint16_t rxchar ;
	while ( ( rxchar = get_fifo128( &Com1_fifo ) ) != 0xFFFF )
	{
		frsky_receive_byte( rxchar ) ;
	}
}


#ifdef PCBSKY

uint32_t txPdcUsart( uint8_t *buffer, uint32_t size, uint32_t receive )
{
  register Usart *pUsart = SECOND_USART;

	if ( pUsart->US_TNCR == 0 )
	{
	if ( RadioType == RADIO_PRO )
	{
		PIOA->PIO_SODR = 0x02000000L ;	// Set bit A25 ON, enable SPort output
	}
		if ( receive == 0 )
		{
			pUsart->US_CR = US_CR_RXDIS ;
		}
		else
		{
  		pUsart->US_CR = US_CR_RXEN ;
		}

	  pUsart->US_TNPR = (uint32_t)buffer ;
		pUsart->US_TNCR = size ;
		pUsart->US_PTCR = US_PTCR_TXTEN ;
		pUsart->US_IER = US_IER_ENDTX ;
		NVIC_SetPriority( USART0_IRQn, 3 ) ; // Quite high priority interrupt
		NVIC_EnableIRQ(USART0_IRQn) ;
		return 1 ;
	}
	return 0 ;
}


//uint32_t txPdcCom1( struct t_serial_tx *data )
//{
//  register Usart *pUsart = SECOND_USART;

//	if ( pUsart->US_TCR == 0 )
//	{
//		Current_Com1 = data ;
//		data->ready = 1 ;
//#ifndef SIMU
//	  pUsart->US_TPR = (uint32_t)data->buffer ;
//#endif
//		pUsart->US_TCR = data->size ;
//		pUsart->US_PTCR = US_PTCR_TXTEN ;
//		pUsart->US_IER = US_IER_TXBUFE ;
//		NVIC_SetPriority( USART0_IRQn, 3 ) ; // Quite high priority interrupt
//		NVIC_EnableIRQ(USART0_IRQn) ;
//		return 1 ;
//	}
//	return 0 ;
//}

static void UART2_Configure( uint32_t baudrate, uint32_t masterClock)
{
////    const Pin pPins[] = CONSOLE_PINS;
  register Usart *pUsart = SECOND_USART;
//	register Pio *pioptr ;

  /* Configure PIO */
	configure_pins( (PIO_PA5 | PIO_PA6), PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;
	
	if ( RadioType == RADIO_PRO )
	{
		configure_pins( PIO_PA25, PIN_ENABLE | PIN_LOW | PIN_OUTPUT | PIN_PORTA | PIN_NO_PULLUP ) ;
	}

//	pioptr = PIOA ;
//  pioptr->PIO_ABCDSR[0] &= ~(PIO_PA5 | PIO_PA6) ;	// Peripheral A
//  pioptr->PIO_ABCDSR[1] &= ~(PIO_PA5 | PIO_PA6) ;	// Peripheral A
//  pioptr->PIO_PDR = (PIO_PA5 | PIO_PA6) ;					// Assign to peripheral

	if ( baudrate < 10 )
	{
		switch ( baudrate )
		{
			case 0 :
			default :
				baudrate = 9600 ;
			break ;
			case 1 :
				baudrate = 19200 ;
			break ;
			case 2 :
				baudrate = 38400 ;
			break ;
			case 3 :
				baudrate = 57600 ;
			break ;
			case 4 :
				baudrate = 115200 ;
			break ;
		}
	}

//  /* Configure PMC */
  PMC->PMC_PCER0 = 1 << SECOND_ID;

//  /* Reset and disable receiver & transmitter */
  pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX
	                 | US_CR_RXDIS | US_CR_TXDIS;

//  /* Configure mode */
  pUsart->US_MR =  0x000008C0 ;  // NORMAL, No Parity, 8 bit

//  /* Configure baudrate */
//  /* Asynchronous, no oversampling */
  pUsart->US_BRGR = (masterClock / baudrate) / 16;

//  /* Disable PDC channel */
  pUsart->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;

//  /* Enable receiver and transmitter */
  pUsart->US_CR = US_CR_RXEN | US_CR_TXEN;
	pUsart->US_IER = US_IER_RXRDY ;

	NVIC_SetPriority( USART0_IRQn, 2 ) ; // Quite high priority interrupt
	NVIC_EnableIRQ(USART0_IRQn) ;
	
}

void com1_Configure( uint32_t baudrate, uint32_t invert, uint32_t parity )
{
	UART2_Configure( baudrate, Master_frequency ) ;	
  register Usart *pUsart = SECOND_USART;
	if ( parity )
	{
  	pUsart->US_MR =  0x000000C0 ;  // NORMAL, Even Parity, 8 bit
	}
	else
	{
		pUsart->US_MR =  0x000008C0 ;  // NORMAL, No Parity, 8 bit
	}
}


extern "C" void USART0_IRQHandler()
{
  register Usart *pUsart = SECOND_USART;
  if ( pUsart->US_IMR & US_IMR_TIMEOUT )
	{
		if ( pUsart->US_CSR & US_CSR_TIMEOUT )
		{
			pUsart->US_RTOR = 0 ;		// Off
			pUsart->US_IDR = US_IDR_TIMEOUT ;
			txPdcUsart( TelemetryTx.SportTx.ptr, TelemetryTx.SportTx.count, 0 ) ;
		}
		
	}
  if ( pUsart->US_IMR & US_IMR_ENDTX )
	{
	 	if ( pUsart->US_CSR & US_CSR_ENDTX )
		{
			TelemetryTx.SportTx.count = 0 ;
			pUsart->US_IER = US_IER_TXEMPTY ;
			pUsart->US_IDR = US_IDR_ENDTX ;
		}
	}
// Disable Tx output
  if ( pUsart->US_IMR & US_IMR_TXEMPTY )
	{
 		if ( pUsart->US_CSR & US_CSR_TXEMPTY )
		{
			if ( RadioType == RADIO_PRO )
			{
				PIOA->PIO_CODR = 0x02000000L ;	// Set bit A25 OFF
			}
			pUsart->US_IDR = US_IDR_TXEMPTY ;
			(void) pUsart->US_RHR ;	// Flush receiver
			pUsart->US_CR = US_CR_RXEN ;
		}
	}
	
	if ( pUsart->US_CSR & US_CSR_RXRDY )
	{
    uint8_t data = pUsart->US_RHR ;
		put_fifo128( &Com1_fifo, data ) ;
	}
}

#endif

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D)

void com1_Configure( uint32_t baudRate, uint32_t invert, uint32_t parity )
{
	// Serial configure  
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN ;		// Enable clock
#ifdef PCB9XT
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
	GPIOB->BSRRH = 0x0004 ;		// output disable
#else
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN ; 		// Enable portD clock
 #ifdef PCBXLITE
			GPIOD->BSRRL = PIN_SPORT_ON ;		// output disable
 #else
			GPIOD->BSRRH = PIN_SPORT_ON ;		// output disable
 #endif
#endif
#ifdef PCB9XT
// PB2 as SPort enable
	configure_pins( 0x00000004, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTB ) ;
	configure_pins( 0x00000008, PIN_PERIPHERAL | PIN_PUSHPULL | PIN_OS25 | PIN_PER_7 | PIN_PORTA ) ;
	configure_pins( 0x00000004, PIN_PERIPHERAL | PIN_PER_7 | PIN_PORTA ) ;
#else
	configure_pins( 0x00000010, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTD ) ;
	GPIOD->MODER = (GPIOD->MODER & 0xFFFFC0FF ) | 0x00002900 ;	// Alternate func.
	GPIOD->AFR[0] = (GPIOD->AFR[0] & 0xF00FFFFF ) | 0x07700000 ;	// Alternate func.
#endif
	if ( baudRate > 10 )
	{
		USART2->BRR = Peri1_frequency / baudRate ;
	}
	else if ( baudRate == 0 )
	{
		USART2->BRR = Peri1_frequency / 57600 ;		// 16.25 divider => 57600 baud
	}
	else
	{
		USART2->BRR = Peri1_frequency / 9600 ;		// 97.625 divider => 9600 baud
	}
	USART2->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE ;
	USART2->CR2 = 0 ;
	USART2->CR3 = 0 ;
	if ( parity )
	{
		USART2->CR1 |= USART_CR1_PCE | USART_CR1_M ;	// Need 9th bit for parity
		USART2->CR2 |= 0x2000 ;	// 2 stop bits
	}
	NVIC_SetPriority( USART2_IRQn, 2 ) ; // Quite high priority interrupt
  NVIC_EnableIRQ(USART2_IRQn);
}

void x9dSPortTxStart( uint8_t *buffer, uint32_t count, uint32_t receive )
{
	SendingSportPacket.buffer = buffer ;
	SendingSportPacket.count = count ;
#ifdef PCB9XT
	GPIOB->BSRRL = 0x0004 ;		// output enable
#else
 #ifdef PCBXLITE
	GPIOD->BSRRH = 0x0010 ;		// output enable
 #else
	GPIOD->BSRRL = 0x0010 ;		// output enable
 #endif
#endif
	if ( receive == 0 )
	{
		USART2->CR1 &= ~USART_CR1_RE ;
	}
	USART2->CR1 |= USART_CR1_TXEIE ;
}

uint16_t RxIntCount ;

extern "C" void USART2_IRQHandler()
{
  uint32_t status;
  uint8_t data;
	
  status = USART2->SR ;

	if ( status & USART_SR_TXE )
	{
		if ( SendingSportPacket.count )
		{
			USART2->DR = *SendingSportPacket.buffer++ ;
			if ( --SendingSportPacket.count == 0 )
			{
				USART2->CR1 &= ~USART_CR1_TXEIE ;	// Stop Tx interrupt
				USART2->CR1 |= USART_CR1_TCIE ;	// Enable complete interrupt
			}
		}
		else
		{
			USART2->CR1 &= ~USART_CR1_TXEIE ;	// Stop Tx interrupt
		}
	}

	if ( ( status & USART_SR_TC ) && (USART2->CR1 & USART_CR1_TCIE ) )
	{
		USART2->CR1 &= ~USART_CR1_TCIE ;	// Stop Complete interrupt
		{
#ifdef PCB9XT
			GPIOB->BSRRH = 0x0004 ;		// output disable

#else
 #ifdef PCBXLITE
			GPIOD->BSRRL = PIN_SPORT_ON ;		// output disable
 #else
			GPIOD->BSRRH = PIN_SPORT_ON ;		// output disable
 #endif
#endif
			TelemetryTx.SportTx.count = 0 ;
			TelemetryTx.SportTx.busy = 0 ;
			USART2->CR1 |= USART_CR1_RE ;
		}
	}

  while (status & (USART_FLAG_RXNE | USART_FLAG_ERRORS))
	{
		
    data = USART2->DR;

    if (!(status & USART_FLAG_ERRORS))
		{
			put_fifo128( &Com1_fifo, data ) ;
		}
		else
		{
			put_fifo128( &Com1_fifo, data ) ;
		}
    status = USART2->SR ;
  }
}

#endif

