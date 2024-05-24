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

// This version for ARM based ERSKY9X board

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "lcd.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_dma2d.h"
//#include "drivers.h"
//#include "logicio.h"

//#include "font.lbm"
//#define font_5x8_x20_x7f (font)

#define LCD_W	480
#define LCD_H	272

#ifdef PCBX10
#define INVERT_DISPLAY 1
#endif

extern uint32_t CurrentFrameBuffer ;

void lcdDrawCharBitmapDma( uint16_t x, uint16_t y, uint8_t chr, uint32_t mode, uint16_t colour, uint16_t background ) ;

// Local data
uint8_t Lcd_lastPos ;

// invers: 0 no 1=yes 2=blink
void lcd_putc(uint8_t x,uint8_t y,const char c )
{
  lcd_putcAtt(x,y,c,0);
}

uint16_t lcd_putcAtt(uint16_t x,uint16_t y,const char c,uint8_t mode )
{
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
		return x ;
	}
	
//	if ( (mode & BLINK) && BLINK_ON_PHASE )
//	{
//		mode ^= INVERS ;
//	}
	lcdDrawCharBitmapDma( x*2, y*2, c, mode & (INVERS | CONDENSED), LCD_BLACK, LCD_WHITE ) ;
	return (mode & CONDENSED) ? x + FWNUM : x + FW ;
}

uint16_t lcd_putcAttColour(uint16_t x,uint16_t y,const char c,uint8_t mode, uint16_t colour, uint16_t background )
{
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
		return x ;
	}

	lcdDrawCharBitmapDma( x*2, y*2, c, mode, colour, background ) ;
	x += FW ;
	return x ;
	
}

void lcd_putsnAttColour( uint16_t x, uint16_t y, const char * s,uint8_t len,uint8_t mode, uint16_t colour, uint16_t background )
{
	register char c ;
  while(len!=0)
	{
    c = *s++ ;
#ifdef BOOT
		if ( c == 0 )
		{
			break ;			
		}
#endif
    x = lcd_putcAttColour( x, y, c, mode, colour, background ) ;
    len--;
  }
}


void lcd_putsnAtt(uint8_t x,uint8_t y, const char * s,uint8_t len,uint8_t mode)
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

void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len)
{
  lcd_putsnAtt( x,y,s,len,0);
}

uint8_t lcd_putsAtt( uint8_t x, uint8_t y, const char *s, uint8_t mode )
{
  while(1)
	{
    char c = *s++ ;
    if(!c) break ;
    x = lcd_putcAtt(x,y,c,mode) ;
  }
  return x;
}

void lcd_puts_Pleft( uint8_t y, const char *s )
{
  lcd_putsAtt( 0, y, s, 0);
}

void lcd_puts_P( uint8_t x, uint8_t y, const char *s )
{
  lcd_putsAtt( x, y, s, 0);
}

#ifdef APP
// Puts sub-string from string options
// First byte of string is sub-string length
// idx is index into string (in length units)
// Output length characters
void lcd_putsAttIdx(uint8_t x,uint8_t y,const char * s,uint8_t idx,uint8_t att)
{
	uint8_t length ;
	length = *s++ ;

  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
}
#endif

void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
{
  register int i ;
  x+=FWNUM*4;
  for(i=0; i<4; i++)
  {
    x-=FWNUM;
    char c = val & 0xf;
    c = c>9 ? c+'A'-10 : c+'0';
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
    val>>=4;
  }
}
uint8_t plotType = PLOT_XOR ;

//void lcd_write_bits( uint8_t *p, uint8_t mask )
//{
//  if(p<DISPLAY_END)
//	{
//		uint8_t temp = *p ;
//		if ( plotType != PLOT_XOR )
//		{
//			temp |= mask ;
//		}
//		if ( plotType != PLOT_BLACK )
//		{
//			temp ^= mask ;
//		}
//		*p = temp ;
//	}
//}

void lcd_plot( uint16_t x, uint16_t y )
{
	uint16_t *p = ( uint16_t *) CurrentFrameBuffer ;
	p += x*2 ;
	p += y * LCD_W * 2 ;
	if ( plotType == PLOT_BLACK )
	{
		*p = 0 ;
		*(p+1) = 0 ;
		*(p+LCD_W) = 0 ;
		*(p+LCD_W+1) = 0 ;
	}
	else if ( plotType == PLOT_BLACK )
	{
		*p = 0xFFFF ;
		*(p+1) = 0xFFFF ;
		*(p+LCD_W) = 0xFFFF ;
		*(p+LCD_W+1) = 0xFFFF ;
	}
	else
	{
		*p ^= 0xFFFF ;
		*(p+1) ^= 0xFFFF ;
		*(p+LCD_W) ^= 0xFFFF ;
		*(p+LCD_W+1) ^= 0xFFFF ;
	}
}

void lcd_hlineStip( uint16_t x, uint16_t y, uint8_t w, uint8_t pat )
{
  if(w<0) {x+=w; w=-w;}

#ifdef INVERT_DISPLAY
	x = (LCD_W/2-1) - x ;
	y = (LCD_H/2-1) - y ;
#endif
	uint16_t *p = ( uint16_t *) CurrentFrameBuffer ;
	p += x*2 ;
	p += y * LCD_W * 2 ;

	while(w)
	{
//    if ( p>=DISPLAY_END)
//    {
//      break ;			
//    }
    if(pat&1)
		{
			if ( plotType == PLOT_BLACK )
			{
				*p = 0 ;
				*(p+1) = 0 ;
				*(p+LCD_W) = 0 ;
				*(p+LCD_W+1) = 0 ;
			}
			else if ( plotType == PLOT_WHITE )
			{
				*p = 0xFFFF ;
				*(p+1) = 0xFFFF ;
				*(p+LCD_W) = 0xFFFF ;
				*(p+LCD_W+1) = 0xFFFF ;
			}
			else
			{
				*p ^= 0xFFFF ;
				*(p+1) ^= 0xFFFF ;
				*(p+LCD_W) ^= 0xFFFF ;
				*(p+LCD_W+1) ^= 0xFFFF ;
			}
      pat = (pat >> 1) | 0x80;
    }
		else
		{
      pat = pat >> 1;
    }
    w -= 1 ;
#ifdef INVERT_DISPLAY
    p -= 2 ;
#else
    p += 2 ;
#endif
  }
}


// Reverse video 8 pixels high, w pixels wide
// Vertically on an 8 pixel high boundary
void lcd_char_inverse( uint16_t x, uint16_t y, uint16_t w, uint8_t blink )
{
#ifdef INVERT_DISPLAY
	x = (LCD_W/2-1) - x ;
	y = (LCD_H/2-1) - y - 7 ;
#endif	
	x *= 2 ;
	w *= 2 ;
	uint16_t *p = ( uint16_t *) CurrentFrameBuffer ;
	p += x ;
	p += y * LCD_W * 2 ;
	
#ifdef INVERT_DISPLAY
	p -= w - 2 ;
//	p -= LCD_W*16 ;
#endif
	uint16_t end = x + w ;

	while ( x < end )
	{
		uint32_t i ;
		uint16_t *q = p ;
		for ( i = 0 ; i < 16 ; i += 1 )
		{
			*q ^= 0xFFFF ;
			q += LCD_W ;
		}
		p += 1 ;
		x += 1 ;
	}
}

void lcd_hline( uint8_t x, uint8_t y, int8_t w )
{
  lcd_hlineStip(x,y,w,0xff);
}

void lcd_vline( uint16_t x, uint16_t y, int8_t h )
{
	int16_t y1 ;
	y1 = y ;
  if (h<0) { y1+=h; h=-h; }
	while ( y1 < 0 )
	{
		y1 += 1 ;
		h -= 1 ;
	}
	if ( h <= 0 )
	{
		return ;
	}
	if ( y1 + h >= 272/2 )
	{
		h -= 272/2 - y1 ;
	}
	if ( h <= 0 )
	{
		return ;
	}

#ifdef INVERT_DISPLAY
	x = (LCD_W/2-1) - x ;
	y1 = (LCD_H/2-1) - y1 ;
#endif
	uint16_t *p = ( uint16_t *) CurrentFrameBuffer ;
	p += x*2 ;
	p += y1 * LCD_W * 2 ;
  
  while( h )
	{
		h -= 1 ;
		if ( plotType == PLOT_BLACK )
		{
			*p = 0 ;
			*(p+1) = 0 ;
			*(p+LCD_W) = 0 ;
			*(p+LCD_W+1) = 0 ;
		}
		else if ( plotType == PLOT_WHITE )
		{
			*p = 0xFFFF ;
			*(p+1) = 0xFFFF ;
			*(p+LCD_W) = 0xFFFF ;
			*(p+LCD_W+1) = 0xFFFF ;
		}
		else
		{
			*p ^= 0xFFFF ;
			*(p+1) ^= 0xFFFF ;
			*(p+LCD_W) ^= 0xFFFF ;
			*(p+LCD_W+1) ^= 0xFFFF ;
		}
#ifdef INVERT_DISPLAY
		p -= LCD_W * 2 ;
#else
		p += LCD_W * 2 ;
#endif
  }
}


