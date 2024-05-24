/*
 * Author - Mike Blandford
 *
 * Based on er9x by Erez Raviv <erezraviv@gmail.com>
 *
 * which was Based on th9x -> http://code.google.com/p/th9x/
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

#ifndef lcd_h
#define lcd_h

#ifdef PCBX9D
#if defined(PCBX7) || defined(PCBXLITE) || defined(PCBX9LITE)
#define DISPLAY_W 128
#define DISPLAY_H  64
#else
#define WIDE_DISPLAY
#define DISPLAY_W 212
#define DISPLAY_H  64
#endif
#endif

#if ( defined(PCBSKY) || defined(PCB9XT) )
#define DISPLAY_W 128
#define DISPLAY_H  64
#endif

#define FW          6
#define FWNUM       5
#define FH          8

/* lcd common flags */
#define INVERS        0x01
#define BLINK         0x02
#define DBLSIZE       0x04
#define CONDENSED     0x08

// putsChnRaw flags
#define MIX_SOURCE    0x10

/* lcd outdez flags */
#define LEADING0      0x10
#define PREC1         0x20
#define PREC2         0x30 /* 4 modes in 2bits! */
#define LEFT          0x40 /* align left */

/* time & telemetry flags */
#define NO_UNIT       0x80

#if defined(PCBX12D) || defined(PCBX10)
#define DISPLAY_W	480
#define DISPLAY_H	272

#define LCD_RED			0xF800
#define LCD_GREEN		0x07E0
#define LCD_BLUE		0x001F
#define LCD_WHITE		0xFFFF
#define LCD_BLACK		0
#define LCD_GREY		0x8410
#define LCD_STATUS_GREY	0x3BEF

#define STATUS_VERTICAL		250


#endif


extern uint8_t LcdLock ;
extern uint8_t LcdInputs ;

extern uint8_t Lcd_lastPos;
extern uint8_t DisplayBuf[] ;

extern const uint8_t font_se_extra[] ;
extern const uint8_t font_fr_extra[] ;
extern const uint8_t font_de_extra[] ;

extern const uint8_t font_se_big_extra[] ;
extern const uint8_t font_fr_big_extra[] ;
extern const uint8_t font_de_big_extra[] ;

extern const uint8_t *ExtraFont ;
extern const uint8_t *ExtraBigFont ;

#define PLOT_XOR		0
#define PLOT_BLACK	1
#define PLOT_WHITE	2

extern uint8_t plotType ;

#if defined(PCBX12D) || defined(PCBX10)
extern uint16_t lcd_putcAtt( uint16_t x, uint16_t y, const char c, uint8_t mode ) ;
extern void lcd_hlineStip( uint16_t x, uint16_t y, uint8_t w, uint8_t pat ) ;
extern void lcd_char_inverse( uint16_t x, uint16_t y, uint16_t w, uint8_t blink ) ;
extern void lcd_vline( uint16_t x, uint16_t y, int8_t h ) ;
extern uint16_t lcd_putcAttColour(uint16_t x,uint16_t y,const char c,uint8_t mode, uint16_t colour, uint16_t background ) ;
extern void lcd_putsnAttColour( uint16_t x, uint16_t y, const char * s,uint8_t len,uint8_t mode, uint16_t colour, uint16_t background ) ;
#else
extern uint8_t lcd_putcAtt( uint8_t x, uint8_t y, const char c, uint8_t mode ) ;
extern void lcd_hlineStip( unsigned char x, unsigned char y, signed char w, uint8_t pat ) ;
extern void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink ) ;
extern void lcd_vline( uint8_t x, uint8_t y, int8_t h ) ;
#endif

extern void lcd_putc(uint8_t x,uint8_t y,const char c ) ;
extern void lcd_putsAttIdx(uint8_t x,uint8_t y,const char * s,uint8_t idx,uint8_t att) ;
extern void lcd_putsnAtt(uint8_t x,uint8_t y,const char * s,uint8_t len,uint8_t mode) ;
extern void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len) ;
extern void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val) ;
extern void lcd_outhex2(uint8_t x,uint8_t y,uint8_t val) ;
extern void lcd_img( uint8_t i_x, uint8_t i_y, const unsigned char *imgdat, uint8_t idx, uint8_t mode ) ;
extern uint8_t lcd_putsAtt( uint8_t x, uint8_t y, const char *s, uint8_t mode ) ;
extern void lcd_puts_Pleft( uint8_t y, const char *s ) ;
extern void lcd_puts_P( uint8_t x, uint8_t y, const char *s ) ;
extern void lcd_outdez( uint8_t x, uint8_t y, int16_t val ) ;
extern void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint8_t mode ) ;
extern void lcd_outdezNAtt( uint8_t x, uint8_t y, int32_t val, uint8_t mode, int8_t len ) ;

extern void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent ) ;
extern void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h ) ;
extern void lcd_plot( uint8_t x, uint8_t y ) ;
extern void lcd_hline( uint8_t x, uint8_t y, int8_t w ) ;
extern void lcd_clear( void ) ;
extern void lcd_init( void ) ;
extern void lcdSetRefVolt(uint8_t val) ;
extern void lcdSendCtl(uint8_t val) ;
extern void refreshDisplay( void ) ;

extern void putsTime(uint8_t x,uint8_t y,int16_t tme,uint8_t att,uint8_t att2) ;
extern void putsVolts(uint8_t x,uint8_t y, uint8_t volts, uint8_t att) ;
extern void putsVBat(uint8_t x,uint8_t y,uint8_t att) ;

#ifdef PCBX9D
extern void backlight_set( uint16_t brightness ) ;
#endif

//#define BLINK_ON_PHASE (g_blinkTmr10ms & (1<<6))
//#define BLINK_SYNC      g_blinkTmr10ms = (3<<5)

#endif


