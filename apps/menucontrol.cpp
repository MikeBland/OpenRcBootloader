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
*  LIMITED TO, THE IMPLIED ARRANTIES OF MERCHANTABILITY AND FITNESS
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
***************************************************************************/

#include <stdint.h>
#include <stdlib.h>

#include "radio.h"
#include "drivers.h"
#include "lcd.h"
#include "menucontrol.h"

void popMenu(bool uppermost) ;
extern uint8_t menuPressed() ;
extern uint32_t keyState(EnumKeys enuk) ;


MenuFuncP g_menuStack[6] ;
uint8_t g_menuStackPtr = 0 ;
uint8_t EnterMenu = 0 ;

bool checkIncDec_Ret ;

static uint8_t Columns ;
uint8_t EditColumns ;
uint8_t s_editMode;
uint8_t s_editing;
uint8_t InverseBlink ;
uint8_t RotaryState ;		// Defaults to ROTARY_MENU_LR
uint8_t g_posHorz ;
uint8_t M_longMenuTimer ;
uint8_t M_lastVerticalPosition ;
int8_t scrollLR ;
int8_t scrollUD ;
uint8_t MaskRotaryLong ;

struct t_p1
{
	int16_t p1val ;
	int16_t p1valdiff ;
  int16_t p1valprev ;
	int16_t p2valprev ;
	int16_t p3valprev ;
} ;

struct t_p1 P1values ;


extern volatile uint8_t  g_blinkTmr10ms ;


int16_t checkIncDec16( int16_t val, int16_t i_min, int16_t i_max, uint8_t event )
{
  int16_t newval = val;
  uint8_t kpl=BOOT_KEY_RIGHT, kmi=BOOT_KEY_LEFT, kother = -1;
	uint8_t editAllowed = 1 ;
	
	if ( s_editMode == 0 )
	{
		editAllowed = 0 ;
	}

  if(event==EVT_KEY_FIRST(kpl) || event== EVT_KEY_REPT(kpl) || (s_editMode && (event==EVT_KEY_FIRST(BOOT_KEY_UP) || event== EVT_KEY_REPT(BOOT_KEY_UP))) )
	{
		if ( editAllowed )
		{
   		newval += 1 ;
//			audioDefevent(AU_KEYPAD_UP);
    	kother=kmi;
		}

  }else if(event==EVT_KEY_FIRST(kmi) || event== EVT_KEY_REPT(kmi) || (s_editMode && (event==EVT_KEY_FIRST(BOOT_KEY_DOWN) || event== EVT_KEY_REPT(BOOT_KEY_DOWN))) )
	{
		if ( editAllowed )
		{
   		newval -= 1 ;
//			audioDefevent(AU_KEYPAD_DOWN);
    	kother=kpl;
		}

  }
  if((kother != (uint8_t)-1) && keyState((EnumKeys)kother))
	{
    newval=-val;
    killEvents(kmi);
    killEvents(kpl);
  }
  if(i_min==0 && i_max==1 )
	{
		if ( (event==EVT_KEY_FIRST(BOOT_KEY_MENU) || event==EVT_KEY_BREAK(BTN_RE)) )
		{
      s_editMode = false;
      newval=!val;
      killEvents(event);
			if ( event==EVT_KEY_BREAK(BTN_RE) )
			{
				RotaryState = ROTARY_MENU_UD ;
			}
			event = 0 ;
		}
		else
		{
			newval &= 1 ;
		}
	}

  //change values based on P1
  newval -= P1values.p1valdiff;
#ifndef PCBX9D
 #ifndef PCB9XT
	if ( RotaryState == ROTARY_VALUE )
	{
		newval += Rotary_diff ;
	}
 #endif  
#endif  
	if(newval>i_max)
  {
    newval = i_max;
    killEvents(event);
  }
  else if(newval < i_min)
  {
    newval = i_min;
    killEvents(event);
  }
  if(newval != val)
	{
    if(newval==0)
		{
   	  pauseEvents(event);
    }
    checkIncDec_Ret = true;
  }
  else
	{
    checkIncDec_Ret = false;
  }
  return newval ;
}

uint8_t checkIndexed( uint8_t y, const char *s, uint8_t value, uint8_t edit, uint8_t event )
{
	uint8_t x ;
	uint8_t max ;
	if ( s )
	{
		x = *s++ ;
		max = *s++ ;
		lcd_putsAttIdx( x, y, s, value, edit ? InverseBlink: 0 ) ;
	}
	else
	{
//		x = PARAM_OFS ;
		max = 1 ;
//		menu_lcd_onoff( x, y, value, edit ) ;
	}
	if ( value > max )
	{
		value = max ;
	}
	if(edit)
	{
		if ( ( EditColumns == 0 ) || ( s_editMode ) )
		{
			value = checkIncDec16( value, 0, max, event ) ;
		}
	}
	return value ;
}

void popMenu(bool uppermost)
{
//	leavingMenu() ;
  if(g_menuStackPtr>0 || uppermost)
	{
    g_menuStackPtr = uppermost ? 0 : g_menuStackPtr-1;
 		EnterMenu = EVT_ENTRY_UP ;
  }
//	else
//	{
//    alert(PSTR(STR_MSTACK_UFLOW));
//  }
}

void chainMenu(MenuFuncP newMenu)
{
//	leavingMenu() ;
  g_menuStack[g_menuStackPtr] = newMenu;
	EnterMenu = EVT_ENTRY ;
}

void pushMenu(MenuFuncP newMenu)
{
//	leavingMenu() ;
  if(g_menuStackPtr >= DIM(g_menuStack)-1)
  {
//    alert(PSTR(STR_MSTACK_OFLOW));
    return;
  }
	EnterMenu = EVT_ENTRY ;
  g_menuStack[++g_menuStackPtr] = newMenu ;
}

uint8_t MState2::check_columns( uint8_t event, uint8_t maxrow)
{
	return check( event, 0, NULL, 0, &Columns, 0, maxrow ) ;
}

uint8_t MAXCOL( uint8_t row, const uint8_t *horTab, uint8_t horTabMax)
{
	return (horTab ? *(horTab+min(row, horTabMax)) : (const uint8_t)0) ;
}

#define INC(val,max) if(val<max) {val++;} else {val=0;}
#define DEC(val,max) if(val>0  ) {val--;} else {val=max;}

uint8_t MState2::check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, const uint8_t *horTab, uint8_t horTabMax, uint8_t maxrow)
{
	uint8_t l_posHorz ;
	l_posHorz = g_posHorz ;
	
//	int16_t c4, c5 ;
//	struct t_p1 *ptrp1 ;

    //check pot 2 - if changed -> scroll menu
    //check pot 3 if changed -> cursor down/up
    //we do this in these brackets to prevent it from happening in the main screen
//	c4 = calibratedStick[4] ;		// Read only once
//	c5 = calibratedStick[5] ;		// Read only once
    
//	ptrp1 = &P1values ;
		
//  scrollLR = ( ptrp1->p2valprev-c4)/SCROLL_TH;
//  scrollUD = ( ptrp1->p3valprev-c5)/SCROLL_TH;

//  if(scrollLR) ptrp1->p2valprev = c4;
//  if(scrollUD) ptrp1->p3valprev = c5;

//  if(scroll_disabled || g_eeGeneral.disablePotScroll)
//  {
//    scrollLR = 0;
//    scrollUD = 0;
//    scroll_disabled = 0;
//  }

	M_lastVerticalPosition = m_posVert ;
  
//	if(scrollLR || scrollUD || ptrp1->p1valdiff)
//	{
//		g_LightOffCounter = g_eeGeneral.lightAutoOff*500; // on keypress turn the light on 5*100
//		InacCounter = 0 ;
//	}

	if (menuTab)
	{
//    uint8_t attr = m_posVert==0 ? INVERS : 0;


    if(m_posVert==0)
    {
#ifndef PCBX9D
 #ifndef PCB9XT
			if ( RotaryState == ROTARY_MENU_LR )
			{
				if ( Rotary_diff > 0 )
				{
   				scrollLR = -1;
				}
				else if ( Rotary_diff < 0 )
				{
   				scrollLR = 1;
				}
				Rotary_diff = 0 ;
        if(event==EVT_KEY_BREAK(BTN_RE))
				{
					RotaryState = ROTARY_MENU_UD ;
		      event = 0 ;
				}
			}
			else if ( RotaryState == ROTARY_MENU_UD )
			{
        if(event==EVT_KEY_BREAK(BTN_RE))
				{
					RotaryState = ROTARY_MENU_LR ;
		      event = 0 ;
				}
			}
 #endif
#endif
      if( scrollLR && !s_editMode)
      {
        int8_t cc = curr - scrollLR ;
        if(cc<1) cc = menuTabSize-1 ;
        if(cc>(menuTabSize-1)) cc = 0 ;

        if(cc!=curr)
        {
          chainMenu((MenuFuncP)(&menuTab[cc]));
					return event ;
        }

        scrollLR = 0;
      }

//      if(event==EVT_KEY_FIRST(BOOT_KEY_LEFT))
//      {
//        uint8_t index ;
//        if(curr>0)
//          index = curr ;
//        else
//          index = menuTabSize ;

//       	chainMenu((MenuFuncP)(&menuTab[index-1]));
//				return event ;
//      }

//      if(event==EVT_KEY_FIRST(BOOT_KEY_RIGHT))
//      {
//        uint8_t index ;
//        if(curr < (menuTabSize-1))
//          index = curr +1 ;
//        else
//          index = 0 ;
//        chainMenu((MenuFuncP)(&menuTab[index]));
//				return event ;
//      }
    }
		else
		{
			if ( s_editMode == 0 ) RotaryState = ROTARY_MENU_UD ;
		}

//    DisplayScreenIndex(curr, menuTabSize, attr);
  }
	else if ( RotaryState == ROTARY_MENU_LR )
	{
		RotaryState = ROTARY_MENU_UD ;
	}
  uint8_t maxcol = MAXCOL(m_posVert, horTab, horTabMax);
		
 if ( maxrow != 0xFF )
 {
	if ( RotaryState == ROTARY_MENU_UD )
	{
		static uint8_t lateUp = 0 ;
		if ( lateUp )
		{
			lateUp = 0 ;
			l_posHorz = MAXCOL(m_posVert, horTab, horTabMax) ;
		}
#ifndef PCBX9D
 #ifndef PCB9XT
		if ( Rotary_diff > 0 )
		{
    	INC(l_posHorz,maxcol) ;
			if ( l_posHorz == 0 )
			{
				INC(m_posVert,maxrow);
			}
		}
		else if ( Rotary_diff < 0 )
		{
			if ( l_posHorz == 0 )
			{
      	DEC(m_posVert,maxrow);
				lateUp = 1 ;
				l_posHorz = 0 ;
			}
			else
			{
      	DEC(l_posHorz,maxcol) ;
			}
		}
		Rotary_diff = 0 ;
 #endif
#endif
    if(event==EVT_KEY_BREAK(BTN_RE))
		{
			RotaryState = ROTARY_VALUE ;
		}
	}
	else if ( RotaryState == ROTARY_VALUE )
	{
		if ( (event==EVT_KEY_BREAK(BTN_RE)) || ( s_editMode == 0 ) )
		{
			RotaryState = ROTARY_MENU_UD ;
		}
	}

	{
		uint8_t timer = M_longMenuTimer ;
		if ( menuPressed() )
		{
			if ( timer < 255 )
			{
				timer += 1 ;
			}
		}
		else
		{
			timer = 0 ;
		}
		if ( timer > 60 )
		{
			s_editMode = 1 ;
			RotaryState = ROTARY_VALUE ;
		}
		M_longMenuTimer = timer ;
	}

 } 

  maxcol = MAXCOL(m_posVert, horTab, horTabMax) ;
	EditColumns = maxcol ;
  l_posHorz = min(l_posHorz, maxcol ) ;

  if(!s_editMode)
  {
//    if(scrollUD)
//    {
//      int8_t cc = m_posVert - scrollUD;
//      if(cc<1) cc = 0;
//      if(cc>=maxrow) cc = maxrow;
//      m_posVert = cc;

//      l_posHorz = min(l_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
//      BLINK_SYNC;

//      scrollUD = 0;
//    }

    if(m_posVert>0 && scrollLR)
    {
      int8_t cc = l_posHorz - scrollLR;
      if(cc<1) cc = 0;
      if(cc>=MAXCOL(m_posVert, horTab, horTabMax)) cc = MAXCOL(m_posVert, horTab, horTabMax);
      l_posHorz = cc;

//      BLINK_SYNC;
      //            scrollLR = 0;
    }
  }

  switch(event)
  {
    case EVT_ENTRY:
        init() ;
        l_posHorz = 0 ;
				M_lastVerticalPosition = m_posVert ;
    case EVT_ENTRY_UP :
        s_editMode = false;
    break;
    case EVT_KEY_BREAK(BTN_RE):
    case EVT_KEY_FIRST(BOOT_KEY_MENU):
        if ( (m_posVert > 0) || (!menuTab) )
				{
	 				if ( maxrow != 0xFF )
					{
						s_editMode = !s_editMode;
						if ( s_editMode )
						{
							RotaryState = ROTARY_VALUE ;
						}
					}
				}
    break;
    case EVT_KEY_LONG(BOOT_KEY_EXIT):
				killEvents(event) ;
        s_editMode = false;
        popMenu(true); //return to uppermost, beeps itself
    break;
        //fallthrough
    case EVT_KEY_LONG(BTN_RE):
			if ( MaskRotaryLong )
			{
				break ;
			}
			killEvents(event) ;
    case EVT_KEY_BREAK(BOOT_KEY_EXIT):
        if(s_editMode)
				{
          s_editMode = false;
          break ;
        }
        popMenu();  //beeps itself
    break;

    case EVT_KEY_REPT(BOOT_KEY_RIGHT):  //inc
        if(l_posHorz==maxcol) break;
    case EVT_KEY_FIRST(BOOT_KEY_RIGHT)://inc
        if(!horTab || s_editMode)break;
        INC(l_posHorz,maxcol);
				if ( maxcol )
				{
					event = 0 ;
				}
//        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(BOOT_KEY_LEFT):  //dec
        if(l_posHorz==0) break;
    case EVT_KEY_FIRST(BOOT_KEY_LEFT)://dec
        if(!horTab || s_editMode)break;
        DEC(l_posHorz,maxcol);
				if ( maxcol )
				{
					event = 0 ;
				}
//        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(BOOT_KEY_DOWN):  //inc
        if(m_posVert==maxrow) break;
    case EVT_KEY_FIRST(BOOT_KEY_DOWN): //inc
        if(s_editMode)break;
        INC(m_posVert,maxrow);
        l_posHorz = min(l_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
//        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(BOOT_KEY_UP):  //dec
        if(m_posVert==0) break;
    case EVT_KEY_FIRST(BOOT_KEY_UP): //dec
        if(s_editMode)break;
        DEC(m_posVert,maxrow);
        l_posHorz = min(l_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
//        BLINK_SYNC;
    break;
  }
	s_editing = s_editMode || P1values.p1valdiff ;
//	InverseBlink = (!maxcol || s_editMode) ? BLINK : INVERS ;
	g_posHorz = l_posHorz ;
	InverseBlink = (s_editMode) ? BLINK : INVERS ;
	Columns = 0 ;
	MaskRotaryLong = 0 ;
	return event ;
}



