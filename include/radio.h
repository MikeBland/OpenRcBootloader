

#include <stdint.h>

#ifdef PCBSKY
#define wdt_reset()	(WDT->WDT_CR = 0xA5000001)
#endif
#ifdef PCBX9D
#define wdt_reset()	(IWDG->KR = 0x0000AAAAL)
#endif
#ifdef PCB9XT
#define wdt_reset()	(IWDG->KR = 0x0000AAAAL)
#endif
#ifdef PCBX12D 
#define wdt_reset()	(IWDG->KR = 0x0000AAAAL)
#endif

#define POWER_OFF			0
#define POWER_ON			1
#define POWER_X9E_STOP	3

#define DIM(arr) (sizeof((arr))/sizeof((arr)[0]))

#define BITMASK(bit) (1<<(bit))

#define _MSK_KEY_REPT    0x40
#define _MSK_KEY_DBL     0x10
#define IS_KEY_BREAK(key)  (((key)&0xf0)        ==  0x20)
#define EVT_KEY_BREAK(key) ((key)|                  0x20)
#define EVT_KEY_FIRST(key) ((key)|    _MSK_KEY_REPT|0x20)
#define EVT_KEY_REPT(key)  ((key)|    _MSK_KEY_REPT     )
#define EVT_KEY_LONG(key)  ((key)|0x80)
#define EVT_KEY_DBL(key)   ((key)|_MSK_KEY_DBL)
//#define EVT_KEY_DBL(key)   ((key)|0x10)
#define EVT_ENTRY               (0xff - _MSK_KEY_REPT)
#define EVT_ENTRY_UP            (0xfe - _MSK_KEY_REPT)
#define EVT_KEY_MASK             0x0f



#if ( defined(PCBSKY) || defined(PCB9XT) )
#define NUM_KEYS 8
enum EnumKeys {
    KEY_MENU ,
    KEY_EXIT ,
    KEY_DOWN ,
    KEY_UP  ,
    KEY_RIGHT ,
    KEY_LEFT,
		KEY_TRN,
	  BTN_RE
} ;
#endif

#ifdef PCBX9D
#define NUM_KEYS 8
enum EnumKeys {
    KEY_MENU ,
    KEY_EXIT ,
    KEY_ENTER ,
    KEY_PAGE ,
    KEY_PLUS ,
    KEY_MINUS,
		KEY_TRN,
	  BTN_RE
} ;
#endif

#ifdef PCBX12D
#define NUM_KEYS 8
enum EnumKeys {
    KEY_MENU ,
    KEY_EXIT ,
    KEY_ENTER ,
    KEY_PAGE ,
    KEY_PLUS ,
    KEY_MINUS,
		KEY_TRN,
	  BTN_RE
} ;
#endif

extern void init_SDcard( void ) ;
extern unsigned long Master_frequency ;

extern void init_soft_power( void ) ;
extern uint32_t check_soft_power( void ) ;
extern void soft_power_off( void ) ;

#if ( defined(PCBSKY) || defined(PCB9XT) )
#define BOOT_KEY_UP			KEY_UP
#define BOOT_KEY_DOWN		KEY_DOWN
#define BOOT_KEY_LEFT		KEY_LEFT
#define BOOT_KEY_RIGHT	KEY_RIGHT
#define BOOT_KEY_MENU		KEY_MENU
#define BOOT_KEY_EXIT		KEY_EXIT
#define DISPLAY_CHAR_WIDTH	21
#endif

#ifdef PCBX12D
#define BOOT_KEY_UP			KEY_PLUS
#define BOOT_KEY_DOWN		KEY_MINUS
#define BOOT_KEY_LEFT		KEY_ENTER
#define BOOT_KEY_RIGHT	KEY_PAGE
#define BOOT_KEY_MENU		KEY_MENU
#define BOOT_KEY_EXIT		KEY_EXIT

#define DISPLAY_CHAR_WIDTH	21

#endif

#ifdef PCBX9D
#ifdef REV9E
#define BOOT_KEY_UP			KEY_PLUS
#define BOOT_KEY_DOWN		KEY_MINUS
#define BOOT_KEY_LEFT		KEY_ENTER
#define BOOT_KEY_RIGHT	KEY_PAGE
#define BOOT_KEY_MENU		KEY_MENU
#define BOOT_KEY_EXIT		KEY_EXIT
#else
#ifdef PCBX7
#define BOOT_KEY_UP			KEY_PLUS
#define BOOT_KEY_DOWN		KEY_MINUS
#define BOOT_KEY_LEFT		KEY_MENU
#define BOOT_KEY_RIGHT	KEY_PAGE
#define BOOT_KEY_MENU		KEY_ENTER
#define BOOT_KEY_EXIT		KEY_EXIT
#else
#define BOOT_KEY_UP			KEY_MENU
#define BOOT_KEY_DOWN		KEY_EXIT
#define BOOT_KEY_LEFT		KEY_PAGE
#define BOOT_KEY_RIGHT	KEY_MINUS
#define BOOT_KEY_MENU		KEY_PLUS
#define BOOT_KEY_EXIT		KEY_ENTER
#endif
#endif
#ifdef PCBX7
#define DISPLAY_CHAR_WIDTH	21
#else // PCBX7
#ifdef REV9E
#define DISPLAY_CHAR_WIDTH	35
#else
#define DISPLAY_CHAR_WIDTH	31
#endif
#endif // PCBX7
#endif


// Rotary encoder movement states
#define	ROTARY_MENU_LR		0
#define	ROTARY_MENU_UD		1
#define	ROTARY_SUBMENU_LR	2
#define	ROTARY_VALUE			3

template<class t> inline t min(t a, t b){ return a<b?a:b; }
template<class t> inline t max(t a, t b){ return a>b?a:b; }
template<class t> inline t limit(t mi, t x, t ma){ return min(max(mi,x),ma); }

#if defined(PCBX9D) || defined(PCB9XT) || defined(PCBX12D)
extern uint32_t Peri1_frequency ;
extern uint32_t Peri2_frequency ;
extern uint32_t Timer_mult1 ;
extern uint32_t Timer_mult2 ;
#endif

