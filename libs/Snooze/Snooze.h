/*
 ||
 || @file 		Snooze.h
 || @version 	5.5.2
 || @author 	duff
 || @contact    http://forum.pjrc.com/members/25610-duff
 ||
 || @description
 || # Low Power Library for Teensy 3.x/LC.
 ||
 || @license
 || | Copyright (c) 2014 Colin Duffy
 || | This library is free software; you can redistribute it and/or
 || | modify it under the terms of the GNU Lesser General Public
 || | License as published by the Free Software Foundation; version
 || | 2.1 of the License.
 || |
 || | This library is distributed in the hope that it will be useful,
 || | but WITHOUT ANY WARRANTY; without even the implied warranty of
 || | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 || | Lesser General Public License for more details.
 || |
 || | You should have received a copy of the GNU Lesser General Public
 || | License along with this library; if not, write to the Free Software
 || | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 || #
 ||
 */
#ifndef Snooze_h
#define Snooze_h

#if( !defined( __arm__ ) && defined( TEENSYDUINO ) )
#error Teensy 3.x and TeensyLC only.
#endif

#include <teensy.h>
#include <tlibcpp/ports.hpp>
#include <tlibcpp/pin.hpp>

/**
 USE_HIBERNATE is advanced feature that allows
 the Teensy to achieve 10 uA Sleep. This puts
 the USB regulator into a low power state and
 configures pin PTA3 as TSI. There are times
 that the mcu will not see any valid reprogram
 signal from the USB when in this sleep mode.
 Use this only when you verfied that your
 program sleep works correctly with deepSleep!
 */
#define USE_HIBERNATE

/**
 extended pinMode types
 */
typedef enum {
    TSI          = 3,
    CMP          = 4,
} PIN_TYPE;

typedef enum {
	INPUT = 0,
	OUTPUT = 1,
	INPUT_PULLUP = 2
} PIN_MODE;

typedef enum {
	LOW = 0,
	HIGH = 1,
	FALLING = 2,
	RISING = 3,
	CHANGE = 4
} PIN_VAL;

/**
 Deep Sleep Modes
 */
typedef enum {
    WAIT,
    VLPW,
    STOP,
    VLPS,
    LLS,
    VLLS3,
    VLLS2,
    VLLS1,
    VLLS0
} SLEEP_MODE;

/*
#include "lvd.h"
#ifdef KINETISK
#include "rtc.h"
#endif
#include "cpu.h"
*/
#include "mcg.h"
#include "llwu.h"
#include "digital.h"
#include "peripheral.h"
#include "util.h"
#include "lptmr.h"
#include "tsi.h"
#include "cmp.h"
#include "smc.h"

#if 0
/****************************************************************************************/
/**
 *  Memory block for wake configurations.
 */
class SnoozeBlock {
private:
    friend class SnoozeClass;
    typedef void*   ISR;
    digital_mask_t  digital_mask;
    lptmr_mask_t    lptmr_mask;
    llwu_mask_t     llwu_mask;
    tsi_mask_t      tsi_mask;
    cmp_mask_t      cmp_mask;
    lvd_mask_t      lvd_mask;
#ifdef KINETISK
    rtc_mask_t      rtc_mask;
#endif
    union periph_t {
        peripheral_mask_t   periph_on_mask;
        peripheral_mask_t   periph_off_mask;
        void operator = ( const SCGC4_OFF_t &rhs ) { peripheral_configure_scgc4_mask( rhs, &periph_off_mask ); }
        void operator = ( const SCGC5_OFF_t &rhs ) { peripheral_configure_scgc5_mask( rhs, &periph_off_mask ); }
        void operator = ( const SCGC6_OFF_t &rhs ) { peripheral_configure_scgc6_mask( rhs, &periph_off_mask ); }
        void operator = ( const SCGC4_ON_t &rhs  ) { peripheral_configure_scgc4_mask( rhs, &periph_on_mask ); }
        void operator = ( const SCGC5_ON_t &rhs  ) { peripheral_configure_scgc5_mask( rhs, &periph_on_mask ); }
        void operator = ( const SCGC6_ON_t &rhs  ) { peripheral_configure_scgc6_mask( rhs, &periph_on_mask ); }
    };
public:
    SnoozeBlock( void );
    /* GPIO, TSI, COMPARE Config */
    void pinMode ( int pin, int mode, int val );
    void pinMode ( int pin, int mode, int type, double val );
    /* LPTMR Config */
    void setTimer( uint16_t period );
#ifdef KINETISK
    /* RTC Config */
    void setAlarm( uint8_t hours, uint8_t minutes, uint8_t seconds );
#endif
    /* Low Voltage Config */
    void setLowVoltage( double threshold );
    /* Peripherals Config */
    periph_t setPeripheral;
};
#endif

namespace SnoozeDetail {

	template< typename Pin, uint8_t Val >
	struct LlwuPinMask;

#define SNOOZE_DEFINE_LLWU_PIN_MASK( port_, pin_, pe_, wupe_ ) \
	template< uint8_t Val > \
	struct LlwuPinMask< tlibcpp::pin< tlibcpp::port_, pin_ >, Val > { \
		static void configure( llwu_mask_t& mask ) { \
			mask.PE ## pe_ |= LLWU_PE ## pe_ ## _WUPE ## wupe_ ( \
				Val == RISING ? LLWU_PIN_RISING : \
				Val == FALLING ? LLWU_PIN_FALLING : \
				LLWU_PIN_ANY ); \
		} \
	}

	SNOOZE_DEFINE_LLWU_PIN_MASK( portd, 0, 4, 12 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( porta, 2, 2, 4 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portd, 4, 4, 14 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portd, 2, 4, 13 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portc, 3, 2, 7 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portc, 4, 3, 8 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portc, 6, 3, 10 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portc, 5, 3, 9 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portb, 0, 2, 5 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portd, 6, 4, 15 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( portc, 1, 2, 6 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( porte, 30, 1, 0 );
#if defined( KINETISK )
	SNOOZE_DEFINE_LLWU_PIN_MASK( 30, 3, 11 );
	SNOOZE_DEFINE_LLWU_PIN_MASK( 33, 1, 3 );
#endif

#undef SNOOZE_DEFINE_LLWU_PIN_MASK

} // namespace SnoozeDetail

namespace SnoozeConfig {

	template< typename Pin, uint8_t Mode, uint8_t Val >
	struct PinMode
	{
		using PinType = Pin;
		using LlwuMask = SnoozeDetail::LlwuPinMask< Pin, Val >;

		static constexpr bool digital = true;
		static constexpr bool llwu = true;
		static constexpr uint8_t mode = Mode;
		static constexpr uint8_t val = Val;
	};

} // namespace SnoozeConfig

/****************************************************************************************/
/**
 *  Low Power modes
 */
class Snooze {
private:
    static SLEEP_MODE sleep_mode;
    static CLOCK_MODE clock_mode;
    friend void wakeup_isr( void );
public:
    Snooze()
    {
		SIM_SOPT1CFG |= SIM_SOPT1CFG_USSWE;
		SIM_SOPT1 &= ~SIM_SOPT1_USBSSTBY;
		//attachInterruptVector( IRQ_LLWU, wakeupISR );
		NVIC_SET_PRIORITY( IRQ_LLWU, 32 );
		clock_mode = mcg_mode( );
	}
    /* helpers functions */
    //void debug      ( Stream *port );
    //int  source     ( void );
    /* sleep functions */
    //void idle       ( void );
    //int  sleep      ( SnoozeBlock &configuration );
    //int  deepSleep  ( SnoozeBlock &configuration, SLEEP_MODE mode = LLS );
    //bool reduceCpuSpeed( uint32_t freq ) {}
#if defined( USE_HIBERNATE )
	template< typename Config, SLEEP_MODE Mode = LLS >
    int hibernate()
    {
		enable_periph_irq = false;
		sleep_mode = Mode;
		// cmp_set( &p->cmp_mask );
		digital_set< Config, false >();
		// lptmr_set( &p->lptmr_mask );
#ifdef KINETISK
		rtc_alarm_set( &p->rtc_mask );
#endif
#ifdef KINETISL
		tlibcpp::digital_pin< 17 >::output();
		tlibcpp::digital_pin< 17 >::clear();
#endif
		// tsi_set( &p->tsi_mask );
		llwu_set< Config >();
		enableHibernate( );
		if ( Mode == LLS )        { enter_lls( ); }
		else if ( Mode == VLLS3 ) { enter_vlls3( ); }
		else if ( Mode == VLLS2 ) { enter_vlls2( ); }
		else if ( Mode == VLLS1 ) { enter_vlls1( ); }
		else if ( Mode == VLLS0 ) { enter_vlls0( ); }
		disableHibernate( );
		llwu_disable( );
		// tsi_disable< Config >();
#ifdef KINETISK
		rtc_disable( &p->rtc_mask );
#endif
		// lptmr_disable( &p->lptmr_mask );
		// cmp_disable( &p->cmp_mask );
		return wakeupSource;
	}
#endif
};
#endif
