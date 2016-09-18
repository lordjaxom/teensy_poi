/***********************************************************************************
 *  digital.h
 *  Teensy 3.x/LC
 *
 * Purpose: Digital Pins
 *
 ***********************************************************************************/
#ifndef __DIGITAL_H__
#define __DIGITAL_H__
#include <tlibcpp/teensy_pins.hpp>
#include "util.h"
/***********************************************************************************/
#if defined(KINETISK)
static void ( * return_porta_irq ) ( void );
static void ( * return_portb_irq ) ( void );
static void ( * return_portc_irq ) ( void );
static void ( * return_portd_irq ) ( void );
static void ( * return_porte_irq ) ( void );
#elif defined(KINETISL)
//static void ( * return_porta_irq ) ( void );
//static void ( * return_portcd_irq )( void );
#endif
/***********************************************************************************/
typedef struct {
#if defined(KINETISK)
    uint64_t pin = 0;
    uint8_t mode_irqType[33];
    bool state = false;
#elif defined(KINETISL)
    uint32_t pin = 0;
    uint8_t mode_irqType[23];
    bool state = false;
#endif
} digital_mask_t;
/***********************************************************************************/

/*******************************************************************************
 *
 *       digital_configure_pin_mask
 *
 *******************************************************************************/
static inline
void digital_configure_pin_mask( uint8_t pin, uint8_t mode, uint8_t type, digital_mask_t *mask )
__attribute__((always_inline, unused));

static inline
void digital_configure_pin_mask( uint8_t pin, uint8_t mode, uint8_t type, digital_mask_t *mask ) {
#if defined(KINETISK)
	mask->pin = mask->pin | ( ( uint64_t )0x01 << pin );// save pin
	mask->mode_irqType[pin] = type;// save type
	mask->mode_irqType[pin] |= mode << 4;// save mode
	mask->state = true;
#elif defined(KINETISL)
	mask->pin = mask->pin | ( ( uint32_t )0x01 << pin );// save pin
	mask->mode_irqType[pin] = type;// save type
	mask->mode_irqType[pin] |= mode << 4;// save mode
	mask->state = true;
#endif
}
/*******************************************************************************
 *
 *       digitalISR
 *
 *******************************************************************************/
static inline
void digitalISR( void )
__attribute__((always_inline, unused));

static inline
void digitalISR( void ) {
	uint32_t isfr_a = PORTA_ISFR;
	PORTA_ISFR = isfr_a;
#if defined(KINETISK)
	uint32_t isfr_b = PORTB_ISFR;
	uint32_t isfr_e = PORTE_ISFR;
	PORTB_ISFR = isfr_b;
	PORTE_ISFR = isfr_e;
#endif
	uint32_t isfr_c = PORTC_ISFR;
	uint32_t isfr_d = PORTD_ISFR;
	PORTC_ISFR = isfr_c;
	PORTD_ISFR = isfr_d;

	if ( isfr_a & (1 << tlibcpp::digital_pin<3>::bit) )       { wakeupSource = 3;  return; }
	else if ( isfr_a & (1 << tlibcpp::digital_pin<4>::bit) )  { wakeupSource = 4;  return; }
#if defined(KINETISK)
	else if ( isfr_a & (1 << tlibcpp::digital_pin<24>::bit) ) { wakeupSource = 24; return; }
	else if ( isfr_a & (1 << tlibcpp::digital_pin<33>::bit) ) { wakeupSource = 33; return; }

	if ( isfr_b & (1 << tlibcpp::digital_pin<0>::bit) )       { wakeupSource = 0;  return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<1>::bit) )  { wakeupSource = 1;  return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<16>::bit) ) { wakeupSource = 16; return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<17>::bit) ) { wakeupSource = 17; return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<18>::bit) ) { wakeupSource = 18; return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<19>::bit) ) { wakeupSource = 19; return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<25>::bit) ) { wakeupSource = 25; return; }
	else if ( isfr_b & (1 << tlibcpp::digital_pin<32>::bit) ) { wakeupSource = 32; return; }
#endif
	if ( isfr_c & (1 << tlibcpp::digital_pin<9>::bit) )       { wakeupSource = 9;  return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<10>::bit) ) { wakeupSource = 10; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<11>::bit) ) { wakeupSource = 11; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<12>::bit) ) { wakeupSource = 12; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<13>::bit) ) { wakeupSource = 13; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<15>::bit) ) { wakeupSource = 15; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<22>::bit) ) { wakeupSource = 22; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<23>::bit) ) { wakeupSource = 23; return; }
#if defined(KINETISK)
	else if ( isfr_c & (1 << tlibcpp::digital_pin<27>::bit) ) { wakeupSource = 27; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<28>::bit) ) { wakeupSource = 28; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<29>::bit) ) { wakeupSource = 29; return; }
	else if ( isfr_c & (1 << tlibcpp::digital_pin<30>::bit) ) { wakeupSource = 30; return; }
#endif
	if ( isfr_d & (1 << tlibcpp::digital_pin<2>::bit) )       { wakeupSource = 2;  return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<5>::bit) )  { wakeupSource = 5;  return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<6>::bit) )  { wakeupSource = 6;  return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<7>::bit) )  { wakeupSource = 7;  return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<8>::bit) )  { wakeupSource = 8;  return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<14>::bit) ) { wakeupSource = 14; return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<20>::bit) ) { wakeupSource = 20; return; }
	else if ( isfr_d & (1 << tlibcpp::digital_pin<21>::bit) ) { wakeupSource = 21; return; }

#if defined(KINETISK)
	if ( isfr_e & (1 << tlibcpp::digital_pin<26>::bit) )      { wakeupSource = 26; return; }
	else if ( isfr_e & (1 << tlibcpp::digital_pin<31>::bit) ) { wakeupSource = 31; return; }
#endif
}

/*******************************************************************************
 *
 *       digital_set
 *
 *******************************************************************************/
template< typename Config, bool EnableIrq >
inline void digital_set() {
	if ( !Config::digital ) return;
#if defined(KINETISK)
	uint64_t _pin = mask->pin;
	if ( enable_periph_irq ) {// if using sleep must setup pin interrupt to wake
		while ( __builtin_popcountll( _pin ) ) {
			uint32_t pin  = 63 - __builtin_clzll( _pin );// get pin
			if ( pin > 33 ) return;
			int priority = nvic_execution_priority( );// get current priority
			// if running from interrupt set priority higher than current interrupt
			priority = ( priority  < 256 ) && ( ( priority - 16 ) > 0 ) ? priority - 16 : 128;
			if ( pin == 3 || pin == 4 || pin == 24 || pin == 33 ) {
				NVIC_SET_PRIORITY( IRQ_PORTA, priority );//set priority to new level
				__disable_irq( );
				return_porta_irq = _VectorsRam[IRQ_PORTA+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTA, digitalISR );// set snooze isr
				__enable_irq( );
			}
			else if ( pin == 0 || pin == 1 || pin == 16 || pin == 17 || pin == 18 || pin == 19 || pin == 25 || pin == 32 ) {
				NVIC_SET_PRIORITY( IRQ_PORTB, priority );//set priority to new level
				__disable_irq( );
				return_portb_irq = _VectorsRam[IRQ_PORTB+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTB, digitalISR );// set snooze isr
				__enable_irq( );
			}
			else if ( pin == 9 || pin == 10 || pin == 11 || pin == 12 || pin == 13 || pin == 15 || pin == 22 || pin == 23 || pin == 27 || pin == 28 || pin == 29 || pin == 30 ) {
				NVIC_SET_PRIORITY( IRQ_PORTC, priority );//set priority to new level
				__disable_irq( );
				return_portc_irq = _VectorsRam[IRQ_PORTC+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTC, digitalISR );// set snooze isr
				__enable_irq( );
			}
			else if ( pin == 2 || pin == 5 || pin == 6 || pin == 7 || pin == 8 || pin == 14 || pin == 20 || pin == 21 )  {
				NVIC_SET_PRIORITY( IRQ_PORTD, priority );//set priority to new level
				__disable_irq( );
				return_portd_irq = _VectorsRam[IRQ_PORTD+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTD, digitalISR );// set snooze isr
				__enable_irq( );
			}
			else if ( pin == 26 || pin == 31 ) {
				NVIC_SET_PRIORITY( IRQ_PORTE, priority );//set priority to new level
				__disable_irq( );
				return_porte_irq = _VectorsRam[IRQ_PORTE+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTE, digitalISR );// set snooze isr
				__enable_irq( );
			}

			_pin &= ~( ( uint64_t )1<<pin );// remove pin from list
		}
		__disable_irq( );// disable all interrupts for attaching an interrupt to a pin
	}
	_pin = mask->pin;
	while ( __builtin_popcountll( _pin ) ) {
		uint32_t pin  = 63 - __builtin_clzll( _pin );// get pin
		uint32_t mode = mask->mode_irqType[pin] >> 4;// get type
		uint32_t type = mask->mode_irqType[pin] & 0x0F;// get mode

		volatile uint32_t *config;
		config = portConfigRegister( pin );

		if ( mode == INPUT || mode == INPUT_PULLUP ) {// setup pin mode/type/interrupt
			*portModeRegister( pin ) = 0;
			if ( mode == INPUT ) *config = PORT_PCR_MUX( 1 );
			else *config = PORT_PCR_MUX( 1 ) | PORT_PCR_PE | PORT_PCR_PS;// pullup
			if ( enable_periph_irq ) attachDigitalInterrupt( pin, type );// set pin interrupt
		} else {
			pinMode( pin, mode );
			digitalWriteFast( pin, type );
		}
		_pin &= ~( ( uint64_t )1<<pin );// remove pin from list
	}

#elif defined(KINETISL)
	static_assert( !EnableIrq, "not implemented" );
#if 0
	if ( EnableIrq ) {// if using sleep must setup pin interrupt to wake
		while ( __builtin_popcount( _pin ) ) {
			uint32_t pin  = 31 - __builtin_clz( _pin );// get pin

			int priority = nvic_execution_priority( );// get current priority
			// if running from handler set priority higher than current handler
			priority = ( priority  < 256 ) && ( (priority - 16) > 0 ) ? priority - 16 : 128;
			if ( pin == 3 || pin == 4 ) {
				NVIC_SET_PRIORITY( IRQ_PORTA, priority );//set priority to new level
				__disable_irq( );
				return_porta_irq = _VectorsRam[IRQ_PORTA+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTA, digitalISR );// set snooze isr
				__enable_irq( );
			}
			else if ( pin == 2 || pin == 5 || pin == 6 || pin == 7 || pin == 8 || pin == 14 || pin == 20 || pin == 21 || pin == 9 || pin == 10 || pin == 11 || pin == 12 || pin == 13 || pin == 15 || pin == 22 || pin == 23 ) {
				NVIC_SET_PRIORITY( IRQ_PORTCD, priority );//set priority to new level
				__disable_irq( );
				return_portcd_irq = _VectorsRam[IRQ_PORTCD+16];// save prev isr handler
				attachInterruptVector( IRQ_PORTCD, digitalISR );// set snooze isr
				__enable_irq( );
			}
			_pin &= ~( ( uint32_t )1<<pin );// remove pin from list
		}
		__disable_irq( );// disable all interrupts for attaching an interrupt to a pin
	}
#endif

	if ( Config::mode == INPUT || Config::mode == INPUT_PULLUP ) {// setup pin mode/type/interrupt
		*Config::PinType::pddr_reg &= ~( 1 << Config::PinType::bit );
		if ( Config::mode == INPUT ) *Config::PinType::pcr_reg = PORT_PCR_MUX( 1 );
		else *Config::PinType::pcr_reg = PORT_PCR_MUX( 1 ) | PORT_PCR_PE | PORT_PCR_PS;// pullup
		static_assert( !EnableIrq, "not implemented" );
		// if ( EnableIrq ) attachDigitalInterrupt( pin, type );// set pin interrupt
	} else {
		Config::PinType::output();
		Config::PinType::set( Config::val == HIGH );
	}
#endif
}
#endif
/*******************************************************************************
 *
 *       digital_disable
 *
 ******************************************************************************/
#if 0
static inline
void digital_disable( digital_mask_t *mask )
__attribute__((always_inline, unused));

static inline
void digital_disable( digital_mask_t *mask ) {
	if ( mask->state == false ) return;
	if (  enable_periph_irq ) {
#if defined(KINETISK)
		uint64_t _pin = mask->pin;
		while ( __builtin_popcountll( _pin ) ) {
			uint32_t pin = 63 - __builtin_clzll( _pin );
			if ( pin > 33 ) {
				__enable_irq( );// enable interrupts from digital_set function
				return;
			}
			detachDigitalInterrupt( pin );// remove pin interrupt
			_pin &= ~( ( uint64_t )1<<pin );// remove pin from list
		}
		__enable_irq( );// enable interrupts from digital_set function

		_pin = mask->pin;
		while ( __builtin_popcountll( _pin ) ) {

			uint32_t pin = 63 - __builtin_clzll( _pin );
			if ( enable_periph_irq ) {// if using sleep, give back previous isr handler
				if ( pin == 3 || pin == 4 || pin == 24 || pin == 33 ) {
					NVIC_SET_PRIORITY( IRQ_PORTA, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTA, return_porta_irq );// return prev interrupt handler
					__enable_irq( );
				}
				else if ( pin == 0 || pin == 1 || pin == 16 || pin == 17 || pin == 18 || pin == 19 || pin == 25 || pin == 32 ) {
					NVIC_SET_PRIORITY( IRQ_PORTB, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTB, return_portb_irq );// return prev interrupt handler
					__enable_irq( );
				}
				else if ( pin == 9 || pin == 10 || pin == 11 || pin == 12 || pin == 13 || pin == 15 || pin == 22 || pin == 23 || pin == 27 || pin == 28 || pin == 29 || pin == 30 ) {
					NVIC_SET_PRIORITY( IRQ_PORTC, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTC, return_portc_irq );// return prev interrupt handler
					__enable_irq( );
				}
				else if ( pin == 2 || pin == 5 || pin == 6 || pin == 7 || pin == 8 || pin == 14 || pin == 20 || pin == 21 ) {
					NVIC_SET_PRIORITY( IRQ_PORTD, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTD, return_portd_irq );// return prev interrupt handler
					__enable_irq( );
				}
				else if ( pin == 26 || pin == 31 ) {
					NVIC_SET_PRIORITY( IRQ_PORTE, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTE, return_porte_irq );// return prev interrupt handler
					__enable_irq( );
				}
			}
			_pin &= ~( ( uint64_t )1<<pin );// remove pin from list
		}
#elif defined(KINETISL)
		uint32_t _pin = mask->pin;
		while ( __builtin_popcount( _pin ) ) {
			uint32_t pin = 31 - __builtin_clz( _pin );
			if ( pin > 33 ) {
				__enable_irq( );// enable interrupts from digital_set function
				return;
			}
			detachDigitalInterrupt( pin );// remove pin interrupt
			_pin &= ~( ( uint32_t )1<<pin );// remove pin from list
		}
		__enable_irq( );// enable interrupts from digital_set function

		_pin = mask->pin;
		while ( __builtin_popcount( _pin ) ) {
			uint32_t pin = 31 - __builtin_clz( _pin );
			if ( enable_periph_irq ) {// if using sleep give back previous isr handler
				if ( pin == 3 || pin == 4 ) {
					NVIC_SET_PRIORITY( IRQ_PORTA, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTA, return_porta_irq );// return prev interrupt handler
					__enable_irq( );
				}
				else if ( pin == 2 || pin == 5 || pin == 6 || pin == 7 || pin == 8 || pin == 14 || pin == 20 || pin == 21 || pin == 9 || pin == 10 || pin == 11 || pin == 12 || pin == 13 || pin == 15 || pin == 22 || pin == 23 ) {
					NVIC_SET_PRIORITY( IRQ_PORTCD, 128 );//return priority to core level
					__disable_irq( );
					attachInterruptVector( IRQ_PORTCD, return_portcd_irq );// return prev interrupt handler
					__enable_irq( );
				}
			}
			_pin &= ~( ( uint32_t )1<<pin );// remove pin from list
		}
#endif
	}
}
/***********************************************************************************/
#endif /* __DIGITAL_H__ */
