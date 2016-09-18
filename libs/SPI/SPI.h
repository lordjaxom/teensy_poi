/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdint.h>
#include <stddef.h>
#include <teensy.h>

#include <tlibcpp/null_pin.hpp>
#include <tlibcpp/pin.hpp>
#include <tlibcpp/ports.hpp>

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use SPI.endTransaction() for
// each SPI.beginTransaction().  Connect a LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define SPI_TRANSACTION_MISMATCH_LED 5

#ifndef __SAM3X8E__
#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#endif

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR


/**********************************************************/
/*     32 bit Teensy-LC					  */
/**********************************************************/

#if defined(KINETISL)

namespace SPIDetail {

	template< uint32_t Clock, uint8_t BitOrder, uint8_t DataMode >
	struct Settings {
		static uint8_t constexpr c1 =
				SPI_C1_MSTR | SPI_C1_SPE |
				( DataMode & 0x04 ? SPI_C1_CPHA : 0 ) |
				( DataMode & 0x08 ? SPI_C1_CPOL : 0 ) |
				( BitOrder == LSBFIRST ? SPI_C1_LSBFE : 0 );

		static uint8_t constexpr br0 =
				(Clock >= F_BUS /   2) ? SPI_BR_SPPR(0) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /   4) ? SPI_BR_SPPR(1) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /   6) ? SPI_BR_SPPR(2) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /   8) ? SPI_BR_SPPR(3) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /  10) ? SPI_BR_SPPR(4) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /  12) ? SPI_BR_SPPR(5) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /  14) ? SPI_BR_SPPR(6) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /  16) ? SPI_BR_SPPR(7) | SPI_BR_SPR(0) :
				(Clock >= F_BUS /  20) ? SPI_BR_SPPR(4) | SPI_BR_SPR(1) :
				(Clock >= F_BUS /  24) ? SPI_BR_SPPR(5) | SPI_BR_SPR(1) :
				(Clock >= F_BUS /  28) ? SPI_BR_SPPR(6) | SPI_BR_SPR(1) :
				(Clock >= F_BUS /  32) ? SPI_BR_SPPR(7) | SPI_BR_SPR(1) :
				(Clock >= F_BUS /  40) ? SPI_BR_SPPR(4) | SPI_BR_SPR(2) :
				(Clock >= F_BUS /  48) ? SPI_BR_SPPR(5) | SPI_BR_SPR(2) :
				(Clock >= F_BUS /  56) ? SPI_BR_SPPR(6) | SPI_BR_SPR(2) :
				(Clock >= F_BUS /  64) ? SPI_BR_SPPR(7) | SPI_BR_SPR(2) :
				(Clock >= F_BUS /  80) ? SPI_BR_SPPR(4) | SPI_BR_SPR(3) :
				(Clock >= F_BUS /  96) ? SPI_BR_SPPR(5) | SPI_BR_SPR(3) :
				(Clock >= F_BUS / 112) ? SPI_BR_SPPR(6) | SPI_BR_SPR(3) :
				(Clock >= F_BUS / 128) ? SPI_BR_SPPR(7) | SPI_BR_SPR(3) :
				(Clock >= F_BUS / 160) ? SPI_BR_SPPR(4) | SPI_BR_SPR(4) :
				(Clock >= F_BUS / 192) ? SPI_BR_SPPR(5) | SPI_BR_SPR(4) :
				(Clock >= F_BUS / 224) ? SPI_BR_SPPR(6) | SPI_BR_SPR(4) :
				(Clock >= F_BUS / 256) ? SPI_BR_SPPR(7) | SPI_BR_SPR(4) :
				(Clock >= F_BUS / 320) ? SPI_BR_SPPR(4) | SPI_BR_SPR(5) :
				(Clock >= F_BUS / 384) ? SPI_BR_SPPR(5) | SPI_BR_SPR(5) :
				(Clock >= F_BUS / 448) ? SPI_BR_SPPR(6) | SPI_BR_SPR(5) :
				(Clock >= F_BUS / 512) ? SPI_BR_SPPR(7) | SPI_BR_SPR(5) :
				(Clock >= F_BUS / 640) ? SPI_BR_SPPR(4) | SPI_BR_SPR(6) :
				SPI_BR_SPPR(5) | SPI_BR_SPR(6);

		static uint8_t constexpr br1 =
				(Clock >= (F_PLL/2) /   2) ? SPI_BR_SPPR(0) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /   4) ? SPI_BR_SPPR(1) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /   6) ? SPI_BR_SPPR(2) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /   8) ? SPI_BR_SPPR(3) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /  10) ? SPI_BR_SPPR(4) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /  12) ? SPI_BR_SPPR(5) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /  14) ? SPI_BR_SPPR(6) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /  16) ? SPI_BR_SPPR(7) | SPI_BR_SPR(0) :
				(Clock >= (F_PLL/2) /  20) ? SPI_BR_SPPR(4) | SPI_BR_SPR(1) :
				(Clock >= (F_PLL/2) /  24) ? SPI_BR_SPPR(5) | SPI_BR_SPR(1) :
				(Clock >= (F_PLL/2) /  28) ? SPI_BR_SPPR(6) | SPI_BR_SPR(1) :
				(Clock >= (F_PLL/2) /  32) ? SPI_BR_SPPR(7) | SPI_BR_SPR(1) :
				(Clock >= (F_PLL/2) /  40) ? SPI_BR_SPPR(4) | SPI_BR_SPR(2) :
				(Clock >= (F_PLL/2) /  48) ? SPI_BR_SPPR(5) | SPI_BR_SPR(2) :
				(Clock >= (F_PLL/2) /  56) ? SPI_BR_SPPR(6) | SPI_BR_SPR(2) :
				(Clock >= (F_PLL/2) /  64) ? SPI_BR_SPPR(7) | SPI_BR_SPR(2) :
				(Clock >= (F_PLL/2) /  80) ? SPI_BR_SPPR(4) | SPI_BR_SPR(3) :
				(Clock >= (F_PLL/2) /  96) ? SPI_BR_SPPR(5) | SPI_BR_SPR(3) :
				(Clock >= (F_PLL/2) / 112) ? SPI_BR_SPPR(6) | SPI_BR_SPR(3) :
				(Clock >= (F_PLL/2) / 128) ? SPI_BR_SPPR(7) | SPI_BR_SPR(3) :
				(Clock >= (F_PLL/2) / 160) ? SPI_BR_SPPR(4) | SPI_BR_SPR(4) :
				(Clock >= (F_PLL/2) / 192) ? SPI_BR_SPPR(5) | SPI_BR_SPR(4) :
				(Clock >= (F_PLL/2) / 224) ? SPI_BR_SPPR(6) | SPI_BR_SPR(4) :
				(Clock >= (F_PLL/2) / 256) ? SPI_BR_SPPR(7) | SPI_BR_SPR(4) :
				(Clock >= (F_PLL/2) / 320) ? SPI_BR_SPPR(4) | SPI_BR_SPR(5) :
				(Clock >= (F_PLL/2) / 384) ? SPI_BR_SPPR(5) | SPI_BR_SPR(5) :
				(Clock >= (F_PLL/2) / 448) ? SPI_BR_SPPR(6) | SPI_BR_SPR(5) :
				(Clock >= (F_PLL/2) / 512) ? SPI_BR_SPPR(7) | SPI_BR_SPR(5) :
				(Clock >= (F_PLL/2) / 640) ? SPI_BR_SPPR(4) | SPI_BR_SPR(6) :
				SPI_BR_SPPR(5) | SPI_BR_SPR(6);
	};

	enum class PinType
	{
		MOSI,
		MISO,
		SCK
	};

	template< PinType Type, typename Pin > struct PinConfig;

	// SCK may not be a null_pin
	template<> struct PinConfig< PinType::SCK, tlibcpp::null_pin >;

	// MISO or MOSI can be null_pins - in which case channel doesn't matter
	template< PinType Type >
	struct PinConfig< Type, tlibcpp::null_pin >
	{
		static bool constexpr is_same_channel( uint8_t channel ) { return true; }
		static void enable() {}
		static void disable() {}
	};

	#define TLIBCPP_SPI_DEFINE_PIN_CONFIG( type_, port_, pin_, channel_, mux_ ) \
		template<> struct PinConfig< PinType::type_, tlibcpp::pin< tlibcpp::port_, pin_ > > { \
			using pin_type = tlibcpp::pin< tlibcpp::port_, pin_ >; \
			static uint8_t constexpr channel = channel_; \
			static bool constexpr is_same_channel( uint8_t channel ) { return channel == channel_; } \
			static void enable() { *pin_type::pcr_reg = PORT_PCR_DSE | PORT_PCR_MUX( mux_ ); } \
			static void disable() { *pin_type::pcr_reg = 0; } \
		}

	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portb, 16, 1, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portb, 17, 1, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portd,  6, 1, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portd,  7, 1, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portd,  2, 0, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portd,  3, 0, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portc,  6, 0, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MOSI, portc,  7, 0, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portb, 16, 1, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portb, 17, 1, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portd,  6, 1, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portd,  7, 1, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portd,  2, 0, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portd,  3, 0, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portc,  6, 0, 5 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( MISO, portc,  7, 0, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( SCK,  portd,  5, 1, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( SCK,  portc,  5, 0, 2 );
	TLIBCPP_SPI_DEFINE_PIN_CONFIG( SCK,  portd,  1, 0, 2 );

	#undef TLIBCPP_SPI_DEFINE_PIN_CONFIG

	template< typename Mosi, typename Miso, typename Sck >
	struct ChannelConfig
	{
		typedef PinConfig< PinType::MOSI, Mosi > MosiConfig;
		typedef PinConfig< PinType::MISO, Miso > MisoConfig;
		typedef PinConfig< PinType::SCK, Sck > SckConfig;

		// SCK must always be a real pin, so it is the reference
		static_assert( MosiConfig::is_same_channel( SckConfig::channel ) && MisoConfig::is_same_channel( SckConfig::channel ), "pins are not on the same SPI channel" );

		static uint8_t constexpr channel = SckConfig::channel;

		static void enable()
		{
			MosiConfig::enable();
			MisoConfig::enable();
			SckConfig::enable();
		}

		static void disable()
		{
			MosiConfig::disable();
			MisoConfig::disable();
			SckConfig::disable();
		}
	};

	template< uint8_t Channel >
	struct ChannelTraits;

	template<>
	struct ChannelTraits< 0 >
	{
		static uint32_t constexpr scgc4 = SIM_SCGC4_SPI0;
		static uint8_t constexpr volatile* c1 = &SPI0_C1;
		static uint8_t constexpr volatile* c2 = &SPI0_C2;
		static uint8_t constexpr volatile* br = &SPI0_BR;
		static uint8_t constexpr volatile* dl = &SPI0_DL;
		static uint8_t constexpr volatile* dh = &SPI0_DH;
		static uint8_t constexpr volatile* s = &SPI0_S;
	};

	template<>
	struct ChannelTraits< 1 >
	{
		static uint32_t constexpr scgc4 = SIM_SCGC4_SPI1;
		static uint8_t constexpr volatile* c1 = &SPI1_C1;
		static uint8_t constexpr volatile* c2 = &SPI1_C2;
		static uint8_t constexpr volatile* br = &SPI1_BR;
		static uint8_t constexpr volatile* dl = &SPI1_DL;
		static uint8_t constexpr volatile* dh_= &SPI1_DH;
		static uint8_t constexpr volatile* s = &SPI1_S;
	};

} // namespace SPIDetail

template< typename Mosi, typename Miso, typename Sck >
class SPI
{
	using ChannelConfig = SPIDetail::ChannelConfig< Mosi, Miso, Sck >;
	using ChannelTraits = SPIDetail::ChannelTraits< ChannelConfig::channel >;

public:
	static uint8_t constexpr channel = ChannelConfig::channel;

	SPI() {
		SIM_SCGC4 |= ChannelTraits::scgc4;
		*ChannelTraits::c1 = SPI_C1_SPE | SPI_C1_MSTR;
		*ChannelTraits::c2 = 0;
		uint8_t tmp __attribute__((unused)) = *ChannelTraits::s;
		ChannelConfig::enable();
	}

	// Disable the SPI bus
	~SPI() {
		ChannelConfig::disable();
		*ChannelTraits::c1 = 0;
		SIM_SCGC4 &= ~ChannelTraits::scgc4;
	}

	// If SPI is to used from within an interrupt, this function registers
	// that interrupt with the SPI library, so beginTransaction() can
	// prevent conflicts.  The input interruptNumber is the number used
	// with attachInterrupt.  If SPI is used from a different interrupt
	// (eg, a timer), interruptNumber should be 255.
	void usingInterrupt(uint8_t n) {
		if (n == 3 || n == 4) {
			usingInterrupt(IRQ_PORTA);
		} else if ((n >= 2 && n <= 15) || (n >= 20 && n <= 23)) {
			usingInterrupt(IRQ_PORTCD);
		}
	}
	void usingInterrupt(IRQ_NUMBER_t interruptName) {
		uint32_t n = (uint32_t)interruptName;
		if (n < NVIC_NUM_INTERRUPTS) interruptMask_ |= (1 << n);
	}
	void notUsingInterrupt(IRQ_NUMBER_t interruptName) {
		uint32_t n = (uint32_t)interruptName;
		if (n < NVIC_NUM_INTERRUPTS) interruptMask_ &= ~(1 << n);
	}

	// Before using SPI.transfer() or asserting chip select pins,
	// this function is used to gain exclusive access to the SPI bus
	// and configure the correct settings.
	template< uint32_t Clock = 4000000, uint8_t BitOrder = MSBFIRST, uint8_t DataMode = SPI_MODE0 >
	void beginTransaction() {
		if (interruptMask_) {
			__disable_irq();
			interruptSave_ = NVIC_ICER0 & interruptMask_;
			NVIC_ICER0 = interruptSave_;
			__enable_irq();
		}
		typedef SPIDetail::Settings< Clock, BitOrder, DataMode > Settings;
		*ChannelTraits::c1 = Settings::c1;
		*ChannelTraits::br = Settings::br0;
	}

	// Write to the SPI bus (MOSI pin) and also receive (MISO pin)
	uint8_t transfer(uint8_t data) {
		*ChannelTraits::dl = data;
		while (!(*ChannelTraits::s & SPI_S_SPRF)) ; // wait
		return *ChannelTraits::dl;
	}
	uint16_t transfer16(uint16_t data) {
		*ChannelTraits::c2 = SPI_C2_SPIMODE;
		*ChannelTraits::s;
		*ChannelTraits::dl = data;
		*ChannelTraits::dh = data >> 8;
		while (!(*ChannelTraits::s & SPI_S_SPRF)) ; // wait
		uint16_t r = *ChannelTraits::dl | (*ChannelTraits::dh << 8);
		*ChannelTraits::c2 = 0;
		*ChannelTraits::s;
		return r;
	}
	void transfer(void *buf, size_t count) {
		if (count == 0) return;
		uint8_t *p = reinterpret_cast<uint8_t *>(buf);
		while (!(*ChannelTraits::s & SPI_S_SPTEF)) ; // wait
		*ChannelTraits::dl = *p;
		while (--count > 0) {
			uint8_t out = *(p + 1);
			while (!(*ChannelTraits::s & SPI_S_SPTEF)) ; // wait
			__disable_irq();
			*ChannelTraits::dl = out;
			while (!(*ChannelTraits::s & SPI_S_SPRF)) ; // wait
			uint8_t in = *ChannelTraits::dl;
			__enable_irq();
			*p++ = in;
		}
		while (!(*ChannelTraits::s & SPI_S_SPRF)) ; // wait
		*p = *ChannelTraits::dl & 255;
	}
	// Write to the SPI bus (MOSI pin) only
	void send(void const*buf, size_t count) {
		if (count == 0) return;
		uint8_t const*p = reinterpret_cast<uint8_t const*>(buf);
		while (!(*ChannelTraits::s & SPI_S_SPTEF)) ; // wait
		*ChannelTraits::dl = *p;
		while (--count > 0) {
			uint8_t out = *(p + 1);
			while (!(*ChannelTraits::s & SPI_S_SPTEF)) ; // wait
			__disable_irq();
			*ChannelTraits::dl = out;
			while (!(*ChannelTraits::s & SPI_S_SPRF)) ; // wait
			__enable_irq();
			++p;
		}
		while (!(*ChannelTraits::s & SPI_S_SPRF)) ; // wait
	}

	// After performing a group of transfers and releasing the chip select
	// signal, this function allows others to access the SPI bus
	void endTransaction(void) {
		if (interruptMask_) {
			NVIC_ISER0 = interruptSave_;
		}
	}

private:
	uint32_t interruptMask_ {};
	uint32_t interruptSave_ {};
};

#endif

#endif
