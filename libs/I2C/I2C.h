/*
    ------------------------------------------------------------------------------------------------------
    i2c_t3 - I2C library for Teensy 3.0/3.1/LC

    - (v8) Modified 02Apr15 by Brian (nox771 at gmail.com)
        - added support for Teensy LC:
            - fully supported (Master/Slave modes, IMM/ISR/DMA operation)
            - Wire: pins 16/17 or 18/19, rate limited to I2C_RATE_1200
            - Wire1: pins 22/23, rate limited to I2C_RATE_2400
        - added timeout on acquiring bus (prevents lockup when bus cannot be acquired)
        - added setDefaultTimeout() function for setting the default timeout to apply to all commands
        - added resetBus() function for toggling SCL to release stuck Slave devices
        - added setRate(rate) function, similar to setClock(freq), but using rate specifiers (does not
                require specifying busFreq)
        - added I2C_AUTO_RETRY user define

    - (v7) Modified 09Jan15 by Brian (nox771 at gmail.com)
        - added support for F_BUS frequencies: 60MHz, 56MHz, 48MHz, 36MHz, 24MHz, 16MHz, 8MHz, 4MHz, 2MHz
        - added new rates: I2C_RATE_1800, I2C_RATE_2800, I2C_RATE_3000
        - added new priority escalation - in cases where I2C ISR is blocked by having a lower priority than
                                          calling function, the I2C will either adjust I2C ISR to a higher priority,
                                          or switch to Immediate mode as needed.
        - added new operating mode control - I2C can be set to operate in ISR mode, DMA mode (Master only),
                                             or Immediate Mode (Master only)
        - added new begin() functions to allow setting the initial operating mode:
            - begin(i2c_mode mode, uint8_t address, i2c_pins pins, i2c_pullup pullup, i2c_rate rate, i2c_op_mode opMode)
            - begin(i2c_mode mode, uint8_t address1, uint8_t address2, i2c_pins pins, i2c_pullup pullup, i2c_rate rate, i2c_op_mode opMode)
        - added new functions:
            - uint8_t setOpMode(i2c_op_mode opMode) - used to change operating mode on the fly (only when bus is idle)
            - void sendTransmission() - non-blocking Tx with implicit I2C_STOP, added for symmetry with endTransmission()
            - uint8_t setRate(uint32_t busFreq, i2c_rate rate) - used to set I2C clock dividers to get desired rate, i2c_rate argument
            - uint8_t setRate(uint32_t busFreq, uint32_t i2cFreq) - used to set I2C clock dividers to get desired SCL freq, uint32_t argument
                                                                    (quantized to nearest i2c_rate)
        - added new Wire compatibility functions:
            - void setClock(uint32_t i2cFreq) - (note: degenerate form of setRate() with busFreq == F_BUS)
            - uint8_t endTransmission(uint8_t sendStop)
            - uint8_t requestFrom(uint8_t addr, uint8_t len)
            - uint8_t requestFrom(uint8_t addr, uint8_t len, uint8_t sendStop)
        - fixed bug in Slave Range code whereby onRequest() callback occurred prior to updating rxAddr instead of after
        - fixed bug in arbitration, was missing from Master Tx mode
        - removed I2C1 defines (now included in kinetis.h)
        - removed all debug code (eliminates rbuf dependency)

    - (v6) Modified 16Jan14 by Brian (nox771 at gmail.com)
        - all new structure using dereferenced pointers instead of hardcoding. This allows functions
          (including ISRs) to be reused across multiple I2C buses.  Most functions moved to static,
          which in turn are called by inline user functions.  Added new struct (i2cData) for holding all
          bus information.
        - added support for Teensy 3.1 and I2C1 interface on pins 29/30 and 26/31.
        - added header define (I2C_BUS_ENABLE n) to control number of enabled buses (eg. both I2C1 & I2C1
          or just I2C1).  When using only I2C1 the code and ram usage will be lower.
        - added interrupt flag (toggles pin high during ISR) with independent defines for I2C1 and
          I2C1 (refer to header file), useful for logic analyzer trigger

    - (v5) Modified 09Jun13 by Brian (nox771 at gmail.com)
        - fixed bug in ISR timeout code in which timeout condition could fail to reset in certain cases
        - fixed bug in Slave mode in sda_rising_isr attach, whereby it was not getting attached on the addr byte
        - moved debug routines so they are entirely defined internal to the library (no end user code req'd)
        - debug routines now use IntervalTimer library
        - added support for range of Slave addresses
        - added getRxAddr() for Slave using addr range to determine its called address
        - removed virtual keyword from all functions (is not a base class)

    - (v1-v4) Modified 26Feb13 by Brian (nox771 at gmail.com)
        - Reworked begin function:
            - added option for pins to use (SCL:SDA on 19:18 or 16:17 - note pin order difference)
            - added option for internal pullup - as mentioned in previous code pullup is very strong,
                                                 approx 190 ohms, but is possibly useful for high speed I2C
            - added option for rates - 100kHz, 200kHz, 300kHz, 400kHz, 600kHz, 800kHz, 1MHz, 1.2MHz, <-- 24/48MHz bus
                                       1.5MHz, 2.0MHz, 2.4MHz                                        <-- 48MHz bus only
        - Removed string.h dependency (memcpy)
        - Changed Master modes to interrupt driven
        - Added non-blocking Tx/Rx routines, and status/done/finish routines:
            - sendTransmission() - non-blocking transmit
            - sendRequest() - non-blocking receive
            - status() - reports current status
            - done() - indicates Tx/Rx complete (for main loop polling if I2C is running in background)
            - finish() - loops until Tx/Rx complete or bus error
        - Added readByte()/peekByte() for uint8_t return values (note: returns 0 instead of -1 if buf empty)
        - Added fixes for Slave Rx mode - in short Slave Rx on this part is fubar
          (as proof, notice the difference in the I2Cx_FLT register in the KL25 Sub-Family parts)
            - the SDA-rising ISR hack can work but only detects STOP conditons.
              A slave Rx followed by RepSTART won't be detected since bus remains busy.
              To fix this if IAAS occurs while already in Slave Rx mode then it will
              assume RepSTART occurred and trigger onReceive callback.
        - Separated Tx/Rx buffer sizes for asymmetric devices (adjustable in i2c_t3.h)
        - Changed Tx/Rx buffer indicies to size_t to allow for large (>256 byte) buffers
        - Left debug routines in place (controlled via header defines - default is OFF).  If debug is
            enabled, note that it can easily overrun the Debug queue on large I2C transfers, yielding
            garbage output.  Adjust ringbuf size (in rbuf.h) and possibly PIT interrupt rate to adjust
            data flow to Serial (note also the buffer in Serial can overflow if written too quickly).
        - Added getError() function to return Wire error code
        - Added pinConfigure() function for changing pins on the fly (only when bus not busy)
        - Added timeouts to endTransmission(), requestFrom(), and finish()
    ------------------------------------------------------------------------------------------------------
    Some code segments derived from:
    TwoWire.cpp - TWI/I2C library for Wiring & Arduino
    Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
    Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
    ------------------------------------------------------------------------------------------------------
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if !defined(I2C_T3_H) && (defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__)) // 3.0/3.1/LC
#define I2C_T3_H

#include <inttypes.h>
#include <stddef.h> // for size_t
#include <string.h>
#include <tstdlib.h>
#include <libcpp/function.hpp>
#include <tlibcpp/ports.hpp>
#include <tlibcpp/pin.hpp>


enum class I2CMasterMode
{
	IMM, // Immediate
	ISR  // ISR
};


namespace I2CDetail {

	enum class PinType
	{
		SDA,
		SCL
	};

	template< PinType Type, typename Pin, bool Pullup > struct PinConfig;

	#define TLIBCPP_I2C_DEFINE_PIN_CONFIG( type_, port_, pin_, bus_, mux_ ) \
	template< bool Pullup > struct PinConfig< PinType::type_, tlibcpp::pin< tlibcpp::port_, pin_ >, Pullup > { \
		using pin_type = tlibcpp::pin< tlibcpp::port_, pin_ >; \
		static constexpr uint8_t bus = bus_; \
		static void enable() { *pin_type::pcr_reg = Pullup ? PORT_PCR_MUX( mux_ ) | PORT_PCR_PE | PORT_PCR_PS : PORT_PCR_MUX( mux_ ) | PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE; } \
		static void disable() { *pin_type::pcr_reg = 0; } \
	}

	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SDA, portb,  3, 0, 2 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SCL, portb,  2, 0, 2 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SDA, portb,  1, 0, 2 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SCL, portb,  0, 0, 2 );
	#if defined( __MK20DX256__ ) // TODO
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SDA, 30, 1, 2 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SCL, 29, 1, 2 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SDA, 31, 1, 6 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SCL, 26, 1, 6 );
	#elif defined( __MKL26Z64__ )
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SDA, portc,  2, 1, 2 );
	TLIBCPP_I2C_DEFINE_PIN_CONFIG( SCL, portc,  1, 1, 2 );
	#endif

	#undef TLIBCPP_I2C_DEFINE_PIN_CONFIG


	template< typename Sda, typename Scl, bool Pullup >
	struct BusConfig
	{
		typedef PinConfig< PinType::SDA, Sda, Pullup > SdaConfig;
		typedef PinConfig< PinType::SCL, Scl, Pullup > SclConfig;

		static_assert( SdaConfig::bus == SclConfig::bus, "pins are not on the same I2C bus" );

		static constexpr uint8_t bus = SdaConfig::bus;

		static void enable()
		{
			SdaConfig::enable();
			SclConfig::enable();
		}

		static void disable()
		{
			SdaConfig::disable();
			SclConfig::disable();
		}
	};


	template< uint8_t Bus >
	struct BusTraits;

	template<>
	struct BusTraits< 0 >
	{
		static constexpr uint32_t scgc4 = SIM_SCGC4_I2C0;
		static constexpr IRQ_NUMBER_t irq = IRQ_I2C0;
		static constexpr uint8_t volatile* A1 = &I2C0_A1;
		static constexpr uint8_t volatile* F = &I2C0_F;
		static constexpr uint8_t volatile* C1 = &I2C0_C1;
		static constexpr uint8_t volatile* S = &I2C0_S;
		static constexpr uint8_t volatile* D = &I2C0_D;
		static constexpr uint8_t volatile* C2 = &I2C0_C2;
		static constexpr uint8_t volatile* FLT = &I2C0_FLT;
		static constexpr uint8_t volatile* RA = &I2C0_RA;
		static constexpr uint8_t volatile* SMB = &I2C0_SMB;
		static constexpr uint8_t volatile* A2 = &I2C0_A2;
		static constexpr uint8_t volatile* SLTH = &I2C0_SLTH;
		static constexpr uint8_t volatile* SLTL = &I2C0_SLTL;
	};

#if defined( __MK20DX256__ ) || defined( __MKL26Z64__ )
	template<>
	struct BusTraits< 1 >
	{
		static constexpr uint32_t scgc4 = SIM_SCGC4_I2C1;
		static constexpr IRQ_NUMBER_t irq = IRQ_I2C1;
		static constexpr uint8_t volatile* A1 = &I2C1_A1;
		static constexpr uint8_t volatile* F = &I2C1_F;
		static constexpr uint8_t volatile* C1 = &I2C1_C1;
		static constexpr uint8_t volatile* S = &I2C1_S;
		static constexpr uint8_t volatile* D = &I2C1_D;
		static constexpr uint8_t volatile* C2 = &I2C1_C2;
		static constexpr uint8_t volatile* FLT = &I2C1_FLT;
		static constexpr uint8_t volatile* RA = &I2C1_RA;
		static constexpr uint8_t volatile* SMB = &I2C1_SMB;
		static constexpr uint8_t volatile* A2 = &I2C1_A2;
		static constexpr uint8_t volatile* SLTH = &I2C1_SLTH;
		static constexpr uint8_t volatile* SLTL = &I2C1_SLTL;
	};
#endif


	template< uint8_t Bus >
	struct IsrHandler
	{
	    static libcpp::function< void () > handler;
	};

	template< uint8_t Bus >
	libcpp::function< void () > IsrHandler< Bus >::handler;


	template< size_t Bus, size_t Freq >
	struct Settings;

	template< size_t Freq >
	struct Settings< 60000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >= 2050000 ? 0x00 :
				Freq >= 1950000 ? 0x01 :
				Freq >= 1800000 ? 0x02 :
				Freq >= 1520000 ? 0x05 :
				Freq >= 1330000 ? 0x06 :
				Freq >= 1100000 ? 0x0b :
				Freq >=  950000 ? 0x0d :
				Freq >=  800000 ? 0x45 :
				Freq >=  650000 ? 0x13 :
				Freq >=  475000 ? 0x19 :
				Freq >=  350000 ? 0x4f :
				Freq >=  250000 ? 0x55 :
				Freq >=  150000 ? 0x24 :
				0x2c;
		static constexpr uint8_t flt = 4;
	};

	template< size_t Freq >
	struct Settings< 56000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >= 1950000 ? 0x00 :
				Freq >= 1800000 ? 0x02 :
				Freq >= 1520000 ? 0x04 :
				Freq >= 1330000 ? 0x09 :
				Freq >= 1100000 ? 0x0a :
				Freq >=  950000 ? 0x0c :
				Freq >=  800000 ? 0x0e :
				Freq >=  650000 ? 0x0f :
				Freq >=  475000 ? 0x15 :
				Freq >=  350000 ? 0x4f :
				Freq >=  250000 ? 0x1e :
				Freq >=  150000 ? 0x24 :
				0x2c;
		static constexpr uint8_t flt = 4;
	};

	template< size_t Freq >
	struct Settings< 48000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >= 1800000 ? 0x00 :
				Freq >= 1520000 ? 0x02 :
				Freq >= 1330000 ? 0x03 :
				Freq >= 1100000 ? 0x09 :
				Freq >=  950000 ? 0x0b :
				Freq >=  800000 ? 0x0d :
				Freq >=  650000 ? 0x45 :
				Freq >=  475000 ? 0x14 :
				Freq >=  350000 ? 0x85 :
				Freq >=  250000 ? 0x1c :
				Freq >=  150000 ? 0x5a :
				0x27;
		static constexpr uint8_t flt = 4;
	};

	template< size_t Freq >
	struct Settings< 36000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >= 1330000 ? 0x00 :
				Freq >= 1100000 ? 0x02 :
				Freq >=  950000 ? 0x05 :
				Freq >=  800000 ? 0x0a :
				Freq >=  650000 ? 0x0c :
				Freq >=  475000 ? 0x45 :
				Freq >=  350000 ? 0x15 :
				Freq >=  250000 ? 0x85 :
				Freq >=  150000 ? 0x55 :
				0x95;
		static constexpr uint8_t flt = 3;
	};

	template< size_t Freq >
	struct Settings< 24000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >=  950000 ? 0x00 :
				Freq >=  800000 ? 0x02 :
				Freq >=  650000 ? 0x05 :
				Freq >=  475000 ? 0x0b :
				Freq >=  350000 ? 0x45 :
				Freq >=  250000 ? 0x14 :
				Freq >=  150000 ? 0x85 :
				0x1f;
		static constexpr uint8_t flt = 2;
	};

	template< size_t Freq >
	struct Settings< 16000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >=  650000 ? 0x00 :
				Freq >=  475000 ? 0x03 :
				Freq >=  350000 ? 0x0b :
				Freq >=  250000 ? 0x43 :
				Freq >=  150000 ? 0x14 :
				0x1d;
		static constexpr uint8_t flt = 1;
	};

	template< size_t Freq >
	struct Settings< 8000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >=  350000 ? 0x00 :
				Freq >=  250000 ? 0x03 :
				Freq >=  150000 ? 0x0b :
				0x14;
		static constexpr uint8_t flt = 1;
	};

	template< size_t Freq >
	struct Settings< 4000000, Freq >
	{
		static constexpr uint8_t f =
				Freq >=  350000 ? 0x00 :
				0x0b;
		static constexpr uint8_t flt = 0;
	};

	template< size_t Freq >
	struct Settings< 2000000, Freq >
	{
		static constexpr uint8_t f = 0x00;
		static constexpr uint8_t flt = 0;
	};


	enum class Status : uint8_t
	{
		WAITING,
		SENDING,
		SEND_ADDR,
		RECEIVING,
		TIMEOUT,
		ADDR_NAK,
		DATA_NAK,
		ARB_LOST,
		BUF_OVF,
		SLAVE_TX,
		SLAVE_RX
	};


} // namespace I2CDetail


template< typename Sda, typename Scl, bool Pullup, size_t Freq, I2CMasterMode OpMode = I2CMasterMode::ISR, uint32_t Timeout = 0, size_t TxSize = 259, size_t RxSize = 259, bool AutoRetry = true >
class I2CMaster
{
	using BusConfig = I2CDetail::BusConfig< Sda, Scl, Pullup >;
	using BusTraits = I2CDetail::BusTraits< BusConfig::bus >;
	using Settings = I2CDetail::Settings< F_BUS, Freq >;
	using IsrHandler = I2CDetail::IsrHandler< BusConfig::bus >;

	using Status = I2CDetail::Status;

public:
	I2CMaster()
		: rxBufferIndex()
		, rxBufferLength()
		, txBufferIndex()
		, txBufferLength()
		, currentStatus( Status::WAITING )
		, rxAddr()
		, reqCount()
		, timeoutRxNAK()
	{
		// Enable I2C internal clock
		SIM_SCGC4 |= BusTraits::scgc4;

		// Set Master address (zeroed to prevent accidental Rx when setup is changed)
		*BusTraits::C2 = I2C_C2_HDRS; // Set high drive select
		*BusTraits::A1 = 0;
		*BusTraits::RA = 0;

		// Setup pins and options (note: does not "unset" unused pins if changed).  As noted in
		// original TwoWire.cpp, internal 3.0/3.1 pullup is strong (about 190 ohms), but it can
		// work if other devices on bus have strong enough pulldown devices (usually true).
		// was: pinConfigure
		BusConfig::enable();
		// end: pinConfigure

	    // Set I2C rate
		// was: setRate
		*BusTraits::F = Settings::f;
		*BusTraits::FLT = Settings::flt;
		// end: setRate

		// Set config registers and operating mode
		// was: setOpMode
		*BusTraits::C1 = I2C_C1_IICEN; // reset I2C modes, stop intr, stop DMA
		*BusTraits::S = I2C_S_IICIF | I2C_S_ARBL; // clear status flags just in case

		if (OpMode == I2CMasterMode::ISR) {
            // attach interrupt handler
			IsrHandler::handler = libcpp__mem_fn( &I2CMaster::isr, *this );
			// Nested Vec Interrupt Ctrl - enable I2C interrupt
			NVIC_ENABLE_IRQ(BusTraits::irq);
			// I2Cx_INTR_FLAG_INIT; // init I2C0 interrupt flag if used
		}
		// end: setOpMode

		*BusTraits::C1 = I2C_C1_IICEN; // Master - enable I2C (hold in Rx mode, intr disabled)
	}

	~I2CMaster()
	{
	    // Nested Vec Interrupt Ctrl - enable I2C interrupt
        NVIC_DISABLE_IRQ(BusTraits::irq);
	    // detach interrupt handler
	    IsrHandler::handler = {};

		// Disable pins
		BusConfig::disable();

		// Disable I2C internal clock
		SIM_SCGC4 &= ~BusTraits::scgc4;
	}

	void beginTransmission(uint8_t address)
	{
		txBuffer[0] = (address << 1); // store target addr
		txBufferLength = 1;
		currentStatus = Status::WAITING; // reset status
	}

	void beginTransmission(int address) { beginTransmission((uint8_t)address); }

	uint8_t endTransmission(bool sendStop, uint32_t timeout)
	{
		sendTransmission(sendStop, timeout);
		finish(timeout);
		return getError();
	}

	uint8_t endTransmission() { return endTransmission(true, 0); }
	uint8_t endTransmission(bool sendStop) { return endTransmission(sendStop, 0); }
	uint8_t endTransmission(uint8_t sendStop) { return endTransmission((bool)sendStop, 0); } // Wire compatibility

	size_t requestFrom(uint8_t addr, size_t len, bool sendStop, uint32_t timeout)
	{
		// exit immediately if request for 0 bytes
		if(len == 0) return 0;

		sendRequest(addr, len, sendStop, timeout);

		// wait for completion or timeout
		if(finish(timeout))
			return rxBufferLength;
		else
			return 0; // NAK, timeout or bus error
	}

	size_t requestFrom(uint8_t addr, size_t len) { return requestFrom(addr, len, true, 0); }
    size_t requestFrom(int addr, int len) { return requestFrom((uint8_t)addr, (size_t)len, true, 0); }
    uint8_t requestFrom(uint8_t addr, uint8_t len) { return (uint8_t)requestFrom(addr, (size_t)len, true, 0); } // Wire compatibility
    size_t requestFrom(uint8_t addr, size_t len, bool sendStop) { return requestFrom(addr, len, sendStop, 0); }
    uint8_t requestFrom(uint8_t addr, uint8_t len, uint8_t sendStop) { return (uint8_t)requestFrom(addr, (size_t)len, (bool)sendStop, 0); } // Wire compatibility

    uint8_t getError() const
    {
   	    // convert status to Arduino return values (give these a higher priority than buf overflow error)
		switch(currentStatus)
		{
		case Status::BUF_OVF:  return 1;
		case Status::ADDR_NAK: return 2;
		case Status::DATA_NAK: return 3;
		case Status::ARB_LOST: return 4;
		case Status::TIMEOUT:  return 4;
		default: break;
		}
		return 0; // no errors
    }

	// ------------------------------------------------------------------------------------------------------
	// Write - write data to Tx buffer
	// return: #bytes written = success, 0=fail
	// parameters:
	//      data = data byte
	//
	size_t write(uint8_t data)
	{
		if(txBufferLength < TxSize)
		{
			txBuffer[txBufferLength++] = data;
			return 1;
		}
		return 0;
	}

	size_t write(unsigned long n) { return write((uint8_t)n); }
    size_t write(long n)          { return write((uint8_t)n); }
    size_t write(unsigned int n)  { return write((uint8_t)n); }
    size_t write(int n)           { return write((uint8_t)n); }

	// ------------------------------------------------------------------------------------------------------
	// Write Array - write length number of bytes from data array to Tx buffer
	// return: #bytes written = success, 0=fail
	// parameters:
	//      data = pointer to uint8_t array of data
	//      length = number of bytes to write
	//
	size_t write(const uint8_t* data, size_t quantity)
	{
		if(txBufferLength < TxSize)
		{
			size_t avail = TxSize - txBufferLength;
			uint8_t* dest = txBuffer + txBufferLength;

			if(quantity > avail)
			{
				quantity = avail; // truncate to space avail if needed
			}
			for(size_t count=quantity; count; count--)
				*dest++ = *data++;
			txBufferLength += quantity;
			return quantity;
		}
		return 0;
	}

    size_t write(const char* str) { return write((const uint8_t*)str, strlen(str)); }

    // ------------------------------------------------------------------------------------------------------
    // Available - returns number of remaining available bytes in Rx buffer
    // return: #bytes available
    //
    int available() const { return rxBufferLength - rxBufferIndex; }

	// ------------------------------------------------------------------------------------------------------
	// Read - returns next data byte (signed int) from Rx buffer
	// return: data, -1 if buffer empty
	//
	int read()
	{
		if(rxBufferIndex >= rxBufferLength) return -1;
		return rxBuffer[rxBufferIndex++];
	}

	// ------------------------------------------------------------------------------------------------------
	// Peek - returns next data byte (signed int) from Rx buffer without removing it from Rx buffer
	// return: data, -1 if buffer empty
	//
	int peek()
	{
		if(rxBufferIndex >= rxBufferLength) return -1;
		return rxBuffer[rxBufferIndex];
	}

	// ------------------------------------------------------------------------------------------------------
	// Read Byte - returns next data byte (uint8_t) from Rx buffer
	// return: data, 0 if buffer empty
	//
	uint8_t readByte()
	{
		if(rxBufferIndex >= rxBufferLength) return 0;
		return rxBuffer[rxBufferIndex++];
	}

	// ------------------------------------------------------------------------------------------------------
	// Peek Byte - returns next data byte (uint8_t) from Rx buffer without removing it from Rx buffer
	// return: data, 0 if buffer empty
	//
	uint8_t peekByte() const
	{
		if(rxBufferIndex >= rxBufferLength) return 0;
		return rxBuffer[rxBufferIndex];
	}

    // ------------------------------------------------------------------------------------------------------
    // Flush (not implemented)
    //
	void flush() {}

    // ------------------------------------------------------------------------------------------------------
    // Get Rx Address - returns target address of incoming I2C command. Used for Slaves operating over an address range.
    // return: rxAddr of last received command
    //
	uint8_t getRxAddr() const { return rxAddr; }

private:
	uint8_t acquireBus(uint32_t timeout, uint8_t& forceImm)
	{
		int irqPriority, currPriority;
		uint32_t ms = millis();

		// start timer, then take control of the bus
		if(*BusTraits::C1 & I2C_C1_MST)
		{
			// we are already the bus master, so send a repeated start
			*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
		}
		else
		{
			while(timeout == 0 || millis() - ms < timeout)
			{
				// we are not currently the bus master, so check if bus ready
				if(!(*BusTraits::S & I2C_S_BUSY))
				{
					// become the bus master in transmit mode (send start)
					*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
					break;
				}
			}
			if (AutoRetry)
			{
				// if not master and auto-retry set, then reset bus and try one last time
				if(!(*BusTraits::C1 & I2C_C1_MST))
				{
					resetBus();
					if(!(*BusTraits::S & I2C_S_BUSY))
					{
						// become the bus master in transmit mode (send start)
						*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
					}
				}
			}
			// check if not master
			if(!(*BusTraits::C1 & I2C_C1_MST))
			{
				currentStatus = Status::TIMEOUT; // bus not acquired, mark as timeout
				return 0;
			}
		}

		// For ISR operation, check if current routine has higher priority than I2C IRQ, and if so
		// either escalate priority of I2C IRQ or send I2C using immediate mode
		if(OpMode == I2CMasterMode::ISR)
		{
			currPriority = nvic_execution_priority();
			irqPriority = NVIC_GET_PRIORITY(BusTraits::irq);
			if(currPriority <= irqPriority)
			{
				if(currPriority < 16)
					forceImm = 1; // current priority cannot be surpassed, force Immediate mode
				else
					NVIC_SET_PRIORITY(BusTraits::irq, currPriority-16);
			}
		}
		return 1;
	}

	void resetBus()
	{
		uint8_t count = 0;

		// change pin mux to digital I/O
		Sda::input();
		if (Pullup) Sda::pullup();
		Scl::set();
		Scl::output();

        while(Sda::read() == 0 && count++ < 10)
        {
        	Scl::clear();
            delayMicroseconds(5);       // 10us period == 100kHz
            Scl::set();
            delayMicroseconds(5);
        }

        // reset status
        currentStatus = Status::WAITING; // reset status

        // reconfigure pins for I2C
		// was: pinConfigure
		BusConfig::enable();
		// end: pinConfigure
	}

	void sendTransmission(bool sendStop, uint32_t timeout)
	{
		uint8_t status, forceImm=0;
		size_t idx;

		// exit immediately if sending 0 bytes
		if(txBufferLength == 0) return;

		// update timeout
		timeout = (timeout == 0) ? Timeout : timeout;

		// clear the status flags
		#if defined(__MKL26Z64__) // LC
			*BusTraits::FLT |= I2C_FLT_STOPF;     // clear STOP intr
		#endif
		*BusTraits::S = I2C_S_IICIF | I2C_S_ARBL; // clear intr, arbl

		// try to take control of the bus
		if(!acquireBus(timeout, forceImm)) return;

		//
		// Immediate mode - blocking
		//
		if(OpMode == I2CMasterMode::IMM || forceImm)
		{
			uint32_t us = micros();
			currentStatus = Status::SENDING;
			currentStop = sendStop;

			for(idx=0; idx < txBufferLength && (timeout == 0 || micros() - us < timeout); idx++)
			{
				// send data, wait for done
				*BusTraits::D = txBuffer[idx];
				i2c_wait();
				status = *BusTraits::S;

				// check arbitration
				if(status & I2C_S_ARBL)
				{
					currentStatus = Status::ARB_LOST;
					*BusTraits::C1 = I2C_C1_IICEN; // change to Rx mode, intr disabled (does this send STOP if ARBL flagged?)
					*BusTraits::S = I2C_S_ARBL; // clear arbl flag
					break;
				}
				// check if slave ACK'd
				else if(status & I2C_S_RXAK)
				{
					if(idx == 0)
						currentStatus = Status::ADDR_NAK; // NAK on Addr
					else
						currentStatus = Status::DATA_NAK; // NAK on Data
					*BusTraits::C1 = I2C_C1_IICEN; // send STOP, change to Rx mode, intr disabled
					break;
				}
			}

			// Set final status
			if(idx < txBufferLength)
				currentStatus = Status::TIMEOUT; // Tx incomplete, mark as timeout
			else
				currentStatus = Status::WAITING; // Tx complete, change to waiting state

			// send STOP if configured
			if(currentStop)
				*BusTraits::C1 = I2C_C1_IICEN; // send STOP, change to Rx mode, intr disabled
			else
				*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX; // no STOP, stay in Tx mode, intr disabled
		}
		//
		// ISR/DMA mode - non-blocking
		//
		else if(OpMode == I2CMasterMode::ISR)
		{
			// send target addr and enable interrupts
			currentStatus = Status::SENDING;
			currentStop = sendStop;
			txBufferIndex = 0;
			// start ISR
			*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX; // enable intr
			*BusTraits::D = txBuffer[0];
		}
	}

	void sendRequest(uint8_t addr, size_t len, bool sendStop, uint32_t timeout)
	{
		uint8_t status, data, chkTimeout=0, forceImm=0;

		// exit immediately if request for 0 bytes or request too large
		if(len == 0) return;
		if(len > RxSize) { currentStatus=Status::BUF_OVF; return; }

		reqCount = len; // store request length
		rxBufferIndex = 0; // reset buffer
		rxBufferLength = 0;
		timeout = (timeout == 0) ? Timeout : timeout;

		// clear the status flags
		#if defined(__MKL26Z64__) // LC
			*BusTraits::FLT |= I2C_FLT_STOPF;     // clear STOP intr
		#endif
		*BusTraits::S = I2C_S_IICIF | I2C_S_ARBL; // clear intr, arbl

		// try to take control of the bus
		if(!acquireBus(timeout, forceImm)) return;

		//
		// Immediate mode - blocking
		//
		if(OpMode == I2CMasterMode::IMM || forceImm)
		{
			uint32_t us = micros();
			currentStatus = Status::SEND_ADDR;
			currentStop = sendStop;

			// Send target address
			*BusTraits::D = (addr << 1) | 1; // address + READ
			i2c_wait();
			status = *BusTraits::S;

			// check arbitration
			if(status & I2C_S_ARBL)
			{
				currentStatus = Status::ARB_LOST;
				*BusTraits::C1 = I2C_C1_IICEN; // change to Rx mode, intr disabled (does this send STOP if ARBL flagged?)
				*BusTraits::S = I2C_S_ARBL; // clear arbl flag
				return;
			}
			// check if slave ACK'd
			else if(status & I2C_S_RXAK)
			{
				currentStatus = Status::ADDR_NAK; // NAK on Addr
				*BusTraits::C1 = I2C_C1_IICEN; // send STOP, change to Rx mode, intr disabled
				return;
			}
			else
			{
				// Slave addr ACK, change to Rx mode
				currentStatus = Status::RECEIVING;
				if(reqCount == 1)
					*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK; // no STOP, Rx, NAK on recv
				else
					*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST; // no STOP, change to Rx
				data = *BusTraits::D; // dummy read

				// Master receive loop
				while(rxBufferLength < reqCount && currentStatus == Status::RECEIVING)
				{
					i2c_wait();
					chkTimeout = (timeout != 0 && us - micros() >= timeout);
					// check if 2nd to last byte or timeout
					if((rxBufferLength+2) == reqCount || (chkTimeout && !timeoutRxNAK))
					{
						*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK; // no STOP, Rx, NAK on recv
					}
					// if last byte or timeout send STOP
					if((rxBufferLength+1) >= reqCount || (chkTimeout && timeoutRxNAK))
					{
						timeoutRxNAK = 0; // clear flag
						// change to Tx mode
						*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
						// grab last data
						data = *BusTraits::D;
						rxBuffer[rxBufferLength++] = data;
						if(chkTimeout)
							currentStatus = Status::TIMEOUT; // Rx incomplete, mark as timeout
						else
							currentStatus = Status::WAITING; // Rx complete, change to waiting state
						if(currentStop) // NAK then STOP
						{
							delayMicroseconds(1); // empirical patch, lets things settle before issuing STOP
							*BusTraits::C1 = I2C_C1_IICEN; // send STOP, change to Rx mode, intr disabled
						}
						// else NAK no STOP
					}
					else
					{
						// grab next data, not last byte, will ACK
						data = *BusTraits::D;
						rxBuffer[rxBufferLength++] = data;
					}
					if(chkTimeout) timeoutRxNAK = 1; // set flag to indicate NAK sent
				}
			}
		}
		//
		// ISR/DMA mode - non-blocking
		//
		else if(OpMode == I2CMasterMode::ISR)
		{
			// send 1st data and enable interrupts
			currentStatus = Status::SEND_ADDR;
			currentStop = sendStop;
			// start ISR
			*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX; // enable intr
			*BusTraits::D = (addr << 1) | 1; // address + READ
		}
	}

	uint8_t done() const
	{
		return (currentStatus==Status::WAITING ||
				currentStatus==Status::ADDR_NAK ||
				currentStatus==Status::DATA_NAK ||
				currentStatus==Status::ARB_LOST ||
				currentStatus==Status::TIMEOUT ||
				currentStatus==Status::BUF_OVF);
	}

	uint8_t finish(uint32_t timeout)
	{
		timeout = (timeout == 0) ? Timeout : timeout;
		uint32_t us = micros();

		// wait for completion or timeout
		while(!done() && (timeout == 0 || micros() - us < timeout));

		// check exit status, if still Tx/Rx then timeout occurred
		if(currentStatus == Status::SENDING ||
		   currentStatus == Status::SEND_ADDR ||
		   currentStatus == Status::RECEIVING)
			currentStatus = Status::TIMEOUT; // set to timeout state

		// delay to allow bus to settle (eg. allow STOP to complete and be recognized,
		//                               not just on our side, but on slave side also)
		delayMicroseconds(4);
		if(currentStatus == Status::WAITING) return 1;
		return 0;
	}

	void i2c_wait()
	{
		while(!(*BusTraits::S & I2C_S_IICIF)){}
		*BusTraits::S = I2C_S_IICIF;
	}

	void isr()
	{
		uint8_t status, c1;

		status = *BusTraits::S;
		c1 = *BusTraits::C1;

		if(c1 & I2C_C1_MST)
		{
			//
			// Master Mode
			//
			if(c1 & I2C_C1_TX)
			{
				// Continue Master Transmit
				// check if Master Tx or Rx
				if(currentStatus == Status::SENDING)
				{
					// check arbitration
					if(status & I2C_S_ARBL)
					{
						// Arbitration Lost
						currentStatus = Status::ARB_LOST;
						*BusTraits::C1 = I2C_C1_IICEN; // change to Rx mode, intr disabled (does this send STOP if ARBL flagged?)
						*BusTraits::S = I2C_S_ARBL | I2C_S_IICIF; // clear arbl flag and intr
						txBufferIndex = 0; // reset Tx buffer index to prepare for resend
						return; // does this need to check IAAS and drop to Slave Rx?
					}
					// check if slave ACK'd
					else if(status & I2C_S_RXAK)
					{
						if(txBufferIndex == 0)
							currentStatus = Status::ADDR_NAK; // NAK on Addr
						else
							currentStatus = Status::DATA_NAK; // NAK on Data
						// send STOP, change to Rx mode, intr disabled
						// note: Slave NAK is an error, so send STOP regardless of setting
						*BusTraits::C1 = I2C_C1_IICEN;
					}
					else
					{
						// check if last byte transmitted
						if(++txBufferIndex >= txBufferLength)
						{
							// Tx complete, change to waiting state
							currentStatus = Status::WAITING;
							// send STOP if configured
							if(currentStop)
								*BusTraits::C1 = I2C_C1_IICEN; // send STOP, change to Rx mode, intr disabled
							else
								*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX; // no STOP, stay in Tx mode, intr disabled
						}
						else
						{
							// ISR transmit next byte
							*BusTraits::D = txBuffer[txBufferIndex];
						}
					}
					*BusTraits::S = I2C_S_IICIF; // clear intr
					return;
				}
				else if(currentStatus == Status::SEND_ADDR)
				{
					// Master Receive, addr sent
					if(status & I2C_S_ARBL)
					{
						// Arbitration Lost
						currentStatus = Status::ARB_LOST;
						*BusTraits::C1 = I2C_C1_IICEN; // change to Rx mode, intr disabled (does this send STOP if ARBL flagged?)
						*BusTraits::S = I2C_S_ARBL | I2C_S_IICIF; // clear arbl flag and intr
						return;
					}
					else if(status & I2C_S_RXAK)
					{
						// Slave addr NAK
						currentStatus = Status::ADDR_NAK; // NAK on Addr
						// send STOP, change to Rx mode, intr disabled
						*BusTraits::C1 = I2C_C1_IICEN;
					}
					else
					{
						// Slave addr ACK, change to Rx mode
						currentStatus = Status::RECEIVING;
						if(reqCount == 1)
							*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // no STOP, Rx, NAK on recv
						else
							*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST; // no STOP, change to Rx
						*BusTraits::D; // dummy read
					}
					*BusTraits::S = I2C_S_IICIF; // clear intr
					return;
				}
				else if(currentStatus == Status::TIMEOUT)
				{
					// send STOP if configured
					if(currentStop)
					{
						// send STOP, change to Rx mode, intr disabled
						*BusTraits::C1 = I2C_C1_IICEN;
					}
					else
					{
						// no STOP, stay in Tx mode, intr disabled
						*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
					}
					*BusTraits::S = I2C_S_IICIF; // clear intr
					return;
				}
				else
				{
					// Should not be in Tx mode if not sending
					// send STOP, change to Rx mode, intr disabled
					*BusTraits::C1 = I2C_C1_IICEN;
					*BusTraits::S = I2C_S_IICIF; // clear intr
					return;
				}
			}
			else
			{
				// Continue Master Receive
				//
				// check if 2nd to last byte or timeout
				if((rxBufferLength+2) == reqCount || (currentStatus == Status::TIMEOUT && !timeoutRxNAK))
				{
					*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // no STOP, Rx, NAK on recv
				}
				// if last byte or timeout send STOP
				if((rxBufferLength+1) >= reqCount || (currentStatus == Status::TIMEOUT && timeoutRxNAK))
				{
					timeoutRxNAK = 0; // clear flag
					if(currentStatus != Status::TIMEOUT)
						currentStatus = Status::WAITING; // Rx complete, change to waiting state
					// change to Tx mode
					*BusTraits::C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
					// grab last data
					rxBuffer[rxBufferLength++] = *BusTraits::D;
					if(currentStop) // NAK then STOP
					{
						delayMicroseconds(1); // empirical patch, lets things settle before issuing STOP
						*BusTraits::C1 = I2C_C1_IICEN; // send STOP, change to Rx mode, intr disabled
					}
					// else NAK no STOP
				}
				else
				{
					// grab next data, not last byte, will ACK
					rxBuffer[rxBufferLength++] = *BusTraits::D;
				}
				if(currentStatus == Status::TIMEOUT && !timeoutRxNAK)
					timeoutRxNAK = 1; // set flag to indicate NAK sent
				*BusTraits::S = I2C_S_IICIF; // clear intr
				return;
			}
		}
	}

    uint8_t  rxBuffer[RxSize];               // Rx Buffer                         (ISR)
    volatile size_t   rxBufferIndex;         // Rx Index                          (User&ISR)
    volatile size_t   rxBufferLength;        // Rx Length                         (ISR)
    uint8_t  txBuffer[TxSize];               // Tx Buffer                         (User)
    volatile size_t   txBufferIndex;         // Tx Index                          (User&ISR)
    volatile size_t   txBufferLength;        // Tx Length                         (User&ISR)
    bool     currentStop;                    // Current Stop                      (User)
    volatile Status   currentStatus;         // Current Status                    (User&ISR)
    uint8_t  rxAddr;                         // Rx Address                        (ISR)
    size_t   reqCount;                       // Byte Request Count                (User)
    uint8_t  timeoutRxNAK;                   // Rx Timeout NAK flag               (ISR)
};

#endif // I2C_T3_H
