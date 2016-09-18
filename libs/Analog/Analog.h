#ifndef Analog_h_
#define Analog_h_

#include <teensy.h>
#include <tlibcpp/teensy_pins.hpp>

enum class AnalogReference : uint8_t
{
	EXTERNAL = 0,
	INTERNAL = 1
};


namespace AnalogDetail {

	// the alternate clock is connected to OSCERCLK (16 MHz).
	// datasheet says ADC clock should be 2 to 12 MHz for 16 bit mode
	// datasheet says ADC clock should be 1 to 18 MHz for 8-12 bit mode

	template< uint32_t Freq >
	struct ConfigImpl;

	template<>
	struct ConfigImpl< 60000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1); // 7.5 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 15 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 15 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 15 MHz
	};

	template<>
	struct ConfigImpl< 56000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1); // 7 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 14 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 14 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 14 MHz
	};

	template<>
	struct ConfigImpl< 48000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 12 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 12 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 12 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1); // 24 MHz
	};

	template<>
	struct ConfigImpl< 40000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 10 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 10 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 10 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1); // 20 MHz
	};

	template<>
	struct ConfigImpl< 36000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1); // 9 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1); // 18 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1); // 18 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1); // 18 MHz
	};

	template<>
	struct ConfigImpl< 24000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0); // 12 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0); // 12 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0); // 12 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 24 MHz
	};

	template<>
	struct ConfigImpl< 16000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 16 MHz
	};

	template<>
	struct ConfigImpl< 8000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 8 MHz
	};

	template<>
	struct ConfigImpl< 4000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 4 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 4 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 4 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 4 MHz
	};

	template<>
	struct ConfigImpl< 2000000 >
	{
		static constexpr uint32_t ADC_CFG1_16BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 2 MHz
		static constexpr uint32_t ADC_CFG1_12BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 2 MHz
		static constexpr uint32_t ADC_CFG1_10BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 2 MHz
		static constexpr uint32_t ADC_CFG1_8BIT = ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0); // 2 MHz
	};


	using Config = ConfigImpl< F_BUS >;


	template< AnalogReference Ref >
	struct Reference;

	template<>
	struct Reference< AnalogReference::EXTERNAL >
	{
		static void init()
		{
#if defined(__MK20DX128__)
			ADC0_SC2 = ADC_SC2_REFSEL(0); // vcc/ext ref
#elif defined(__MK20DX256__)
			ADC0_SC2 = ADC_SC2_REFSEL(0); // vcc/ext ref
			ADC1_SC2 = ADC_SC2_REFSEL(0); // vcc/ext ref
#elif defined(__MKL26Z64__)
			ADC0_SC2 = ADC_SC2_REFSEL(0); // external AREF
#endif
		}
	};

	template<>
	struct Reference< AnalogReference::INTERNAL >
	{
		static void init()
		{
#if defined(__MK20DX128__)
			ADC0_SC2 = ADC_SC2_REFSEL(1); // 1.2V ref
#elif defined(__MK20DX256__)
			ADC0_SC2 = ADC_SC2_REFSEL(1); // 1.2V ref
			ADC1_SC2 = ADC_SC2_REFSEL(1); // 1.2V ref
#elif defined(__MKL26Z64__)
			ADC0_SC2 = ADC_SC2_REFSEL(1); // vcc
#endif
		}
	};


	template< uint8_t Res >
	struct Resolution;

	template<>
	struct Resolution< 8 >
	{
		static void init()
		{
			ADC0_CFG1 = Config::ADC_CFG1_8BIT + ADC_CFG1_MODE(0);
			ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(3);
#if defined(__MK20DX256__)
			ADC1_CFG1 = Config::ADC_CFG1_8BIT + ADC_CFG1_MODE(0);
			ADC1_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(3);
#endif
		}
	};

	template<>
	struct Resolution< 10 >
	{
		static void init()
		{
			ADC0_CFG1 = Config::ADC_CFG1_10BIT + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP;
			ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(3);
#if defined(__MK20DX256__)
			ADC1_CFG1 = Config::ADC_CFG1_10BIT + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP;
			ADC1_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(3);
#endif
		}
	};

	template<>
	struct Resolution< 12 >
	{
		static void init()
		{
			ADC0_CFG1 = Config::ADC_CFG1_12BIT + ADC_CFG1_MODE(1) + ADC_CFG1_ADLSMP;
			ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(2);
#if defined(__MK20DX256__)
			ADC1_CFG1 = Config::ADC_CFG1_12BIT + ADC_CFG1_MODE(1) + ADC_CFG1_ADLSMP;
			ADC1_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(2);
#endif
		}
	};

	template<>
	struct Resolution< 16 >
	{
		static void init()
		{
			ADC0_CFG1 = Config::ADC_CFG1_16BIT + ADC_CFG1_MODE(3) + ADC_CFG1_ADLSMP;
			ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(2);
#if defined(__MK20DX256__)
			ADC1_CFG1 = Config::ADC_CFG1_16BIT + ADC_CFG1_MODE(3) + ADC_CFG1_ADLSMP;
			ADC1_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(2);
#endif
		}
	};


	template< uint8_t Avg >
	struct Averaging;

	template<>
	struct Averaging< 0 >
	{
		static void init()
		{
			ADC0_SC3 = ADC_SC3_CAL;  // begin cal
#if defined(__MK20DX256__)
			ADC1_SC3 = ADC_SC3_CAL;  // begin cal
#endif
		}
	};

	template<>
	struct Averaging< 4 >
	{
		static void init()
		{
			ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(0);
#if defined(__MK20DX256__)
			ADC1_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(0);
#endif
		}
	};

	template<>
	struct Averaging< 8 >
	{
		static void init()
		{
			ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(1);
#if defined(__MK20DX256__)
			ADC1_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(1);
#endif
		}
	};

	template<>
	struct Averaging< 16 >
	{
		static void init()
		{
			ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(2);
#if defined(__MK20DX256__)
			ADC1_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(2);
#endif
		}
	};

	template<>
	struct Averaging< 32 >
	{
		static void init()
		{
			ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(3);
#if defined(__MK20DX256__)
			ADC1_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(3);
#endif
		}
	};


	struct InputTemperature {};
	struct InputVoltage {};


	template< typename Input >
	struct Channel;

	template<>
	struct Channel< InputTemperature >
	{
		static constexpr uint8_t channel = 26;
	};

	template<>
	struct Channel< InputVoltage >
	{
		static constexpr uint8_t channel =
#if defined(__MK20DX128__)
				22
#elif defined(__MK20DX256__)
				18+128
#elif defined(__MKL26Z64__)
				27
#endif
				;
	};

#define ANALOG_DEFINE_ANALOG_CHANNEL( port_, pin_, channel_ ) \
    template<> struct Channel< tlibcpp::pin< tlibcpp::port_, pin_ > > { \
        static constexpr uint8_t channel = channel_; \
    }

#if defined( __MKL26Z64__ )

        ANALOG_DEFINE_ANALOG_CHANNEL( portd, 1, 5 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portc, 0, 14 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portb, 0, 8 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portb, 1, 9 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portb, 3, 13 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portb, 2, 12 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portd, 5, 6 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portd, 6, 7 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portc, 1, 15 );
        ANALOG_DEFINE_ANALOG_CHANNEL( portc, 2, 11 );
        ANALOG_DEFINE_ANALOG_CHANNEL( porte, 20, 0 );
        ANALOG_DEFINE_ANALOG_CHANNEL( porte, 21, 4+64 );
        ANALOG_DEFINE_ANALOG_CHANNEL( porte, 30, 23 );

#endif

#undef ANALOG_DEFINE_ANALOG_CHANNEL

} // namespace AnalogDetail


template< AnalogReference Ref = AnalogReference::EXTERNAL, uint8_t Res = 10, uint32_t Avg = 4 >
class Analog
{
	using Resolution = AnalogDetail::Resolution< Res >;
	using Reference = AnalogDetail::Reference< Ref >;
	using Averaging = AnalogDetail::Averaging< Avg >;

public:
	using InputTemperature = AnalogDetail::InputTemperature;
	using InputVoltage = AnalogDetail::InputVoltage;

	Analog()
	{
		#if defined(__MK20DX128__) || defined(__MK20DX256__)
		VREF_TRM = 0x60;
		VREF_SC = 0xE1;		// enable 1.2 volt ref
		#endif

		Resolution::init();
		Reference::init();
		Averaging::init();

		// wait_for_cal:

#if defined(__MK20DX128__)
		while (ADC0_SC3 & ADC_SC3_CAL) {
			// wait
		}
#elif defined(__MK20DX256__)
		while ((ADC0_SC3 & ADC_SC3_CAL) || (ADC1_SC3 & ADC_SC3_CAL)) {
			// wait
		}
#endif
		__disable_irq();
		uint16_t sum;
		sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
		sum = (sum / 2) | 0x8000;
		ADC0_PG = sum;
		sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
		sum = (sum / 2) | 0x8000;
		ADC0_MG = sum;
#if defined(__MK20DX256__)
		sum = ADC1_CLPS + ADC1_CLP4 + ADC1_CLP3 + ADC1_CLP2 + ADC1_CLP1 + ADC1_CLP0;
		sum = (sum / 2) | 0x8000;
		ADC1_PG = sum;
		sum = ADC1_CLMS + ADC1_CLM4 + ADC1_CLM3 + ADC1_CLM2 + ADC1_CLM1 + ADC1_CLM0;
		sum = (sum / 2) | 0x8000;
		ADC1_MG = sum;
	#endif
		__enable_irq();
	}

	template< typename Input >
	uint32_t read() const
	{
		uint8_t channel = AnalogDetail::Channel< Input >::channel;
		volatile uint8_t analogReadBusyADC0 = 0;
#if defined(__MK20DX256__)
		volatile uint8_t analogReadBusyADC1 = 0;
#endif
		uint32_t result;

#if defined(__MK20DX256__)
		if (channel & 0x80) goto beginADC1;
#endif

		__disable_irq();
startADC0:
#if defined(__MKL26Z64__)
		if (channel & 0x40) {
			ADC0_CFG2 &= ~ADC_CFG2_MUXSEL;
			channel &= 0x3F;
		} else {
			ADC0_CFG2 |= ADC_CFG2_MUXSEL;
		}
#endif
		ADC0_SC1A = channel;
		analogReadBusyADC0 = 1;
		__enable_irq();
		while (1) {
			__disable_irq();
			if ((ADC0_SC1A & ADC_SC1_COCO)) {
				result = ADC0_RA;
				analogReadBusyADC0 = 0;
				__enable_irq();
				return result;
			}
			// detect if analogRead was used from an interrupt
			// if so, our analogRead got canceled, so it must
			// be restarted.
			if (!analogReadBusyADC0) goto startADC0;
			__enable_irq();
		}

#if defined(__MK20DX256__)
beginADC1:
		__disable_irq();
startADC1:
		// ADC1_CFG2[MUXSEL] bit selects between ADCx_SEn channels a and b.
		if (channel & 0x40) {
			ADC1_CFG2 &= ~ADC_CFG2_MUXSEL;
		} else {
			ADC1_CFG2 |= ADC_CFG2_MUXSEL;
		}
		ADC1_SC1A = channel & 0x3F;
		analogReadBusyADC1 = 1;
		__enable_irq();
		while (1) {
			__disable_irq();
			if ((ADC1_SC1A & ADC_SC1_COCO)) {
				result = ADC1_RA;
				analogReadBusyADC1 = 0;
				return result;
			}
			// detect if analogRead was used from an interrupt
			// if so, our analogRead got canceled, so it must
			// be restarted.
			if (!analogReadBusyADC1) goto startADC1;
			__enable_irq();
		}
#endif
	}
};

#endif // Analog_h_
