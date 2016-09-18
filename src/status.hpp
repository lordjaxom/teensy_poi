#ifndef TEENSY_POI_STATUS_HPP
#define TEENSY_POI_STATUS_HPP

#include <algorithm>
#include <iterator>

#include <libcpp/function.hpp>
#include <libcpp/numeric.hpp>
#include <libcpp/sample_buffer.hpp>

#include <Analog.h>

namespace tp {

    template< typename ChargePin, typename VoltagePin >
    class Status
    {
    	static constexpr uint32_t usBetweenVoltage = 1000000;
    	static constexpr uint16_t maximumVoltage = 4200;
    	static constexpr uint16_t lowVoltageThreshold = 3200;
    	static constexpr size_t voltageSampleCount = 10;

    	using AnalogType = Analog< AnalogReference::INTERNAL, 12, 32 >;

    public:
        explicit Status( Stopwatch const& stopwatch )
			: stopwatch_( stopwatch )
			, nextVoltageCountdown_()
        {
            PMC_REGSC |= PMC_REGSC_BGBE; // 39=bandgap ref (PMC_REGSC |= PMC_REGSC_BGBE)
            ChargePin::input();
            ChargePin::pullup();
        }

        Status( Status const& ) = delete;

        void advance()
        {
        	if ( stopwatch_.countdown( nextVoltageCountdown_) == 0 ) {
				voltageSamples_.push_back( currentVoltage() );
				averageVoltage_ = voltageSamples_.average();
				if ( averageVoltage_ < lowVoltageThreshold && onLowVoltage ) {
					onLowVoltage();
				}
				nextVoltageCountdown_ = usBetweenVoltage;
        	}
        }

        bool charging() const { return !ChargePin::read(); }
        bool connected() const { return !( USB0_OTGSTAT & ( 1 << 5 ) ); }

        uint16_t voltage() const
        {
			return !voltageSamples_.empty() ? averageVoltage_ : maximumVoltage;
        }

        uint16_t currentVoltage() const
        {
            auto pin = voltagePin();
            auto vin = voltageVin();
            if ( pin >= 3400 ) {
                return pin;
            }
            if ( vin >= 3200 ) {
                return ( pin + vin ) / 2;
            }
            return vin;
        }

        uint8_t percentage() const
        {
        	if ( voltageSamples_.empty() ) {
				return 100;
        	}

            auto input = ( maximumVoltage - averageVoltage_ ) / 100 + 1;
            auto percent = 0.37f * input * input * input - 6.1562f * input * input + 15.55f * input + 90.712;
            if ( percent > 100.0f ) {
                return 100;
            }
            if ( percent < 0.0f ) {
                return 0;
            }
            return (uint8_t) percent;
        }

        libcpp::function< void () > onLowVoltage;

	private:
	    uint16_t voltagePin() const
	    {
	        uint32_t x = analog_.read< VoltagePin >();
	        return x * 1000 / 911;
	    }

       	uint16_t voltageVin() const
       	{
       	    // for Teensy LC, Vin, only valid between 1.73V and 3.3V. Returns in millivolts.
			uint32_t x = analog_.read< AnalogType::InputVoltage >();
			return (255*x*x + 2435256940 - 1377272 * x) / 338460;
		}

		uint16_t voltage3v3() const
		{
		    // for Teensy LC, 3v3 input, only valid between 1.65V and 3.5V. Returns in millivolts.
			uint32_t x = analog_.read< AnalogType::InputVoltage  >();
			return (266*x*x + 2496026531 - 1431005 * x) / 342991;
		}

		Stopwatch const& stopwatch_;
		AnalogType analog_;
		uint32_t nextVoltageCountdown_;
		libcpp::sample_buffer< uint16_t, voltageSampleCount > voltageSamples_;
		uint16_t averageVoltage_;
    };

} // namespace tp

#endif // TEENSY_POI_STATUS_HPP
