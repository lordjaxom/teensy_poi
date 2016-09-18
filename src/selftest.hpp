#ifndef TEENSY_POI_SELFTEST_HPP
#define TEENSY_POI_SELFTEST_HPP

#include "leds.hpp"
#include "led_algorithm.hpp"

namespace tp {

	namespace SelftestDetail {

		struct BootColors
		{
			static constexpr LedPixel colorMin = LedColor::BLACK;
			static constexpr LedPixel colorMax = LedColor::RED;
		};

		using BootAnimation = LedFadeAnimation< 100, BootColors >;


		struct BatteryColors
		{
			static constexpr LedPixel colorOn = LedColor::DIM_CYAN;
			static constexpr LedPixel colorOff = LedColor::FAINT_CYAN;
		};

		using BatteryProgress = LedProgress< 6, 10, BatteryColors >;

	} // namespace SelftestDetail

	template< typename Manager >
	static void selftest( Manager& manager )
	{
		typename Manager::Leds leds;
		auto&& pixels = leds.pixels();

		for ( uint16_t i = 0 ; i < 100 ; ++i ) {
			pixels[ 0 ] = SelftestDetail::BootAnimation::animate( i );
			leds.send();
			delay( 10 );
		}
		leds.clear();
		delay( 500 );

		debug( "TeensyPOI ready, performing self test..." );

		debug( "1. LEDs... see for yourself" );
		pixels[ 0 ] = LedColor::DIM_GREEN;
		leds.send();
		delay( 500 );

		debug( "2. Status... USB ", manager.status().connected() ? "" : "not ", "connected, Battery ", manager.status().charging() ? "" : "not ", "charging" );
		pixels[ 1 ] = LedColor::DIM_GREEN;
		leds.send();
		delay( 500 );

		debugn( "3. Voltage... " );
		auto voltage = manager.status().currentVoltage();
		bool voltageOk = voltage >= 3200 && voltage <= 4500;
		debug( voltage, voltageOk ? " (makes sense)" : " (doesn't make sense)" );
		pixels[ 2 ] = voltageOk ? LedColor::DIM_GREEN : LedColor::DIM_RED;
		leds.send();
		delay( 500 );

		debugn( "4. Flash... " );
		typename Manager::Flash flash;
		auto image = flash.next( typename Manager::Flash::Image() );
		if ( flash && image ) {
			debug( "OK" );
			pixels[ 3 ] = LedColor::DIM_GREEN;
		}
		else if ( flash ) {
			debug( "EMPTY" );
			pixels[ 3 ] = LedColor::DIM_YELLOW;
		}
		else {
			debug( "ERROR" );
			pixels[ 3 ] = LedColor::DIM_RED;
		}
		leds.send();
		delay( 500 );

		debugn( "5. Motion... " );
		typename Manager::Motion motion( manager.stopwatch() );
		debug( motion ? "OK" : "ERROR" );
		pixels[ 4 ] = motion ? LedColor::DIM_GREEN : LedColor::DIM_RED;
		leds.send();
		delay( 500 );

		auto percent = manager.status().percentage();
		debugn( "Battery is ", percent, "% full" );
		SelftestDetail::BatteryProgress::percentage( pixels, percent );
		leds.send();

		debug( "...done!" );
		delay( 2000 );
	}

} // namespace tp

#endif // TEENSY_POI_SELFTEST_HPP
