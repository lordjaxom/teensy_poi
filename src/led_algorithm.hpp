#ifndef TEENSY_POI_LEGALGO_HPP
#define TEENSY_POI_LEGALGO_HPP

#include <algorithm>

#include <libcpp/numeric.hpp>

#include "leds.hpp"

namespace tp {

	template< size_t Start, size_t Count, typename Colors >
	struct LedProgress
	{
		template< typename Pixels >
		static void progress( Pixels& pixels, size_t value )
		{
			std::fill( &pixels[ Start ], &pixels[ Start + value ], LedPixel( Colors::colorOn ) );
			std::fill( &pixels[ Start + value ], &pixels[ Start + Count ], LedPixel( Colors::colorOff ) );
		}

		template< typename Pixels >
		static void percentage( Pixels& pixels, uint8_t percentage )
		{
			progress( pixels, Count * percentage / 100 );
		}
	};


	template< uint32_t Runtime, typename Colors >
	struct LedFadeAnimation
	{
		static LedPixel animate( uint32_t position )
		{
			position %= Runtime;
			uint8_t state = ( position < Runtime / 2 ? position : 2 * Runtime - position - 1 ) * 255 / ( Runtime - 1 );
			return LedPixel(
					Colors::colorMin.r + state * ( Colors::colorMax.r - Colors::colorMin.r ) / 255,
					Colors::colorMin.g + state * ( Colors::colorMax.g - Colors::colorMin.g ) / 255,
					Colors::colorMin.b + state * ( Colors::colorMax.b - Colors::colorMin.b ) / 255 );
		}
	};


	template< uint32_t OffTime, uint32_t OnTime, typename Colors >
	struct LedFlashAnimation
	{
		static constexpr uint32_t runtime = OnTime + OffTime;

		static LedPixel animate( uint32_t position )
		{
			return position % runtime >= OffTime ? LedPixel( Colors::colorOn ) : LedPixel( Colors::colorOff );
		}
	};

} // namespace tp

#endif // TEENSY_POI_LEGALGO_HPP
