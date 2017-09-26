#ifndef TEENSY_POI_LEDALGO_HPP
#define TEENSY_POI_LEDALGO_HPP

#include <algorithm>

#include <libcpp/numeric.hpp>

#include "leds.hpp"

namespace tp {

	template< size_t Start, size_t Count, uint32_t OffColor, uint32_t OnColor >
	struct LedProgress
	{
		template< typename Pixels >
		static void progress( Pixels& pixels, size_t value )
		{
			std::fill( &pixels[ Start ], &pixels[ Start + value ], LedPixel( OnColor ) );
			std::fill( &pixels[ Start + value ], &pixels[ Start + Count ], LedPixel( OffColor ) );
		}

		template< typename Pixels >
		static void percentage( Pixels& pixels, uint8_t percentage )
		{
			progress( pixels, Count * percentage / 100 );
		}
	};


	template< uint32_t Runtime, uint32_t MinColor, uint32_t MaxColor >
	struct LedFadeAnimation
	{
		static constexpr uint32_t runtime = Runtime;
		static constexpr LedPixel colorMin = MinColor;
		static constexpr LedPixel colorMax = MaxColor;

		static constexpr LedPixel animate( uint32_t position )
		{
			position %= runtime;
			uint8_t state = ( position < runtime / 2 ? position : 2 * runtime - position - 1 ) * 255 / ( runtime - 1 );
			return LedPixel(
					colorMin.r + state * ( colorMax.r - colorMin.r ) / 255,
					colorMin.g + state * ( colorMax.g - colorMin.g ) / 255,
					colorMin.b + state * ( colorMax.b - colorMin.b ) / 255 );
		}
	};


	template< uint32_t OffTime, uint32_t OnTime, uint32_t OffColor, uint32_t OnColor >
	struct LedFlashAnimation
	{
		static constexpr uint32_t runtime = OnTime + OffTime;

		static constexpr LedPixel animate( uint32_t position )
		{
			return position % runtime >= OffTime ? OnColor : OffColor;
		}
	};

} // namespace tp

#endif // TEENSY_POI_LEDALGO_HPP
