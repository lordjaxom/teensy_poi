#ifndef TEENSY_POI_COLORWHEEL_HPP
#define TEENSY_POI_COLORWHEEL_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>

#include <libcpp/numeric.hpp>
#include <libcpp/value_table.hpp>

#include "maths.hpp"

namespace tp {

	template< size_t Size >
	struct Colorwheel
	{
		static_assert( Size >= 3, "colorwheel with less than three colors makes no sense" );

		static constexpr size_t size = Size;

		struct Function
		{
			static constexpr uint8_t i( size_t input ) { return input * 3 / size; }
			static constexpr double  f( size_t input ) { return pi * ( 3.0 * input - i( input ) * size ) / ( 2.0 * size ); }
			static constexpr int     s( size_t input ) { return 255.0 * std::sin( f( input ) ); }
			static constexpr int     c( size_t input ) { return 255.0 * std::cos( f( input ) ); }
			static constexpr uint8_t r( size_t input ) { return (uint8_t) ( ( i( input ) == 0 ? 1 : 0 ) * s( input ) + ( i( input ) == 1 ? 1 : 0 ) * c( input ) ); }
			static constexpr uint8_t g( size_t input ) { return (uint8_t) ( ( i( input ) == 1 ? 1 : 0 ) * s( input ) + ( i( input ) == 2 ? 1 : 0 ) * c( input ) ); }
			static constexpr uint8_t b( size_t input ) { return (uint8_t) ( ( i( input ) == 2 ? 1 : 0 ) * s( input ) + ( i( input ) == 0 ? 1 : 0 ) * c( input ) ); }

			static constexpr uint32_t apply( size_t input )
			{
				return ( r( input ) << 16 ) | ( g( input ) << 8 ) | b( input );
			}
		};

		static constexpr uint32_t get( size_t index )
		{
			return libcpp::value_table< Function, 0, size >::get( index );
		}
	};

} // namespace tp

#endif // TEENSY_POI_COLORWHEEL_HPP
