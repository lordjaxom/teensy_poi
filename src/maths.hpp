#ifndef TEENSY_POI_MATHS_HPP
#define TEENSY_POI_MATHS_HPP

#include <math.h>
#include <stdint.h>
#include <type_traits>

#include "random.h"

namespace tp {

	template< typename A, typename B, typename T >
	constexpr typename std::common_type< A, B, T >::type linear_interpolate( A a, B b, T t )
	{
		return a * ( 1 - t ) + b * t;
	}


	template< typename T, typename std::enable_if< std::is_integral< T >::value >::type* = nullptr >
	inline T random( T min, T max )
	{
		return mersenne_twister() % ( max - min + 1 ) + min;
	}

	template< typename T, typename std::enable_if< std::is_floating_point< T >::value >::type* = nullptr >
	inline T random( T min, T max )
	{
		auto rn = mersenne_twister();
		return ( (T) rn * ( max - min ) / std::numeric_limits< decltype( rn ) >::max() ) + min;
	}

	static constexpr float pi = 3.14159265359f;

} // namespace tp

#endif // TEENSY_POI_MATHS_HPP
