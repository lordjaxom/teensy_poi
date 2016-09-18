#ifndef TP_GAMMA_TABLE_HPP
#define TP_GAMMA_TABLE_HPP

#include <cmath>
#include <limits>

#include <libcpp/value_table.hpp>

namespace tp {

namespace detail {

template< typename Gamma >
struct gamma_function
{
	static uint8_t constexpr const max = std::numeric_limits< uint8_t >::max();

	static uint8_t constexpr apply( uint8_t input )
	{
		return std::pow( (double) input / max, ( 1.0 / Gamma::value ) ) * max;
	}
};

} // namespace detail

template< typename Gamma >
struct basic_gamma_table
{
	static uint8_t get( uint8_t input )
	{
		return libcpp::value_table< detail::gamma_function< Gamma > >::get( input );
	}

	static uint8_t revert( uint8_t result )
	{
		return libcpp::value_table< detail::gamma_function< Gamma > >::revert( result );
	}
};

struct gamma_value
{
	static double constexpr const value = 0.4;
};

typedef basic_gamma_table< gamma_value > gamma_table;

} // namespace tp

#endif // TP_GAMMA_TABLE_HPP
