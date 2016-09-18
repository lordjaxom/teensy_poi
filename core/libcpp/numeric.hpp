#ifndef LIBCPP_NUMERIC_HPP
#define LIBCPP_NUMERIC_HPP

#include <stdint.h>
#include <limits>
#include <ratio>
#include <type_traits>

namespace libcpp {

#if 0
	namespace detail {

		template< typename MaxT, MaxT N, typename... T >
		struct least_impl;

		template< typename MaxT, MaxT N, typename T0 >
		struct least_impl< MaxT, N, T0 >
		{
			using type = T0;
		};

		template< typename MaxT, MaxT N, typename T0, typename T1, typename... T >
		struct least_impl< MaxT, N, T0, T1, T... >
		{
			static_assert(
					std::numeric_limits< T0 >::max() <= std::numeric_limits< T1 >::max(),
					"types not in ascending order regarding sizes" );

			using type = typename std::conditional<
					N >= std::numeric_limits< T0 >::min() && N <= std::numeric_limits< T0 >::max(),
					T0, typename least_impl< MaxT, N, T1, T... >::type
				>::type;
		};

	} // namespace detail

	template< uintmax_t I >
	using uint_least_t = typename detail::least_impl< uintmax_t, I, uint8_t, uint16_t, uint32_t, uint64_t >::type;


#define LIBCPP_DEFINE_INTEGRAL_CONSTANT( type_ ) \
	template< type_ N > using type_ ## _constant = std::integral_constant< type_, N >

	LIBCPP_DEFINE_INTEGRAL_CONSTANT( char );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( short );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( int );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( long );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( int8_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( int16_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( int32_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( int64_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( intmax_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uint8_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uint16_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uint32_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uint64_t );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uintmax_t );

#undef LIBCPP_DEFINE_INTEGRAL_CONSTANT

#define LIBCPP_DEFINE_INTEGRAL_CONSTANT( name_, type_ ) \
	template< type_ N > using name_ ## _constant = std::integral_constant< type_, N >

	LIBCPP_DEFINE_INTEGRAL_CONSTANT( schar, signed char );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uchar, unsigned char );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( ushort, unsigned short );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( uint, unsigned int );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( ulong, unsigned long );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( longlong, long long );
	LIBCPP_DEFINE_INTEGRAL_CONSTANT( ulonglong, unsigned long long );

#undef LIBCPP_DEFINE_INTEGRAL_CONSTANT

	template< uintmax_t N > using uint_least_constant = std::integral_constant< uint_least_t< N >, N >;


	template< typename T, intmax_t Num, intmax_t Den = 1 >
	struct floating_point_constant : std::ratio< Num, Den >
	{
		using value_type = T;

		static constexpr value_type value = (value_type) Num / Den;
	};

	template< intmax_t Num, intmax_t Den = 1 > using float_constant = floating_point_constant< float, Num, Den >;
	template< intmax_t Num, intmax_t Den = 1 > using double_constant = floating_point_constant< double, Num, Den >;
	template< intmax_t Num, intmax_t Den = 1 > using longdouble_constant = floating_point_constant< long double, Num, Den >;


	template< typename Value >
	using value_type = typename Value::value_type;


	namespace detail {

		template< typename Value, typename Enable = void >
		struct index_bound;

		template< typename Value >
		struct index_bound<
			Value,
			typename std::enable_if< std::is_unsigned< typename Value::value_type >::value >::type >
		{
			using value = uint_least_constant< Value::value - 1 >;
		};

	} // namespace detail

	template< typename Value >
	using index_bound = typename detail::index_bound< Value >::value;

	template< typename Value >
	using index_type = value_type< index_bound< Value > >;


	template< typename Value >
	using promoted_type = decltype( +std::declval< typename Value::value_type >() );
#endif


	template< typename T, intmax_t Num, intmax_t Denom = 1 >
	struct floating_point_constant : std::ratio< Num, Denom >
	{
		using base_type = std::ratio< Num, Denom >;
		using base_type::num;
		using base_type::den;

		using value_type = T;

		static constexpr value_type value = (value_type) num / den;
	};


	template< typename T0 >
	constexpr T0 min( T0 const& arg )
	{
		return arg;
	}

	template< typename T0, typename... T >
	constexpr typename std::common_type< T0, T... >::type min( T0 const& arg0, T const&... args )
	{
		return arg0 < min( args... ) ? arg0 : min( args... );
	}


	template< typename T0 >
	constexpr T0 max( T0 const& arg )
	{
		return arg;
	}

	template< typename T0, typename... T >
	constexpr typename std::common_type< T0, T... >::type max( T0 const& arg0, T const&... args )
	{
		return arg0 > max( args... ) ? arg0 : max( args... );
	}


	template< typename T, typename Max >
	T clip( T value, Max )
	{
		while ( value > Max::value ) {
			value -= Max::value;
		}
		return value;
	}

} // namespace libcpp

#endif // LIBCPP_NUMERIC_HPP
