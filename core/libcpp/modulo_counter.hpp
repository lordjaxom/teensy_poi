#ifndef LIBCPP_MODULO_COUNTER_HPP
#define LIBCPP_MODULO_COUNTER_HPP

#include <libcpp/numeric.hpp>

namespace libcpp {

	template< typename Type, Type Denominator >
	class modulo_counter
	{
	public:
		modulo_counter( Type value = {} )
			: value_( value )
		{
		}

		operator Type() const { return value_; }

		template< typename T, typename std::enable_if< std::is_convertible< T, Type >::value >::type* = nullptr >
		bool operator==( T other ) const { return value_ == other; }

		template< typename T >
		bool operator!=( T other ) const { return !( *this == other ); }

		modulo_counter& operator++()
		{
			increment();
			return *this;
		}

		modulo_counter operator++( int )
		{
			modulo_counter result( *this );
			++*this;
			return result;
		}

	private:
		void increment()
		{
			value_ = value_ < Denominator - 1 ? value_ + 1 : 0;
		}

		Type value_;
	};

} // namespace libcpp

#endif // LIBCPP_MODULO_COUNTER_HPP
