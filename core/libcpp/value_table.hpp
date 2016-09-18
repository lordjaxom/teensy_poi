#ifndef LIBCPP_VALUE_TABLE_HPP
#define LIBCPP_VALUE_TABLE_HPP

#include <cstdlib>
#include <limits>

namespace libcpp {

namespace detail {

template< typename R, typename I >
struct value_table_function_traits
{
	typedef R result_type;
	typedef I input_type;
};

template< typename R, typename I >
value_table_function_traits< R, I > get_value_table_function_traits( R ( *func )( I ) );

template< typename F >
struct value_table_traits
{
	typedef decltype( get_value_table_function_traits( &F::apply ) ) function_traits;
	typedef typename function_traits::result_type result_type;
	typedef typename function_traits::input_type input_type;
};

template< 
	typename F,
	typename value_table_traits< F >::input_type I,
	typename value_table_traits< F >::input_type To,
	typename value_table_traits< F >::result_type... Values
>
struct value_table_values
{
	typedef value_table_traits< F > traits_type;
	typedef typename traits_type::result_type result_type;
	typedef typename traits_type::input_type input_type;

	typedef value_table_values< F, I + 1, To, Values..., F::apply( I ) > next_type;
	static decltype( next_type::values ) constexpr& values = next_type::values;
};

template< 
	typename F,
	typename value_table_traits< F >::input_type I,
	typename value_table_traits< F >::input_type To,
	typename value_table_traits< F >::result_type... Values
>
decltype( value_table_values< F, I, To, Values... >::next_type::values ) constexpr& value_table_values< F, I, To, Values... >::values;

template< 
	typename F, 
	typename value_table_traits< F >::input_type To,
	typename value_table_traits< F >::result_type... Values
>
struct value_table_values< F, To, To, Values... >
{
	typedef value_table_traits< F > traits_type;
	typedef typename traits_type::result_type result_type;
	
	static result_type constexpr const values[] { Values..., F::apply( To ) };
};


template< 
	typename F, 
	typename value_table_traits< F >::input_type To,
	typename value_table_traits< F >::result_type... Values
>
typename value_table_traits< F >::result_type constexpr const value_table_values< F, To, To, Values... >::values[];

} // namespace detail

template< 
	typename F, 
	typename detail::value_table_traits< F >::input_type From = std::numeric_limits< typename detail::value_table_traits< F >::input_type >::min(), 
	typename detail::value_table_traits< F >::input_type To   = std::numeric_limits< typename detail::value_table_traits< F >::input_type >::max()
>
class value_table
{
	typedef detail::value_table_traits< F > traits_type;
	typedef typename traits_type::result_type result_type;
	typedef typename traits_type::input_type input_type;

	typedef detail::value_table_values< F, From, To > values_type;

public:
	static result_type constexpr get( input_type input )
	{
		return values_type::values[ input - From ];
	}
};

} // namespace libcpp

#endif // LIBCPP_VALUE_TABLE_HPP
