#ifndef TLIBCPP_PERIODIC_HPP
#define TLIBCPP_PERIODIC_HPP

#include <cstdint>

#include <teensy.h>

#include <tlibcpp/nvic.hpp>

namespace tlibcpp {

namespace detail {

template< size_t N, size_t Vector > 
struct periodic_timer_traits_impl
{
	static size_t const interrupt_vector = Vector;

	template< void (*Func)() >
	static void isr()
	{
		PIT_TFLG(N) = PIT_TFLG_TIF_MASK;
		Func();
	}
};

template< size_t N > struct periodic_timer_traits;
template<> struct periodic_timer_traits< 0 > : periodic_timer_traits_impl< 0, INT_PIT0 > {};
template<> struct periodic_timer_traits< 1 > : periodic_timer_traits_impl< 1, INT_PIT1 > {};
template<> struct periodic_timer_traits< 2 > : periodic_timer_traits_impl< 2, INT_PIT2 > {};
template<> struct periodic_timer_traits< 3 > : periodic_timer_traits_impl< 3, INT_PIT3 > {};

} // namespace detail

template< size_t N >
struct periodic_timer
{
	static void init()
	{
		SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; // enable PIT clocks
		PIT_MCR = 0; // turn on PIT
		PIT_TCTRL( N ) = PIT_TCTRL_TIE_MASK; // enable interrupt generation

		nvic< detail::periodic_timer_traits< N >::interrupt_vector >::clear_pending();
		nvic< detail::periodic_timer_traits< N >::interrupt_vector >::enable();
		nvic< detail::periodic_timer_traits< N >::interrupt_vector >::set_priority(0);

		//__interrupt_vector_table[detail::periodic_timer_traits< N >::interrupt_vector] = &detail::periodic_timer_traits< N >::template isr< Isr >;
	}

	static void init( uint32_t usecs )
	{
		init();
		start( usecs );
	}

	static void stop()
	{
		PIT_TCTRL( N ) &= ~PIT_TCTRL_TEN_MASK;
	}

	static void start()
	{
		PIT_TCTRL( N ) |= PIT_TCTRL_TEN_MASK;
	}

	static void start( uint32_t usecs )
	{
		PIT_LDVAL( N ) = ( periph_clk_khz / 1000 ) * usecs - 1;
		start();
	}

	static void restart( uint32_t usecs )
	{
		stop();
		start( usecs );
	}
};

} // namespace tlibcpp

#endif // TLIBCPP_PERIODIC_HPP
