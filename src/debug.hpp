#ifndef TEENSY_POI_DEBUG_HPP
#define TEENSY_POI_DEBUG_HPP

#include <cstdlib>
#include <utility>
#include <type_traits>
#include <string.h>

#include "usb_serial.h"
#include "tstdlib.h"

namespace detail {

	template< typename T >
	struct debug_printer;

	template< typename T >
	struct debug_printer< T* >
	{
		static void print( T* arg )
		{
			char buf[ 32 ];
			ultoa( (uint32_t) arg, buf, 10 );
			usb_serial_write( buf, strlen( buf ) );
		}
	};

	template< typename T >
	struct debug_printer< T* volatile >
	{
		static void print( T* arg )
		{
			char buf[ 32 ];
			ultoa( (uint32_t) arg, buf, 10 );
			usb_serial_write( buf, strlen( buf ) );
		}
	};

	template<>
	struct debug_printer< char const* >
	{
		static void print( char const* arg, size_t len )
		{
			usb_serial_write( arg, len );
		}

		static void print( char const* arg )
		{
			print( arg, strlen( arg ) );
		}
	};

	template< size_t N >
	struct debug_printer< char [ N ] >
	{
		static void print( char const ( &arg )[ N ] )
		{
			usb_serial_write( arg, N - 1 );
		}
	};

	template<>
	struct debug_printer< bool >
	{
		static void print( bool arg )
		{
			arg ? usb_serial_write( "true", 4 ) : usb_serial_write( "false", 5 );
		}
	};

    template< typename T >
	struct debug_printer
	{
	    static void print( uint8_t arg ) { print_ul( arg ); }
	    static void print( uint16_t arg ) { print_ul( arg ); }
	    static void print( uint32_t arg ) { print_ul( arg ); }
	    static void print( uint64_t arg ) { print_ul( arg ); }
	    static void print( int8_t arg ) { print_l( arg ); }
	    static void print( int16_t arg ) { print_l( arg ); }
	    static void print( int32_t arg ) { print_l( arg ); }
	    static void print( int64_t arg ) { print_l( arg ); }

	    static void print( float arg )
	    {
	        int32_t whole = arg;
	        int32_t fract = std::abs( (int32_t) ( arg * 1000.0f ) ) % 1000;
	        char buf[ 8 ], *p = buf;
	        *p++ = '.';
	        if ( fract < 100 ) *p++ = '0';
	        if ( fract < 10 ) *p++ = '0';
	        ultoa( fract, p, 10 );
	        debug_printer< int32_t >::print( whole );
	        debug_printer< char const* >::print( buf );
	    }

	    static void print_ul( T arg )
	    {
	        char buf[ 32 ];
			ultoa( arg, buf, 10 );
			usb_serial_write( buf, strlen( buf ) );
	    }

	    static void print_l( T arg )
	    {
	        char buf[ 32 ];
			ltoa( arg, buf, 10 );
			usb_serial_write( buf, strlen( buf ) );
	    }
	};


	inline void debug_impl()
	{
	}

	template< typename T0, typename... T >
	void debug_impl( T0 const& arg0, T&&... args )
	{
		detail::debug_printer< T0 >::print( arg0 );
		debug_impl( std::forward< T >( args )... );
	}


} // namespace detail

template< typename... T >
void debug( T&&... args )
{
	detail::debug_impl( std::forward< T >( args )..., "\n" );
}

template< typename... T >
void debugn( T&&... args )
{
	detail::debug_impl( std::forward< T >( args )... );
}

#endif // TEENSY_POI_DEBUG_HPP
