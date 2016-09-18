#ifndef TLIBCPP_TEENSY_PINS_HPP
#define TLIBCPP_TEENSY_PINS_HPP

#include <tlibcpp/ports.hpp>
#include <tlibcpp/pin.hpp>

namespace tlibcpp {

    namespace detail {

        template< uint8_t N >
        struct digital_pin;

#define TLIBCPP_DEFINE_DIGITAL_PIN( no_, port_, pin_ ) \
        template<> struct digital_pin< no_ > { \
            using type = pin< port_, pin_ >; \
        }

#if defined( __MKL26Z64__ )

        TLIBCPP_DEFINE_DIGITAL_PIN(  0, portb, 16 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  1, portb, 17 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  2, portd,  0 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  3, porta,  1 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  4, porta,  2 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  5, portd,  7 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  6, portd,  4 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  7, portd,  2 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  8, portd,  3 );
        TLIBCPP_DEFINE_DIGITAL_PIN(  9, portc,  3 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 10, portc,  4 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 11, portc,  6 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 12, portc,  7 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 13, portc,  5 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 14, portd,  1 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 15, portc,  0 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 16, portb,  0 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 17, portb,  1 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 18, portb,  3 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 19, portb,  2 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 20, portd,  5 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 21, portd,  6 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 22, portc,  1 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 23, portc,  2 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 24, porte, 20 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 25, porte, 21 );
        TLIBCPP_DEFINE_DIGITAL_PIN( 26, porte, 30 );

#endif

#undef TLIBCPP_DEFINE_DIGITAL_PIN

    } // namespace detail


    template< uint8_t N > using digital_pin = typename detail::digital_pin< N >::type;


    namespace detail {

        template< uint8_t N >
        struct analog_pin;


#define TLIBCPP_DEFINE_ANALOG_PIN( no_, port_, pin_ ) \
        template<> struct analog_pin< no_ > { \
            using type = pin< port_, pin_ >; \
        }

#if defined( __MKL26Z64__ )

        TLIBCPP_DEFINE_ANALOG_PIN( 0, portd, 1 );
        TLIBCPP_DEFINE_ANALOG_PIN( 1, portc, 0 );
        TLIBCPP_DEFINE_ANALOG_PIN( 2, portb, 0 );
        TLIBCPP_DEFINE_ANALOG_PIN( 3, portb, 1 );
        TLIBCPP_DEFINE_ANALOG_PIN( 4, portb, 3 );
        TLIBCPP_DEFINE_ANALOG_PIN( 5, portb, 2 );
        TLIBCPP_DEFINE_ANALOG_PIN( 6, portd, 5 );
        TLIBCPP_DEFINE_ANALOG_PIN( 7, portd, 6 );
        TLIBCPP_DEFINE_ANALOG_PIN( 8, portc, 1 );
        TLIBCPP_DEFINE_ANALOG_PIN( 9, portc, 2 );
        TLIBCPP_DEFINE_ANALOG_PIN( 10, porte, 20 );
        TLIBCPP_DEFINE_ANALOG_PIN( 11, porte, 21 );
        TLIBCPP_DEFINE_ANALOG_PIN( 12, porte, 30 );

#endif

#undef TLIBCPP_DEFINE_ANALOG_PIN

    } // namespace detail


    template< uint8_t N > using analog_pin = typename detail::analog_pin< N >::type;

} // namespace tlibcpp

#endif // TLIBCPP_TEENSY_PINS_HPP
