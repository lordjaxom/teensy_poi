#ifndef TLIBCPP_PORTA_HPP
#define TLIBCPP_PORTA_HPP

#include <stdint.h>
#include "teensy.h"

#if defined( KINETISK )
#   define TLIBCPP_GPIOx_PDOR( port_ ) GPIO ## port_ ## _PDOR
#   define TLIBCPP_GPIOx_PSOR( port_ ) GPIO ## port_ ## _PSOR
#   define TLIBCPP_GPIOx_PCOR( port_ ) GPIO ## port_ ## _PCOR
#   define TLIBCPP_GPIOx_PTOR( port_ ) GPIO ## port_ ## _PTOR
#   define TLIBCPP_GPIOx_PDDR( port_ ) GPIO ## port_ ## _PDDR
#   define TLIBCPP_GPIOx_PDIR( port_ ) GPIO ## port_ ## _PDIR
#elif defined( KINETISL )
#   define TLIBCPP_GPIOx_PDOR( port_ ) FGPIO ## port_ ## _PDOR
#   define TLIBCPP_GPIOx_PSOR( port_ ) FGPIO ## port_ ## _PSOR
#   define TLIBCPP_GPIOx_PCOR( port_ ) FGPIO ## port_ ## _PCOR
#   define TLIBCPP_GPIOx_PTOR( port_ ) FGPIO ## port_ ## _PTOR
#   define TLIBCPP_GPIOx_PDDR( port_ ) FGPIO ## port_ ## _PDDR
#   define TLIBCPP_GPIOx_PDIR( port_ ) FGPIO ## port_ ## _PDIR
#endif

#define TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, bit_ ) \
	template<> struct port ## name_ ## _pin< bit_ > { \
		static constexpr uint32_t volatile* pcr_reg = &PORT ## port_ ## _PCR ## bit_; \
	}

#define TLIBCPP_DEFINE_PORT( name_, port_ ) \
	namespace detail { \
		template< uint8_t Bit > struct port ## name_ ## _pin; \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 0 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 1 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 2 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 3 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 4 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 5 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 6 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 7 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 8 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 9 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 10 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 11 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 12 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 13 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 14 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 15 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 16 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 17 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 18 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 19 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 20 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 21 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 22 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 23 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 24 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 25 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 26 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 27 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 28 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 29 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 30 ); \
		TLIBCPP_PORT_DEFINE_PIN_CONFIG( name_, port_, 31 ); \
	} \
	struct port ## name_ { \
		static constexpr uint32_t volatile* pdor_reg = &TLIBCPP_GPIOx_PDOR( port_ ); \
		static constexpr uint32_t volatile* psor_reg = &TLIBCPP_GPIOx_PSOR( port_ ); \
		static constexpr uint32_t volatile* pcor_reg = &TLIBCPP_GPIOx_PCOR( port_ ); \
		static constexpr uint32_t volatile* ptor_reg = &TLIBCPP_GPIOx_PTOR( port_ ); \
		static constexpr uint32_t volatile* pddr_reg = &TLIBCPP_GPIOx_PDDR( port_ ); \
		static constexpr uint32_t volatile* pdir_reg = &TLIBCPP_GPIOx_PDIR( port_ ); \
		template< uint8_t Bit > using pinx = detail::port ## name_ ## _pin< Bit >; \
	}

namespace tlibcpp {

TLIBCPP_DEFINE_PORT( a, A );
TLIBCPP_DEFINE_PORT( b, B );
TLIBCPP_DEFINE_PORT( c, C );
TLIBCPP_DEFINE_PORT( d, D );
TLIBCPP_DEFINE_PORT( e, E );

} // namespace tlibcpp

#undef TLIBCPP_PORT_DEFINE_PIN_CONFIG
#undef TLIBCPP_DEFINE_PORT
#undef TLIBCPP_GPIOx_PDOR
#undef TLIBCPP_GPIOx_PSOR
#undef TLIBCPP_GPIOx_PCOR
#undef TLIBCPP_GPIOx_PTOR
#undef TLIBCPP_GPIOx_PDDR
#undef TLIBCPP_GPIOx_PDIR

#endif // TLIBCPP_PORTA_HPP
