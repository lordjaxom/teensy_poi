#ifndef TEENSY_POI_LEDS_HPP
#define TEENSY_POI_LEDS_HPP

#include <libcpp/numeric.hpp>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <iterator>

#include <SPI.h>
#include <DmaSpi.h>

#include "gamma_table.hpp"
#include "power.hpp"

namespace tp {

	struct LedPixel
	{
		LedPixel() = default;
		constexpr LedPixel( uint8_t r, uint8_t g, uint8_t b ) : b( b ), g( g ), r( r ) {}
		constexpr LedPixel( uint32_t color ) : b( (uint8_t) ( color & 0xff ) ), g( (uint8_t) ( ( color >> 8 ) & 0xff ) ), r( (uint8_t) ( ( color >> 16 ) & 0xff ) ) {}

    private:
		uint8_t global = 0xff;
	public:
		uint8_t b;
		uint8_t g;
		uint8_t r;
	} __attribute__(( packed ));


	template< size_t N >
	struct LedPixelBuffer
	{
		static constexpr size_t size = N;

		using Pixel = LedPixel;

		Pixel& operator[]( size_t index ) { return pixels[ index ]; }
		Pixel const& operator[]( size_t index ) const { return pixels[ index ]; }

    private:
		uint32_t start = 0x00;
		Pixel pixels[ size ];
		uint32_t end = 0xff;
	} __attribute__(( packed ));


	namespace LedColor {

		static constexpr LedPixel BLACK( 0x00, 0x00, 0x00 );

		static constexpr LedPixel RED( 0xff, 0x00, 0x00 );
		static constexpr LedPixel GREEN( 0x00, 0xff, 0x00 );
		static constexpr LedPixel BLUE( 0x00, 0x00, 0xff );
		static constexpr LedPixel YELLOW( 0xff, 0xff, 0x00 );
		static constexpr LedPixel PURPLE( 0xff, 0x00, 0xff );
		static constexpr LedPixel CYAN( 0x00, 0xff, 0xff );
		static constexpr LedPixel WHITE( 0xff, 0xff, 0xff );

		static constexpr LedPixel DIM_RED( 0x24, 0x00, 0x00 );
		static constexpr LedPixel DIM_GREEN( 0x00, 0x24, 0x00 );
		static constexpr LedPixel DIM_BLUE( 0x00, 0x00, 0x24 );
		static constexpr LedPixel DIM_YELLOW( 0x24, 0x24, 0x00 );
		static constexpr LedPixel DIM_PURPLE( 0x24, 0x00, 0x24 );
		static constexpr LedPixel DIM_CYAN( 0x00, 0x24, 0x24 );
		static constexpr LedPixel DIM_WHITE( 0x24, 0x24, 0x24 );

		static constexpr LedPixel FAINT_RED( 0x01, 0x00, 0x00 );
		static constexpr LedPixel FAINT_GREEN( 0x00, 0x01, 0x00 );
		static constexpr LedPixel FAINT_BLUE( 0x00, 0x00, 0x01 );
		static constexpr LedPixel FAINT_YELLOW( 0x01, 0x01, 0x00 );
		static constexpr LedPixel FAINT_PURPLE( 0x01, 0x00, 0x01 );
		static constexpr LedPixel FAINT_CYAN( 0x00, 0x01, 0x01 );
		static constexpr LedPixel FAINT_WHITE( 0x01, 0x01, 0x01 );

	} // namespace LedColor


	namespace LedsDetail {

		template< size_t Count, typename TData, typename TClock, typename TPower, uint32_t Speed >
		struct LedTraits
		{
			static constexpr size_t count = Count;
			static constexpr uint32_t speed = Speed;

			using Data = TData;
			using Clock = TClock;
			using Power = TPower;
		};


		template< typename Derived, typename Traits >
		class LedsBase
		{
            static PowerGuard< typename Traits::Power > power;

		protected:
            using SPI = ::SPI< typename Traits::Data, tlibcpp::null_pin, typename Traits::Clock >;

        public:
            using Pixel = LedPixel;
            using Output = LedPixelBuffer< Traits::count >;

			static_assert( Output::size % 2 == 0, "number of leds must be even" );
            static constexpr size_t pixelCount = Output::size / 2;

            using Pixels = LedPixelBuffer< pixelCount >;

            LedsBase()
                : pixels_()
                , output_()
            {
                power.enable();
            }

            LedsBase( LedsBase const& ) = delete;

            ~LedsBase()
            {
                power.disable();
            }

            Pixels& pixels() { return pixels_; }
            Pixels const& pixels() const { return pixels_; }

			void clear()
			{
				self().waitForTransfer();
				this->output_ = {};
				self().sendPixels();
			}

			void send()
			{
				self().waitForTransfer();
				std::transform( &pixels_[ 0 ], &pixels_[ pixelCount ], &output_[ 0 ],
								[]( LedPixel const& px ) {
									return LedPixel(
											gamma_table::get( px.r ),
											gamma_table::get( px.g ),
											gamma_table::get( px.b ) );
								} );
                std::reverse_copy( &output_[ 0 ], &output_[ pixelCount ], &output_[ pixelCount ] );
				self().sendPixels();
			}

        protected:
			SPI spi_;
            Pixels pixels_;
            Output output_;

		private:
			Derived& self() { return *static_cast< Derived* >( this ); }
		};

        template< typename Derived, typename Traits >
        PowerGuard< typename Traits::Power > LedsBase< Derived, Traits >::power;


        template< typename Traits >
        struct LedsSpiWrapper
        {
        	class LedsSpi : public LedsBase< LedsSpi, Traits >
        	{
				using Base = LedsBase< LedsSpi, Traits >;
				friend Base;

				using typename Base::SPI;

			public:
				using typename Base::Pixel;
				using typename Base::Pixels;

				using Base::pixelCount;

				LedsSpi( LedsSpi const& ) = delete;

				~LedsSpi()
				{
					clear();
				}

				using Base::pixels;
				using Base::clear;
				using Base::send;

			private:
				void sendPixels()
				{
					this->spi_.template beginTransaction< Traits::speed >();
					this->spi_.send( &this->output_, sizeof( this->output_ ) );
					this->spi_.endTransaction();
				}

				void waitForTransfer()
				{
				}
        	};
        };

        template< typename Traits >
        struct LedsDmaWrapper
        {
        	class LedsDma : public LedsBase< LedsDma, Traits >
        	{
				using Base = LedsDetail::LedsBase< LedsDma, Traits >;
				friend Base;

				using typename Base::SPI;

				using DmaSpi = ::DmaSpi< SPI, tlibcpp::null_pin, Traits::speed >;

			public:
				using typename Base::Pixel;
				using typename Base::Pixels;

				using Base::pixelCount;

				LedsDma()
					: dmaSpi_( this->spi_ )
				{
					dmaSpi_.start();
				}

				LedsDma( LedsDma const& ) = delete;

				~LedsDma()
				{
					clear();
					waitForTransfer();
				}

				using Base::pixels;
				using Base::clear;
				using Base::send;

			private:
				void sendPixels()
				{
					transfer_ = DmaSpiTransfer( reinterpret_cast< uint8_t const* >( &this->output_ ), sizeof( this->output_ ) );
					dmaSpi_.registerTransfer( transfer_ );
				}

				void waitForTransfer()
				{
					while ( transfer_.busy() ) {}
				}

				DmaSpi dmaSpi_;
				DmaSpiTransfer transfer_;
        	};
        };

	} // namespace LedsDetail


	template<
		size_t Count, typename TData, typename TClock, typename TPower, uint32_t Speed,
		typename Traits = LedsDetail::LedTraits< Count, TData, TClock, TPower, Speed > >
	using LedsSpi = typename LedsDetail::LedsSpiWrapper< Traits >::LedsSpi;

	template<
		size_t Count, typename TData, typename TClock, typename TPower, uint32_t Speed,
		typename Traits = LedsDetail::LedTraits< Count, TData, TClock, TPower, Speed > >
	using LedsDma = typename LedsDetail::LedsDmaWrapper< Traits >::LedsDma;

} // namespace tp

#endif // TEENSY_POI_LEDS_HPP
