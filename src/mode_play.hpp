#ifndef TEENSY_POI_MODE_PLAY_HPP
#define TEENSY_POI_MODE_PLAY_HPP

#include <limits>
//#include <peripheral.h>

#include <libcpp/variant.hpp>

#include "led_algorithm.hpp"

namespace tp {

	template< typename Manager >
	class ModePlay
	{
		static constexpr auto pixelCount = Manager::Leds::pixelCount;

        using ErrorAnimation = LedFlashAnimation< 500000, 500000, LedColor::BLACK, LedColor::DIM_RED >;

	public:
		explicit ModePlay( Manager const& manager )
			: manager_( manager )
			, motion_( manager_.stopwatch() )
			, lastFrameTimestamp_()
		{
			next();
		}

		ModePlay( ModePlay const& ) = delete;

		void buttonClicked()
		{
			next();
		}

		void advance()
		{
		    if ( !motion_ || !flash_ || !image_ ) {
                auto& pixels = leds_.pixels();
                auto timestamp = manager_.stopwatch().timestamp();
                pixels[ 0 ] = !motion_ ? ErrorAnimation::animate( timestamp ) : LedColor::BLACK;
                pixels[ 1 ] = !flash_ ? ErrorAnimation::animate( timestamp ) : LedColor::BLACK;
                pixels[ 2 ] = !image_ ? ErrorAnimation::animate( timestamp ) : LedColor::BLACK;
                leds_.send();
                return;
		    }

		    auto rps = motion_.roundsPerSecond();
		    auto frameDuration = rps > 0.01f ? (uint32_t) ( 1000000.f / ( rps * image_.content().linesPerRound() ) ) : std::numeric_limits< uint32_t >::max();

		    auto elapsed = manager_.stopwatch().difference( lastFrameTimestamp_ );
		    if ( elapsed >= frameDuration ) {
                //debug( "lpr ", image_.content().linesPerRound(), " @ ", rps, " duration was ", frameDuration, ", waited ", elapsed - frameDuration, " more" );
                lastFrameTimestamp_ = manager_.stopwatch().timestamp();

                auto& pixels = leds_.pixels();
                flash_.read( image_, reinterpret_cast< uint8_t* >( &pixels[ 0 ] ), sizeof( LedPixel ) * pixelCount );
                leds_.send();
		    }

		    motion_.advance();
		}

	private:
		void next()
		{
			leds_.clear();
			image_ = flash_.next( image_ );
		}

		Manager const& manager_;
		typename Manager::Leds leds_;
		typename Manager::Motion motion_;
		typename Manager::Flash flash_;
		typename Manager::Flash::Image image_;
		uint32_t lastFrameTimestamp_;
	};

} // namespace tp

#endif // TEENSY_POI_MODE_PLAY_HPP
