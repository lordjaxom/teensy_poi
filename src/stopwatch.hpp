#ifndef TEENSY_POI_STOPWATCH_HPP
#define TEENSY_POI_STOPWATCH_HPP

#include <limits>
#include <stdint.h>
#include <tstdlib.h>

namespace tp {

	class Stopwatch
	{
	public:
		static uint32_t difference( uint32_t start, uint32_t end )
		{
			return end >= start ? end - start : std::numeric_limits< uint32_t >::max() - start + end;
		}

		Stopwatch()
			: timestamp_( micros() )
			, lastTotal_()
		{
		}

		Stopwatch( Stopwatch const& ) = delete;

		void advance()
		{
			uint32_t lastTimestamp = timestamp_;
			timestamp_ = micros();
			lastTotal_ = difference( lastTimestamp, timestamp_ );
		}

		uint32_t timestamp() const
		{
			return timestamp_;
		}

		uint32_t lastTotal() const
		{
			return lastTotal_;
		}

		uint32_t elapsed() const
		{
			return difference( timestamp_, micros() );
		}

		uint32_t difference( uint32_t start ) const
		{
		    return difference( start, timestamp_ );
		}

		uint32_t remaining( uint32_t timeout ) const
		{
			uint32_t mark = elapsed();
			return timeout > mark ? timeout - mark : 0;
		}

		template< typename T >
		T countdown( T& remaining ) const
		{
		    return remaining = lastTotal_ < remaining ? remaining - lastTotal_ : 0;
		}

	private:
		uint32_t timestamp_;
		uint32_t lastTotal_;
	};

} // namespace tp

#endif // TEENSY_POI_STOPWATCH_HPP
