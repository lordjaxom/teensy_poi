#ifndef LIBCPP_SAMPLE_BUFFER_HPP
#define LIBCPP_SAMPLE_BUFFER_HPP

#include <algorithm>
#include <utility>

#include <libcpp/modulo_counter.hpp>

namespace libcpp {

	template< typename T, size_t Size >
	class sample_buffer
	{
	public:
		explicit sample_buffer()
			: used_()
		{
		}

		size_t size() const { return used_; }
		bool empty() const { return used_ == 0; }

		void push_back( T const& value )
		{
			samples_[ index_ ] = value;
			increment();
		}

		void push_back( T&& value )
		{
			samples_[ index_ ] = std::move( value );
			increment();
		}

		T average() const
		{
			return std::accumulate( &samples_[ 0 ], &samples_[ used_ ], T() ) / used_;
		}

	private:
		void increment()
		{
			++index_;
			if ( used_ < Size ) {
				++used_;
			}
		}

		T samples_[ Size ];
		size_t used_;
		modulo_counter< size_t, Size > index_;
	};

} // namespace libcpp

#endif // LIBCPP_SAMPLE_BUFFER_HPP
