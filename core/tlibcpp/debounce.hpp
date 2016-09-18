#ifndef TLIBCPP_DEBOUNCE_HPP
#define TLIBCPP_DEBOUNCE_HPP

namespace tlibcpp {

template< typename Pin, size_t Freq, size_t Millis = 80 >
struct debounce_pin
{
	static_assert( 1000 / Freq * 2 <= Millis, "frequency too small" );

	typedef Pin pin_type;
	static size_t const frequency = Freq;
	static size_t const milliseconds = Millis;

	static void poll()
	{
		if ( pin_type::read() ) {
			state += 1000 / Freq;
		}
		else {
			if ( state >= milliseconds ) {
				clicked = true;
			}
			state = 0;
		}
	}

	static bool is_clicked()
	{
		bool result = clicked;
		clicked = false;
		return result;
	}

private:
	static size_t state;
	static bool clicked;
};

template< typename Pin, size_t Freq, size_t Millis > 
size_t debounce_pin< Pin, Freq, Millis >::state;

template< typename Pin, size_t Freq, size_t Millis > 
bool debounce_pin< Pin, Freq, Millis >::clicked;

} // namespace tlibcpp

#endif // TLIBCPP_DEBOUNCE_HPP
