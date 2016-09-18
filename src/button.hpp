#ifndef TEENSY_POI_BUTTON_HPP
#define TEENSY_POI_BUTTON_HPP

#include <limits>
#include <stddef.h>

#include <libcpp/function.hpp>

#include "stopwatch.hpp"

namespace tp {

	template< typename Pin, uint32_t DebounceUs = 50000 >
	class Debounce
	{
		static constexpr uint32_t debounceUs = DebounceUs;

		struct State
		{
			uint32_t counter;
			bool lastState;
			bool pressed;
		};

	public:
		explicit Debounce( Stopwatch const& stopwatch )
			: stopwatch_( stopwatch )
			, state_()
		{
		}

		Debounce( Debounce const& ) = delete;

		void advance()
		{
			bool current = !Pin::read();
			if ( current != state_.lastState ) {
				state_.counter = debounceUs;
			}
			else if ( state_.counter > 0 ) {
                if ( stopwatch_.countdown( state_.counter ) == 0 ) {
                    state_.pressed = current;
                }
			}
			state_.lastState = current;
		}

		bool pressed() const
		{
			return state_.pressed;
		}

		bool polling() const
		{
			return state_.counter > 0;
		}

	private:
		Stopwatch const& stopwatch_;
		State state_;
	};

	template< typename Pin, size_t LongpressUs = 2000000, size_t DebounceUs = 50000 >
	class Button
	{
	public:
		using PinType = Pin;

		static constexpr size_t debounceUs = DebounceUs;
		static constexpr size_t longpressUs = LongpressUs;

	private:
		static constexpr uint8_t NoneState = 0x00;
		static constexpr uint8_t ClickState = 0x01;
		static constexpr uint8_t LongpressState = 0x02;

		static constexpr uint32_t counter_npos = 0x00ffffff;

		struct State
		{
			uint32_t counter;
			uint8_t  state;
		};

	public:
		explicit Button( Stopwatch const& stopwatch )
			: stopwatch_( stopwatch )
			, debounce_( stopwatch_ )
			, state_()
		{
			Pin::input();
			Pin::pullup();
		}

		Button( Button const& ) = delete;

		void advance()
		{
			state_.state = NoneState;
			debounce_.advance();
			if ( debounce_.pressed() ) {
				if ( state_.counter == 0 ) {
					// button got initially pressed, start counting
					state_.counter = longpressUs;
				}
				else if ( state_.counter != counter_npos ) {
                    if ( stopwatch_.countdown( state_.counter ) == 0 ) {
						// button was pressed longer than longpress_ticks
						state_.counter = counter_npos;
						state_.state = LongpressState;
						if ( onLongpressed ) {
							onLongpressed();
						}
					}
				}
			}
			else if ( state_.counter == counter_npos ) {
				// button was released after longpress, just reset state
				state_.counter = 0;
			}
			else if ( state_.counter > 0 ) {
				// button was released before longpress_ticks, report and reset state
				state_.counter = 0;
				state_.state = ClickState;
				if ( onClicked ) {
					onClicked();
				}
			}
		}

		uint8_t state() const
		{
			return state_;
		}

		bool clicked() const
		{
			return state_ == ClickState;
		}

		bool longpressed() const
		{
			return state_ == LongpressState;
		}

		bool polling() const
		{
			return debounce_.pressed() || debounce_.polling();
		}

		libcpp::function< void () > onClicked;
		libcpp::function< void () > onLongpressed;

	private:
		Stopwatch const& stopwatch_;
		Debounce< Pin, debounceUs > debounce_;
		State state_;
	};

} // namespace tp

#endif // TEENSY_POI_BUTTON_HPP
