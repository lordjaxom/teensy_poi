#ifndef TEENSY_POI_MODE_OFF_HPP
#define TEENSY_POI_MODE_OFF_HPP

#include <Snooze.h>

namespace tp {

	template< typename Manager >
	class ModeOff
	{
		using SnoozeConfigType =
				SnoozeConfig::PinMode< typename Manager::Button::PinType, INPUT_PULLUP, HIGH >;

	public:
		explicit ModeOff( Manager const& manager )
			: manager_( manager )
		{
		}

		ModeOff( ModeOff const& ) = delete;

		void buttonClicked()
		{
		}

		void advance()
		{
			if ( !manager_.button().polling() ) {
				snooze_.hibernate< SnoozeConfigType >();
			}
		}

	private:
		Manager const& manager_;
		Snooze snooze_;
	};

} // namespace tp

#endif // TEENSY_POI_MODE_OFF_HPP
