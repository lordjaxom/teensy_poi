#include <tlibcpp/teensy_pins.hpp>

#include "debug.hpp"

#include "config.hpp"

#include "button.hpp"
#include "flash.hpp"
#include "leds.hpp"
#include "manager.hpp"
#include "mode_off.hpp"
#include "mode_play.hpp"
#include "mode_program.hpp"
#include "motion.hpp"
#include "selftest.hpp"
#include "status.hpp"

using Status = tp::Status< tlibcpp::digital_pin< chargePin >, tlibcpp::analog_pin< voltagePin > >;
using Button = tp::Button< tlibcpp::digital_pin< buttonPin >, longpressUs, debounceUs >;
using Leds = tp::LedsDma< ledCount, tlibcpp::digital_pin< ledDataPin >, tlibcpp::digital_pin< ledClockPin >, tlibcpp::digital_pin< ledPowerPin >, 8000000 >;
using Flash = tp::Flash< tlibcpp::digital_pin< flashMOSIPin >, tlibcpp::digital_pin< flashMISOPin >, tlibcpp::digital_pin< flashSCKPin >, tlibcpp::digital_pin< flashSSPin > >;
using Motion = tp::Motion< tlibcpp::digital_pin< i2cSdaPin >, tlibcpp::digital_pin< i2cSclPin >, i2cPullup, tlibcpp::digital_pin< i2cPowerPin > >;
using Manager = tp::Manager< Status, Button, Leds, Flash, Motion, tp::ModeOff, tp::ModePlay, tp::ModeProgram >;

static Manager manager;

int main()
{
	selftest( manager );

	while ( true ) {
		manager.advance();
	}
}

