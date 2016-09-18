#ifndef TEENSY_POI_CONFIG_HPP
#define TEENSY_POI_CONFIG_HPP

#include <stddef.h>

#define REVISION 2

#if REVISION == 1

static constexpr size_t pixelCount = 72;

static constexpr size_t ledPowerPin = 16;
static constexpr size_t ledDataPin = 21;
static constexpr size_t ledClockPin = 20;

static constexpr size_t chargePin = 0;
static constexpr size_t voltagePin = 9;
static constexpr size_t buttonPin = 6;

static constexpr size_t flashSSPin = 9;
static constexpr size_t flashMISOPin = 8;
static constexpr size_t flashMOSIPin = 7;
static constexpr size_t flashSCKPin = 14;

static constexpr size_t i2cSdaPin = 18;
static constexpr size_t i2cSclPin = 19;
static constexpr bool i2cPullup = true;

#elif REVISION == 2

static constexpr size_t pixelCount = 60;

static constexpr size_t ledPowerPin = 10;
static constexpr size_t ledDataPin = 11;
static constexpr size_t ledClockPin = 13;

static constexpr size_t chargePin = 2;
static constexpr size_t voltagePin = 9;
static constexpr size_t buttonPin = 22;

static constexpr size_t flashSSPin = 6;
static constexpr size_t flashMISOPin = 5;
static constexpr size_t flashMOSIPin = 21;
static constexpr size_t flashSCKPin = 20;

static constexpr size_t i2cPowerPin = 19;
static constexpr size_t i2cSdaPin = 17;
static constexpr size_t i2cSclPin = 16;
static constexpr bool i2cPullup = true;

#else // REVISION
#   error Unknown REVISION
#endif

static constexpr size_t ledCount = pixelCount * 2;

static constexpr size_t debounceUs = 50000;
static constexpr size_t longpressUs = 1500000;

#endif // TEENSY_POI_CONFIG_HPP
