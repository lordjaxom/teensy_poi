cmake_minimum_required(VERSION 3.5)
project(TeensyPoi C CXX)

include_directories(core)
include_directories(libs/Analog)
include_directories(libs/DMAChannel)
include_directories(libs/DmaSpi)
include_directories(libs/I2C)
include_directories(libs/I2Cdev)
include_directories(libs/MPU6050)
include_directories(libs/Snooze)
include_directories(libs/SPI)
include_directories(libs/SPIFlash)

add_definitions(-DLIBCPP_NO_EXCEPTIONS)

link_libraries(m arm_cortexM0l_math)

add_executable(teensy_poi
        core/libcpp/function.hpp
        core/libcpp/modulo_counter.hpp
        core/libcpp/numeric.hpp
        core/libcpp/polymorphic_ptr.hpp
        core/libcpp/sample_buffer.hpp
        core/libcpp/type_traits.hpp
        core/libcpp/value_table.hpp
        core/libcpp/variant.hpp
        core/tlibcpp/debounce.hpp
        core/tlibcpp/null_pin.hpp
        core/tlibcpp/nvic.hpp
        core/tlibcpp/periodic.hpp
        core/tlibcpp/pin.hpp
        core/tlibcpp/pingroup.hpp
        core/tlibcpp/ports.hpp
        core/tlibcpp/teensy_pins.hpp
        core/util/atomic.h
        core/nonstd.c
        core/teensy.c
        core/teensy.h
        core/tstdlib.c
        core/tstdlib.h
        core/usb_desc.c
        core/usb_desc.h
        core/usb_dev.c
        core/usb_dev.h
        core/usb_mem.c
        core/usb_mem.h
        core/usb_serial.c
        core/usb_serial.h
        libs/Analog/Analog.h
        libs/DMAChannel/DMAChannel.cpp
        libs/DMAChannel/DMAChannel.h
        libs/DmaSpi/DmaSpi.h
        libs/I2C/I2C.cpp
        libs/I2C/I2C.h
        libs/I2Cdev/I2Cdev.h
        libs/MPU6050/helper_3dmath.h
        libs/MPU6050/MPU6050.h
        libs/MPU6050/MPU6050_dmp41.cpp
        libs/MPU6050/MPU6050_dmp41_impl.h
        libs/MPU6050/MPU6050_impl.h
        libs/Snooze/bitband.h
        libs/Snooze/cmp.h
        libs/Snooze/cpu.h
        libs/Snooze/digital.h
        libs/Snooze/kinetis_ext.h
        libs/Snooze/llwu.h
        libs/Snooze/lptmr.h
        libs/Snooze/lvd.h
        libs/Snooze/mcg.c
        libs/Snooze/mcg.h
        libs/Snooze/peripheral.h
        libs/Snooze/rtc.h
        libs/Snooze/smc.h
        libs/Snooze/Snooze.cpp
        libs/Snooze/Snooze.h
        libs/Snooze/tsi.h
        libs/Snooze/util.h
        libs/SPI/SPI.h
        libs/SPIFlash/SPIFlash.h
        src/button.hpp
        src/colorwheel.hpp
        src/config.hpp
        src/debug.hpp
        src/flash.hpp
        src/gamma_table.hpp
        src/io_service.hpp
        src/led_algorithm.hpp
        src/leds.hpp
        src/manager.hpp
        src/maths.hpp
        src/mode_off.hpp
        src/mode_play.hpp
        src/mode_program.hpp
        src/motion.hpp
        src/power.hpp
        src/protocol.hpp
        src/random.c
        src/random.h
        src/selftest.hpp
        src/status.hpp
        src/stopwatch.hpp
        src/teensy_poi.cpp
        src/usb_serial.hpp)

add_custom_command(TARGET "teensy_poi" POST_BUILD
        COMMAND "${TEENSY_SIZE}" "$<TARGET_FILE:teensy_poi>")
add_custom_command(TARGET "teensy_poi" POST_BUILD
        COMMAND "${TEENSY_OBJCOPY}" -O ihex -R .eeprom "$<TARGET_FILE:teensy_poi>" "$<TARGET_FILE:teensy_poi>.hex")
