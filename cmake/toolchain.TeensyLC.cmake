set(CMAKE_SYSTEM_NAME TeensyLC)
set(CMAKE_SYSTEM_VERSION 1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

set(TEENSY_TOOLS_PATH "${ARDUINO_HOME}/hardware/tools")
set(TEENSY_COMPILER_PATH "${TEENSY_TOOLS_PATH}/arm/bin")

set(TEENSY_C_COMPILER "${TEENSY_COMPILER_PATH}/arm-none-eabi-gcc.exe")
set(TEENSY_CXX_COMPILER "${TEENSY_COMPILER_PATH}/arm-none-eabi-g++.exe")
set(TEENSY_OBJCOPY "${TEENSY_COMPILER_PATH}/arm-none-eabi-objcopy.exe")
set(TEENSY_SIZE "${TEENSY_COMPILER_PATH}/arm-none-eabi-size.exe")

set(CMAKE_C_COMPILER "${TEENSY_C_COMPILER}")
set(CMAKE_CXX_COMPILER "${TEENSY_CXX_COMPILER}")

set(COMMON_FLAGS "-Os -Wall -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD -mcpu=cortex-m0plus -D__MKL26Z64__ -DF_CPU=48000000 -DSERIAL_NUMBER={'9','0','1','0','0','0','0','0','0','1'} -DUSB_SERIAL -DLAYOUT_US_ENGLISH -DNO_RAM_INTERRUPT_VECTOR -DUSING_MAKEFILE")

set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=c11" CACHE STRING "C compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11 -felide-constructors -fno-exceptions -fno-rtti" CACHE STRING "C++ compiler flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-Os -Wl,--gc-sections,--relax -mthumb --specs=nano.specs -mcpu=cortex-m0plus -L${CMAKE_CURRENT_SOURCE_DIR}/core -Tmkl26z64.ld" CACHE STRING "Linker flags" FORCE)

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> -o <TARGET>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -o <TARGET> <OBJECTS> <LINK_LIBRARIES>")
