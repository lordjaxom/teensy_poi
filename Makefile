# The name of your project (used to name the compiled .hex file)
TARGET = teensy_poi

# The teensy version to use, 30, 31, or LC
TEENSY = LC

# Set to 24000000, 48000000, or 96000000 to set CPU core speed
TEENSY_CORE_SPEED = 48000000

# Some libraries will require this to be defined
# If you define this, you will break the default main.cpp
#ARDUINO = 10600

# configurable options
OPTIONS = -DSERIAL_NUMBER="{'9','0','1','0','0','0','0','0','0','1'}" -DUSB_SERIAL -DLAYOUT_US_ENGLISH -DNO_RAM_INTERRUPT_VECTOR -DLIBCPP_NO_EXCEPTIONS

# directory to build in
BUILDDIR = build

#************************************************************************
# Location of Teensyduino utilities, Toolchain, and Arduino Libraries.
# To use this makefile without Arduino, copy the resources from these
# locations and edit the pathnames.  The rest of Arduino is not needed.
#************************************************************************

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH    ?= $(ARDUINO_HOME)/hardware/tools

# path location for the arm-none-eabi compiler
COMPILERPATH ?= $(TOOLSPATH)/arm/bin

ifeq ($(OS),Windows_NT)
	RM-RF = busybox rm -rf
	MKDIR = busybox mkdir -p
else
	RM-RF = rm -rf
	MKDIR = mkdir -p
endif

# path location for Teensy 3 core
COREPATH = core

# path location for Arduino libraries
LIBSPATH = libs

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Os -Wall -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD $(OPTIONS) -DF_CPU=$(TEENSY_CORE_SPEED) -Isrc -I$(COREPATH) -IC:\Users\lordjaxom\Programme\MinGW\include

# compiler options for C++ only
CXXFLAGS = -std=c++11 -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS = -std=c11

# linker options
LDFLAGS = -Os -Wl,--gc-sections,--relax -mthumb --specs=nano.specs -Wl,-Map=$(BUILDDIR)/$(TARGET).map

# additional libraries to link
LIBS = -lm

# compiler options specific to teensy version
ifeq ($(TEENSY), 30)
    CPPFLAGS += -D__MK20DX128__ -mcpu=cortex-m4
    LDSCRIPT = $(COREPATH)/mk20dx128.ld
    LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
else
    ifeq ($(TEENSY), 31)
        CPPFLAGS += -D__MK20DX256__ -mcpu=cortex-m4
        LDSCRIPT = $(COREPATH)/mk20dx256.ld
        LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
    else
        ifeq ($(TEENSY), LC)
            CPPFLAGS += -D__MKL26Z64__ -mcpu=cortex-m0plus
            LDSCRIPT = $(COREPATH)/mkl26z64.ld
            LDFLAGS += -mcpu=cortex-m0plus -T$(LDSCRIPT)
            LIBS += -larm_cortexM0l_math
        else
            $(error Invalid setting for TEENSY)
        endif
    endif
endif

# set arduino define if given
ifdef ARDUINO
	CPPFLAGS += -DARDUINO=$(ARDUINO)
else
	CPPFLAGS += -DUSING_MAKEFILE
endif

# names for the compiler programs
CC = $(COMPILERPATH)/arm-none-eabi-gcc
CXX = $(COMPILERPATH)/arm-none-eabi-g++
OBJCOPY = $(COMPILERPATH)/arm-none-eabi-objcopy
SIZE = $(COMPILERPATH)/arm-none-eabi-size

# include paths for libraries
L_INC := $(foreach lib,$(filter %/, $(wildcard $(LIBSPATH)/*/)), -I$(lib))

PROJ_SOURCES := $(wildcard src/*.cpp) $(wildcard src/*.c)
CORE_SOURCES := $(wildcard $(COREPATH)/*.c) $(wildcard $(COREPATH)/*.cpp)
LIBS_CSOURCES := $(wildcard $(LIBSPATH)/*/*.c)
LIBS_COBJECTS := $(foreach src,$(LIBS_CSOURCES),$(BUILDDIR)/libs/$(notdir $(patsubst %/,%,$(dir $(src))))/$(addsuffix .o,$(notdir $(src))))
LIBS_CXXSOURCES := $(wildcard $(LIBSPATH)/*/*.cpp)
LIBS_CXXOBJECTS := $(foreach src,$(LIBS_CXXSOURCES),$(BUILDDIR)/libs/$(notdir $(patsubst %/,%,$(dir $(src))))/$(addsuffix .o,$(notdir $(src))))

OBJECTS := $(foreach src,$(PROJ_SOURCES),$(BUILDDIR)/src/$(addsuffix .o,$(notdir $(src)))) \
           $(foreach src,$(CORE_SOURCES),$(BUILDDIR)/core/$(addsuffix .o,$(notdir $(src)))) \
           $(LIBS_CXXOBJECTS) $(LIBS_COBJECTS)

.SECONDEXPANSION:

all: hex

build: $(TARGET).elf

hex: $(TARGET).hex

post_compile: $(TARGET).hex
	@$(TOOLSPATH)/teensy_post_compile -file="$(basename $<)" -path=$(CURDIR) -tools="$(TOOLSPATH)"

reboot:
	@-$(TOOLSPATH)/teensy_reboot

upload: post_compile reboot

$(BUILDDIR)/src/%.c.o: src/%.c
	@echo "[CC]    $<"
	@$(MKDIR) "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/src/%.cpp.o: src/%.cpp
	@echo "[CXX]   $<"
	@$(MKDIR) "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/core/%.c.o: $(COREPATH)/%.c
	@echo "[CC]    $<"
	@$(MKDIR) "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/core/%.cpp.o: $(COREPATH)/%.cpp
	@echo "[CXX]   $<"
	@$(MKDIR) "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(LIBS_COBJECTS): $(LIBSPATH)/$$(notdir $$(patsubst %/,%,$$(dir $$@)))/$$(notdir $$(patsubst %.c.o,%.c,$$@))
	@echo "[CC]    $<"
	@$(MKDIR) "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(LIBS_CXXOBJECTS): $(LIBSPATH)/$$(notdir $$(patsubst %/,%,$$(dir $$@)))/$$(notdir $$(patsubst %.cpp.o,%.cpp,$$@))
	@echo "[CXX]   $<"
	@$(MKDIR) "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	@echo "[LD]    $@"
	$(CC) $(LDFLAGS) -o "$@" $(OBJECTS) $(LIBS)

%.hex: %.elf
	@echo "[HEX]   $@"
	@$(SIZE) "$<"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

# compiler generated dependency info
-include $(OBJECTS:.o=.d)

clean:
	@echo Cleaning...
	@$(RM-RF) "$(BUILDDIR)"
	@$(RM-RF) "$(TARGET).elf" "$(TARGET).hex"
