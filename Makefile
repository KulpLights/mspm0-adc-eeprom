# Processor Companion Utility (PCU) Project Makefile
# Copyright (c) 2024, Texas Instruments Incorporated

include Makefile.defs

SRC_DIR = ./source
BRD_DIR = $(SRC_DIR)/board
MOD_DIR = $(SRC_DIR)/modules
SDK_DIR = $(SRC_DIR)/mspm0-sdk

BRD_INC = -I$(BRD_DIR)
MOD_INC = -I$(MOD_DIR)
SDK_INC = -I$(SDK_DIR) -I$(SDK_DIR)/third_party/CMSIS/Core/Include
INCLUDE_PATHS = $(BRD_INC) $(MOD_INC) $(SDK_INC)
INCLUDE_DEPS = $(shell find $(SRC_DIR) -name '*.h')

BRD_SOURCES = $(shell find $(BRD_DIR) -name '*.c')
MOD_SOURCES = $(shell find $(MOD_DIR) -name '*.c')
SDK_SOURCES = $(SDK_DIR)/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0l110x_ticlang.c
SOURCES = $(BRD_SOURCES) $(MOD_SOURCES) $(SDK_SOURCES)

BRD_OBJS = $(patsubst %.c,%.o, $(notdir $(BRD_SOURCES)))
MOD_OBJS = $(patsubst %.c,%.o, $(notdir $(MOD_SOURCES)))
SDK_OBJS = $(patsubst %.c,%.o, $(notdir $(SDK_SOURCES)))
OBJECTS = $(BRD_OBJS) $(MOD_OBJS) $(SDK_OBJS)

SDK_LIBS = $(SDK_DIR)/ti/driverlib/lib/ticlang/m0p/mspm0l11xx_l13xx/driverlib.a
LIBS = $(SDK_LIBS)

LINKER_CMD_FILE = $(BRD_DIR)/linker.cmd


ADC_EEPROM_NAME = mspm0-adc-eeprom
ADC_EEPROM_OUT_FILE = $(ADC_EEPROM_NAME).out
ADC_EEPROM_TXT_FILE = $(ADC_EEPROM_NAME).txt
ADC_EEPROM_MAP_FILE = $(ADC_EEPROM_NAME).map

ADC_NAME = mspm0-adc
ADC_OUT_FILE = $(ADC_NAME).out
ADC_TXT_FILE = $(ADC_NAME).txt
ADC_MAP_FILE = $(ADC_NAME).map

# Same apps, emulating an AD7998 instead of an AD7291
AD7998_EEPROM_NAME = mspm0-ad7998-eeprom
AD7998_EEPROM_OUT_FILE = $(AD7998_EEPROM_NAME).out
AD7998_EEPROM_TXT_FILE = $(AD7998_EEPROM_NAME).txt
AD7998_EEPROM_MAP_FILE = $(AD7998_EEPROM_NAME).map

AD7998_NAME = mspm0-ad7998
AD7998_OUT_FILE = $(AD7998_NAME).out
AD7998_TXT_FILE = $(AD7998_NAME).txt
AD7998_MAP_FILE = $(AD7998_NAME).map

flasher/flasher : flasher/flasher.cpp
	ccache $(CXX) -I/opt/fpp/src -o flasher/flasher flasher/flasher.cpp  -L/opt/fpp/src -Wl,-rpath=/opt/fpp/src:. -lfpp

all : $(ADC_EEPROM_TXT_FILE)  $(ADC_TXT_FILE) $(AD7998_EEPROM_TXT_FILE) $(AD7998_TXT_FILE) flasher/flasher
	@echo " Removing object files..."
	@rm $(shell find . -name '*.o')
	@echo "Done."

$(ADC_EEPROM_TXT_FILE) : $(ADC_EEPROM_OUT_FILE)
	$(HEX) $(HEXFLAGS) -o $(ADC_EEPROM_TXT_FILE) $(ADC_EEPROM_OUT_FILE)

$(ADC_EEPROM_OUT_FILE) : $(OBJECTS) $(LINKER_CMD_FILE) adc-eeprom.o
	$(CC) $(LFLAGS)  -Wl,-m$(ADC_EEPROM_MAP_FILE) -o $(ADC_EEPROM_OUT_FILE) $(OBJECTS) adc-eeprom.o $(LIBS) -Wl,$(LINKER_CMD_FILE)

$(ADC_TXT_FILE) : $(ADC_OUT_FILE)
	$(HEX) $(HEXFLAGS) -o $(ADC_TXT_FILE) $(ADC_OUT_FILE)

$(ADC_OUT_FILE) : $(OBJECTS) $(LINKER_CMD_FILE) adc-only.o
	$(CC) $(LFLAGS)  -Wl,-m$(ADC_MAP_FILE) -o $(ADC_OUT_FILE) $(OBJECTS) adc-only.o $(LIBS) -Wl,$(LINKER_CMD_FILE)

$(AD7998_EEPROM_TXT_FILE) : $(AD7998_EEPROM_OUT_FILE)
	$(HEX) $(HEXFLAGS) -o $(AD7998_EEPROM_TXT_FILE) $(AD7998_EEPROM_OUT_FILE)

$(AD7998_EEPROM_OUT_FILE) : $(OBJECTS) $(LINKER_CMD_FILE) adc7998-eeprom.o
	$(CC) $(LFLAGS)  -Wl,-m$(AD7998_EEPROM_MAP_FILE) -o $(AD7998_EEPROM_OUT_FILE) $(OBJECTS) adc7998-eeprom.o $(LIBS) -Wl,$(LINKER_CMD_FILE)

$(AD7998_TXT_FILE) : $(AD7998_OUT_FILE)
	$(HEX) $(HEXFLAGS) -o $(AD7998_TXT_FILE) $(AD7998_OUT_FILE)

$(AD7998_OUT_FILE) : $(OBJECTS) $(LINKER_CMD_FILE) adc7998-only.o
	$(CC) $(LFLAGS)  -Wl,-m$(AD7998_MAP_FILE) -o $(AD7998_OUT_FILE) $(OBJECTS) adc7998-only.o $(LIBS) -Wl,$(LINKER_CMD_FILE)

$(OBJECTS) : $(SOURCES) $(INCLUDE_DEPS)
	$(CC) -c $(CFLAGS) $(INCLUDE_PATHS) $(SOURCES)

adc-only.o : source/app/adc-only.c $(INCLUDE_DEPS)
	$(CC) -c $(CFLAGS) $(INCLUDE_PATHS) source/app/adc-only.c
	
adc-eeprom.o : source/app/adc-eeprom.c $(INCLUDE_DEPS)
	$(CC) -c $(CFLAGS) $(INCLUDE_PATHS) source/app/adc-eeprom.c

adc7998-only.o : source/app/adc-only.c $(INCLUDE_DEPS)
	$(CC) -c $(CFLAGS) -DADC_AD7998 $(INCLUDE_PATHS) source/app/adc-only.c -o adc7998-only.o

adc7998-eeprom.o : source/app/adc-eeprom.c $(INCLUDE_DEPS)
	$(CC) -c $(CFLAGS) -DADC_AD7998 $(INCLUDE_PATHS) source/app/adc-eeprom.c -o adc7998-eeprom.o

.PHONY clean :
	@echo " Cleaning..."
	@rm -f flasher/flasher
	@rm -f $(ADC_OUT_FILE) $(ADC_MAP_FILE) $(ADC_TXT_FILE)
	@rm -f $(ADC_EEPROM_OUT_FILE) $(ADC_EEPROM_MAP_FILE) $(ADC_EEPROM_TXT_FILE)
	@rm -f $(AD7998_OUT_FILE) $(AD7998_MAP_FILE) $(AD7998_TXT_FILE)
	@rm -f $(AD7998_EEPROM_OUT_FILE) $(AD7998_EEPROM_MAP_FILE) $(AD7998_EEPROM_TXT_FILE)
	@rm -f $(shell find . -name '*.o')
	@echo "Done."
