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
OUT_FILE = $(PRJ_NAME).out
TXT_FILE = $(PRJ_NAME).txt
MAP_FILE = $(PRJ_NAME).map

flasher/flasher : flasher/flasher.cpp
	ccache $(CXX) -I/opt/fpp/src -o flasher/flasher flasher/flasher.cpp  -L/opt/fpp/src -Wl,-rpath=/opt/fpp/src:. -lfpp

all : $(TXT_FILE) flasher/flasher
	@echo " TI-TXT image at " $(TXT_FILE)
	@echo " Out file at " $(OUT_FILE)
	@echo " Linker map file at " $(MAP_FILE)
	@echo " Removing object files..."
	@rm $(shell find . -name '*.o')
	@cp $(TXT_FILE) ../msp-m0-python-flasher/firmware.txt
	@echo "Done."

$(TXT_FILE) : $(OUT_FILE)
	$(HEX) $(HEXFLAGS) -o $(TXT_FILE) $(OUT_FILE)

$(OUT_FILE) : $(OBJECTS)
	$(CC) $(LFLAGS) -o $(OUT_FILE) $(OBJECTS) $(LIBS) -Wl,$(LINKER_CMD_FILE)

$(OBJECTS) : $(SOURCES) $(INCLUDE_DEPS)
	$(CC) -c $(CFLAGS) $(INCLUDE_PATHS) $(SOURCES)

.PHONY clean :
	@echo " Cleaning..."
	@rm -f flasher/flasher
	@rm -f $(OUT_FILE) $(MAP_FILE) $(TXT_FILE)
	@rm -f $(shell find . -name '*.o')
	@echo "Done."
