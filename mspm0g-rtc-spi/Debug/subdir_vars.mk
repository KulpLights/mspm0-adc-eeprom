################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../mspm0g3507.cmd 

SYSCFG_SRCS += \
../mspm0-linux-spi-rtc.syscfg 

C_SRCS += \
../mspm0-linux-spi-rtc.c \
./syscfg/ti_msp_dl_config.c \
../rtc.c \
../spi_target.c 

GEN_FILES += \
./syscfg/ti_msp_dl_config.c 

GEN_MISC_DIRS += \
./syscfg/ 

C_DEPS += \
./mspm0-linux-spi-rtc.d \
./syscfg/ti_msp_dl_config.d \
./rtc.d \
./spi_target.d 

OBJS += \
./mspm0-linux-spi-rtc.o \
./syscfg/ti_msp_dl_config.o \
./rtc.o \
./spi_target.o 

GEN_MISC_FILES += \
./syscfg/ti_msp_dl_config.h \
./syscfg/Event.dot 

GEN_MISC_DIRS__QUOTED += \
"syscfg\" 

OBJS__QUOTED += \
"mspm0-linux-spi-rtc.o" \
"syscfg\ti_msp_dl_config.o" \
"rtc.o" \
"spi_target.o" 

GEN_MISC_FILES__QUOTED += \
"syscfg\ti_msp_dl_config.h" \
"syscfg\Event.dot" 

C_DEPS__QUOTED += \
"mspm0-linux-spi-rtc.d" \
"syscfg\ti_msp_dl_config.d" \
"rtc.d" \
"spi_target.d" 

GEN_FILES__QUOTED += \
"syscfg\ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../mspm0-linux-spi-rtc.c" \
"./syscfg/ti_msp_dl_config.c" \
"../rtc.c" \
"../spi_target.c" 

SYSCFG_SRCS__QUOTED += \
"../mspm0-linux-spi-rtc.syscfg" 


