################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../mspm0g3507.cmd 

SYSCFG_SRCS += \
../mspm0-linux-rtc.syscfg 

C_SRCS += \
../i2c_target.c \
../mspm0-linux-rtc.c \
./syscfg/ti_msp_dl_config.c \
../rtc.c 

GEN_FILES += \
./syscfg/ti_msp_dl_config.c 

GEN_MISC_DIRS += \
./syscfg/ 

C_DEPS += \
./i2c_target.d \
./mspm0-linux-rtc.d \
./syscfg/ti_msp_dl_config.d \
./rtc.d 

OBJS += \
./i2c_target.o \
./mspm0-linux-rtc.o \
./syscfg/ti_msp_dl_config.o \
./rtc.o 

GEN_MISC_FILES += \
./syscfg/ti_msp_dl_config.h \
./syscfg/Event.dot 

GEN_MISC_DIRS__QUOTED += \
"syscfg\" 

OBJS__QUOTED += \
"i2c_target.o" \
"mspm0-linux-rtc.o" \
"syscfg\ti_msp_dl_config.o" \
"rtc.o" 

GEN_MISC_FILES__QUOTED += \
"syscfg\ti_msp_dl_config.h" \
"syscfg\Event.dot" 

C_DEPS__QUOTED += \
"i2c_target.d" \
"mspm0-linux-rtc.d" \
"syscfg\ti_msp_dl_config.d" \
"rtc.d" 

GEN_FILES__QUOTED += \
"syscfg\ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../i2c_target.c" \
"../mspm0-linux-rtc.c" \
"./syscfg/ti_msp_dl_config.c" \
"../rtc.c" 

SYSCFG_SRCS__QUOTED += \
"../mspm0-linux-rtc.syscfg" 


