################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc" -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/Debug" -I"C:/ti/mspm0_sdk_0_52_00_03_eng/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_0_52_00_03_eng/source" -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1610832609: ../mspm0-linux-rtc.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1.15.0/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_0_52_00_03_eng/.metadata/product.json" --script "C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/mspm0-linux-rtc.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_msp_dl_config.c: build-1610832609 ../mspm0-linux-rtc.syscfg
syscfg/ti_msp_dl_config.h: build-1610832609
syscfg/Event.dot: build-1610832609
syscfg/: build-1610832609

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc" -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/Debug" -I"C:/ti/mspm0_sdk_0_52_00_03_eng/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_0_52_00_03_eng/source" -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


