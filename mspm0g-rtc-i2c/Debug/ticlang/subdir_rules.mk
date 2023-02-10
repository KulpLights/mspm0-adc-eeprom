################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ticlang/%.o: ../ticlang/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1210/ccs/tools/compiler/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc" -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/Debug" -I"C:/ti/mspm0_sdk_0_52_00_03_eng/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_0_52_00_03_eng/source" -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"ticlang/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0497614/Documents/BU Rotation/MSPM0 x AM62/mspm0-linux-rtc/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


