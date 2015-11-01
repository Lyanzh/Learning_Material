################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(RTC)/rtc.c" \

C_SRCS += \
$(RTC)/rtc.c \

OBJS += \
./project/rtc/rtc.o \

C_DEPS += \
./project/rtc/rtc.d \

OBJS_QUOTED += \
"./project/rtc/rtc.o" \

C_DEPS_QUOTED += \
"./project/rtc/rtc.d" \

OBJS_OS_FORMAT += \
./project/rtc/rtc.o \


# Each subdirectory must supply rules for building sources it contributes
project/rtc/rtc.o: $(RTC)/rtc.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/rtc/rtc.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/rtc/rtc.o"
	@echo 'Finished building: $<'
	@echo ' '


