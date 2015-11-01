################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(ADC16)/adc16.c" \

C_SRCS += \
$(ADC16)/adc16.c \

OBJS += \
./project/adc16/adc16.o \

C_DEPS += \
./project/adc16/adc16.d \

OBJS_QUOTED += \
"./project/adc16/adc16.o" \

C_DEPS_QUOTED += \
"./project/adc16/adc16.d" \

OBJS_OS_FORMAT += \
./project/adc16/adc16.o \


# Each subdirectory must supply rules for building sources it contributes
project/adc16/adc16.o: $(ADC16)/adc16.c
	@echo 'Building file: $<'
	@echo 'Executing target #20 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/adc16/adc16.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/adc16/adc16.o"
	@echo 'Finished building: $<'
	@echo ' '


