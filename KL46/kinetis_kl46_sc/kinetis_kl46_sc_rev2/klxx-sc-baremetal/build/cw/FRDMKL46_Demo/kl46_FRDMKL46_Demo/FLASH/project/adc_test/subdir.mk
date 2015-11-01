################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(ADC_TEST)/adc_test.c" \

C_SRCS += \
$(ADC_TEST)/adc_test.c \

OBJS += \
./project/adc_test/adc_test.o \

C_DEPS += \
./project/adc_test/adc_test.d \

OBJS_QUOTED += \
"./project/adc_test/adc_test.o" \

C_DEPS_QUOTED += \
"./project/adc_test/adc_test.d" \

OBJS_OS_FORMAT += \
./project/adc_test/adc_test.o \


# Each subdirectory must supply rules for building sources it contributes
project/adc_test/adc_test.o: $(ADC_TEST)/adc_test.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/adc_test/adc_test.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/adc_test/adc_test.o"
	@echo 'Finished building: $<'
	@echo ' '


