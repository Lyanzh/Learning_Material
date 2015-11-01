################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(ECOMPASS)/sensorCompass.c" \

C_SRCS += \
$(ECOMPASS)/sensorCompass.c \

OBJS += \
./project/ecompass/sensorCompass.o \

C_DEPS += \
./project/ecompass/sensorCompass.d \

OBJS_QUOTED += \
"./project/ecompass/sensorCompass.o" \

C_DEPS_QUOTED += \
"./project/ecompass/sensorCompass.d" \

OBJS_OS_FORMAT += \
./project/ecompass/sensorCompass.o \


# Each subdirectory must supply rules for building sources it contributes
project/ecompass/sensorCompass.o: $(ECOMPASS)/sensorCompass.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/ecompass/sensorCompass.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/ecompass/sensorCompass.o"
	@echo 'Finished building: $<'
	@echo ' '


