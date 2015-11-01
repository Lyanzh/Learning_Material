################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(PIT)/pit.c" \

C_SRCS += \
$(PIT)/pit.c \

OBJS += \
./project/pit/pit.o \

C_DEPS += \
./project/pit/pit.d \

OBJS_QUOTED += \
"./project/pit/pit.o" \

C_DEPS_QUOTED += \
"./project/pit/pit.d" \

OBJS_OS_FORMAT += \
./project/pit/pit.o \


# Each subdirectory must supply rules for building sources it contributes
project/pit/pit.o: $(PIT)/pit.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/pit/pit.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/pit/pit.o"
	@echo 'Finished building: $<'
	@echo ' '


