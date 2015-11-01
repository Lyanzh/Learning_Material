################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(LCD)/LCD.c" \
"$(LCD)/s401.c" \

C_SRCS += \
$(LCD)/LCD.c \
$(LCD)/s401.c \

OBJS += \
./project/lcd/LCD.o \
./project/lcd/s401.o \

C_DEPS += \
./project/lcd/LCD.d \
./project/lcd/s401.d \

OBJS_QUOTED += \
"./project/lcd/LCD.o" \
"./project/lcd/s401.o" \

C_DEPS_QUOTED += \
"./project/lcd/LCD.d" \
"./project/lcd/s401.d" \

OBJS_OS_FORMAT += \
./project/lcd/LCD.o \
./project/lcd/s401.o \


# Each subdirectory must supply rules for building sources it contributes
project/lcd/LCD.o: $(LCD)/LCD.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/lcd/LCD.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/lcd/LCD.o"
	@echo 'Finished building: $<'
	@echo ' '

project/lcd/s401.o: $(LCD)/s401.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/lcd/s401.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/lcd/s401.o"
	@echo 'Finished building: $<'
	@echo ' '


