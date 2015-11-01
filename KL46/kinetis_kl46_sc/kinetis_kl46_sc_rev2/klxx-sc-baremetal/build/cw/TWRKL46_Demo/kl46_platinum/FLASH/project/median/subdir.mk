################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(MEDIAN)/median.c" \

C_SRCS += \
$(MEDIAN)/median.c \

OBJS += \
./project/median/median.o \

C_DEPS += \
./project/median/median.d \

OBJS_QUOTED += \
"./project/median/median.o" \

C_DEPS_QUOTED += \
"./project/median/median.d" \

OBJS_OS_FORMAT += \
./project/median/median.o \


# Each subdirectory must supply rules for building sources it contributes
project/median/median.o: $(MEDIAN)/median.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/median/median.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/median/median.o"
	@echo 'Finished building: $<'
	@echo ' '


