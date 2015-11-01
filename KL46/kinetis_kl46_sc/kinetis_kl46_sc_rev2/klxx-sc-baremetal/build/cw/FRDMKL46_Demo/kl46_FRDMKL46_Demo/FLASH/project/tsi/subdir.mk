################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(TSI)/TSIDrv.c" \

C_SRCS += \
$(TSI)/TSIDrv.c \

OBJS += \
./project/tsi/TSIDrv.o \

C_DEPS += \
./project/tsi/TSIDrv.d \

OBJS_QUOTED += \
"./project/tsi/TSIDrv.o" \

C_DEPS_QUOTED += \
"./project/tsi/TSIDrv.d" \

OBJS_OS_FORMAT += \
./project/tsi/TSIDrv.o \


# Each subdirectory must supply rules for building sources it contributes
project/tsi/TSIDrv.o: $(TSI)/TSIDrv.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/tsi/TSIDrv.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/tsi/TSIDrv.o"
	@echo 'Finished building: $<'
	@echo ' '


