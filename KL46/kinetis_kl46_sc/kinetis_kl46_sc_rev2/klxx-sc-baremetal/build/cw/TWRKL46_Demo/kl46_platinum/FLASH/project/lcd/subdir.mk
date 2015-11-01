################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(LCD)/GD-6363P.c" \
"$(LCD)/LcdDrv.c" \

C_SRCS += \
$(LCD)/GD-6363P.c \
$(LCD)/LcdDrv.c \

OBJS += \
./project/lcd/GD-6363P.o \
./project/lcd/LcdDrv.o \

C_DEPS += \
./project/lcd/GD-6363P.d \
./project/lcd/LcdDrv.d \

OBJS_QUOTED += \
"./project/lcd/GD-6363P.o" \
"./project/lcd/LcdDrv.o" \

C_DEPS_QUOTED += \
"./project/lcd/GD-6363P.d" \
"./project/lcd/LcdDrv.d" \

OBJS_OS_FORMAT += \
./project/lcd/GD-6363P.o \
./project/lcd/LcdDrv.o \


# Each subdirectory must supply rules for building sources it contributes
project/lcd/GD-6363P.o: $(LCD)/GD-6363P.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/lcd/GD-6363P.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/lcd/GD-6363P.o"
	@echo 'Finished building: $<'
	@echo ' '

project/lcd/LcdDrv.o: $(LCD)/LcdDrv.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/lcd/LcdDrv.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/lcd/LcdDrv.o"
	@echo 'Finished building: $<'
	@echo ' '


