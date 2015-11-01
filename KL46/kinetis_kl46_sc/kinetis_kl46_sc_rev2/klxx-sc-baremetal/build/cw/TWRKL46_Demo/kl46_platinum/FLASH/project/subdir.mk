################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(TWRKL46_DEMO)/GPIO.c" \
"$(TWRKL46_DEMO)/TWRKL46_Demo.c" \
"$(TWRKL46_DEMO)/global.c" \

C_SRCS += \
$(TWRKL46_DEMO)/GPIO.c \
$(TWRKL46_DEMO)/TWRKL46_Demo.c \
$(TWRKL46_DEMO)/global.c \

OBJS += \
./project/GPIO.o \
./project/TWRKL46_Demo.o \
./project/global.o \

C_DEPS += \
./project/GPIO.d \
./project/TWRKL46_Demo.d \
./project/global.d \

OBJS_QUOTED += \
"./project/GPIO.o" \
"./project/TWRKL46_Demo.o" \
"./project/global.o" \

C_DEPS_QUOTED += \
"./project/GPIO.d" \
"./project/TWRKL46_Demo.d" \
"./project/global.d" \

OBJS_OS_FORMAT += \
./project/GPIO.o \
./project/TWRKL46_Demo.o \
./project/global.o \


# Each subdirectory must supply rules for building sources it contributes
project/GPIO.o: $(TWRKL46_DEMO)/GPIO.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/GPIO.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/GPIO.o"
	@echo 'Finished building: $<'
	@echo ' '

project/TWRKL46_Demo.o: $(TWRKL46_DEMO)/TWRKL46_Demo.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/TWRKL46_Demo.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/TWRKL46_Demo.o"
	@echo 'Finished building: $<'
	@echo ' '

project/global.o: $(TWRKL46_DEMO)/global.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/global.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/global.o"
	@echo 'Finished building: $<'
	@echo ' '


