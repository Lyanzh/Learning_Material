################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(FRDMKL46_DEMO)/FRDMKL46_Demo.c" \
"$(FRDMKL46_DEMO)/GPIO.c" \
"$(FRDMKL46_DEMO)/global.c" \

C_SRCS += \
$(FRDMKL46_DEMO)/FRDMKL46_Demo.c \
$(FRDMKL46_DEMO)/GPIO.c \
$(FRDMKL46_DEMO)/global.c \

OBJS += \
./project/FRDMKL46_Demo.o \
./project/GPIO.o \
./project/global.o \

C_DEPS += \
./project/FRDMKL46_Demo.d \
./project/GPIO.d \
./project/global.d \

OBJS_QUOTED += \
"./project/FRDMKL46_Demo.o" \
"./project/GPIO.o" \
"./project/global.o" \

C_DEPS_QUOTED += \
"./project/FRDMKL46_Demo.d" \
"./project/GPIO.d" \
"./project/global.d" \

OBJS_OS_FORMAT += \
./project/FRDMKL46_Demo.o \
./project/GPIO.o \
./project/global.o \


# Each subdirectory must supply rules for building sources it contributes
project/FRDMKL46_Demo.o: $(FRDMKL46_DEMO)/FRDMKL46_Demo.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/FRDMKL46_Demo.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/FRDMKL46_Demo.o"
	@echo 'Finished building: $<'
	@echo ' '

project/GPIO.o: $(FRDMKL46_DEMO)/GPIO.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/GPIO.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/GPIO.o"
	@echo 'Finished building: $<'
	@echo ' '

project/global.o: $(FRDMKL46_DEMO)/global.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/global.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/global.o"
	@echo 'Finished building: $<'
	@echo ' '


