################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(ANGLE_CALCULATION)/angle_cal.c" \
"$(ANGLE_CALCULATION)/asin.c" \
"$(ANGLE_CALCULATION)/sqrt16.c" \

C_SRCS += \
$(ANGLE_CALCULATION)/angle_cal.c \
$(ANGLE_CALCULATION)/asin.c \
$(ANGLE_CALCULATION)/sqrt16.c \

OBJS += \
./project/angle_calculation/angle_cal.o \
./project/angle_calculation/asin.o \
./project/angle_calculation/sqrt16.o \

C_DEPS += \
./project/angle_calculation/angle_cal.d \
./project/angle_calculation/asin.d \
./project/angle_calculation/sqrt16.d \

OBJS_QUOTED += \
"./project/angle_calculation/angle_cal.o" \
"./project/angle_calculation/asin.o" \
"./project/angle_calculation/sqrt16.o" \

C_DEPS_QUOTED += \
"./project/angle_calculation/angle_cal.d" \
"./project/angle_calculation/asin.d" \
"./project/angle_calculation/sqrt16.d" \

OBJS_OS_FORMAT += \
./project/angle_calculation/angle_cal.o \
./project/angle_calculation/asin.o \
./project/angle_calculation/sqrt16.o \


# Each subdirectory must supply rules for building sources it contributes
project/angle_calculation/angle_cal.o: $(ANGLE_CALCULATION)/angle_cal.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/angle_calculation/angle_cal.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/angle_calculation/angle_cal.o"
	@echo 'Finished building: $<'
	@echo ' '

project/angle_calculation/asin.o: $(ANGLE_CALCULATION)/asin.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/angle_calculation/asin.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/angle_calculation/asin.o"
	@echo 'Finished building: $<'
	@echo ' '

project/angle_calculation/sqrt16.o: $(ANGLE_CALCULATION)/sqrt16.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/angle_calculation/sqrt16.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/angle_calculation/sqrt16.o"
	@echo 'Finished building: $<'
	@echo ' '


