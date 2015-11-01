################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(I2C)/hal_dev_mag3110.c" \
"$(I2C)/hal_dev_mma8451.c" \
"$(I2C)/hal_i2c.c" \

C_SRCS += \
$(I2C)/hal_dev_mag3110.c \
$(I2C)/hal_dev_mma8451.c \
$(I2C)/hal_i2c.c \

OBJS += \
./project/I2C/hal_dev_mag3110.o \
./project/I2C/hal_dev_mma8451.o \
./project/I2C/hal_i2c.o \

C_DEPS += \
./project/I2C/hal_dev_mag3110.d \
./project/I2C/hal_dev_mma8451.d \
./project/I2C/hal_i2c.d \

OBJS_QUOTED += \
"./project/I2C/hal_dev_mag3110.o" \
"./project/I2C/hal_dev_mma8451.o" \
"./project/I2C/hal_i2c.o" \

C_DEPS_QUOTED += \
"./project/I2C/hal_dev_mag3110.d" \
"./project/I2C/hal_dev_mma8451.d" \
"./project/I2C/hal_i2c.d" \

OBJS_OS_FORMAT += \
./project/I2C/hal_dev_mag3110.o \
./project/I2C/hal_dev_mma8451.o \
./project/I2C/hal_i2c.o \


# Each subdirectory must supply rules for building sources it contributes
project/I2C/hal_dev_mag3110.o: $(I2C)/hal_dev_mag3110.c
	@echo 'Building file: $<'
	@echo 'Executing target #21 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/I2C/hal_dev_mag3110.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/I2C/hal_dev_mag3110.o"
	@echo 'Finished building: $<'
	@echo ' '

project/I2C/hal_dev_mma8451.o: $(I2C)/hal_dev_mma8451.c
	@echo 'Building file: $<'
	@echo 'Executing target #22 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/I2C/hal_dev_mma8451.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/I2C/hal_dev_mma8451.o"
	@echo 'Finished building: $<'
	@echo ' '

project/I2C/hal_i2c.o: $(I2C)/hal_i2c.c
	@echo 'Building file: $<'
	@echo 'Executing target #23 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/I2C/hal_i2c.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/I2C/hal_i2c.o"
	@echo 'Finished building: $<'
	@echo ' '


