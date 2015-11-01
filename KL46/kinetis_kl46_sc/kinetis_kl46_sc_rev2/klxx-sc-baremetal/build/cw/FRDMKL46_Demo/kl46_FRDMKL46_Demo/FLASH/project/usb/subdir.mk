################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(USB)/ring_buffer.c" \
"$(USB)/usb.c" \
"$(USB)/usb_cdc.c" \
"$(USB)/usb_reg.c" \

C_SRCS += \
$(USB)/ring_buffer.c \
$(USB)/usb.c \
$(USB)/usb_cdc.c \
$(USB)/usb_reg.c \

OBJS += \
./project/usb/ring_buffer.o \
./project/usb/usb.o \
./project/usb/usb_cdc.o \
./project/usb/usb_reg.o \

C_DEPS += \
./project/usb/ring_buffer.d \
./project/usb/usb.d \
./project/usb/usb_cdc.d \
./project/usb/usb_reg.d \

OBJS_QUOTED += \
"./project/usb/ring_buffer.o" \
"./project/usb/usb.o" \
"./project/usb/usb_cdc.o" \
"./project/usb/usb_reg.o" \

C_DEPS_QUOTED += \
"./project/usb/ring_buffer.d" \
"./project/usb/usb.d" \
"./project/usb/usb_cdc.d" \
"./project/usb/usb_reg.d" \

OBJS_OS_FORMAT += \
./project/usb/ring_buffer.o \
./project/usb/usb.o \
./project/usb/usb_cdc.o \
./project/usb/usb_reg.o \


# Each subdirectory must supply rules for building sources it contributes
project/usb/ring_buffer.o: $(USB)/ring_buffer.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/usb/ring_buffer.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/usb/ring_buffer.o"
	@echo 'Finished building: $<'
	@echo ' '

project/usb/usb.o: $(USB)/usb.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/usb/usb.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/usb/usb.o"
	@echo 'Finished building: $<'
	@echo ' '

project/usb/usb_cdc.o: $(USB)/usb_cdc.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/usb/usb_cdc.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/usb/usb_cdc.o"
	@echo 'Finished building: $<'
	@echo ' '

project/usb/usb_reg.o: $(USB)/usb_reg.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/usb/usb_reg.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/usb/usb_reg.o"
	@echo 'Finished building: $<'
	@echo ' '


