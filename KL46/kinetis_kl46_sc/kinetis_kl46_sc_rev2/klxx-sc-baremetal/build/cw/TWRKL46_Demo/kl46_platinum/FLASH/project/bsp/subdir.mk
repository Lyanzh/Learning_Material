################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(BSP)/bsp_KL25.c" \

C_SRCS += \
$(BSP)/bsp_KL25.c \

OBJS += \
./project/bsp/bsp_KL25.o \

C_DEPS += \
./project/bsp/bsp_KL25.d \

OBJS_QUOTED += \
"./project/bsp/bsp_KL25.o" \

C_DEPS_QUOTED += \
"./project/bsp/bsp_KL25.d" \

OBJS_OS_FORMAT += \
./project/bsp/bsp_KL25.o \


# Each subdirectory must supply rules for building sources it contributes
project/bsp/bsp_KL25.o: $(BSP)/bsp_KL25.c
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/bsp/bsp_KL25.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/bsp/bsp_KL25.o"
	@echo 'Finished building: $<'
	@echo ' '


