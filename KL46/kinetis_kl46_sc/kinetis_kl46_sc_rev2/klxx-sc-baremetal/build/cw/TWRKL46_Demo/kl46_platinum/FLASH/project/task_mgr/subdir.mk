################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(TASK_MGR)/task_mgr.c" \

C_SRCS += \
$(TASK_MGR)/task_mgr.c \

OBJS += \
./project/task_mgr/task_mgr.o \

C_DEPS += \
./project/task_mgr/task_mgr.d \

OBJS_QUOTED += \
"./project/task_mgr/task_mgr.o" \

C_DEPS_QUOTED += \
"./project/task_mgr/task_mgr.d" \

OBJS_OS_FORMAT += \
./project/task_mgr/task_mgr.o \


# Each subdirectory must supply rules for building sources it contributes
project/task_mgr/task_mgr.o: $(TASK_MGR)/task_mgr.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"project/task_mgr/task_mgr.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"project/task_mgr/task_mgr.o"
	@echo 'Finished building: $<'
	@echo ' '


