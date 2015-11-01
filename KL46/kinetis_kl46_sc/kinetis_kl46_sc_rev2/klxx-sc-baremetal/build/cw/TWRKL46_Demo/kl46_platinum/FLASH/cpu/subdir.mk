################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(CPU)/arm_cm0.c" \
"$(CPU)/nmi_isr_handler.c" \
"$(CPU)/start.c" \
"$(CPU)/sysinit.c" \
"$(CPU)/vectors.c" \

C_SRCS += \
$(CPU)/arm_cm0.c \
$(CPU)/nmi_isr_handler.c \
$(CPU)/start.c \
$(CPU)/sysinit.c \
$(CPU)/vectors.c \

S_SRCS += \
$(CPU)/cw_crt0.s \

S_SRCS_QUOTED += \
"$(CPU)/cw_crt0.s" \

S_DEPS_QUOTED += \
"./cpu/cw_crt0.d" \

OBJS += \
./cpu/arm_cm0.o \
./cpu/cw_crt0.o \
./cpu/nmi_isr_handler.o \
./cpu/start.o \
./cpu/sysinit.o \
./cpu/vectors.o \

C_DEPS += \
./cpu/arm_cm0.d \
./cpu/nmi_isr_handler.d \
./cpu/start.d \
./cpu/sysinit.d \
./cpu/vectors.d \

S_DEPS += \
./cpu/cw_crt0.d \

OBJS_QUOTED += \
"./cpu/arm_cm0.o" \
"./cpu/cw_crt0.o" \
"./cpu/nmi_isr_handler.o" \
"./cpu/start.o" \
"./cpu/sysinit.o" \
"./cpu/vectors.o" \

C_DEPS_QUOTED += \
"./cpu/arm_cm0.d" \
"./cpu/nmi_isr_handler.d" \
"./cpu/start.d" \
"./cpu/sysinit.d" \
"./cpu/vectors.d" \

OBJS_OS_FORMAT += \
./cpu/arm_cm0.o \
./cpu/cw_crt0.o \
./cpu/nmi_isr_handler.o \
./cpu/start.o \
./cpu/sysinit.o \
./cpu/vectors.o \


# Each subdirectory must supply rules for building sources it contributes
cpu/arm_cm0.o: $(CPU)/arm_cm0.c
	@echo 'Building file: $<'
	@echo 'Executing target #31 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/arm_cm0.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/arm_cm0.o"
	@echo 'Finished building: $<'
	@echo ' '

cpu/cw_crt0.o: $(CPU)/cw_crt0.s
	@echo 'Building file: $<'
	@echo 'Executing target #32 $<'
	@echo 'Invoking: ARM Ltd Windows GCC Assembler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/cw_crt0.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -MT"cpu/cw_crt0.d" -o"cpu/cw_crt0.o"
	@echo 'Finished building: $<'
	@echo ' '

cpu/nmi_isr_handler.o: $(CPU)/nmi_isr_handler.c
	@echo 'Building file: $<'
	@echo 'Executing target #33 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/nmi_isr_handler.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/nmi_isr_handler.o"
	@echo 'Finished building: $<'
	@echo ' '

cpu/start.o: $(CPU)/start.c
	@echo 'Building file: $<'
	@echo 'Executing target #34 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/start.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/start.o"
	@echo 'Finished building: $<'
	@echo ' '

cpu/sysinit.o: $(CPU)/sysinit.c
	@echo 'Building file: $<'
	@echo 'Executing target #35 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/sysinit.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/sysinit.o"
	@echo 'Finished building: $<'
	@echo ' '

cpu/vectors.o: $(CPU)/vectors.c
	@echo 'Building file: $<'
	@echo 'Executing target #36 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/vectors.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/vectors.o"
	@echo 'Finished building: $<'
	@echo ' '


