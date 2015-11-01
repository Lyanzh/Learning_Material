################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"$(COMMON)/alloc.c" \
"$(COMMON)/assert.c" \
"$(COMMON)/io.c" \
"$(COMMON)/memtest.c" \
"$(COMMON)/printf.c" \
"$(COMMON)/queue.c" \
"$(COMMON)/startup.c" \
"$(COMMON)/stdlib.c" \
"$(COMMON)/uif.c" \

C_SRCS += \
$(COMMON)/alloc.c \
$(COMMON)/assert.c \
$(COMMON)/io.c \
$(COMMON)/memtest.c \
$(COMMON)/printf.c \
$(COMMON)/queue.c \
$(COMMON)/startup.c \
$(COMMON)/stdlib.c \
$(COMMON)/uif.c \

OBJS += \
./common/alloc.o \
./common/assert.o \
./common/io.o \
./common/memtest.o \
./common/printf.o \
./common/queue.o \
./common/startup.o \
./common/stdlib.o \
./common/uif.o \

C_DEPS += \
./common/alloc.d \
./common/assert.d \
./common/io.d \
./common/memtest.d \
./common/printf.d \
./common/queue.d \
./common/startup.d \
./common/stdlib.d \
./common/uif.d \

OBJS_QUOTED += \
"./common/alloc.o" \
"./common/assert.o" \
"./common/io.o" \
"./common/memtest.o" \
"./common/printf.o" \
"./common/queue.o" \
"./common/startup.o" \
"./common/stdlib.o" \
"./common/uif.o" \

C_DEPS_QUOTED += \
"./common/alloc.d" \
"./common/assert.d" \
"./common/io.d" \
"./common/memtest.d" \
"./common/printf.d" \
"./common/queue.d" \
"./common/startup.d" \
"./common/stdlib.d" \
"./common/uif.d" \

OBJS_OS_FORMAT += \
./common/alloc.o \
./common/assert.o \
./common/io.o \
./common/memtest.o \
./common/printf.o \
./common/queue.o \
./common/startup.o \
./common/stdlib.o \
./common/uif.o \


# Each subdirectory must supply rules for building sources it contributes
common/alloc.o: $(COMMON)/alloc.c
	@echo 'Building file: $<'
	@echo 'Executing target #37 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/alloc.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/alloc.o"
	@echo 'Finished building: $<'
	@echo ' '

common/assert.o: $(COMMON)/assert.c
	@echo 'Building file: $<'
	@echo 'Executing target #38 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/assert.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/assert.o"
	@echo 'Finished building: $<'
	@echo ' '

common/io.o: $(COMMON)/io.c
	@echo 'Building file: $<'
	@echo 'Executing target #39 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/io.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/io.o"
	@echo 'Finished building: $<'
	@echo ' '

common/memtest.o: $(COMMON)/memtest.c
	@echo 'Building file: $<'
	@echo 'Executing target #40 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/memtest.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/memtest.o"
	@echo 'Finished building: $<'
	@echo ' '

common/printf.o: $(COMMON)/printf.c
	@echo 'Building file: $<'
	@echo 'Executing target #41 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/printf.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/printf.o"
	@echo 'Finished building: $<'
	@echo ' '

common/queue.o: $(COMMON)/queue.c
	@echo 'Building file: $<'
	@echo 'Executing target #42 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/queue.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/queue.o"
	@echo 'Finished building: $<'
	@echo ' '

common/startup.o: $(COMMON)/startup.c
	@echo 'Building file: $<'
	@echo 'Executing target #43 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/startup.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/startup.o"
	@echo 'Finished building: $<'
	@echo ' '

common/stdlib.o: $(COMMON)/stdlib.c
	@echo 'Building file: $<'
	@echo 'Executing target #44 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/stdlib.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/stdlib.o"
	@echo 'Finished building: $<'
	@echo ' '

common/uif.o: $(COMMON)/uif.c
	@echo 'Building file: $<'
	@echo 'Executing target #45 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/uif.args" -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/uif.o"
	@echo 'Finished building: $<'
	@echo ' '


