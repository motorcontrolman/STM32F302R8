################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MCLIB/ControlFunctions.c \
../Core/MCLIB/GeneralFunctions.c \
../Core/MCLIB/GlobalVariables.c \
../Core/MCLIB/Sequence.c \
../Core/MCLIB/SignalReadWrite.c \
../Core/MCLIB/SixsStep.c \
../Core/MCLIB/VectorControl.c 

OBJS += \
./Core/MCLIB/ControlFunctions.o \
./Core/MCLIB/GeneralFunctions.o \
./Core/MCLIB/GlobalVariables.o \
./Core/MCLIB/Sequence.o \
./Core/MCLIB/SignalReadWrite.o \
./Core/MCLIB/SixsStep.o \
./Core/MCLIB/VectorControl.o 

C_DEPS += \
./Core/MCLIB/ControlFunctions.d \
./Core/MCLIB/GeneralFunctions.d \
./Core/MCLIB/GlobalVariables.d \
./Core/MCLIB/Sequence.d \
./Core/MCLIB/SignalReadWrite.d \
./Core/MCLIB/SixsStep.d \
./Core/MCLIB/VectorControl.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MCLIB/%.o Core/MCLIB/%.su Core/MCLIB/%.cyclo: ../Core/MCLIB/%.c Core/MCLIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/r720r/STM32CubeIDE/workspace_1.12.1/STM32F302R8/Core/MCLIB" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MCLIB

clean-Core-2f-MCLIB:
	-$(RM) ./Core/MCLIB/ControlFunctions.cyclo ./Core/MCLIB/ControlFunctions.d ./Core/MCLIB/ControlFunctions.o ./Core/MCLIB/ControlFunctions.su ./Core/MCLIB/GeneralFunctions.cyclo ./Core/MCLIB/GeneralFunctions.d ./Core/MCLIB/GeneralFunctions.o ./Core/MCLIB/GeneralFunctions.su ./Core/MCLIB/GlobalVariables.cyclo ./Core/MCLIB/GlobalVariables.d ./Core/MCLIB/GlobalVariables.o ./Core/MCLIB/GlobalVariables.su ./Core/MCLIB/Sequence.cyclo ./Core/MCLIB/Sequence.d ./Core/MCLIB/Sequence.o ./Core/MCLIB/Sequence.su ./Core/MCLIB/SignalReadWrite.cyclo ./Core/MCLIB/SignalReadWrite.d ./Core/MCLIB/SignalReadWrite.o ./Core/MCLIB/SignalReadWrite.su ./Core/MCLIB/SixsStep.cyclo ./Core/MCLIB/SixsStep.d ./Core/MCLIB/SixsStep.o ./Core/MCLIB/SixsStep.su ./Core/MCLIB/VectorControl.cyclo ./Core/MCLIB/VectorControl.d ./Core/MCLIB/VectorControl.o ./Core/MCLIB/VectorControl.su

.PHONY: clean-Core-2f-MCLIB

