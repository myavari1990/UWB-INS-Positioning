################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/sys/syscalls.c \
../src/sys/unistd.c 

OBJS += \
./src/sys/syscalls.o \
./src/sys/unistd.o 

C_DEPS += \
./src/sys/syscalls.d \
./src/sys/unistd.d 


# Each subdirectory must supply rules for building sources it contributes
src/sys/%.o: ../src/sys/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_OTG_Driver\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\Include" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_Device_Library\Class\audio\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_Device_Library\Class\cdc\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_Device_Library\Class\dfu\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_Device_Library\Class\hid\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_Device_Library\Class\msc\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32_USB_Device_Library\Core\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32F10x_StdPeriph_Driver\inc" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\STM32F10x_StdPeriph_Driver\src" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\CMSIS\CM3\CoreSupport" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\src\usb" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\src\application" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\src\compiler" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\src\decadriver" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\src\platform" -I"C:\Users\user\MovedFromM07\Decawave-Complete\Deca\src\sys" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


