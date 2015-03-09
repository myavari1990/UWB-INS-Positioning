################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/STM32_USB_Device_Library/Class/audio/src/usbd_audio_core.c \
../Libraries/STM32_USB_Device_Library/Class/audio/src/usbd_audio_out_if.c 

OBJS += \
./Libraries/STM32_USB_Device_Library/Class/audio/src/usbd_audio_core.o \
./Libraries/STM32_USB_Device_Library/Class/audio/src/usbd_audio_out_if.o 

C_DEPS += \
./Libraries/STM32_USB_Device_Library/Class/audio/src/usbd_audio_core.d \
./Libraries/STM32_USB_Device_Library/Class/audio/src/usbd_audio_out_if.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/STM32_USB_Device_Library/Class/audio/src/%.o: ../Libraries/STM32_USB_Device_Library/Class/audio/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_OTG_Driver\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_Device_Library\Class\audio\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_Device_Library\Class\cdc\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_Device_Library\Class\dfu\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_Device_Library\Class\hid\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_Device_Library\Class\msc\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32_USB_Device_Library\Core\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32F10x_StdPeriph_Driver\inc" -I"C:\DecawavePLUSvcp\Deca\Libraries\STM32F10x_StdPeriph_Driver\src" -I"C:\DecawavePLUSvcp\Deca\Libraries\CMSIS\CM3\CoreSupport" -I"C:\DecawavePLUSvcp\Deca\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x" -I"C:\DecawavePLUSvcp\Deca\src\usb" -I"C:\DecawavePLUSvcp\Deca\src\application" -I"C:\DecawavePLUSvcp\Deca\src\compiler" -I"C:\DecawavePLUSvcp\Deca\src\decadriver" -I"C:\DecawavePLUSvcp\Deca\src\platform" -I"C:\DecawavePLUSvcp\Deca\src\sys" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


