################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/gui/src/screen1_screen/Screen1Presenter.cpp \
../TouchGFX/gui/src/screen1_screen/Screen1View.cpp 

OBJS += \
./TouchGFX/gui/src/screen1_screen/Screen1Presenter.o \
./TouchGFX/gui/src/screen1_screen/Screen1View.o 

CPP_DEPS += \
./TouchGFX/gui/src/screen1_screen/Screen1Presenter.d \
./TouchGFX/gui/src/screen1_screen/Screen1View.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/gui/src/screen1_screen/%.o TouchGFX/gui/src/screen1_screen/%.su: ../TouchGFX/gui/src/screen1_screen/%.cpp TouchGFX/gui/src/screen1_screen/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F429xx -c -I../TouchGFX/gui/include -I../TouchGFX/generated/fonts/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../TouchGFX/generated/texts/include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../TouchGFX/generated/images/include -I../Middlewares/ST/touchgfx/framework/include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../TouchGFX/target/generated -I../Drivers/CMSIS/Include -I../Core/Inc -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/App -I../TouchGFX/target -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TouchGFX-2f-gui-2f-src-2f-screen1_screen

clean-TouchGFX-2f-gui-2f-src-2f-screen1_screen:
	-$(RM) ./TouchGFX/gui/src/screen1_screen/Screen1Presenter.d ./TouchGFX/gui/src/screen1_screen/Screen1Presenter.o ./TouchGFX/gui/src/screen1_screen/Screen1Presenter.su ./TouchGFX/gui/src/screen1_screen/Screen1View.d ./TouchGFX/gui/src/screen1_screen/Screen1View.o ./TouchGFX/gui/src/screen1_screen/Screen1View.su

.PHONY: clean-TouchGFX-2f-gui-2f-src-2f-screen1_screen

