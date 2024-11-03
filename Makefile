##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.19.2] date: [Sat Dec 09 14:32:18 CST 2023] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------


######################################
# target
######################################
TARGET = basic_framework


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/gpio.c \
Src/freertos.c \
Src/adc.c \
Src/can.c \
Src/crc.c \
Src/dac.c \
Src/dma.c \
Src/i2c.c \
Src/rng.c \
Src/rtc.c \
Src/spi.c \
Src/tim.c \
Src/usart.c \
Src/usb_device.c \
Src/usbd_conf.c \
Src/usbd_desc.c \
Src/usbd_cdc_if.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Src/stm32f4xx_hal_timebase_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Src/system_stm32f4xx.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c \
Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c \
bsp/dwt/bsp_dwt.c \
bsp/pwm/bsp_pwm.c \
bsp/gpio/bsp_gpio.c \
bsp/spi/bsp_spi.c \
bsp/iic/bsp_iic.c \
bsp/can/bsp_can.c \
bsp/usart/bsp_usart.c \
bsp/usb/bsp_usb.c \
bsp/log/bsp_log.c \
bsp/flash/bsp_flash.c \
bsp/bsp_tools.c \
modules/algorithm/controller.c \
modules/algorithm/kalman_filter.c \
modules/algorithm/QuaternionEKF.c \
modules/algorithm/crc8.c \
modules/algorithm/crc16.c \
modules/algorithm/user_lib.c \
modules/bluetooth/HC05.c \
modules/BMI088/bmi088.c \
modules/imu/BMI088driver.c \
modules/imu/BMI088Middleware.c \
modules/imu/ins_task.c \
modules/ist8310/ist8310.c \
modules/master_machine/master_process.c \
modules/master_machine/seasky_protocol.c \
modules/motor/DJImotor/dji_motor.c \
modules/motor/HTmotor/HT04.c \
modules/motor/LKmotor/LK9025.c \
modules/motor/DMmotor/dmmotor.c \
modules/motor/step_motor/step_motor.c \
modules/motor/servo_motor/servo_motor.c \
modules/motor/motor_task.c \
modules/oled/oled.c \
modules/referee/crc_ref.c \
modules/referee/rm_referee.c \
modules/referee/referee_UI.c \
modules/referee/referee_task.c \
modules/remote/remote_control.c \
modules/super_cap/super_cap.c \
modules/can_comm/can_comm.c \
modules/message_center/message_center.c \
modules/daemon/daemon.c \
modules/alarm/buzzer.c \
application/gimbal/gimbal.c \
application/chassis/chassis.c \
application/shoot/shoot.c \
application/cmd/robot_cmd.c \
application/robot.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s \
Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb -mthumb-interwork $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-DARM_MATH_CM4 \
-DDISABLE_LOG_SYSTEM 

# AS includes
AS_INCLUDES =  \
-IInc

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IMiddlewares/ST/ARM/DSP/Include/dsp \
-IMiddlewares/ST/ARM/DSP/Include \
-IMiddlewares/Third_Party/SEGGER/RTT \
-IMiddlewares/Third_Party/SEGGER/Config \
-Iapplication/chassis \
-Iapplication/shoot \
-Iapplication/gimbal \
-Iapplication/cmd \
-Iapplication \
-Ibsp/dwt \
-Ibsp/can \
-Ibsp/usart \
-Ibsp/usb \
-Ibsp/gpio \
-Ibsp/spi \
-Ibsp/iic \
-Ibsp/log \
-Ibsp/flash \
-Ibsp/pwm \
-Ibsp \
-Imodules/algorithm \
-Imodules/bluetooth \
-Imodules/BMI088 \
-Imodules/imu \
-Imodules/ist8310 \
-Imodules/master_machine \
-Imodules/motor/DJImotor \
-Imodules/motor/LKmotor \
-Imodules/motor/HTmotor \
-Imodules/motor/step_motor \
-Imodules/motor/servo_motor \
-Imodules/motor/DMmotor \
-Imodules/motor \
-Imodules/oled \
-Imodules/referee \
-Imodules/remote \
-Imodules/super_cap \
-Imodules/can_comm \
-Imodules/message_center \
-Imodules/daemon \
-Imodules/alarm \
-Imodules  \
-IMiddlewares/ST/ARM/DSP/Inc


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -fdata-sections -ffunction-sections -fmessage-length=0 -Werror

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407IGHx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys  \
-l:libCMSISDSP.a
LIBDIR =  \
-LMiddlewares/ST/ARM/DSP/Lib
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -flto -Wl,--no-warn-rwx-segments -Wl,--print-memory-usage

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	@mkdir $@		


#######################################
# clean up
#######################################
clean:
	rd $(BUILD_DIR) /s/q


#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)


#######################################
# download directl without debugging
#######################################
download_dap:
	openocd -f openocd_dap.cfg -c init -c halt -c "flash write_image erase $(BUILD_DIR)/$(TARGET).bin 0x08000000" -c reset -c shutdown
download_jlink:
	JFlash -openprj'stm32.jflash' -open'$(BUILD_DIR)/$(TARGET).hex',0x8000000 -auto -startapp -exit

# *** EOF ***