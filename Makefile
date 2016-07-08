# Source files 
SRCS = main.c LED_PWM.c system_stm32f4xx.c funcs1.s stm32f4xx_usart.c stm32f4xx_syscfg.c

ACCELEROMETER_SRCS = accelerometers/accelerometers.c accelerometers/tm_accelerometers/tm_stm32f4_lis302dl_lis3dsh.c accelerometers/tm_accelerometers/tm_stm32f4_spi.c
SRCS += $(ACCELEROMETER_SRCS)

RNG_SRCS = RNG/random_number_generator.c
SRCS += $(RNG_SRCS)

TIM_SRC = STM32F4-Discovery_FW_V1.1.0/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
SRCS += $(TIM_SRC)

TEMPERATURE_SRCS = temperature/temperature.c
SRCS += $(TEMPERATURE_SRCS)

HCSR04_SRCS = tm_stm32f4_hcsr04.c
SRCS += $(HCSR04_SRCS)

GPIO_SRCS = tm_stm32f4_gpio.c
SRCS += $(GPIO_SRCS)

DELAY_SRCS = tm_stm32f4_delay.c
SRCS += $(DELAY_SRCS)

TIMER_SRCS = tm_stm32f4_timer_properties.c
SRCS += $(TIMER_SRCS)
# Binary will be generated with this name (with .elf filename extension)
PROJ_NAME=STM32_example

# You should not need to change anything below this line!
#######################################################################################

BUILDDIR = build

# STM32F4 library code directory
STM_COMMON=STM32F4-Discovery_FW_V1.1.0

SERIAL_PORT_USB_SRCS = stm32f4xx_it.c serial_port_usb/serial_port_usb.c serial_port_usb/usb_bsp.c serial_port_usb/usbd_cdc_vcp.c serial_port_usb/usbd_desc.c serial_port_usb/usbd_usr.c
SERIAL_PORT_USB_SRCS += serial_port_usb/USB_OTG/src/usb_core.c serial_port_usb/USB_OTG/src/usb_dcd.c serial_port_usb/USB_OTG/src/usb_dcd_int.c serial_port_usb/USB_OTG/src/usb_hcd.c serial_port_usb/USB_OTG/src/usb_hcd_int.c serial_port_usb/USB_OTG/src/usb_otg.c
USB_DEVICE_LIBRARY = $(STM_COMMON)/Libraries/STM32_USB_Device_Library
SERIAL_PORT_USB_SRCS += $(USB_DEVICE_LIBRARY)/Core/src/usbd_core.c $(USB_DEVICE_LIBRARY)/Core/src/usbd_ioreq.c $(USB_DEVICE_LIBRARY)/Core/src/usbd_req.c
SERIAL_PORT_USB_SRCS += $(USB_DEVICE_LIBRARY)/Class/cdc/src/usbd_cdc_core.c

SRCS += $(SERIAL_PORT_USB_SRCS)


# Tools
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE=arm-none-eabi-size
ifeq ($(OS),Windows_NT)
MKDIR=busybox mkdir
else
MKDIR=mkdir
endif

#Debug
CFLAGS  = -ggdb -O0 -Wall

CFLAGS += -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -D__FPU_PRESENT=1 -DSTM32F40_41xxx -DUSE_STM32F4_DISCO -DSTM32F4XX -DARM_MATH_CM4

CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -Xassembler -mimplicit-it=always
CFLAGS += -fsingle-precision-constant -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.

# Include files from STM32 libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -Iserial_port_usb -I$(USB_DEVICE_LIBRARY)/Core/inc -I$(USB_DEVICE_LIBRARY)/Class/cdc/inc -Iserial_port_usb/USB_OTG/inc

# Linker script
LDSCRIPT = stm32_flash.ld
LDFLAGS += -T$(LDSCRIPT)

# add startup file to build
SRCS += $(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s

# add required files from STM32 standard peripheral library
STM_SRCDIR = $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src
SRCS += $(STM_SRCDIR)/stm32f4xx_gpio.c $(STM_SRCDIR)/stm32f4xx_spi.c $(STM_SRCDIR)/stm32f4xx_rcc.c $(STM_SRCDIR)/stm32f4xx_rng.c $(STM_SRCDIR)/stm32f4xx_adc.c $(STM_SRCDIR)/stm32f4xx_exti.c $(STM_SRCDIR)/misc.c

OBJS = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(SRCS))))

SEMIHOSTING_FLAGS = --specs=rdimon.specs -lc -lrdimon

ELF = $(PROJ_NAME).elf
BIN = $(PROJ_NAME).bin


$(BUILDDIR)/%.o: %.c
	$(MKDIR) -p $(dir $@)
	$(CC) -c $(SEMIHOSTING_FLAGS) $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.s
	$(MKDIR) -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY: all

all: $(ELF) $(BIN)

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) $(SEMIHOSTING_FLAGS) $(CFLAGS) -o $@ $(OBJS) $(STM_COMMON)/Libraries/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a
	$(SIZE) $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@


clean:
	rm -f $(ELF) $(BIN) $(OBJS)


#######################################################
# Debugging targets
#######################################################
gdb: all
	arm-none-eabi-gdb -x .gdbinit $(ELF)

# Start OpenOCD GDB server (supports semihosting)
openocd: 
	openocd -f board/stm32f4discovery.cfg 

flash: $(BIN)
	st-flash write $(BIN) 0x8000000

