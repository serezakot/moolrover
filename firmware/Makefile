# Имя компилятора
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Путь к первой папке (где нашелся core_cm3.h)
INC_CORE ?= $(HOME)/.local/share/cmsis/STM32CubeF1/Drivers/CMSIS/Core/Include
# Путь ко второй папке (где лежит stm32f1xx.h)
INC_DEV  ?= $(HOME)/.local/share/cmsis/cmsis_device_f1/Include

SRCS = main.c rcc.c startup.c

CFLAGS = -mcpu=cortex-m3 -mthumb -O0 -Wall -I$(INC_CORE) -I$(INC_DEV) -DSTM32F103xB
# -nostartfiles: у нас свой Reset_Handler в startup.c. Без этого флага линкер
# тянет стандартный crt0 из newlib и падает (undefined _exit, _sbrk, __bss_start__).
LDFLAGS = -T linker_script.ld -nostartfiles

# Главная цель
all: firmware.bin

# Собираем .elf
firmware.elf: $(SRCS) linker_script.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -o firmware.elf
	$(SIZE) firmware.elf

# Конвертируем .elf в чистый бинарник .bin для прошивки
firmware.bin: firmware.elf
	$(OBJCOPY) -O binary firmware.elf firmware.bin

# Очистка мусора
clean:
	rm -f *.elf *.bin

.PHONY: all clean
