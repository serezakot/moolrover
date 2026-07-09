# Имя компилятора
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
# Путь к первой папке (где нашелся core_cm3.h)
INC_CORE = $(HOME)/.local/share/cmsis/STM32CubeF1/Drivers/CMSIS/Core/Include

# Путь ко второй папке (где теперь лежит stm32f1xx.h)
INC_DEV  = $(HOME)/.local/share/cmsis/cmsis_device_f1/Include

CFLAGS = -mcpu=cortex-m3 -mthumb -O0 -Wall -I$(INC_CORE) -I$(INC_DEV) -DSTM32F103xB# Флаги линкера (указываем наш скрипт линкера)
LDFLAGS = -T linker_script.ld 

# Главная цель
all: firmware.bin 

# Собираем .elf со всей отладочной информацией
firmware.elf: main.c startup.c 
	$(CC) $(CFLAGS) $(LDFLAGS) main.c startup.c -o firmware.elf 

# Конвертируем .elf в чистый бинарник .bin для прошивки
firmware.bin: firmware.elf 
	$(OBJCOPY) -O binary firmware.elf firmware.bin 

# Очистка мусора
clean:
	rm -f *.elf *.bin
