# Имя компилятора
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# Флаги компилятора под архитектуру Cortex-M3 (Blue Pill)
CFLAGS = -mcpu=cortex-m3 -mthumb -O0 -Wall

# Флаги линкера (указываем наш скрипт линкера)
LDFLAGS = -T linker_script.ld -nostdlib

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
