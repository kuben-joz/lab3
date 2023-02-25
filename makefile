CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
FLAGS = -mthumb -mcpu=cortex-m4
CPPFLAGS = -DSTM32F411xE
CFLAGS = $(FLAGS) -Wall -g  \
-O2 -ffunction-sections -fdata-sections \
-I/home/jakub/Documents/microcontrollers/stm32/inc \
-I/home/jakub/Documents/microcontrollers/stm32/CMSIS/Include \
-I/home/jakub/Documents/microcontrollers/stm32/CMSIS/Device/ST/STM32F4xx/Include 
LDFLAGS = $(FLAGS) -Wl,--gc-sections -nostartfiles \
-L/home/jakub/Documents/microcontrollers/stm32/lds -Tstm32f411re.lds
vpath %.c /home/jakub/Documents/microcontrollers/stm32/src
OBJECTS = uart_main.o  buffer.o startup_stm32.o gpio.o
TARGET = uart
.SECONDARY: $(TARGET).elf $(OBJECTS)
all: $(TARGET).bin
%.elf : $(OBJECTS)
		$(CC) $(LDFLAGS) $^ -o $@

%.bin : %.elf
		$(OBJCOPY) $< $@ -O binary
clean :
		rm -f *.bin *.elf *.hex *.d *.o *.bak *~