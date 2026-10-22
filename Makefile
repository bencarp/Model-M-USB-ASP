
MCU = atmega8
F_CPU = 12000000UL

CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CFLAGS += -Os -std=gnu11 -Wall -Wextra
CFLAGS += -I. -Iusbdrv -funsigned-char -funsigned-bitfields
CFLAGS += -ffunction-sections -fdata-sections

LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections

OBJECTS = main.o usbdrv/usbdrv.o usbdrv/usbdrvasm.o

all: modelm.hex

modelm.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS)
	$(SIZE) $@

modelm.hex: modelm.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

%.o: %.c usbconfig.h
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S usbconfig.h
	$(CC) $(CFLAGS) -x assembler-with-cpp -c -o $@ $<

clean:
	rm -f $(OBJECTS) modelm.elf modelm.hex

.PHONY: all clean
