CC=avr-gcc
CHIP=attiny85
MMCU=-mmcu=$(CHIP)
MCU=--mcu=$(CHIP)
COMMFLAGS=-g $(MMCU)
CFLAGS=-Os -Wall -DF_CPU=1000000UL -I.
SRCS=$(wildcard *.c)
OUTPUT=out.hex
ELF=$(OUTPUT:.hex=.elf)
OBJS=$(SRCS:.c=.o)

all: $(OUTPUT)

$(OUTPUT): $(ELF)
	avr-objcopy -j .text -j .data -O ihex $(ELF) $(OUTPUT)
	avr-size --format=avr $(MCU) $(ELF)

$(ELF): $(OBJS)
	$(CC) $(COMMFLAGS) $(OBJS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(COMMFLAGS) -c $< -o $@

flash:
	avrdude -p $(CHIP) -c usbasp -P usb -U flash:w:$(OUTPUT)

clean:
	rm -rf *.hex *.o *.elf
