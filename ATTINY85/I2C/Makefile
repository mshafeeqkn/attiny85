# Copyright (c) 2024 Mohammed Shafeeque
# 
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
# 
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
# 
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
CC=avr-gcc
CHIP=attiny85
MMCU=-mmcu=$(CHIP)
MCU=--mcu=$(CHIP)
COMMFLAGS=-g $(MMCU)
CFLAGS=-Os -Wall -DF_CPU=8000000UL -I.
SRCS=$(wildcard *.c)
OUTPUT=$(SRCS:.c=.hex)
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

flash: $(OUTPUT)
	# 8MHz frequency fuse
	# avrdude -p $(CHIP) -c usbasp -P usb -U flash:w:$(OUTPUT) -U lfuse:w:0xE2:m
	# Default FUSE
	# avrdude -p $(CHIP) -c usbasp -P usb -U flash:w:$(OUTPUT) -U lfuse:w:0x62:m
	avrdude -p $(CHIP) -c usbasp -P usb -U flash:w:$(OUTPUT)

clean:
	rm -rf *.hex *.o *.elf
