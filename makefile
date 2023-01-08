TARGET=chronometer.hex

# Link minimal version of printf.
CFLAGS=-mmcu=atmega48pa -Os -Wall -Wextra -std=c++11 -Wl,-u,vfprintf -lprintf_min
#CFLAGS=-mmcu=atmega328p -Os -Wall -Wextra -std=c++11

AVRDUDE=avrdude -p m48p -c usbasp-clone -B 8
#AVRDUDE=avrdude -p m328p -c usbasp-clone -B 8
 
all:	$(TARGET)
 
upload:	$(TARGET)
	$(AVRDUDE) -e -U flash:w:$(TARGET)
 
test:
	$(AVRDUDE) -n  -v
 
%.elf:	%.c
	avr-g++ $(CFLAGS) $< -o $@
 
%.hex:	%.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

