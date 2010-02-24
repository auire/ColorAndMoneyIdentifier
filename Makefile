MCU=atmega16
CC=avr-gcc
OBJCOPY=avr-objcopy
# optimize for size:
CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -mcall-prologues
#-------------------
all: usart.hex
#-------------------
usart.hex : usart.out 
	$(OBJCOPY) -R .eeprom -O ihex usart.out usart.hex 
usart.out : usart.o 
	$(CC) $(CFLAGS) -o usart.out -Wl,-Map,usart.map usart.o 
usart.o : usart.c 
	$(CC) $(CFLAGS) -Os -c usart.c
# you need to erase first before loading the program.
# load (program) the software into the eeprom:
load: usart.hex
	uisp -dlpt=/dev/parport0 --erase  -dprog=dapa
	uisp -dlpt=/dev/parport0 --upload if=usart.hex -dprog=dapa  -v=3 --hash=32
#-------------------
clean:
	rm -f *.o *.map *.out
#-------------------