avr-gcc -Wall -O2 -mmcu=atmega16 -o %1 %1.c
avr-objcopy -O ihex %1 %1.hex