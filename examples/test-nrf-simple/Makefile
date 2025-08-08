#CONTIKI_PROJECT = test-nrf
#all: $(CONTIKI_PROJECT)

#CONTIKI = ../..
#TARGET = arduino
#ARDUINO_MODEL = Uno
#CONTIKI_WITH_RIME = 1
#LDFLAGS += -Wl,--gc-sections
#PROJECT_SOURCEFILES += test-nrf.c nrf24l01_atmega328p.c spi.c
#CFLAGS += -I$(CONTIKI)/cpu/avr/dev

#include $(CONTIKI)/Makefile.include

CONTIKI_PROJECT = simple-receiver simple-sender writing-config-reg
CONTIKI = ../..
TARGET = arduino
ARDUINO_MODEL = Uno

PROJECT_SOURCEFILES += nrf24l01_atmega328p_aux.c spi.c
CFLAGS += -I$(CONTIKI)/cpu/avr -I$(CONTIKI)/cpu/avr/dev

include ../../Makefile.include
