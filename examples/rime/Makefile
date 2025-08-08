CONTIKI_PROJECT = example-broadcast-2
all: $(CONTIKI_PROJECT)
CONTIKI_WITH_RIME = 1

LDFLAGS += -Wl,--gc-sections

#UIP_CONF_IPV6=1 (se o wifi exigir)
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
