CFLAGS=-g -O0 -Wall -Wformat
LDFLAGS=-lm

all: bin/ws2812-RPi


bin/ws2812-RPi: ws2812-RPi.c ws2812-RPi.h demo.c
	mkdir -p bin
	gcc ${CFLAGS} ${LDFLAGS} $+ -o $@
