CC=gcc
CFLAGS=-g -O0 -Wall -Wformat
CXXFLAGS=-g -O0 -Wall -Wformat
LDFLAGS=-lm

all: bin/ws2812-RPi bin/ws2812-RPi-cxx

bin/ws2812-RPi: ws2812-RPi.c ws2812-RPi.h demo.c
	mkdir -p bin
	$(CC) $(CFLAGS) $+ $(LDFLAGS) -o $@

bin/ws2812-RPi-cxx: ws2812-RPi.c ws2812-RPi.h demo.c
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $+ $(LDFLAGS) -o $@

