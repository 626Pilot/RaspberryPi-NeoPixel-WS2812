
all: bin/ws2812-RPi

bin/ws2812-RPi: ws2812-RPi.c
	mkdir -p bin
	gcc $+ -o $@
