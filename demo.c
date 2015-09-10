// Set tabs to 4 spaces.

// =================================================================================================
//
//		 __      __  _________________   ______  ____________   ____________________.__ 
//		/  \    /  \/   _____/\_____  \ /  __  \/_   \_____  \  \______   \______   \__|
//		\   \/\/   /\_____  \  /  ____/ >      < |   |/  ____/   |       _/|     ___/  |
//		 \        / /        \/       \/   --   \|   /       \   |    |   \|    |   |  |
//		  \__/\  / /_______  /\_______ \______  /|___\_______ \  |____|_  /|____|   |__|
//		       \/          \/         \/      \/             \/         \/              
//
// WS2812 NeoPixel driver
// Based on code by Richard G. Hirst and others
// Adapted for the WS2812 by 626Pilot, April/May 2014
// Huge ASCII art section labels are from http://patorjk.com/software/taag/
//
// License: GPL
//
// You are using this at your OWN RISK. I believe this software is reasonably safe to use (aside
// from the intrinsic risk to those who are photosensitive - see below), although I can't be certain
// that it won't trash your hardware or cause property damage.
//
// Speaking of risk, WS2812 pixels are bright enough to cause eye pain and (for all I know) possibly
// retina damage when run at full strength. It's a good idea to set the brightness at 0.2 or so for
// direct viewing (whether you're looking directly at the pixels or not), or to put some diffuse
// material between you and the LEDs.
//
// PHOTOSENSITIVITY WARNING:
// Patterns of light and darkness (stationary or moving), flashing lights, patterns and backgrounds
// on screens, and the like, may cause epilleptic seizures in some people. This is a danger EVEN IF
// THE PERSON (WHICH MAY BE *YOU*) HAS NEVER KNOWINGLY HAD A PHOTOSENSITIVE EPISODE BEFORE. It's up
// to you to learn the warning signs, but symptoms may include dizziness, nausea, vision changes,
// convlusions, disorientation, involuntary movements, and eye twitching. (This list is not
// necessarily exhaustive.)
//
// NEOPIXEL BEST PRACTICES: https://learn.adafruit.com/adafruit-neopixel-uberguide/best-practices
//
// Connections:
//		Positive to Raspberry Pi's 3.3v
//		Negative to Raspberry Pi's ground
// 		Data to GPIO18 (Pin 12) (through a resistor, which you should know from the Best
// 		Practices guide!)
//
// GitHub (source, support, etc.): https://github.com/626Pilot/RaspberryPi-NeoPixel-WS2812
//    Buy WS2812-based stuff from: http://adafruit.com
//                   Compile with: make
//                      Test with: sudo ./bin/ws2812-RPi
//                                 (it needs to be root so it can map the peripherals' registers)
//
// =================================================================================================

// This is for the WS2812 LEDs. It won't work with the older WS2811s, although it could be modified
// for that without too much trouble. Preliminary driver used Frank Buss' servo driver, but I moved
// to Richard Hirst's memory mapping/access model because his code already works with DMA, and has
// what I think is a slightly cleaner way of accessing the registers: register[name] rather than
// *(register + name).

// At the time of writing, there's a lot of confusing "PWM DMA" code revolving around simulating
// an FM signal. Usually this is done without properly initializing certain registers, which is
// OK for their purpose, but I needed to be able to transfer actual coherent data and have it wind
// up in a proper state once it was transferred. This has proven to be a somewhat painful task.
// The PWM controller likes to ignore the RPTL1 bit when the data is in a regular, repeating
// pattern. I'M NOT MAKING IT UP! It really does that. It's bizarre. There are lots of other
// strange irregularities as well, which had to be figured out through trial and error. It doesn't
// help that the BCM2835 ARM Peripherals manual contains outright errors and omissions!

// Many examples of this kind of code have magic numbers in them. If you don't know, a magic number
// is one that either lacks an obvious structure (e.g. 0x2020C000) or purpose. Please don't use
// that stuff in any code you release! All magic numbers found in reference code have been changed
// to DEFINEs. That way, instead of seeing some inscrutable number, you see (e.g.) PWM_CTL.

// References - BCM2835 ARM Peripherals:
//              http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
//
//              Raspberry Pi low-level peripherals:
//              http://elinux.org/RPi_Low-level_peripherals
//
//				Richard Hirst's nice, clean code:
//				https://github.com/richardghirst/PiBits/blob/master/PiFmDma/PiFmDma.c
//
//              PWM clock register:
//              http://www.raspberrypi.org/forums/viewtopic.php?t=8467&p=124620
//
//				Simple (because it's in assembly) PWM+DMA setup:
//				https://github.com/mikedurso/rpi-projects/blob/master/asm-nyancat/rpi-nyancat.s
//
//				Adafruit's NeoPixel driver:
//				https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.cpp


// =================================================================================================
//	.___              .__            .___             
//	|   | ____   ____ |  |  __ __  __| _/____   ______
//	|   |/    \_/ ___\|  | |  |  \/ __ |/ __ \ /  ___/
//	|   |   |  \  \___|  |_|  |  / /_/ \  ___/ \___ \ 
//	|___|___|  /\___  >____/____/\____ |\___  >____  >
//	         \/     \/                \/    \/     \/ 
// =================================================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include "ws2812-RPi.h"

// =================================================================================================
//	   _____         .__        
//	  /     \ _____  |__| ____  
//	 /  \ /  \\__  \ |  |/    \ 
//	/    Y    \/ __ \|  |   |  \
//	\____|__  (____  /__|___|  /
//	        \/     \/        \/ 
// =================================================================================================

void effectsDemo() {

	int i, j, ptr;
	float k;

	// Default effects from the Arduino lib
	colorWipe(Color(255, 0, 0), 50); // Red
	colorWipe(Color(0, 255, 0), 50); // Green
	colorWipe(Color(0, 0, 255), 50); // Blue
	theaterChase(Color(127, 127, 127), 50); // White
	theaterChase(Color(127,   0,   0), 50); // Red
	theaterChase(Color(  0,   0, 127), 50); // Blue
	rainbow(5);
	rainbowCycle(5);
	theaterChaseRainbow(50);

	// Watermelon fade :)
	for(k=0; k<0.5; k+=.01) {
		ptr=0;
		setBrightness(k);
		for(i=0; i<numLEDs; i++) {
			setPixelColor(i, i*5, 64, i*2);
		}
		show();
	}
	for(k=0.5; k>=0; k-=.01) {
		ptr=0;
		setBrightness(k);
		for(i=0; i<numLEDs; i++) {
			setPixelColor(i, i*5, 64, i*2);
		}
		show();
	}
	usleep(1000);

	// Random color fade
	srand(time(NULL));
	uint8_t lastRed = 0;
	uint8_t lastGreen = 0;
	uint8_t lastBlue = 0;
	uint8_t red, green, blue;
	Color_t curPixel;
	setBrightness(DEFAULT_BRIGHTNESS);
	for(j=1; j<16; j++) {
		ptr = 0;
		if(j % 3) {
			red = 120;
			green = 64;
			blue = 48;
		} else if(j % 7) {
			red = 255;
			green = 255;
			blue = 255;
		} else {
			red = rand();
			green = rand();
			blue = rand();
		}
		for(k=0; k<1; k+=.01) {
			for(i=0; i<numLEDs; i++) {
				setPixelColor(
					i,
					(red * k) + (lastRed * (1-k)),
					i * (255 / numLEDs), //(green * k) + (lastGreen * (1-k)),
					(blue * k) + (lastBlue * (1-k))
					);
				curPixel = getPixelColor(i);
			}
			show();
		}
		lastRed = red;
		lastGreen = green;
		lastBlue = blue;
	}
}


int main(int argc, char **argv) { 
	// Check "Single Instance"
	int pid_file = open("/var/run/whatever.pid", O_CREAT | O_RDWR, 0666);
	int rc = flock(pid_file, LOCK_EX | LOCK_NB);
	if(rc) {
	    if(EWOULDBLOCK == errno)
	    {
	        // another instance is running
	        printf("Instance already running\n");
	        exit(EXIT_FAILURE);
	    }
	}

	// Catch all signals possible - it's vital we kill the DMA engine on process exit!
	int i;
	for (i = 0; i < 64; i++) {
		struct sigaction sa;
		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}

	// Don't buffer console output
	setvbuf(stdout, NULL, _IONBF, 0);

	// How many LEDs?
	numLEDs = 24;

	// How bright? (Recommend 0.2 for direct viewing @ 3.3V)
	setBrightness(DEFAULT_BRIGHTNESS);

	// Init PWM generator and clear LED buffer
	initHardware();
	clearLEDBuffer();

	// Show some effects
	while(true) {
		effectsDemo();
	}

	// Exit cleanly, freeing memory and stopping the DMA & PWM engines
	// We trap all signals (including Ctrl+C), so even if you don't get here, it terminates correctly
	terminate(0);

	return 0;
}
