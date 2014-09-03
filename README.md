RaspberryPi-NeoPixel-WS2812
===========================

Library for driving WS2812 pixels (also known as "NeoPixels" when sold by Adafruit) from a Raspberry Pi. Unlike other solutions, this DOES NOT require an Arduino or other external controller. The Raspberry Pi has a DMA controller that is perfectly capable of doing the job. All you need is a resistor and a capacitor, and you're done!

Wishlist:
* Modify DMA code so it can use more than one 4K page, enabling >450 pixels (some people have thousands!)
* Turn this into a FIFO daemon, like ServoBlaster
* There are a few stupid magic numbers left that I haven't changed to DEFINEs yet
* Make it immediately return after initiating DMA transfer, so we can begin building the next frame (for higher framerate on huge lengths of pixels)

Done:
* Add whatever functions are present in the Adafruit Arduino library, but not implemented here
* Change calculated delay after DMA transfer start to reflect number of pixel commands sent (plus one word, to ensure low latch signal is sent) rather than the length of the entire buffer
* Fix high CPU usage
