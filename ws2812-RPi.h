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

/*
// =================================================================================================
//	.___              .__            .___             
//	|   | ____   ____ |  |  __ __  __| _/____   ______
//	|   |/    \_/ ___\|  | |  |  \/ __ |/ __ \ /  ___/
//	|   |   |  \  \___|  |_|  |  / /_/ \  ___/ \___ \ 
//	|___|___|  /\___  >____/____/\____ |\___  >____  >
//	         \/     \/                \/    \/     \/ 
// =================================================================================================
*/

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
#include <signal.h>
#include <sys/file.h>	// Used for single instance check



/*
// =================================================================================================
//	________          _____.__                         ____    ____   ____                    
//	\______ \   _____/ ____\__| ____   ____   ______  /  _ \   \   \ /   /____ _______  ______
//	 |    |  \_/ __ \   __\|  |/    \_/ __ \ /  ___/  >  _ </\  \   Y   /\__  \\_  __ \/  ___/
//	 |    `   \  ___/|  |  |  |   |  \  ___/ \___ \  /  <_\ \/   \     /  / __ \|  | \/\___ \ 
//	/_______  /\___  >__|  |__|___|  /\___  >____  > \_____\ \    \___/  (____  /__|  /____  >
//	        \/     \/              \/     \/     \/         \/                \/           \/ 
// =================================================================================================
*/

// Base addresses for GPIO, PWM, PWM clock, and DMA controllers (physical, not bus!)
// These will be "memory mapped" into virtual RAM so that they can be written and read directly.
// -------------------------------------------------------------------------------------------------
#define DMA_BASE		0x20007000
#define DMA_LEN			0x24
#define PWM_BASE		0x2020C000
#define PWM_LEN			0x28
#define CLK_BASE	    0x20101000
#define CLK_LEN			0xA8
#define GPIO_BASE		0x20200000
#define GPIO_LEN		0xB4

// GPIO
// -------------------------------------------------------------------------------------------------
#define GPFSEL0			0x20200000			// GPIO function select, pins 0-9 (bits 30-31 reserved)
#define GPFSEL1			0x20200004			// Pins 10-19
#define GPFSEL2			0x20200008			// Pins 20-29
#define GPFSEL3			0x2020000C			// Pins 30-39
#define GPFSEL4			0x20200010			// Pins 40-49
#define GPFSEL5			0x20200014			// Pins 50-53
#define GPSET0			0x2020001C			// Set (turn on) pin
#define GPCLR0			0x20200028			// Clear (turn off) pin
#define GPPUD			0x20200094			// Internal pullup/pulldown resistor control
#define GPPUDCLK0		0x20200098			// PUD clock for pins 0-31
#define GPPUDCLK1		0x2020009C			// PUD clock for pins 32-53

// Memory offsets for the PWM clock register, which is undocumented! Please fix that, Broadcom!
// -------------------------------------------------------------------------------------------------
#define	PWM_CLK_CNTL 	40		// Control (on/off)
#define	PWM_CLK_DIV  	41		// Divisor (bits 11:0 are *quantized* floating part, 31:12 integer part)

// PWM Register Addresses (page 141)
// These are divided by 4 because the register offsets in the guide are in bytes (8 bits) but
// the pointers we use in this program are in words (32 bits). Buss' original defines are in
// word offsets, e.g. PWM_RNG1 was 4 and PWM_DAT1 was 5. This is functionally the same, but it
// matches the numbers supplied in the guide.
// -------------------------------------------------------------------------------------------------
#define	PWM_CTL  0x00		// Control Register
#define PWM_STA  (0x04 / 4)	// Status Register
#define PWM_DMAC (0x08 / 4)	// DMA Control Register
#define PWM_RNG1 (0x10 / 4)	// Channel 1 Range
#define PWM_DAT1 (0x14 / 4)	// Channel 1 Data
#define PWM_FIF1 (0x18 / 4)	// FIFO (for both channels - bytes are interleaved if both active)
#define PWM_RNG2 (0x20 / 4)	// Channel 2 Range
#define PWM_DAT2 (0x24 / 4)	// Channel 2 Data

// PWM_CTL register bit offsets
// Note: Don't use MSEN1/2 for this purpose. It will screw things up.
// -------------------------------------------------------------------------------------------------
#define PWM_CTL_MSEN2	15	// Channel 2 - 0: Use PWM algorithm. 1: Use M/S (serial) algorithm.
#define PWM_CTL_USEF2	13	// Channel 2 - 0: Use PWM_DAT2. 1: Use FIFO.
#define PWM_CTL_POLA2	12	// Channel 2 - Invert output polarity (if set, 0=high and 1=low)
#define PWM_CTL_SBIT2	11	// Channel 2 - Silence bit (default line state when not transmitting)
#define PWM_CTL_RPTL2	10	// Channel 2 - Repeat last data in FIFO
#define PWM_CTL_MODE2	9	// Channel 2 - Mode. 0=PWM, 1=Serializer
#define PWM_CTL_PWEN2	8	// Channel 2 - Enable PWM
#define	PWM_CTL_CLRF1	6	// Clear FIFO
#define	PWM_CTL_MSEN1	7	// Channel 1 - 0: Use PWM algorithm. 1: Use M/S (serial) algorithm.
#define	PWM_CTL_USEF1	5	// Channel 1 - 0: Use PWM_DAT1. 1: Use FIFO.
#define	PWM_CTL_POLA1	4	// Channel 1 - Invert output polarity (if set, 0=high and 1=low)
#define	PWM_CTL_SBIT1	3	// Channel 1 - Silence bit (default line state when not transmitting)
#define	PWM_CTL_RPTL1	2	// Channel 1 - Repeat last data in FIFO
#define	PWM_CTL_MODE1	1	// Channel 1 - Mode. 0=PWM, 1=Serializer
#define	PWM_CTL_PWEN1	0	// Channel 1 - Enable PWM

// PWM_STA register bit offsets
// -------------------------------------------------------------------------------------------------
#define PWM_STA_STA4	12	// Channel 4 State
#define PWM_STA_STA3	11	// Channel 3 State
#define PWM_STA_STA2	10	// Channel 2 State
#define PWM_STA_STA1	9	// Channel 1 State
#define PWM_STA_BERR	8	// Bus Error
#define PWM_STA_GAPO4	7	// Gap Occurred on Channel 4
#define PWM_STA_GAPO3	6	// Gap Occurred on Channel 3
#define PWM_STA_GAPO2	5	// Gap Occurred on Channel 2
#define PWM_STA_GAPO1	4	// Gap Occurred on Channel 1
#define PWM_STA_RERR1	3	// FIFO Read Error
#define PWM_STA_WERR1	2	// FIFO Write Error
#define PWM_STA_EMPT1	1	// FIFO Empty
#define PWM_STA_FULL1	0	// FIFO Full

// PWM_DMAC bit offsets
// -------------------------------------------------------------------------------------------------
#define PWM_DMAC_ENAB	31	// 0: DMA Disabled. 1: DMA Enabled.
#define PWM_DMAC_PANIC	8	// Bits 15:8. Threshold for PANIC signal. Default 7.
#define PWM_DMAC_DREQ	0	// Bits 7:0. Threshold for DREQ signal. Default 7.

// PWM_RNG1, PWM_RNG2
// --------------------------------------------------------------------------------------------------
// Defines the transmission range. In PWM mode, evenly spaced pulses are sent within a period
// of length defined in these registers. In serial mode, serialized data is sent within the
// same period. The value is normally 32. If less, data will be truncated. If more, data will
// be padded with zeros.

// DAT1, DAT2
// --------------------------------------------------------------------------------------------------
// NOTE: These registers are not useful for our purposes - we will use the FIFO instead!
// Stores 32 bits of data to be sent when USEF1/USEF2 is 0. In PWM mode, defines how many
// pulses will be sent within the period specified in PWM_RNG1/PWM_RNG2. In serializer mode,
// defines a 32-bit word to be transmitted.

// FIF1
// --------------------------------------------------------------------------------------------------
// 32-bit-wide register used to "stuff" the FIFO, which has 16 32-bit words. (So, if you write
// it 16 times, it will fill the FIFO.)
// See also:	PWM_STA_EMPT1 (FIFO empty)
//				PWM_STA_FULL1 (FIFO full)
//				PWM_CTL_CLRF1 (Clear FIFO)

// DMA
// --------------------------------------------------------------------------------------------------
// DMA registers (divided by four to convert form word to byte offsets, as with the PWM registers)
#define DMA_CS				(0x00 / 4)	// Control & Status register
#define DMA_CONBLK_AD		(0x04 /	4)	// Address of Control Block (must be 256-BYTE ALIGNED!!!)
#define DMA_TI				(0x08 /	4)	// Transfer Information (populated from CB)
#define DMA_SOURCE_AD		(0x0C /	4)	// Source address, populated from CB. Physical address.
#define DMA_DEST_AD			(0x10 /	4)	// Destination address, populated from CB. Bus address.
#define DMA_TXFR_LEN		(0x14 /	4)	// Transfer length, populated from CB
#define DMA_STRIDE			(0x18 /	4)	// Stride, populated from CB
#define DMA_NEXTCONBK		(0x1C /	4)	// Next control block address, populated from CB
#define DMA_DEBUG			(0x20 /	4)	// Debug settings

// DMA Control & Status register bit offsets
#define DMA_CS_RESET		31			// Reset the controller for this channel
#define DMA_CS_ABORT		30			// Set to abort transfer
#define DMA_CS_DISDEBUG		29			// Disable debug pause signal
#define DMA_CS_WAIT_FOR		28			// Wait for outstanding writes
#define DMA_CS_PANIC_PRI	20			// Panic priority (bits 23:20), default 7		
#define DMA_CS_PRIORITY		16			// AXI priority level (bits 19:16), default 7		
#define DMA_CS_ERROR		8			// Set when there's been an error		
#define DMA_CS_WAITING_FOR	6			// Set when the channel's waiting for a write to be accepted		
#define DMA_CS_DREQ_STOPS_DMA 5			// Set when the DMA is paused because DREQ is inactive		
#define DMA_CS_PAUSED		4			// Set when the DMA is paused (active bit cleared, etc.)
#define DMA_CS_DREQ			3			// Set when DREQ line is high
#define DMA_CS_INT			2			// If INTEN is set, this will be set on CB transfer end
#define DMA_CS_END			1			// Set when the current control block is finished
#define DMA_CS_ACTIVE		0			// Enable DMA (CB_ADDR must not be 0)
// Default CS word
#define DMA_CS_CONFIGWORD	(8 << DMA_CS_PANIC_PRI) | \
							(8 << DMA_CS_PRIORITY) | \
							(1 << DMA_CS_WAIT_FOR)

// DREQ lines (page 61, most DREQs omitted)
#define DMA_DREQ_ALWAYS		0
#define DMA_DREQ_PCM_TX		2
#define DMA_DREQ_PCM_RX		3
#define DMA_DREQ_PWM		5
#define DMA_DREQ_SPI_TX		6
#define DMA_DREQ_SPI_RX		7
#define DMA_DREQ_BSC_TX		8
#define DMA_DREQ_BSC_RX		9

// DMA Transfer Information register bit offsets
// We don't write DMA_TI directly. It's populated from the TI field in a control block.
#define DMA_TI_NO_WIDE_BURSTS	26		// Don't do wide writes in 2-beat bursts
#define DMA_TI_WAITS			21		// Wait this many cycles after end of each read/write
#define DMA_TI_PERMAP			16		// Peripheral # whose ready signal controls xfer rate (pwm=5)
#define DMA_TI_BURST_LENGTH		12		// Length of burst in words (bits 15:12)
#define DMA_TI_SRC_IGNORE		11		// Don't perform source reads (for fast cache fill)
#define DMA_TI_SRC_DREQ			10		// Peripheral in PERMAP gates source reads
#define DMA_TI_SRC_WIDTH		9		// Source transfer width - 0=32 bits, 1=128 bits
#define DMA_TI_SRC_INC			8		// Source address += SRC_WITH after each read
#define DMA_TI_DEST_IGNORE		7		// Don't perform destination writes
#define DMA_TI_DEST_DREQ		6		// Peripheral in PERMAP gates destination writes
#define DMA_TI_DEST_WIDTH		5		// Destination transfer width - 0=32 bits, 1=128 bits
#define DMA_TI_DEST_INC			4		// Dest address += DEST_WIDTH after each read
#define DMA_TI_WAIT_RESP		3		// Wait for write response
#define DMA_TI_TDMODE			1		// 2D striding mode
#define DMA_TI_INTEN			0		// Interrupt enable
// Default TI word
#define DMA_TI_CONFIGWORD		(1 << DMA_TI_NO_WIDE_BURSTS) | \
								(1 << DMA_TI_SRC_INC) | \
								(1 << DMA_TI_DEST_DREQ) | \
								(1 << DMA_TI_WAIT_RESP) | \
								(1 << DMA_TI_INTEN) | \
								(DMA_DREQ_PWM << DMA_TI_PERMAP)

// DMA Debug register bit offsets
#define DMA_DEBUG_LITE					28		// Whether the controller is "Lite"
#define DMA_DEBUG_VERSION				25		// DMA Version (bits 27:25)
#define DMA_DEBUG_DMA_STATE				16		// DMA State (bits 24:16)
#define DMA_DEBUG_DMA_ID				8		// DMA controller's AXI bus ID (bits 15:8)
#define DMA_DEBUG_OUTSTANDING_WRITES	4		// Outstanding writes (bits 7:4)
#define DMA_DEBUG_READ_ERROR			2		// Slave read response error (clear by setting)
#define DMA_DEBUG_FIFO_ERROR			1		// Operational read FIFO error (clear by setting)
#define DMA_DEBUG_READ_LAST_NOT_SET		0		// AXI bus read last signal not set (clear by setting)

// Control Block (CB) - this tells the DMA controller what to do.
typedef struct {
	unsigned int
		info,		// Transfer Information (TI)
		src,		// Source address (physical)
		dst,		// Destination address (bus)
		length,		// Length in bytes (not words!)
		stride,		// We don't care about this
		next,		// Pointer to next control block
		pad[2];		// These are "reserved" (unused)
} dma_cb_t;

// The page map contains pointers to memory that we will allocate below. It uses two pointers
// per address. This is because the software (this program) deals only in virtual addresses,
// whereas the DMA controller can only access RAM via physical address. (If that's not confusing
// enough, it writes to peripherals by their bus addresses.)
typedef struct {
	uint8_t *virtaddr;
	uint32_t physaddr;
} page_map_t;


// Contains arrays of control blocks and their related samples.
// One pixel needs 72 bits (24 bits for the color * 3 to represent them on the wire).
// 		 768 words = 341.3 pixels
// 		1024 words = 455.1 pixels
// The highest I can make this number is 1016. Any higher, and it will start copying garbage to the
// PWM controller. I think it might be because of the virtual->physical memory mapping not being
// contiguous, so *pointer+1016 isn't "next door" to *pointer+1017 for some weird reason.
// However, that's still enough for 451.5 color instructions! If someone has more pixels than that
// to control, they can figure it out. I tried Hirst's message of having one CB per word, which
// seems like it might fix that, but I couldn't figure it out.
#define NUM_DATA_WORDS 1016
struct control_data_s {
	dma_cb_t cb[1];
	uint32_t sample[NUM_DATA_WORDS];
};


#define PAGE_SIZE	4096					// Size of a RAM page to be allocated
#define PAGE_SHIFT	12						// This is used for address translation
#define NUM_PAGES	((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define SETBIT(word, bit) word |= 1<<bit
#define CLRBIT(word, bit) word &= ~(1<<bit)
#define GETBIT(word, bit) word & (1 << bit) ? 1 : 0
#define true 1
#define false 0

// GPIO
#define INP_GPIO(g) *(gpio_reg+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio_reg+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio_reg+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET *(gpio_reg+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio_reg+10) // clears bits which are 1 ignores bits which are 0


// =================================================================================================
//	  ________                                  .__   
//	 /  _____/  ____   ____   ________________  |  |  
//	/   \  ____/ __ \ /    \_/ __ \_  __ \__  \ |  |  
//	\    \_\  \  ___/|   |  \  ___/|  | \// __ \|  |__
//	 \______  /\___  >___|  /\___  >__|  (____  /____/
//	        \/     \/     \/     \/           \/      
// =================================================================================================

void printBinary(unsigned int i, unsigned int bits);
unsigned int reverseWord(unsigned int word);

void terminate(int dummy);

void fatal(char *fmt, ...);

// Memory management
// --------------------------------------------------------------------------------------------------
unsigned int mem_virt_to_phys(void *virt);
unsigned int mem_phys_to_virt(uint32_t phys);
void * map_peripheral(uint32_t base, uint32_t len);


/*
// =================================================================================================
//	.____     ___________________      _________ __          _____  _____ 
//	|    |    \_   _____/\______ \    /   _____//  |_ __ ___/ ____\/ ____\
//	|    |     |    __)_  |    |  \   \_____  \\   __\  |  \   __\\   __\ 
//	|    |___  |        \ |    `   \  /        \|  | |  |  /|  |   |  |   
//	|_______ \/_______  //_______  / /_______  /|__| |____/ |__|   |__|   
//	        \/        \/         \/          \/                           
// =================================================================================================
*/

// Brightness - I recommend 0.2 for direct viewing at 3.3v.
#define DEFAULT_BRIGHTNESS 1.0
extern float brightness;

// LED buffer (this will be translated into pulses in PWMWaveform[])
typedef struct {
	unsigned char r;
	unsigned char g;
	unsigned char b;
} Color_t;

unsigned int numLEDs;		// How many LEDs there are on the chain

#define LED_BUFFER_LENGTH 24
Color_t LEDBuffer[LED_BUFFER_LENGTH];

// PWM waveform buffer (in words), 16 32-bit words are enough to hold 170 wire bits.
// That's OK if we only transmit from the FIFO, but for DMA, we will use a much larger size.
// 1024 (4096 bytes) should be enough for over 400 elements. It can be bumped up if you need more!
unsigned int PWMWaveform[NUM_DATA_WORDS];

// Set brightness
unsigned char setBrightness(float b);

// Zero out the PWM waveform buffer
void clearPWMBuffer();

// Zero out the LED buffer
void clearLEDBuffer();

// Turn r, g, and b into a Color_t struct
Color_t RGB2Color(unsigned char r, unsigned char g, unsigned char b);

// Alias for the above
Color_t Color(unsigned char r, unsigned char g, unsigned char b);

// Set pixel color (24-bit color)
unsigned char setPixelColor(unsigned int pixel, unsigned char r, unsigned char g, unsigned char b);

// Set pixel color, by a direct Color_t
unsigned char setPixelColorT(unsigned int pixel, Color_t c);

// Get pixel color
Color_t getPixelColor(unsigned int pixel);

// Return # of pixels
unsigned int numPixels();

// Return pointer to pixels (FIXME: dunno if this works!)
Color_t* getPixels();

// Set an individual bit in the PWM output array, accounting for word boundaries
// The (31 - bitIdx) is so that we write the data backwards, correcting its endianness
// This means getPWMBit will return something other than what was written, so it would be nice
// if the logic that calls this function would figure it out instead. (However, that's trickier)
void setPWMBit(unsigned int bitPos, unsigned char bit);

// Get an individual bit from the PWM output array, accounting for word boundaries
unsigned char getPWMBit(unsigned int bitPos); 


/*
// =================================================================================================
//	________        ___.
//	\______ \   ____\_ |__  __ __  ____  
//	 |    |  \_/ __ \| __ \|  |  \/ ___\ 
//	 |    `   \  ___/| \_\ \  |  / /_/  >
//	 /_______  /\___  >___  /____/\___  / 
//	         \/     \/    \/     /_____/  
// =================================================================================================
*/

// Dump contents of LED buffer
void dumpLEDBuffer();

// Dump contents of PWM waveform
// The last number dumped may not have a multiple of 3 digits (our basic unit of data is 3 bits,
// whereas the RAM comprising the buffer has to be a multiple of 2 bits in size)
void dumpPWMBuffer();

// Display the status of the PWM's control register
void dumpPWMStatus();

// Display the settings in a PWM control word
// If you want to dump the register directly, use this: dumpPWMControl(*(pwm + PWM_CTL));
void dumpPWMControl(unsigned int word);

// Display the settings in the PWM DMAC word
void dumpPWMDMAC();

// Display all PWM registers
void dumpPWM();

// Display all PWM control registers
void dumpDMARegs();

// Display the contents of a Control Block
void dumpControlBlock(dma_cb_t *c);

// Display the contents of a Transfer Information word
void dumpTransferInformation(unsigned int TI);

// Display the readable DMA registers
void dumpDMA();


/*
// =================================================================================================
//	.___       .__  __      ___ ___                  .___                              
//	|   | ____ |__|/  |_   /   |   \_____ _______  __| _/_  _  _______ _______   ____  
//	|   |/    \|  \   __\ /    ~    \__  \\_  __ \/ __ |\ \/ \/ /\__  \\_  __ \_/ __ \ 
//	|   |   |  \  ||  |   \    Y    // __ \|  | \/ /_/ | \     /  / __ \|  | \/\  ___/ 
//	|___|___|  /__||__|    \___|_  /(____  /__|  \____ |  \/\_/  (____  /__|    \___  >
//	         \/                  \/      \/           \/              \/            \/ 
// =================================================================================================
*/

void initHardware();

// Begin the transfer
void startTransfer();



/*
// =================================================================================================
//	  ____ ___            .___       __           .____     ___________________          
//	 |    |   \______   __| _/____ _/  |_  ____   |    |    \_   _____/\______ \   ______
//	 |    |   /\____ \ / __ |\__  \\   __\/ __ \  |    |     |    __)_  |    |  \ /  ___/
//	 |    |  / |  |_> > /_/ | / __ \|  | \  ___/  |    |___  |        \ |    `   \\___ \ 
//	 |______/  |   __/\____ |(____  /__|  \___  > |_______ \/_______  //_______  /____  >
//	           |__|        \/     \/          \/          \/        \/         \/     \/ 
// =================================================================================================
*/

void show();

/*
// =================================================================================================
//	___________ _____  _____              __          
//	\_   _____// ____\/ ____\____   _____/  |_  ______
//	 |    __)_\   __\\   __\/ __ \_/ ___\   __\/  ___/
//	 |        \|  |   |  | \  ___/\  \___|  |  \___ \ 
//	/_______  /|__|   |__|  \___  >\___  >__| /____  >
//	        \/                  \/     \/          \/ 
// =================================================================================================
// The effects in this section are adapted from the Adafruit NeoPixel library at:
// https://github.com/adafruit/Adafruit_NeoPixel/blob/master/examples/strandtest/strandtest.ino
*/

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
Color_t Wheel(uint8_t WheelPos); 

// Fill the dots one after the other with a color
void colorWipe(Color_t c, uint8_t wait);

// Rainbow
void rainbow(uint8_t wait);

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait);

//Theatre-style crawling lights.
void theaterChase(Color_t c, uint8_t wait);

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait);

