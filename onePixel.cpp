/* ------------------------------------------------------------------------
  An attempt to create The One Routine to run all pixels from all AVRs
  Cut-n-pasted by David R Ratliff (1ChicagoDave) 2014


  -------------------------------------------------------------------------
  Arduino library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for RGB or GRB color order.  8 MHz MCUs provide
  output on PORTB and PORTD, while 16 MHz chips can handle most output pins
  (possible exception with upper PORT registers on the Arduino Mega).

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC and other members of the open source community.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "onePixel.h"

Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t n, uint8_t p, uint8_t t) : numLEDs(n), numBytes(n * 3), pin(p), pixels(NULL)
#if defined(NEO_RGB) || defined(NEO_KHZ400)
  ,type(t)
#endif
#ifdef __AVR__
  ,port(portOutputRegister(digitalPinToPort(p))),
   pinMask(digitalPinToBitMask(p))
#endif
{
  if((pixels = (uint8_t *)malloc(numBytes))) {
    memset(pixels, 0, numBytes);
  }
}


Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t n, uint8_t p) : numLEDs(n), numBytes(n * 3), pin(p), pixels(NULL)

#ifdef __AVR__
  ,port(portOutputRegister(digitalPinToPort(p))),
   pinMask(digitalPinToBitMask(p))
#endif
{
  if((pixels = (uint8_t *)malloc(numBytes))) {
    memset(pixels, 0, numBytes);
  }
}


Adafruit_NeoPixel::~Adafruit_NeoPixel() {
  if(pixels) free(pixels);
  pinMode(pin, INPUT);
}

void Adafruit_NeoPixel::begin(void) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Adafruit_NeoPixel::show(void) {

  if(!pixels) return;

  // Data latch = 50+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while((micros() - endTime) < 50L);
  // endTime is a private member (rather than global var) so that multiple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  noInterrupts(); // Need 100% focus on instruction timing

#ifdef __AVR__

  volatile uint16_t
    i   = numBytes; // Loop counter
  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b   = *ptr++,   // Current byte value
    hi,             // PORT w/output bit set high
    lo;             // PORT w/output bit set low



                  // 8 MHz(ish) AVR ---------------------------------------------------------
//  Define clock speed -                   
#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)
  #define 8MHZ 
#elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)
  #define 12MHZ
#elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)
  #define 16MHZ

    volatile uint8_t n1, n2 = 0;  // First, next bits out


/* ***************************************************** The One Routine (AVR) ****************************************** */
    //  Based on 400KHZ 8MHZ Adafruit instructions. Adjusted to meet timing specs from datasheet at 600MHZ.
    //  Extra NOPs added to maintain timing at all clock speeds (MHZ)
    //  Maintains 'active' bit timing, only changes "duty cycle" - gap beween bits...? (for KHZ ajust)
    //  Precompiler instructions added to control port used.
    //  - 1CD
    
    

    //***  ONLY add ticks to end of 'bit-sandwich' to change KHZ "duty cycle", leave inter-bit-timing alone.
    
    // for 8MHZ 400KHZ
    // 18 inst. clocks per bit: HHHxxxxxxxLLLLLLLL
    // ST instructions:         ^  ^      ^        (T=1,4,11)

    // At 8 Mhz, each clock tick is about 125 nanoseconds
    // Signal timing for each bit with this code should be:
    // HIGH 375ns + Bit 875ns + LOW 1000ns = 2250ns/bit
    // *****
    
    //  For 12 MHZ 400 KHz ?
    // 30 instruction clocks per bit: HHHHHHxxxxxxxxxLLLLLLLLLLLLLLL
    // ST instructions:               ^     ^        ^    (T=0,6,15)
    
    // For 16 MHz 400 KHZ 
    // 40 inst. clocks per bit: HHHHHHHHxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
    // ST instructions:         ^       ^           ^         (T=0,8,20)


    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
      // This runs for all bits (0's and 1's)
                                                    //        (8 MHZ)       (16MHZ)
     "head20:"                  "\n\t" // Clk  Pseudocode    (T =  0) 
      "out   %[port], %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)	port
      
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 128)
      "mov  %[next], %[hi]"     "\n\t" // 0-1  next = hi     (T =  3)
      "out   %[port], %[next]"  "\n\t" // 1    PORT = next   (T =  4)	port
      
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo     (T =  5)
      "dec  %[bit]"             "\n\t" // 1    bit--         (T =  6)
      "breq nextbyte20"         "\n\t" // 1-2  if(bit == 0) skip to code at 'nextbyte20:'

  // This runs if next bit == 1

      "rol  %[byte]"            "\n\t" // 1    b <<= 1       (T = 8)
      "nop"                     "\n\t" // 1    nop           (T = 18)	
      "nop"                     "\n\t" // 1    nop           (T = 18)	

      "out   %[port], %[lo]"    "\n\t" // 1    PORT = lo     (T = 11)	port
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 13)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 15)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 17)	
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 17)	

      "rjmp head20"             "\n\t" // 2    -> head20 (next bit out). Back to the top!
    
    
   // This runs if next bit == 0
   
      "nextbyte20:"             "\n\t" //                    (T = 8)
      "nop"                     "\n\t" // 1    nop           (T = 9)
      "nop"                     "\n\t" // 1    nop           (T = 10)
      "out   %[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 11)	port
      "nop"                     "\n\t" // 1    nop           (T = 9)
      "ldi  %[bit]  , 8"        "\n\t" // 1    bit = 8       (T = 14)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 16)
      "sbiw %[count], 1"        "\n\t" // 2    i--           (T = 18)
      "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)   Back to the top!

      : [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
#ifdef PORTD
      : [port]   "I" (_SFR_IO_ADDR(PORTD)),
#endif // PORTD
#ifdef PORTB
      : [port] "I" (_SFR_IO_ADDR(PORTB)),
#endif // PORTB
        [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));
        
  }
#endif  // AVR

// 12 MHz(ish) AVR --------------------------------------------------------


    // 15 instruction clocks per bit: HHHHxxxxxxLLLLL
    // OUT instructions:              ^   ^     ^     (T=0,4,10)

 
      // Don't "optimize" the OUT calls into the bitTime subroutine;
      // we're exploiting the RCALL and RET as 3- and 4-cycle NOPs!
   
   
   // ASSEMBLER REMOVED **********
   

      // Same as above, just set for PORTB & stripped of comments
     
      //  ASSEMBLER REMOVED **********
      
   

    // 30 instruction clocks per bit: HHHHHHxxxxxxxxxLLLLLLLLLLLLLLL
    // ST instructions:               ^     ^        ^    (T=1,7,16)


   // ASSEMBLER REMOVED **********
   


// 16 MHz(ish) AVR --------------------------------------------------------


    // WS2811 and WS2812 have different hi/lo duty cycles; this is
    // similar but NOT an exact copy of the prior 400-on-8 code.

    // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
    // ST instructions:         ^    ^       ^       (T=0,5,13)


    // The 400 KHz clock on 16 MHz MCU is the most 'relaxed' version.

    // 40 inst. clocks per bit: HHHHxxxxxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
    // ST instructions:         ^   ^               ^         (T=1,5,21)

    // At 16 Mhz each clock tick lasts about 62.5 nanoseconds 
    // Timing for each the sequence representing one bit of data:
    //     HIGH 250ns + Bit 1000ns + LOW 1250ns = 2500ns/bit

   
#endif

#else
 #error "CPU SPEED NOT SUPPORTED"
#endif

//  END OF AVR  --- BEGIN ARM (TEENSY 3.0 3.1  *********
)
#elif defined(__arm__)

#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 & 3.1
#define CYCLES_800_T0H  (F_CPU / 2500000)    
#define CYCLES_800_T1H  (F_CPU / 1250000)
#define CYCLES_800      (F_CPU /  800000)
#define CYCLES_400_T0H  (F_CPU / 2000000)
#define CYCLES_400_T1H  (F_CPU /  833333)
#define CYCLES_400      (F_CPU /  400000)

  uint8_t          *p   = pixels,
                   *end = p + numBytes, pix, mask;
  volatile uint8_t *set = portSetRegister(pin),
                   *clr = portClearRegister(pin);
  uint32_t          cyc;

  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

#ifdef NEO_KHZ400
  if((type & NEO_SPDMASK) == NEO_KHZ800) { // 800 KHz bitstream
#endif
    cyc = ARM_DWT_CYCCNT + CYCLES_800;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
#ifdef NEO_KHZ400
  } else { // 400 kHz bitstream
    cyc = ARM_DWT_CYCCNT + CYCLES_400;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_400);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_400_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_400_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_400);
  }
#endif

#else // Arduino Due

  #define SCALE      VARIANT_MCK / 2UL / 1000000UL
  #define INST       (2UL * F_CPU / VARIANT_MCK)
  #define TIME_800_0 ((int)(0.40 * SCALE + 0.5) - (5 * INST))
  #define TIME_800_1 ((int)(0.80 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_800 ((int)(1.25 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_0 ((int)(0.50 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_1 ((int)(1.20 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_400 ((int)(2.50 * SCALE + 0.5) - (5 * INST))

  int             pinMask, time0, time1, period, t;
  Pio            *port;
  volatile WoReg *portSet, *portClear, *timeValue, *timeReset;
  uint8_t        *p, *end, pix, mask;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC3_IRQn);
  TC_Configure(TC1, 0,
    TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Start(TC1, 0);

  pinMask   = g_APinDescription[pin].ulPin; // Don't 'optimize' these into
  port      = g_APinDescription[pin].pPort; // declarations above.  Want to
  portSet   = &(port->PIO_SODR);            // burn a few cycles after
  portClear = &(port->PIO_CODR);            // starting timer to minimize
  timeValue = &(TC1->TC_CHANNEL[0].TC_CV);  // the initial 'while'.
  timeReset = &(TC1->TC_CHANNEL[0].TC_CCR);
  p         =  pixels;
  end       =  p + numBytes;
  pix       = *p++;
  mask      = 0x80;

#ifdef NEO_KHZ400
  if((type & NEO_SPDMASK) == NEO_KHZ800) { // 800 KHz bitstream
#endif
    time0 = TIME_800_0;
    time1 = TIME_800_1;
    period = PERIOD_800;
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    time0 = TIME_400_0;
    time1 = TIME_400_1;
    period = PERIOD_400;
  }
#endif

  for(t = time0;; t = time0) {
    if(pix & mask) t = time1;
    while(*timeValue < period);
    *portSet   = pinMask;
    *timeReset = TC_CCR_CLKEN | TC_CCR_SWTRG;
    while(*timeValue < t);
    *portClear = pinMask;
    if(!(mask >>= 1)) {   // This 'inside-out' loop logic utilizes
      if(p >= end) break; // idle time to minimize inter-byte delays.
      pix = *p++;
      mask = 0x80;
    }
  }
  while(*timeValue < period); // Wait for last bit
  TC_Stop(TC1, 0);

#endif // end Arduino Due

#endif // end Architecture select

  interrupts();
  endTime = micros(); // Save EOD time for latch on next call
}

// Set the output pin number
void Adafruit_NeoPixel::setPin(uint8_t p) {
  pinMode(pin, INPUT);
  pin = p;
  pinMode(p, OUTPUT);
  digitalWrite(p, LOW);
#ifdef __AVR__
  port    = portOutputRegister(digitalPinToPort(p));
  pinMask = digitalPinToBitMask(p);
#endif
}

// Set pixel color from separate R,G,B components:
void Adafruit_NeoPixel::setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if(n < numLEDs) {
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    uint8_t *p = &pixels[n * 3];
#ifdef NEO_RGB
    if((type & NEO_COLMASK) == NEO_GRB) {
#endif
      *p++ = g;
      *p++ = r;
#ifdef NEO_RGB
    } else {
      *p++ = r;
      *p++ = g;
    }
#endif
    *p = b;
  }
}

// Set pixel color from 'packed' 32-bit RGB color:
void Adafruit_NeoPixel::setPixelColor(uint16_t n, uint32_t c) {
  if(n < numLEDs) {
    uint8_t
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
      b = (uint8_t)c;
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    uint8_t *p = &pixels[n * 3];
#ifdef NEO_RGB
    if((type & NEO_COLMASK) == NEO_GRB) {
#endif
      *p++ = g;
      *p++ = r;
#ifdef NEO_RGB
    } else {
      *p++ = r;
      *p++ = g;
    }
#endif
    *p = b;
  }
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t Adafruit_NeoPixel::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

// Query color from previously-set pixel (returns packed 32-bit RGB value)
uint32_t Adafruit_NeoPixel::getPixelColor(uint16_t n) const {

  if(n < numLEDs) {
    uint16_t ofs = n * 3;
    return (uint32_t)(pixels[ofs + 2]) |
#ifdef NEO_RGB
      (((type & NEO_COLMASK) == NEO_GRB) ?
#endif
        ((uint32_t)(pixels[ofs    ]) <<  8) |
        ((uint32_t)(pixels[ofs + 1]) << 16)
#ifdef NEO_RGB
      :
        ((uint32_t)(pixels[ofs    ]) << 16) |
        ((uint32_t)(pixels[ofs + 1]) <<  8) )
#endif
      ;
  }

  return 0; // Pixel # is out of bounds
}

uint8_t *Adafruit_NeoPixel::getPixels(void) const {
  return pixels;
}

uint16_t Adafruit_NeoPixel::numPixels(void) const {
  return numLEDs;
}

// Adjust output brightness; 0=darkest (off), 255=brightest.  This does
// NOT immediately affect what's currently displayed on the LEDs.  The
// next call to show() will refresh the LEDs at this level.  However,
// this process is potentially "lossy," especially when increasing
// brightness.  The tight timing in the WS2811/WS2812 code means there
// aren't enough free cycles to perform this scaling on the fly as data
// is issued.  So we make a pass through the existing color data in RAM
// and scale it (subsequent graphics commands also work at this
// brightness level).  If there's a significant step up in brightness,
// the limited number of steps (quantization) in the old data will be
// quite visible in the re-scaled version.  For a non-destructive
// change, you'll need to re-render the full strip data.  C'est la vie.
void Adafruit_NeoPixel::setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed.
  // This simplifies the actual scaling math later, allowing a fast
  // 8x8-bit multiply and taking the MSB.  'brightness' is a uint8_t,
  // adding 1 here may (intentionally) roll over...so 0 = max brightness
  // (color values are interpreted literally; no scaling), 1 = min
  // brightness (off), 255 = just below max brightness.
  uint8_t newBrightness = b + 1;
  if(newBrightness != brightness) { // Compare against prior value
    // Brightness has changed -- re-scale existing data in RAM
    uint8_t  c,
            *ptr           = pixels,
             oldBrightness = brightness - 1; // De-wrap old brightness value
    uint16_t scale;
    if(oldBrightness == 0) scale = 0; // Avoid /0
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    for(uint16_t i=0; i<numBytes; i++) {
      c      = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}
