#ifndef ADDR7SEG_H
#define ADDR7SEG_H

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

#ifdef __AVR__
#include <avr/power.h>
#endif

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B   // HEX representation
#define NEO_RGB  ((0 << 6) | (0 << 4) | (1 << 2) | (2)) // 0x06
#define NEO_RBG  ((0 << 6) | (0 << 4) | (2 << 2) | (1)) // 0x09
#define NEO_GRB  ((1 << 6) | (1 << 4) | (0 << 2) | (2)) // 0x52
#define NEO_GBR  ((2 << 6) | (2 << 4) | (0 << 2) | (1)) // 0xA1
#define NEO_BRG  ((1 << 6) | (1 << 4) | (2 << 2) | (0)) // 0x58
#define NEO_BGR  ((2 << 6) | (2 << 4) | (1 << 2) | (0)) // 0xA4

// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B   // HEX representation
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3)) // 0x1B
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2)) // 0x1E
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3)) // 0x27
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2)) // 0x36
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1)) // 0x2D
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1)) // 0x39

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3)) // 0x4B
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2)) // 0x4E
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3)) // 0x87
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2)) // 0xC6
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1)) // 0x8D
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1)) // 0xC9

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3)) // 0x63
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2)) // 0x72
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3)) // 0x93
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2)) // 0xD2
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1)) // 0xB1
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1)) // 0xE1

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0)) // 0x6C
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0)) // 0x78
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0)) // 0x9C
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0)) // 0xD8
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0)) // 0xB4
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0)) // 0xE4

// Add NEO_KHZ400 to the color order value to indicate a 400 KHz
// device.  All but the earliest v1 NeoPixels expect an 800 KHz data
// stream, this is the default if unspecified.  Because flash space
// is very limited on ATtiny devices (e.g. Trinket, Gemma), v1
// NeoPixels aren't handled by default on those chips, though it can
// be enabled by removing the ifndef/endif below -- but code will be
// bigger.  Conversely, can disable the NEO_KHZ400 line on other MCUs
// to remove v1 support and save a little space.

#define NEO_KHZ800 0x0000 // 800 KHz datastream
#ifndef __AVR_ATtiny85__
#define NEO_KHZ400 0x0100 // 400 KHz datastream
#endif

// If 400 KHz support is enabled, the third parameter to the constructor
// requires a 16-bit value (in order to select 400 vs 800 KHz speed).
// If only 800 KHz is enabled (as is default on ATtiny), an 8-bit value
// is sufficient to encode pixel color order, saving some space.

#ifdef NEO_KHZ400
typedef uint16_t neoPixelType;
#else
typedef uint8_t  neoPixelType;
#endif

class Addr7Seg {

public:
  Addr7Seg(uint8_t pin, uint8_t digits, neoPixelType t);
  ~Addr7Seg(void);
  void begin(void);
  void show(void);
  void writeDigit(uint8_t digit, uint8_t num, uint8_t brightness);
  void slideDown(int digit);
  void changeDEC(int digit);
  void slideUp(int digit);
  void rotateCW(int digit);
  void rotateCCW(int digit);
  void pulseNum(int digit, int num, uint8_t maxb);

private:
  const uint8_t numGRB[10] = {0b01101111, 0b00001100, 0b11100101, 0b10101101, 0b10001110, 0b10101011, 0b11101011, 0b00001101, 0b11101111, 0b10001111};
  uint8_t wOffset;
  uint8_t rOffset;
  uint8_t gOffset;
  uint8_t bOffset;
  #ifdef NEO_KHZ400
  boolean is800KHz = false;
  #endif
  uint8_t *setDEC;
  boolean _begun = false;
  uint8_t _pin;
  uint8_t _digits;
  uint16_t _numChips;
  uint16_t _numBytes;      // Size of 'pixels' buffer below (3 or 4 bytes/pixel)
  uint32_t _endTime;       // Latch timing reference
  uint8_t *pixels;
  #ifdef __AVR__
    volatile uint8_t
      *port;         // Output PORT register
    uint8_t
      pinMask;       // Output PORT bitmask
  #endif
  inline bool canShow(void) { return (micros() - _endTime) >= 300L; }
  void clearDigit(int digit);
  void setSeg(int digit, int seg, uint8_t brightness);
  void updateLength(uint16_t n);
  void setSubSegment(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  void updateType(neoPixelType t);
  void setPin(uint8_t p);
};

#endif
