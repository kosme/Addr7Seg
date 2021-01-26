#ifndef ADDR7SEG_H
#define ADDR7SEG_H

#include "Adafruit_NeoPixel.h"

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
  Adafruit_NeoPixel neoPix;
  const uint8_t numGRB[10] = {0b01101111, 0b00001100, 0b11100101, 0b10101101, 0b10001110, 0b10101011, 0b11101011, 0b00001101, 0b11101111, 0b10001111};
  uint8_t *setDEC;
  uint8_t _pin;
  uint16_t _numChips;
  void clearDigit(int digit);
  void setSeg(int digit, int seg, uint8_t brightness);
  void setSubSegment(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
};

#endif
