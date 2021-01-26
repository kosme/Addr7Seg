#include "Addr7Seg.h"
//#define DEBUG

Addr7Seg::Addr7Seg(uint8_t pin, uint8_t digits, neoPixelType t)
{
  neoPix = Adafruit_NeoPixel(digits, pin, t);
  _pin=pin;
  _numChips=digits*3;
  memset(setDEC, 0, digits);
  for (uint8_t x = 0; x < digits; x++) {
    setDEC[x] = 0;
  }
}

Addr7Seg::~Addr7Seg()
{
  neoPix.~Adafruit_NeoPixel();
  if(_pin >= 0) pinMode(_pin, INPUT);
  free(setDEC);
}

void Addr7Seg::begin(void)
{
  neoPix.begin();
}

void Addr7Seg::show(void) {
  neoPix.show();
}

void Addr7Seg::writeDigit(uint8_t digit, uint8_t num, uint8_t brightness)
{
  uint16_t firstChip = digit * 3;

  uint8_t BIT[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t SET[8] = {0, 0, 0, 0, 0, 0, 0, 0};
#ifdef DEBUG
  Serial.print(num);
  Serial.print(" ");
  Serial.println(numGRB[num], BIN);
#endif
  for (int x = 0; x < 8; x++) {

    BIT[x] = bitRead(numGRB[num], x);
    if (BIT[x]) {
      SET[x] = brightness;
    }
#ifdef DEBUG
    Serial.print(SET[x]);
    Serial.print(", ");
#endif
  }
  if (setDEC[digit] == 1) {
    SET[4] = brightness;
  }
  else {
    SET[4] = 0;
  }
#ifdef DEBUG
  Serial.println();
#endif
  setSubSegment(firstChip++, SET[0], SET[1], SET[2]);
  setSubSegment(firstChip++, SET[3], SET[4], SET[5]);
  setSubSegment(firstChip++, SET[6], SET[7], SET[8]);
}

void Addr7Seg::slideDown(int digit) {
  clearDigit(digit);
  setSeg(digit, 0, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 7, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 5, 100);
  show();
  delay(100);
}

void Addr7Seg::clearDigit(int digit) {
  int firstChip = digit * 3;
  setSubSegment(firstChip++, 0, 0, 0);
  setSubSegment(firstChip++, 0, 0, 0);
  setSubSegment(firstChip++, 0, 0, 0);
}

void Addr7Seg::setSeg(int digit, int seg, uint8_t brightness)
{
  uint8_t offset = seg/3;
  switch(seg%3)
  {
    case 0:
      setSubSegment((digit * 3) + offset, brightness, 0, 0);
      break;
    case 1:
      setSubSegment((digit * 3) + offset, 0, brightness, 0);
      break;
    case 2:
      setSubSegment((digit * 3) + offset, 0, 0, brightness);
      break;
  }
}

void Addr7Seg::changeDEC(int digit)
{
  if (setDEC[digit] == 0) {
    setDEC[digit] = 1;
  }
  else {
    setDEC[digit] = 0;
  }
}

void Addr7Seg::slideUp(int digit)
{
  clearDigit(digit);
  setSeg(digit, 5, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 7, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 0, 100);
  show();
  delay(100);
}

void Addr7Seg::rotateCW(int digit)
{
  clearDigit(digit);
  setSeg(digit, 0, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 2, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 3, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 5, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 6, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 1, 100);
  show();
  delay(100);
}

void Addr7Seg::rotateCCW(int digit)
{
  clearDigit(digit);
  setSeg(digit, 0, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 1, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 6, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 5, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 3, 100);
  show();
  delay(100);
  clearDigit(digit);
  setSeg(digit, 2, 100);
  show();
  delay(100);
}

void Addr7Seg::pulseNum(int digit, int num, uint8_t maxb)
{
  int    brightness = 10;
  int     rate = 5;
  while (brightness > 0) {
    if (brightness > maxb) {
      rate = rate * -1;
    }
    writeDigit(digit, num, brightness);
    delay(10);
    brightness += rate;
    show();
  }
  writeDigit(digit, num, 0);
  show();
}

void Addr7Seg::setSubSegment(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
    neoPix.setPixelColor(n, r, g, b);
}
