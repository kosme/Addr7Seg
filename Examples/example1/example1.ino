#include "Addr7Seg.h"

#define PIN             9             //The Pin that connects to the data line of the FIRST (Rightmost) digit.
#define NUMDIGITS       4             //Number of DIGITs Daisy Chained.
#define brightness 5

Addr7Seg display = Addr7Seg(PIN, NUMDIGITS, NEO_GRB + NEO_KHZ400);


void setup() {
  display.begin(); // Starts library.
  Serial.begin(115200);
}

uint8_t number = 0;

void loop() {
  number = 0;
  while (number < 10) {
    display.writeDigit(0, number, brightness);
    display.writeDigit(1, number, brightness);
    display.writeDigit(2, number, brightness);
    number++;
    display.show();
    delay(2000);
  }
}
