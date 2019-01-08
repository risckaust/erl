#include <FastLED.h>

// Number of LED in each arm
#define NUM_LEDS 1

// Pins where LEDs are connected (Arduino Micro)
#define DATA_PIN2 2 
#define DATA_PIN3 3
#define DATA_PIN4 4
#define DATA_PIN5 5

CRGB leds2[NUM_LEDS], leds3[NUM_LEDS], leds4[NUM_LEDS], leds5[NUM_LEDS];

void setup() {
  FastLED.addLeds<NEOPIXEL, DATA_PIN2>(leds2, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN3>(leds3, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN4>(leds4, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, DATA_PIN5>(leds5, NUM_LEDS);
}

void loop() {
  
  // Make LED oraneg for 0.5 sec
  leds2[0] = CRGB::OrangeRed;
  leds3[0] = CRGB::OrangeRed;
  leds4[0] = CRGB::OrangeRed;
  leds5[0] = CRGB::OrangeRed;
  FastLED.show();
  delay(500);

  // Turn LED off for 0.5 sec
  leds2[0] = CRGB::Black;
  leds3[0] = CRGB::Black;
  leds4[0] = CRGB::Black;
  leds5[0] = CRGB::Black;
  FastLED.show();
  delay(500);

}
