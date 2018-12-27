/*
 * To use this, you will need to plug an Addressable RGB LED
 * strip into pin 2.
 */

#include <PololuLedStrip.h>

// Create an ledStrip object and specify the pin it will use.
PololuLedStrip<2> ledStrip;

// Create a buffer for holding the colors (3 bytes per color).
#define LED_COUNT 30
rgb_color colors[LED_COUNT];

void setup()
{
  
}


void loop()
{
  // Update the colors.
  for(uint16_t i = 0; i < LED_COUNT; i++)
  {
    colors[i] = rgb_color(255, 69, 0); // Orange color rgb values
  }
  // Write the colors to the LED strip.
  ledStrip.write(colors, LED_COUNT);
  delay(2000);

  //Turn off LED's
  for(uint16_t i = 0; i < LED_COUNT; i++)
  {
    colors[i] = rgb_color(0, 0, 0);
  }
  // Write the colors to the LED strip.
  ledStrip.write(colors, LED_COUNT);
  delay(500);

  
}
