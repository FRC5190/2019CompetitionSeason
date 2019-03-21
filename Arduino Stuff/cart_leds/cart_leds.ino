#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    160

CRGB leds[NUM_LEDS];

uint8_t gHue = 0;

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  
  FastLED.setBrightness(255);
}

void loop() {
  fill_rainbow( leds, NUM_LEDS, gHue, 10);
  FastLED.show();

  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}
