#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    20

CRGB leds[NUM_LEDS];

byte ledmode = 0;

uint8_t gHue = 0;

void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
  // initialize serial:
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();
  switch(ledmode) {
    case 0:
      // clear
      fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
      break;
    case 1:
      // disabled
      fill_rainbow( leds, NUM_LEDS, gHue, 10);
      break;
    case 2:
      // emergency
      if(currentMillis % 500 > 250) {
        fill_solid(leds, NUM_LEDS, CRGB(255, 0, 0));
      }else {
        fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
      }
      break;
    case 3: {
      // vision
      int i = ((currentMillis % 2000) / 1000.0) * NUM_LEDS / 2;
      if(i > NUM_LEDS / 2) {
        i = NUM_LEDS / 2 - (i - NUM_LEDS / 2);
      }
      for (int k=0; k<NUM_LEDS; k++) {
        leds[k].r = 0;
        if(k == i || k - NUM_LEDS / 2 == i) {
          leds[k].g = 255;
        }else{
          leds[k].g = 128;
        }
        leds[k].b = 0;
      }
      break;
    }
    case 4: {
      // emergency
      if(currentMillis % 500 > 250) {
        fill_solid(leds, NUM_LEDS, CRGB(255, 127, 0));
      }else {
        fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
      }
      break;
    }
  }
  FastLED.show();

  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

void serialEvent() {
  while (Serial.available()) {
    int data = Serial.read(); 
    if(isDigit(data)) {
      ledmode = data - '0';
      Serial.println("Ok");
    }
  }
}
