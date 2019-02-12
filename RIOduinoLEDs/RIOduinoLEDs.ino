#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    40

CRGB leds[NUM_LEDS];

byte ledmode = -1;
unsigned long ledmodeStart = 0;

uint8_t gHue = 0;

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  
  // initialize serial:
  Serial.begin(9600);
}

void loop() {

  if(ledmode == 1) {
    FastLED.setBrightness(255 / 2);
  }else {
    FastLED.setBrightness(255);
  }
  
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
      int s = 1500;
      int i = ((currentMillis % s) / (s / 2.0)) * NUM_LEDS / 2;
      if(i > NUM_LEDS / 2) {
        i = NUM_LEDS / 2 - (i - NUM_LEDS / 2);
      }
      for (int k=0; k<NUM_LEDS / 2; k++) {
        if(k == i || k == i - 1 || k == i + 1) {
          leds[k].r = 255;
          leds[k].g = 0;
          leds[k].b = 0;
        }else{ 
          leds[k].r = 0;
          leds[k].g = 0;
          leds[k].b = 0;
        }
      }
      
      for (int k=NUM_LEDS / 2; k<NUM_LEDS; k++) {
        if(k - NUM_LEDS / 2 == i || k - NUM_LEDS / 2 == i - 1 || k - NUM_LEDS / 2 == i + 1) {
          leds[k].r = 255;
          leds[k].g = 0;
          leds[k].b = 0;
        }else{ 
          leds[k].r = 0;
          leds[k].g = 0;
          leds[k].b = 0;
        }
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
    case 5: {
      // emergency
      if(currentMillis - ledmodeStart < 2000) {
        if(currentMillis % 400 > 200) {
          fill_solid(leds, NUM_LEDS, CRGB(0, 255, 0));
        }else {
          fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
        }
      }else{
          fill_solid(leds, NUM_LEDS, CRGB(0, 255, 0));
      }
      break;
    }
  }
  FastLED.show();

  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

String serialBuffer = "";

void serialEvent() {
  while (Serial.available()) {
    char readChar = Serial.read();
    serialBuffer += readChar; 
    if(readChar == '\n') {
      ledmode = serialBuffer.toInt();
      ledmodeStart = millis();
      serialBuffer = "";
      Serial.println("Ok");
    }
  }
}
