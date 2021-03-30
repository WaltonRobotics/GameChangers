#include "FastLED.h"

#define NUM_LEDS 60
#define LED_PIN 5

DEFINE_GRADIENT_PALETTE(idlePalette) {
  0,     255,  0,  0,   //black
255,   0,0,0 }; //full white

CRGBPalette16 idleGradient = idlePalette;

CRGB leds[NUM_LEDS];

int leftSideIndex = NUM_LEDS / 2; // 29 -> 0
int rightSideIndex = NUM_LEDS / 2 - 1; // 30 -> 59

void setup()
{
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );

  rainbowCycle(10);
  blackout();
}

// *** REPLACE FROM HERE ***
void loop() {
  // ---> here we call the effect function <---
  showIdleState(20);
}

void rainbowCycle(int SpeedDelay) {
    byte *c;
    uint16_t i, j;
  
    for(j=0; j<256; j++) {
      for(i=0; i< NUM_LEDS; i++) {
        c=Wheel(((i * 256 / NUM_LEDS) + j) & 255);
        setPixel(i, *c, *(c+1), *(c+2));
      }
      showStrip();
      delay(SpeedDelay);
    }  
}

void showIdleState(int SpeedDelay) {

  for(int i = 0; i < NUM_LEDS / 2; i++) {
    if(i >= leftSideIndex) {
      leds[i] = ColorFromPalette(idleGradient, round((float)(NUM_LEDS / 2 - 1 - i) / (NUM_LEDS / 2 - 1) * 255.0f));
    } else {
      leds[i] = CRGB::Black;
    }
  }

  for(int i = NUM_LEDS / 2; i < NUM_LEDS; i++) {
    if(i <= rightSideIndex) {
      leds[i] = ColorFromPalette(idleGradient, round((float)(i - NUM_LEDS / 2) / (NUM_LEDS / 2 - 1) * 255.0f));
    } else {
      leds[i] = CRGB::Black;
    }
  }

  showStrip();

  delay(SpeedDelay);

  leftSideIndex--;

  if (leftSideIndex < 0) {
    leftSideIndex = NUM_LEDS / 2 - 1;
  }
  
  rightSideIndex++; 

  if(rightSideIndex >= NUM_LEDS) {
    rightSideIndex = NUM_LEDS / 2;
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
 
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}

// ---> here we define the effect function <---
// *** REPLACE TO HERE ***

void showStrip() {
  FastLED.show();
}

void setPixel(int pixel, byte red, byte green, byte blue) {
 leds[pixel].r = red;
 leds[pixel].g = green;
 leds[pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

void blackout() {
  setAll(0, 0, 0);
}
