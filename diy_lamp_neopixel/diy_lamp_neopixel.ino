#include <Adafruit_NeoPixel.h>

#define LED_PIN 0

#define COOLING  40           // defines the level at which the lighting effect fades before a new "flame" generates
#define SPARKING 100          // defines the rate of flicker which we will see from the flame animation

#define COLOR_BUTTON 2
#define SHIFT_BUTTON 5
#define UP_BUTTON 12
#define DOWN_BUTTON 4

#define LED_COUNT 32
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRBW);

uint32_t relaxWhite = strip.gamma32(strip.Color(255,190,0,255/2)); // strip.ColorHSV(65536*(50/360), 0.6 * 255, 255);
uint32_t readWhite = strip.gamma32(strip.Color(255,190,50,160)); //strip.ColorHSV(65536*(47/360), 0.3 * 255, 255);
uint32_t middleWhite = strip.gamma32(strip.Color(255,195,146,200)); // strip.ColorHSV(65536*(27/360), 0.43 * 225, 255);
uint32_t concetrateWhite = strip.gamma32(strip.Color(255,217,200,220)); //strip.ColorHSV(65536*(29/360), 0.31 * 255, 255);
uint32_t energizeWhite = strip.gamma32(strip.Color(242,242,255,255)); //strip.ColorHSV(65536*(41/360), 0.028 * 255, 255);

uint32_t red_hh = strip.ColorHSV(2000);
uint32_t red_h = strip.ColorHSV(1000);
uint32_t red = strip.ColorHSV(0);
uint32_t red_l = strip.ColorHSV(65536 - 1000);
uint32_t red_ll = strip.ColorHSV(65536 - 2000);

uint32_t green_hh = strip.ColorHSV(65536 / 3 + 2000);
uint32_t green_h = strip.ColorHSV(65536 / 3 + 1000);
uint32_t green = strip.ColorHSV(65536 / 3);
uint32_t green_l = strip.ColorHSV(65536 / 3 - 1500);
uint32_t green_ll = strip.ColorHSV(65536 / 3 - 3000);

uint32_t blue_hh = strip.ColorHSV(65536 * 2 / 3 + 4000);
uint32_t blue_h = strip.ColorHSV(65536 * 2 / 3 + 2000);
uint32_t blue = strip.ColorHSV(65536 * 2 / 3);
uint32_t blue_l = strip.ColorHSV(65536 * 2 / 3 - 2000);
uint32_t blue_ll = strip.ColorHSV(65536 * 2 / 3 - 4000);

uint32_t colors_set[4][5] = {{relaxWhite, readWhite, middleWhite, concetrateWhite, energizeWhite}, {red_hh, red_h, red, red_l, red_ll}, {green_hh, green_h, green, green_l, green_ll}, {blue_hh, blue_h, blue, blue_l, blue_ll}};

int colorsLength = sizeof(colors_set) / sizeof(colors_set[0]);
int colorIndex = 0;

int effectsIndex = 3;

bool isShiftMode = false;
bool lightUpdates = true;
bool isEffectMode = false;
int currentEffectTime = 0;

uint32_t currentColor = middleWhite;
int currentColorSubIndex = 2;
int maxSubColors = 5;

int maxBrightness = 255;
int minBrightness = 5;
int brightness = 191;
int brightnessStep = 32;

int colorButtonState = -1;
int colorButtonStatePrev = 0;

int shiftButtonState = -1;
int shiftButtonStatePrev = 0;

int upButtonState = 0;
int upButtonStatePrev = 0;

int downButtonState = 0;
int downButtonStatePrev = 0;

void setup() {
  strip.begin();

  pinMode(COLOR_BUTTON, INPUT_PULLUP);
  pinMode(SHIFT_BUTTON, INPUT_PULLUP);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);

  colorButtonState = digitalRead(COLOR_BUTTON);
  shiftButtonState = digitalRead(SHIFT_BUTTON);
  upButtonState = digitalRead(UP_BUTTON);
  downButtonState = digitalRead(DOWN_BUTTON);

  shiftButtonStatePrev = shiftButtonState;
  colorButtonStatePrev = colorButtonState;
  upButtonStatePrev = upButtonState;
  downButtonStatePrev = downButtonState;

  Serial.begin(9600);

  delay(1000);
  Serial.println("Lamp start");
}

void loop() {

  checkShiftButton();
  checkColorButton();
  checkUpButton();
  checkDownButton();

  if (lightUpdates == true) {
    lightUpdates = false;

    strip.setBrightness(brightness);
    colorSet(currentColor);
    strip.show();
  } else if (isEffectMode == true) {
    switch (effectsIndex) {
      case 0  : {
          Fire(55, 120, 30);
          break;
        }

      case 1  : {
          rainbowCycle(40);
          break;
        }

      case 2  : {
          colorWipeCycleSmooth(50);
          break;
        }

      case 3  : {
          colorWipeCycleSmoothParallel(50);
          break;
        }

    }
  }
}

void demoColor() {
  static int buttonLastTime = 0;

  if (millis() - buttonLastTime > 1000) {
    buttonLastTime = millis();

    moveColor(true);
  }
}

void checkShiftButton() {
  static int buttonLastTime = 0;

  if (millis() - buttonLastTime > 100) {
    buttonLastTime = millis();

    shiftButtonState = digitalRead(SHIFT_BUTTON);

    if (shiftButtonState != shiftButtonStatePrev) {
      if (shiftButtonState == LOW) {
        isShiftMode = true;
      } else {
        isShiftMode = false;
      }

      shiftButtonStatePrev = shiftButtonState;
    }
  }
}

void checkColorButton() {
  static int buttonLastTime = 0;

  if (millis() - buttonLastTime > 100) {
    buttonLastTime = millis();

    colorButtonState = digitalRead(COLOR_BUTTON);

    if (colorButtonState != colorButtonStatePrev) {
      if (colorButtonState == LOW) {
        chooseNextColor();
      }

      colorButtonStatePrev = colorButtonState;
    }
  }
}

void checkUpButton() {
  static int buttonLastTime = 0;

  if (millis() - buttonLastTime > 100) {
    buttonLastTime = millis();

    upButtonState = digitalRead(UP_BUTTON);

    if (upButtonState != upButtonStatePrev) {
      if (upButtonState == LOW) {
        if (isShiftMode) {
          moveColor(true);
        } else {
          moveBrightness(true);
        }
      }

      upButtonStatePrev = upButtonState;
    }
  }
}

void checkDownButton() {
  static int buttonLastTime = 0;
  if (millis() - buttonLastTime > 100) {
    buttonLastTime = millis();

    downButtonState = digitalRead(DOWN_BUTTON);

    if (downButtonState != downButtonStatePrev) {
      if (downButtonState == LOW) {
        if (isShiftMode) {
          moveColor(false);
        } else {
          moveBrightness(false);
        }
      }

      downButtonStatePrev = downButtonState;
    }
  }
}

void chooseNextColor() { // From NeoPixel Library
  Serial.print("chooseNextColor, shift:"); Serial.println(isShiftMode);

  if (isShiftMode) {
    if (isEffectMode) {
      // select next effect only if we have already in effects mode
      effectsIndex++;
      if (effectsIndex > 4) {
        effectsIndex = 0;
      }
    }
    isEffectMode = true;

    Serial.println("effect");
    Serial.println(effectsIndex);
  } else {
    lightUpdates = true;

    if (!isEffectMode) {
      colorIndex++;
      if (colorIndex >= colorsLength) {
        colorIndex = 0;
      }
    }

    isEffectMode = false;

    currentColorSubIndex = 2;
    maxSubColors = 5;
    currentColor = colors_set[colorIndex][currentColorSubIndex];

    Serial.print(colorIndex); Serial.print(" : "); Serial.println(currentColorSubIndex);
  }
}

void moveBrightness(bool dir) {
  lightUpdates = true;

  Serial.println("moveBrightness");
  Serial.println(dir);
  if (dir) {
    if (brightness == minBrightness) {
      brightness = brightnessStep;
    } else {
      brightness += brightnessStep;
    }
  } else {
    brightness -= brightnessStep;
  }

  if (brightness > maxBrightness) {
    brightness = maxBrightness;
  } else if (brightness < minBrightness) {
    brightness = minBrightness;
  }
}

void moveColor(bool dir) {
  lightUpdates = true;
  if (dir) {
    currentColorSubIndex++;
  } else {
    currentColorSubIndex--;
  }

  if (currentColorSubIndex >= maxSubColors) {
    currentColorSubIndex = 0; // maxSubColors - 1;
  } else if (currentColorSubIndex < 0) {
    currentColorSubIndex = maxSubColors - 1; // 0
  }

  currentColor = colors_set[colorIndex][currentColorSubIndex];
}

void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[LED_COUNT];
  int cooldown;
  static int lastEffectTime = millis();
  static const int halfLedCount = LED_COUNT / 2;

  if (millis() - lastEffectTime >= SpeedDelay) {
    // Step 1.  Cool down every cell a little
    for ( int i = 0; i < halfLedCount; i++) {
      cooldown = random(0, ((Cooling * 10) / halfLedCount) + 2);

      if (cooldown > heat[i]) {
        heat[i] = 0;
      } else {
        heat[i] = heat[i] - cooldown;
      }
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for ( int k = halfLedCount - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if ( random(255) < Sparking ) {
      int y = random(7);
      heat[y] = heat[y] + random(100, 200);
      //heat[y] = random(160,255);
    }

    // Step 3.1.  Copy half led to another
    for ( int j = halfLedCount - 1; j >= 0; j--) {
      heat[LED_COUNT - j - 1] = heat[j];
    }

    // Step 4.  Convert heat to LED colors
    for ( int j = 0; j < LED_COUNT; j++) {
      setPixelHeatColor(j, heat[j]);
    }

    strip.show();
    lastEffectTime = millis();
  }
}

void rainbowCycle(int SpeedDelay) {
  byte *c;
  uint16_t i, j;
  static int lastEffectTime = millis();
  static int cyclesIterator = 0;

  if (millis() - lastEffectTime >= SpeedDelay) {
    for (i = 0; i < LED_COUNT; i++) {
      c = Wheel(((i * 256 / LED_COUNT) + cyclesIterator) & 255);
      setPixel(i, *c, *(c + 1), *(c + 2));
    }
    strip.show();
    lastEffectTime = millis();

    cyclesIterator++;
    if (cyclesIterator >= 256 * 5) {
      cyclesIterator = 0;
    }
  }
}

void colorWipe(int SpeedDelay) {
  static int lastEffectTime = 0;
  static int startIndex = 0;
  static int endIndex = 0;
  static uint32_t color;
  static bool update = true;

  if (millis() - lastEffectTime >= SpeedDelay) {
    if (startIndex == 0 && endIndex == 0) {
      color = strip.Color(random(255), random(255), random(255));
    }

    if (endIndex < LED_COUNT) {
      endIndex++;
      update = true;
    } else {
      if (startIndex < LED_COUNT - 1) {
        startIndex++;
        update = true;
      } else {
        endIndex = 0;
        startIndex = 0;
        update = false;
      }
    }

    if (update) {
      clearStrip();
      strip.fill(color, startIndex, endIndex - startIndex);
      strip.show();
    }


    lastEffectTime = millis();
  }
}

void colorWipeCycleSmooth(int SpeedDelay) {
  static const int tail = 10;
  static const int middle = 10;
  static const int Q = LED_COUNT * 2 + middle + tail * 2;

  static uint32_t ledArr[LED_COUNT];

  static int lastEffectTime = 0;
  static int iteration = 0;
  static int color = random(65536);
  static int saturation = random(120, 255);


  if (millis() - lastEffectTime >= SpeedDelay) {
    //Serial.print("Q:");Serial.print(Q);Serial.print(" | ");
    //Serial.print("it:");Serial.print(iteration);Serial.print(" | ");

    if (iteration <= Q) {
      const int toEnd = Q - iteration;
      int pixelNumber = 0;
      int i;

      //Serial.print("toEnd:");Serial.print(toEnd);Serial.print(" | ");

      if (toEnd < LED_COUNT + tail) {
        for (i = iteration; i > iteration - LED_COUNT; i--) {
          if (i > Q - LED_COUNT) {
            ledArr[pixelNumber] = strip.ColorHSV(0, 0, 0);
          } else if (i > Q - LED_COUNT - tail) {
            const int left = Q - LED_COUNT - i + 1;
            const float progress = 100 * left / (tail + 1);
            const int br = brightness * progress / 100;
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, br);
          } else {
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, brightness);
          }
          pixelNumber++;
        }
      } else {
        for (i = iteration; i > iteration - LED_COUNT; i--) {
          if (i <= 0) {
            ledArr[pixelNumber] = strip.ColorHSV(0, 0, 0);
          } else if (i <= tail) {
            const float progress = 100 * i / (tail + 1);
            const int br = brightness * progress / 100;
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, br);
          } else if (i > tail) {
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, brightness);
          }
          pixelNumber++;
        }
      }

      iteration++;

      colorSetArray(ledArr);
      strip.show();
    } else {
      iteration = 0;
      color = random(65536);
      saturation = random(120, 255);
    }

    // Serial.println("=");

    lastEffectTime = millis();
  }
}

void colorWipeCycleSmoothParallel(int SpeedDelay) {
  static const int tail = 10;
  static const int middle = 10;
  static const int halfLedCount = LED_COUNT / 2;

  static const int Q = halfLedCount * 2 + middle + tail * 2;

  static uint32_t ledArr[LED_COUNT];

  static int lastEffectTime = 0;
  static int iteration = 0;
  static int color = random(65536);
  static int saturation = random(120, 255);


  if (millis() - lastEffectTime >= SpeedDelay) {
    //Serial.print("Q:");Serial.print(Q);Serial.print(" | ");
    Serial.print("it:"); Serial.print(iteration); Serial.print(" | ");

    if (iteration <= Q) {
      const int toEnd = Q - iteration;
      int pixelNumber = 0;
      int i;

      //Serial.print("toEnd:");Serial.print(toEnd);Serial.print(" | ");

      if (toEnd < halfLedCount + tail) {
        for (i = iteration; i > iteration - halfLedCount; i--) {
          if (i > Q - halfLedCount) {
            ledArr[pixelNumber] = strip.ColorHSV(0, 0, 0);
          } else if (i > Q - halfLedCount - tail) {
            const int left = Q - halfLedCount - i + 1;
            const float progress = 100 * left / (tail + 1);
            const int br = brightness * progress / 100;
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, br);
          } else {
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, brightness);
          }
          pixelNumber++;
        }
      } else {
        for (i = iteration; i > iteration - halfLedCount; i--) {
          if (i <= 0) {
            ledArr[pixelNumber] = strip.ColorHSV(0, 0, 0);
          } else if (i <= tail) {
            const float progress = 100 * i / (tail + 1);
            const int br = brightness * progress / 100;
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, br);
          } else if (i > tail) {
            ledArr[pixelNumber] = strip.ColorHSV(color, saturation, brightness);
          }
          pixelNumber++;
        }
      }

      // Copy half led to another
      for ( int j = halfLedCount - 1; j >= 0; j--) {
        ledArr[LED_COUNT - j - 1] = ledArr[j];
      }

      iteration++;

      colorSetArray(ledArr);
      strip.show();
    } else {
      iteration = 0;
      color = random(65536);
      saturation = random(120, 255);
    }

    Serial.println("=");

    lastEffectTime = millis();
  }
}

// used by rainbowCycle and theaterChaseRainbow
byte * Wheel(byte WheelPos) {
  static byte c[3];

  if (WheelPos < 85) {
    c[0] = WheelPos * 3;
    c[1] = 255 - WheelPos * 3;
    c[2] = 0;
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    c[0] = 255 - WheelPos * 3;
    c[1] = 0;
    c[2] = WheelPos * 3;
  } else {
    WheelPos -= 170;
    c[0] = 0;
    c[1] = WheelPos * 3;
    c[2] = 255 - WheelPos * 3;
  }

  return c;
}

void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature / 255.0) * 191);

  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  // figure out which third of the spectrum we're in:
  if ( t192 > 0x80) {                    // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else if ( t192 > 0x40 ) {            // middle
    setPixel(Pixel, 255, heatramp, 0);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);
  }
}

// Set a LED color (not yet visible)
void setPixel(int Pixel, byte red, byte green, byte blue) {
  strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

void colorSet(uint32_t color) {
  strip.fill(color, 0, strip.numPixels() - 1);
}

void clearStrip() {
  strip.fill(strip.Color(0, 0, 0));
}

void setAll(byte red, byte green, byte blue) {
  for (int i = 0; i < LED_COUNT; i++ ) {
    setPixel(i, red, green, blue);
  }
}

void colorSetArray(uint32_t colors[]) {
  for ( int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, colors[i]);
  }
}
