#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
VL53L0X sensor;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();

  delay(1000);

  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // LONG RANGE ///////////
  //////////////////////////
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // text display big!
  display.setTextSize(1); 
  display.setTextColor(WHITE);
}

void loop() {
  int distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) {

  } else {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(distance/10);
    display.print("sm");
    display.display();
  }
}
