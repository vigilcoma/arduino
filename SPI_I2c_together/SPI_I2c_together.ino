#include "Wire.h"
#include <BMx280I2C.h>

#include "OLED_Driver.h"
#include "GUI_paint.h"
#include "DEV_Config.h"
#include "Debug.h"
#include "ImageData.h"

UBYTE *BlackImage;

#define BMP280_ADDRESS      0x76
#define SEA_LEVEL_PRESSURE  1013.25   // sea level pressure

BMx280I2C bmx280(BMP280_ADDRESS);

// --- Global vars
float pressureOutput = 0.0;
float tempOutput = 0.0;
float altitudeOutput = 0.0;

// ---- Roll chart box
const int chartRollX = 1;
const int chartRollY = 36;
const int chartRollW = 80;
const int chartRollH = 28;

bool bmpMeasureFlag = false;
bool isBarometerReady = false;


// ----- Timings
unsigned long last_disp_time;
unsigned long last_sensors_time;
int disp_update_interval = 500;
int sensors_update_interval = 50;


// ================================================================
// ===               SETUP ROUTINE                              ===
// ================================================================
void setup() {
  delay(1000);
  System_Init();
  delay(100);

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  
  delay(100);

  Serial.println(">> INIT <<");

  initDisplay();

  Serial.println("Init barometer...");

  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
	//and reads compensation parameters.
	if (bmx280.begin())
	{
    isBarometerReady = true;

    if (bmx280.isBME280())
      Serial.println("sensor is a BME280");
    else
      Serial.println("sensor is a BMP280");

    //reset sensor to default parameters.
    bmx280.resetToDefaults();

    //by default sensing is disabled and must be enabled by setting a non-zero
    //oversampling setting.
    //set an oversampling setting for pressure and temperature measurements. 
    bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
    bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
    bmx280.writeFilterSetting(BMx280MI::FILTER_x04);

    delay(100);
    Serial.println("Done!");
    delay(500);
	} else {
    isBarometerReady = false;
    
    delay(100);
    Serial.println("Error!");
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    delay(1000);
  }

  last_disp_time = millis();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (millis() - last_sensors_time >= sensors_update_interval) {  // Get the Latest packet
    last_sensors_time = millis();

    if(isBarometerReady) {
      calcAltitude();
    }
  }

  if(millis() - last_disp_time >= disp_update_interval) {
    last_disp_time = millis();

    updateDisplayData();
  }

}

void updateDisplayData() {
  Paint_Clear(BLACK);

  updateBigDisplayBarometerData();

  OLED_1IN51_Display(BlackImage);
}

void updateBigDisplayBarometerData() {
  String altitude = String(int(altitudeOutput + 0.5));
  altitude += "m";

  int leftStartX = chartRollX;
  int leftStartY = chartRollY + chartRollH;
  int leftMidX = chartRollX + 5;
  int leftMidY = chartRollY + chartRollH - 8;
  int leftEndX = chartRollX + 10;
  int leftEndY = chartRollY + chartRollH;

  int midStartX = leftEndX - 2;
  int midStartY = leftStartY - 4;
  int midMidX = leftEndX + 3;
  int midMidY = leftMidY - 5;
  int midEndX = leftEndX + 9;
  int midEndY = leftEndY;

  int rStartX = midEndX - 1;
  int rStartY = midEndY - 2;
  int rMidX = midEndX + 2;
  int rMidY = midEndY - 5;
  int rEndX = midEndX + 5;
  int rEndY = leftEndY;

  // draw mauntains icon
  Paint_DrawLine(leftStartX, leftStartY, leftMidX, leftMidY, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(leftMidX, leftMidY, leftEndX, leftEndY, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  Paint_DrawLine(midStartX, midStartY, midMidX, midMidY, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(midMidX, midMidY, midEndX, midEndY, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  Paint_DrawLine(midMidX - 2, midMidY + 5, midMidX + 2, midMidY + 5, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  Paint_DrawLine(rStartX, rStartY, rMidX, rMidY, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  Paint_DrawLine(rMidX, rMidY, rEndX, rEndY, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  // draw altitude value
  Paint_DrawString(64, 32 - 2, altitude, &Font24, 0.5, 1);
  // draw presseru value
  Paint_DrawString(64, 32 + 2, String(pressureOutput, 0) + " hPa", &Font12, 0.5, 0);
  
  // draw temp value
  Paint_DrawString(midMidX, midMidY - 2, String(tempOutput, 0) + "C", &Font8, 0.5, 1);
}

void initDisplay() {
  Serial.println(F("OLED_Init()...\r\n"));
  OLED_1IN51_Init();
  //Driver_Delay_ms(500); 
  OLED_1IN51_Clear(); 

  //0.Create a new image cache
  UWORD Imagesize = ((OLED_1IN51_WIDTH%8==0)? (OLED_1IN51_WIDTH/8): (OLED_1IN51_WIDTH/8+1)) * OLED_1IN51_HEIGHT;
  if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) { 
      Serial.print("Failed to apply for black memory...\r\n");
      return;
  }
  Paint_NewImage(BlackImage, OLED_1IN51_WIDTH, OLED_1IN51_HEIGHT, 270, BLACK);  

  //1.Select Image
  Paint_SelectImage(BlackImage);
  Paint_Clear(BLACK);

  Serial.println("OLED_Init done");
}

void calcAltitude() {
  if(!bmpMeasureFlag) {
    //start a measurement
    bmx280.measure();
    bmpMeasureFlag = true;
  }

  if(bmpMeasureFlag && bmx280.hasValue()) {
      bmpMeasureFlag = false;

      float pressure = bmx280.getPressure(); // in Si units for Pascal
      pressure /= 100.0F;

      float temp = bmx280.getTemperature();

      
      float altitude = 44330 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));

      if (!isnan(pressure)) {
        pressureOutput = 0.9*pressureOutput + 0.1*pressure;
      }

      if (!isnan(altitude)) {
        altitudeOutput = 0.9*altitudeOutput + 0.1*altitude;
      }

      if (!isnan(temp)) {
        tempOutput = 0.9*tempOutput + 0.1*temp;
      }
    }
}
