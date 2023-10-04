#include "Wire.h"
#include <MPU9250_WE.h>
#include <BMx280I2C.h>

#include "helper_3dmath.h"

#include "OLED_Driver.h"
#include "GUI_paint.h"
#include "DEV_Config.h"
#include "Debug.h"
#include "ImageData.h"

UBYTE *BlackImage;

// extern "C" {
// #include <user_interface.h>
// }

#define LED_PIN LED_BUILTIN  //13
#define BUTTON_PIN 3

#define MPU9250_ADDR        0x68
#define BMP280_ADDRESS      0x76
#define SEA_LEVEL_PRESSURE  1013.25   // sea level pressure


/* There are several ways to create your MPU9250 object:
 * MPU9250_WE myMPU9250 = MPU9250_WE()              -> uses Wire / I2C Address = 0x68
 * MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR)  -> uses Wire / MPU9250_ADDR
 * MPU9250_WE myMPU9250 = MPU9250_WE(&wire2)        -> uses the TwoWire object wire2 / MPU9250_ADDR
 * MPU9250_WE myMPU9250 = MPU9250_WE(&wire2, MPU9250_ADDR) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
BMx280I2C bmx280(BMP280_ADDRESS);

// ------- VERSION
String version1 = "Version:";
String version2 = "2.0";
String version3 = "MPU9250 + BMP280";

// ------- VIEW BOX POSITIONS
// ---- Acceleration box
const int chartAccX = 1;
const int chartAccY = 16;
const int chartAccW = 80;
const int chartAccH = 20;
// ---- Roll chart box
const int chartRollX = 1;
const int chartRollY = 36;
const int chartRollW = 80;
const int chartRollH = 28;
const int chartRollWidth = chartRollW;

// --- Global vars
bool otaServer = true;
float rollOutput, pitchOutput;
float pressureOutput = 0.0;
float tempOutput = 0.0;
float altitudeOutput = 0.0;
const int chartTotalTime = 30;  // minutes

// --- Init display messages
int startMessageYPad = 0;

// -- Button state
byte lastButtonState = HIGH;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;
long buttonTimer = 0;
long longButtonPressTime = 300;
bool longPressActive = false;
bool buttonActive = false;

// -- Display state
int currentDispPresentation = 0;
int maxDispPresentation = 4;
int defaultDispPresentation = 1;
const byte numChars = 20;
const char dispNames[][numChars] = {"Roll chart", "Altitude", "Altitude chart", "Empty"};

// ------- INTRO MODE VARS
bool introMode = true;
int introlCurrLogo = 0;
unsigned long last_intro_time;
int intro_update_interval = 1000;


// ------ INSTALLATION ROLL CORRECTION
int rollCorrection = -1;
int pitchCorrection = 2;

// ------ COMPASS VARS
const char compass_queue_new[][10] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW", "N" };

// ----- Flags
bool compassReady = false;
bool blinkState = false;
bool CalibrationMode = false;
bool bmpMeasureFlag = false;
bool isBarometerReady = false;
bool serialDebug = false;
bool showDisplayName = true;

// ----- Timings
unsigned long last_disp_time;
unsigned long last_sensors_time;
int disp_update_interval = 500;
int sensors_update_interval = 50;
unsigned long last_compass_time;
int compass_update_interval = 100;
int compass_calibration_time = 30000;
int display_name_interval = 3000;
int display_name_time;
unsigned long last_chart_calc_time;
const int oneChartBarTimeInterval = (chartTotalTime * 60 / chartRollWidth) * 1000;

void showStartDisplayMessage(String message, bool clear = true, UWORD Ystart = 0) {
  if(clear) {
    startMessageYPad = Ystart + 0;
    Paint_Clear(BLACK);
  }

  Paint_DrawString(0, startMessageYPad, message, &Font8, 0, 0, BLACK, WHITE);

  startMessageYPad += int((message.length() * 5 / 128) + 1) * 9;

  OLED_1IN51_Display(BlackImage);
}

// ================================================================
// ===               SETUP ROUTINE                              ===
// ================================================================
void setup() {
  wificonfig_wifiOff();

  delay(1000);
  System_Init();
  delay(100);

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  
  delay(100);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println(">> INIT <<");

  initDisplay();

  delay(1000);

  showStartDisplayMessage(version1, false);
  delay(100);
  showStartDisplayMessage(version2, false);
  delay(100);
  showStartDisplayMessage(version3, false);
  delay(1000);

  showStartDisplayMessage("Init gyro...", true);
  
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
    showStartDisplayMessage("Error!", false);
    while(true) {
      delay(1000);
    }
  } else {
    Serial.println("MPU9250 is connected");
    delay(100);
    showStartDisplayMessage("Done!", false);
    delay(1000);
  }

  showStartDisplayMessage("Init AK8963 compass...", false);
  initAK8963();
  compassReady = true;
  Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  delay(100);
  showStartDisplayMessage("Done!", false);
  delay(1000);


  showStartDisplayMessage("Init barometer...", true);

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
    showStartDisplayMessage("Done!", false);
    delay(500);
	} else {
    isBarometerReady = false;
    
    delay(100);
    showStartDisplayMessage("Error!", false);
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    delay(1000);
  }

  currentDispPresentation = defaultDispPresentation;

  if(CalibrationMode == true)
  {
    showStartDisplayMessage("Start calibration!", true);
    delay(100);
    showStartDisplayMessage("Start calibration!", false);
    delay(100);
    showStartDisplayMessage("Start calibration!", false);
    delay(100);

    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  } else {
    showStartDisplayMessage("Apply calibration data!", false);
  
    applyMPUSettings();
    delay(500);

    Serial.println("Done!");
    showStartDisplayMessage("Init successful!", false);

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);

    delay(500);

    //////// create wifi AP for OTA
    if (otaServer) {
      createWifiAP();
    }
  }

  last_disp_time = millis();
  last_intro_time = millis();
  display_name_time = millis();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if(CalibrationMode == true)
  {
    showStartDisplayMessage("Calibration!", true);

    delay(1000);
    
    showStartDisplayMessage("Gyroscope", false);
    calibrateGyro();
    showStartDisplayMessage("Done", false);
    
    delay(1000);
    
    showStartDisplayMessage("Compass", false);
    magcalMPU9250();

    showStartDisplayMessage("Done", false);
    delay(1000);

    CalibrationMode = false;

    applyMPUSettings();
  } else {
    if (millis() - display_name_time >= display_name_interval) {  // Get the Latest packet
      display_name_time = millis();

      showDisplayName = false;
    }

    processButton();

    calcAcceleration();
    
    if (millis() - last_sensors_time >= sensors_update_interval) {  // Get the Latest packet
      last_sensors_time = millis();

      float temp = myMPU9250.getTemperature();
    
      calcPitchRoll();
      
      calcHeading();

      process_roll_chart_data();
      process_altitude_chart_data();

      if (millis() - last_chart_calc_time >= oneChartBarTimeInterval) {
        last_chart_calc_time = millis();

        calc_roll_chart_data();
        calc_altitude_chart_data();
      }

      if(isBarometerReady) {
        calcAltitude();
      }
    }

    if(millis() - last_disp_time >= disp_update_interval) {
      last_disp_time = millis();

      if(introMode) {
        showIntro();
      } else {
        updateDisplayData();
      }
    }
  }

  if (otaServer) {
    handleWifi();
  }
}

void processButton() {
  // Check button press
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    byte buttonState = digitalRead(BUTTON_PIN);
    
    if (buttonState != lastButtonState) {
      lastTimeButtonStateChanged = millis();
      lastButtonState = buttonState;

      if(buttonState == LOW) {
        if (buttonActive == false) {
          buttonActive = true;
          buttonTimer = millis();
        }
      } else if (buttonState == HIGH) {
        if (buttonActive == true) {
          buttonActive = false;
        }

        if(!longPressActive) {
          onSingleShortClick();
        }

        longPressActive = false;
      }
    }
  }

  if ((buttonActive == true) && (millis() - buttonTimer > longButtonPressTime) && (longPressActive == false)) {
    longPressActive = true;

    onSingleLongClick();
  }
}

void onSingleShortClick() {
  currentDispPresentation++;
  if(currentDispPresentation >= maxDispPresentation) {
    currentDispPresentation = 0;
  }

  display_name_time = millis();
  showDisplayName = true;
}

void onSingleLongClick() {
  nextAccelerationFilter();
}


void showIntro() {
  if (millis() - last_intro_time >= intro_update_interval) {
    introlCurrLogo++;

    if(introlCurrLogo == 1) {
      OLED_1IN51_Display_Array(gImage_RavLogo);
    } else if(introlCurrLogo = 2) {
      introMode = false;
    }
  }
}

void updateDisplayData() {
  Paint_Clear(BLACK);

  switch (currentDispPresentation) {
    case 0:
      updateDisplayData0();
      break;
    case 1:
      updateDisplayData1();
      break;
    case 2:
      updateDisplayData2();
      break;
    default:
      // nothing
      break;
  }

  if(showDisplayName) {
    updateBigDisplayName();
  }

  OLED_1IN51_Display(BlackImage);
}

void updateDisplayData0() {
  updateBigDisplayCompassData();
  updateBigDisplayRollChart();
  updateBigDisplayGyroPitchData();
  updateBigDisplayGyroRollData();
  updateBigDisplayRollMinMaxData();
  updateBigDisplayAccelerationData();
  
  Paint_DrawLine(82,0,82,63,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
  Paint_DrawLine(0,36,127,36,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
  Paint_DrawLine(0,18,82,18,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);

  if(serialDebug) {
    //Print the results
    Serial.print("Roll: ");
    Serial.print(int(rollOutput + 0.5));
    Serial.print("\t");

    Serial.print("Pitch: ");
    Serial.print(int(pitchOutput + 0.5));
    Serial.print("\t");
  
    // Serial.print("Heading: ");
    // Serial.print(headingDegrees);
    // Serial.print("\t");

    xyzFloat acc = getAccSmoothValues();

    Serial.print("Acc x: ");
    Serial.print(acc.x);
    Serial.print("\t");
    Serial.print("Acc y: ");
    Serial.print(acc.y);
    Serial.print("\t");


    Serial.println();
  }
}

void updateDisplayData1() {
  updateBigDisplayCompassData();
  updateBigDisplayGyroPitchData();
  updateBigDisplayGyroRollData();
  updateBigDisplayAccelerationData();
  updateBigDisplayBarometerData();
  
  Paint_DrawLine(82,0,82,63,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
  Paint_DrawLine(0,36,127,36,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
  Paint_DrawLine(0,18,82,18,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);

  if(serialDebug) {
    if(isBarometerReady) {
      Serial.print("Pressure: "); Serial.print(pressureOutput);Serial.print("\t");
      Serial.print("Alt: "); Serial.print(altitudeOutput);Serial.print("\t");
      Serial.println();
    }
  }
}

void updateDisplayData2() {
  updateBigDisplayCompassData();
  updateBigDisplayAltitudeChart();
  updateBigDisplayGyroPitchData();
  updateBigDisplayGyroRollData();
  updateBigDisplayAltMinMaxData();
  updateBigDisplayAccelerationData();
  
  Paint_DrawLine(82,0,82,63,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
  Paint_DrawLine(0,36,127,36,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
  Paint_DrawLine(0,18,82,18,WHITE,DOT_PIXEL_1X1,LINE_STYLE_DOTTED);
}

// ↑↓
void updateBigDisplayGyroRollData() {
  int boxX = 83;
  int boxY = 1;
  int boxW = 46;
  int boxH = 36;

  //Paint_DrawRectangle(boxX, boxY, boxX + boxW, boxY + boxH, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

  int arrow_head_w = 6;
  int roll_arrow_x = boxX + boxW - arrow_head_w - 1;
  int roll_arrow_height = 20;
  int roll_arrow_y = boxY + abs((boxH - roll_arrow_height)/2);

  int curr_roll_angle_int = int(rollOutput + 0.5);

  // draw number sign and black box
  String rollSign;
  float rollAlign = 0.5;
  if (curr_roll_angle_int > 0) {
    //Paint_DrawString(boxX + 2, boxY + boxH/2, "+", &Font16, 0, 0.5);
    rollSign = "+";
    rollAlign = 0;
  } else if (curr_roll_angle_int < 0) {
    //Paint_DrawString(boxX + 2, boxY + boxH/2, "-", &Font16, 0, 0.5);
    rollSign = "-";
    rollAlign = 0;
  }

  String rollValue = rollSign + String(abs(curr_roll_angle_int));

  Paint_DrawString(boxX  + boxW*rollAlign, boxY + boxH/2, rollValue, &Font16, rollAlign, 0.5);

  // draw roll arrow
  if (abs(curr_roll_angle_int) > 0) {
    Paint_DrawLine(roll_arrow_x, roll_arrow_y, roll_arrow_x, roll_arrow_y + roll_arrow_height, GRAY2, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    if (curr_roll_angle_int > 0) {
      Paint_DrawTriangle(roll_arrow_x, roll_arrow_y, 8, arrow_head_w, DOT_PIXEL_1X1, 1);
    } else {
      Paint_DrawTriangle(roll_arrow_x, roll_arrow_y + roll_arrow_height, 8, arrow_head_w, DOT_PIXEL_1X1, 0);
    }
  }
}

// ← →
void updateBigDisplayGyroPitchData() {
  int boxX = 83;
  int boxY = 36;
  int boxW = 46;
  int boxH = 28;

  //Paint_DrawRectangle(boxX, boxY, boxX + boxW, boxY + boxH, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

  int curr_pitch_angle_int = 1 * int(pitchOutput + 0.5);

  String pitchSign;
  float pitchAlign;

  if (curr_pitch_angle_int > 0) {
    pitchSign = '+';
    pitchAlign = 0.1;
  } else if (curr_pitch_angle_int < 0) {
    pitchSign = '-';
    pitchAlign = 0.9;
  } else {
    pitchSign = String("");
    pitchAlign = 0.5;
  }

  String pitchValue = pitchSign + String(abs(curr_pitch_angle_int));

  Paint_DrawString(boxX + boxW * pitchAlign, boxY + boxH/2 + 5, pitchValue, &Font16, pitchAlign, 0.5);

  // // draw pitch arrow
  if (abs(curr_pitch_angle_int) > 0) {
    int topY = boxY + 3;
    int pitchArrowY = topY + 10;
    int xGap = 3;

    int x1, x2, x3;
    int y1, y2, y3;
    int triDir;

    if (curr_pitch_angle_int < 0) {
      x1 = boxX + xGap;
      y1 = pitchArrowY;
      x2 = x1 + (pitchArrowY - topY);
      y2 = topY;
      x3 = x2 + 20;
      y3 = topY;
      triDir = 0;
    } else {
      x1 = boxX + boxW - xGap;
      y1 = pitchArrowY;
      x2 = x1 - (pitchArrowY - topY);
      y2 = topY;
      x3 = x2 - 20;
      y3 = topY;
      triDir = 1;
    }

    Paint_DrawLine(x1, y1, x2, y2, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x2, y2, x3, y3, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    
    Paint_DrawRightTriangle(x1, y1, 8, 2, DOT_PIXEL_1X1, triDir);
  }
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
  Paint_DrawString(9 + chartRollX + chartRollW/2, chartRollY + chartRollH/2 - 3, altitude, &Font16, 0.5, 0.5);
  // draw presseru value
  Paint_DrawString(9 + chartRollX + chartRollW/2, chartRollY + chartRollH - 1, String(pressureOutput, 0), &Font8, 0.5, 1);
  
  // draw temp value
  if(altitudeOutput > 999) {
    Paint_DrawString(chartRollX + 1, midMidY - 2, String(tempOutput, 0) + "C", &Font8, 0, 1);
  } else {
    Paint_DrawString(midMidX, midMidY - 2, String(tempOutput, 0) + "C", &Font8, 0.5, 1);
  }
  
}

void updateBigDisplayName() {
  String name = String(dispNames[currentDispPresentation]);
  String page = String(currentDispPresentation + 1) + "/" + maxDispPresentation;

  int width = 110, height = 40, border = 1;


  Paint_DrawRectangle(64 - width/2 - border, 32 - height/2 - border, 64 + width/2 + border, 32 + height/2 + border, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawRectangle(64 - width/2, 32 - height/2, 64 + width/2, 32 + height/2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  
  Paint_DrawString(64, 32 - 2, page, &Font12, 0.5, 1);
  Paint_DrawString(64, 32 + 2, name, &Font12, 0.5, 0);
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

  OLED_1IN51_Display_Array(gImage_ToyotaLogo);

  Serial.println("OLED_Init done");
}

void applyMPUSettings() {
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);

  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_5);

  applyGyroOffsets();
  applyAccOffsets();
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
