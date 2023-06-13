// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <HMC5883L.h>
#include <Adafruit_SSD1306.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

extern "C" {
#include <user_interface.h>
}

// instantiation
MPU6050 mpu;
//Adafruit_SSD1306 displaySmall = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_SSD1306 displayBig = Adafruit_SSD1306(128, 64, &Wire);

HMC5883L compass;

MDNSResponder mdns;
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

///////////
////VERSION
///////////

String version = "Version:\n0.9\nchart";


///////////
////INSTALLATION CORRECTION
///////////
int rollCorrection = 17;

//////////////
/// COMPASS VARS
/////////////////////////

// compass calibration data
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

const char compass_queue_new[][10] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"};

unsigned long last_compass_time;
int compass_update_interval = 100;
int compass_calibration_time = 30000;

bool compassReady = false;
bool compassCalibration = false;
bool compassCalibrationDone = false;

//// ::END OF COMPASS VARS

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define LED_PIN LED_BUILTIN  //13
#define INTERRUPT_PIN D4     //D8  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//
int init_error_code = -1;

unsigned long last_disp_time;
int disp_update_interval = 500;


// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
int acc_counter = 0;
float acc_yaw_rad = 0;
float acc_pitch_rad = 0;
float acc_roll_rad = 0;

float curr_yaw_rad = 0;
float curr_pitch_rad = 0;
float curr_roll_rad = 0;

float curr_yaw_angle = 0;
float curr_pitch_angle = 0;
float curr_roll_angle = 0;
float curr_roll_angle_smooth = 0;

// data in radians
void set_last_read_angle_data(float x, float y, float z) {
  acc_yaw_rad += x;
  acc_pitch_rad += y;
  acc_roll_rad += z;

  acc_counter += 1;
}

void calc_angle_data() {
  curr_yaw_rad = acc_yaw_rad / acc_counter;
  curr_pitch_rad = acc_pitch_rad / acc_counter;
  curr_roll_rad = acc_roll_rad / acc_counter;

  curr_yaw_angle = curr_yaw_rad * 180 / M_PI;
  curr_pitch_angle = curr_pitch_rad * 180 / M_PI;
  curr_roll_angle = curr_roll_rad * 180 / M_PI;

  acc_yaw_rad = curr_yaw_rad;
  acc_pitch_rad = curr_pitch_rad;
  acc_roll_rad = curr_roll_rad;

  curr_roll_angle_smooth = 0.8 * curr_roll_angle_smooth + 0.2 * (curr_roll_angle + rollCorrection);

  acc_counter = 1;
  // Serial.print("curr_yaw_rad=");Serial.print(curr_yaw_rad);
  // Serial.print(" curr_pitch_rad=");Serial.print(curr_pitch_rad);
  // Serial.print(" curr_roll_rad=");Serial.println(curr_roll_rad);
  // Serial.print("curr_yaw_angle=");Serial.print(curr_yaw_angle);
  // Serial.print(" curr_pitch_angle=");Serial.print(curr_pitch_angle);
  // Serial.print(" curr_roll_angle=");Serial.println(curr_roll_angle);
}

//////////////
/// ELEVATION CHART VARS
/////////////////////////

const int chartWidth = 60;
const int chartTotalTime = 5;  // minutes
const int oneChartBarTimeInterval = (chartTotalTime * 60 / chartWidth) * 1000 / 2;
float chartBars[chartWidth];
int chartBarsPointer = 0;
float maxChartValue = 0.00;
int maxChartValueIndex = 0;
float minChartValue = 1000.00;
int minChartValueIndex = 0;
boolean chartBarCycle = false;
float curr_roll_angle_smooth_accum = 0.00;
int curr_roll_angle_smooth_counter = 0;
unsigned long last_chart_calc_time;

///END:: ELEVATION CHART VARS

void calc_elevation_chart_data() {
  curr_roll_angle_smooth_accum += curr_roll_angle_smooth;
  curr_roll_angle_smooth_counter++;

  if (millis() - last_chart_calc_time >= oneChartBarTimeInterval) {
    last_chart_calc_time = millis();

    float value = (curr_roll_angle_smooth_accum / curr_roll_angle_smooth_counter); //(M_PI / 180) * 
    bool update = false;

    //Serial.print(" value=");Serial.println(value);

    chartBars[chartBarsPointer] = value;

    if(chartBarCycle) {
      if(chartBarsPointer == maxChartValueIndex || chartBarsPointer == minChartValueIndex) {
        maxChartValue = chartBars[0];
        minChartValue = chartBars[0];

        for (int i = 0; i < chartWidth; i++) {
          if(chartBars[i] > maxChartValue) {
            maxChartValue = chartBars[i];
            maxChartValueIndex = chartBarsPointer;
          }

          if(chartBars[i] < minChartValue) {
            minChartValue = chartBars[i];
            minChartValueIndex = chartBarsPointer;
          }
        }

        update = true;
      }
    }
    
    if(value > maxChartValue) {
      maxChartValue = value;
      maxChartValueIndex = chartBarsPointer;

      update = true;
    }

    if(value < minChartValue) {
      minChartValue = value;
      minChartValueIndex = chartBarsPointer;
      
      update = true;
    }

    if(update) {
      Serial.print(" minChartValue=");Serial.print(minChartValue);
      Serial.print(" maxChartValue=");Serial.println(maxChartValue);
    }

    //Serial.print(" chartBarsPointer=");Serial.println(chartBarsPointer);
    //Serial.print(" bar=");Serial.println(chartBars[chartBarsPointer]);
    

    if (chartBarsPointer >= chartWidth - 1) {
      chartBarCycle = true;
      chartBarsPointer = 0;
    } else {
      chartBarsPointer++;
    }

    //Serial.print(" chartBarsPointerNext=");Serial.println(chartBarsPointer);

    curr_roll_angle_smooth_accum = 0;
    curr_roll_angle_smooth_counter = 0;
  }
}

// Compass average
int acc_compas_counter = 0;
float acc_head = 0;
float acc_head_prev = 0;
int curr_head = 0;

void set_last_read_compas_data(float h) {
  // if(abs(h - acc_head_prev) > 90) {
  //   acc_head = h;
  //   acc_compas_counter = 1;
  // } else {
  //   acc_head += h;
  //   acc_compas_counter += 1;
  // }

  // acc_head_prev = h;


  acc_head = h;
  acc_compas_counter = 1;
}

void calc_compass_average_data() {
  curr_head = int((acc_head / acc_compas_counter) + 0.5);

  acc_head = curr_head;

  acc_compas_counter = 1;
}

void wificonfig_wifiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  delay(100);
}

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF
bool otaServer = true;
int otaServerExpireTime = 1000 * 60 * 5; //5 mins


void wificonfig_wifiOff() {
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
  delay(100);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
ICACHE_RAM_ATTR void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               SETUP ROUTINE                              ===
// ================================================================
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  wificonfig_wifiOff();

  // initialize serial communication
  Serial.begin(38400);
  delay(100);

  // init big display
  if (displayBig.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayBig.display();
  } else {
    Serial.println(F("SSD1306 allocation failed - big"));
    init_error_code = 2;
  }

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // init displays
  configDisplays();

  showStartDisplayMessage("Init...");
  delay(200);

  showStartDisplayMessage(version);
  delay(2000);

  // 0x0D - compass addr
  // 0x68 - gyro addr
  // 0x3C - small disp
  // 0x3D - big disp

  // load and configure the DMP
  showStartDisplayMessage("Init gyroscope...");

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  delay(100);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    showStartDisplayMessage("Calibrate gyroscope...");

    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(10);
    // mpu.CalibrateGyro(10);

    mpu.setXAccelOffset(1799);
    mpu.setYAccelOffset(2);
    mpu.setZAccelOffset(1585);
    mpu.setXGyroOffset(169);
    mpu.setYGyroOffset(15);
    mpu.setZGyroOffset(6);

    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));

    showStartDisplayMessage("Init error occured. Please restart device.");
  }

  delay(500);

  showStartDisplayMessage("Init compass...");

  // Initialize HMC5883L
  while (!compass.begin()) {
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  if (!compassCalibration) {
    compass.setOffset(140, -42);
  }

  compassReady = true;

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  delay(200);

  //////// create wifi AP for OTA
  if(otaServer) {
    showStartDisplayMessage("Setup AP for OTA updates...");

    wificonfig_wifiOn();
    boolean result = WiFi.softAP("ESP_Tilt_AP", "12345678");

    delay(100);
    showStartDisplayMessage(result == true ? "AP setup OK" : "AP setup failed");
    Serial.println(result == true ? "AP setup OK" : "AP setup failed");

    delay(200);
    IPAddress myIP = WiFi.softAPIP();  
    Serial.print("Access Point IP address: ");Serial.println(myIP);
    if (mdns.begin("espotaserver", myIP)) {
      Serial.println("MDNS responder started");
    }
    httpUpdater.setup(&httpServer);
    httpServer.begin();
    
    delay(100);
    
    showStartDisplayMessage("HTTPUpdateServer ready! Open " + myIP.toString() + "/update in your browser");
    Serial.println("HTTPUpdateServer ready! Open http://espotaserver.local/update");
    Serial.printf("or http://");Serial.print(myIP);Serial.println("/update in your browser");

    delay(500);
  }
  
  delay(500);

  last_disp_time = millis();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // read a packet from FIFO
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    set_last_read_angle_data(ypr[0], ypr[1], ypr[2]);

// #ifdef OUTPUT_READABLE_QUATERNION
//     // display quaternion values in easy matrix form: w x y z
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     // Serial.print("quat\t");
//     // Serial.print(q.w);
//     // Serial.print("\t");
//     // Serial.print(q.x);
//     // Serial.print("\t");
//     // Serial.print(q.y);
//     // Serial.print("\t");
//     // Serial.println(q.z);
// #endif

// #ifdef OUTPUT_READABLE_EULER
//     // display Euler angles in degrees
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetEuler(euler, &q);
//     // Serial.print("euler\t");
//     // Serial.print(euler[0] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.print(euler[1] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.println(euler[2] * 180 / M_PI);
// #endif

// #ifdef OUTPUT_READABLE_YAWPITCHROLL
//     // display Euler angles in degrees
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetGravity(&gravity, &q);
//     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

//     // set_last_read_angle_data(ypr[0] * 180 / M_PI, ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);
//     set_last_read_angle_data(ypr[0], ypr[1], ypr[2]);

//     // Serial.print("ypr\t");
//     // Serial.print(ypr[0] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.print(ypr[1] * 180 / M_PI);
//     // Serial.print("\t");
//     // Serial.println(ypr[2] * 180 / M_PI);
// #endif

// #ifdef OUTPUT_READABLE_REALACCEL
//     // display real acceleration, adjusted to remove gravity
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetAccel(&aa, fifoBuffer);
//     mpu.dmpGetGravity(&gravity, &q);
//     mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//     // Serial.print("areal\t");
//     // Serial.print(aaReal.x);
//     // Serial.print("\t");
//     // Serial.print(aaReal.y);
//     // Serial.print("\t");
//     // Serial.println(aaReal.z);
// #endif

// #ifdef OUTPUT_READABLE_WORLDACCEL
//     // display initial world-frame acceleration, adjusted to remove gravity
//     // and rotated based on known orientation from quaternion
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetAccel(&aa, fifoBuffer);
//     mpu.dmpGetGravity(&gravity, &q);
//     mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//     mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//     // Serial.print("aworld\t");
//     // Serial.print(aaWorld.x);
//     // Serial.print("\t");
//     // Serial.print(aaWorld.y);
//     // Serial.print("\t");
//     // Serial.println(aaWorld.z);
// #endif
  }

  ///////////////////
  // COMPASS
  ///////////////////
  if (compassCalibration) {
    if(!compassCalibrationDone) {
      doCompassCalibration();
    }
  } else {
    processCompass();
  }

  if(otaServer) {
    httpServer.handleClient();
    if(millis() > otaServerExpireTime) {
      otaServer = false;
      wificonfig_wifiOff();
      Serial.println("Wifi off");
    }
  }

  if (millis() - last_disp_time >= disp_update_interval) {
    last_disp_time = millis();

    calc_angle_data();
    calc_compass_average_data();
    calc_elevation_chart_data();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    updateDisplayData();
  }
}

void doCompassCalibration() {
  Vector mag = compass.readRaw();

  // Determine Min / Max values
  minX = min(minX, int(mag.XAxis));
  maxX = max(maxX, int(mag.XAxis));
  minY = min(minY, int(mag.YAxis));
  maxY = max(maxY, int(mag.YAxis));

  if(millis() > compass_calibration_time) {
    // Calculate offsets
    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;

    compassCalibrationDone = true;

    Serial.println("==============================================");
    Serial.print(minX);
    Serial.print(":");
    Serial.print(maxX);
    Serial.print(":");
    Serial.print(minY);
    Serial.print(":");
    Serial.print(maxY);
    Serial.print(":");
    Serial.print(offX);
    Serial.print(":");
    Serial.println(offY);
    Serial.println("==============================================");
  }
}

void processCompass() {
  if (compassReady && millis() - last_compass_time >= compass_update_interval) {
    last_compass_time = millis();

    Vector norm = compass.readNormalize();

    // Calculate heading
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    float declinationAngle = (8 + 22 / 60) * M_PI / 180;  //0.148; //radians
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * M_PI;

    // Check for wrap due to addition of declination.
    if (heading >= 2 * M_PI)
      heading -= 2 * M_PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;
    headingDegrees += 90;

    if(headingDegrees >= 360) {
      headingDegrees -= 360;
    }

    // Fix HMC5883L issue with angles
    float fixedHeadingDegrees;

    if (headingDegrees >= 1 && headingDegrees < 240) {
      fixedHeadingDegrees = map(headingDegrees, 0, 239, 0, 179);
    } else if (headingDegrees >= 240) {
      fixedHeadingDegrees = map(headingDegrees, 240, 360, 180, 360);
    }

    set_last_read_compas_data(headingDegrees);
  }
}

void updateDisplayData() {
  displayBig.clearDisplay();

  updateBigDisplayCompassData();
  updateBigDisplayElevationChart();
  updateBigDisplayGyroData();

  displayBig.display();

  // updateSmallDisplayData();
}

void updateSmallDisplayData() {
  //   displaySmall.clearDisplay();
  //   displaySmall.setCursor(0, 0);

  //   displaySmall.fillTriangle(61,0,67,0,64,3,WHITE);
  //   displaySmall.fillTriangle(61,31,67,31,64,28,WHITE);

  //   int nearestSideIndex = int(round(curr_head/90));//%4;
  //   int nearestSideAngle = nearestSideIndex * 90;
  //   int head_diff = nearestSideAngle - curr_head;
  //   float anglePixels = float(128)/float(compassFOV); // 128 - width of the screen
  //   float x_diff = head_diff * anglePixels;

  //   displaySmall.setTextColor(WHITE);

  //   // Serial.print("nearestSideIndex="); Serial.print(nearestSideIndex);
  //   // Serial.print(" nearestSideAngle="); Serial.print(nearestSideAngle);
  //   // Serial.print(" head_diff="); Serial.print(head_diff);
  //   // Serial.print(" x_diff="); Serial.print(x_diff);
  //   // Serial.print(" anglePixels="); Serial.println(anglePixels);

  //   int midXPoint = int(64 + x_diff);
  //   int leftXPoint = int(midXPoint - 90*anglePixels);
  //   int rightXPoint = int(midXPoint + 90*anglePixels);

  //   int16_t x1, y1;
  //   uint16_t w, h;

  //   displaySmall.setTextSize(compassSideSize);

  //   displaySmall.getTextBounds(String(compass_queue[nearestSideIndex + 2]), midXPoint, compassSideYPosition, &x1, &y1, &w, &h);
  //   displaySmall.drawChar(midXPoint - w/2, compassSideYPosition, compass_queue[nearestSideIndex + 2], WHITE, BLACK, compassSideSize);

  //   displaySmall.getTextBounds(String(compass_queue[nearestSideIndex + 3]), rightXPoint, compassSideYPosition, &x1, &y1, &w, &h);
  //   displaySmall.drawChar(rightXPoint - w/2, compassSideYPosition, compass_queue[nearestSideIndex + 3], WHITE, BLACK, compassSideSize);

  //   displaySmall.getTextBounds(String(compass_queue[nearestSideIndex + 1]), leftXPoint, compassSideYPosition, &x1, &y1, &w, &h);
  //   displaySmall.drawChar(leftXPoint - w/2, compassSideYPosition, compass_queue[nearestSideIndex + 1], WHITE, BLACK, compassSideSize);

  //   float sectorWidth = (rightXPoint - midXPoint)/(compassQuaterSectors + 1);

  //   int centerNotch = floor(compassQuaterSectors/2) + 1;

  //   for (int i = 1; i <= compassQuaterSectors; i++) {
  //     displaySmall.drawFastVLine(midXPoint + sectorWidth*i, i == centerNotch ? 13 : 15, i == centerNotch ? 5 : 3, WHITE);
  //     displaySmall.drawFastVLine(midXPoint - sectorWidth*i, i == centerNotch ? 13 : 15, i == centerNotch ? 5 : 3, WHITE);

  //     // draw frontier notches
  //     if (leftXPoint - sectorWidth*i >= 0) {
  //       displaySmall.drawFastVLine(leftXPoint - sectorWidth*i, i == centerNotch ? 13 : 15, i == centerNotch ? 5 : 3, WHITE);
  //     }
  //     if(rightXPoint + sectorWidth*i <= 127) {
  //       displaySmall.drawFastVLine(rightXPoint + sectorWidth*i, i == centerNotch ? 13 : 15, i == centerNotch ? 5 : 3, WHITE);
  //     }
  //   }

  //   displaySmall.display();

  Serial.print("Heading (degrees): ");
  Serial.println(curr_head);
}

void updateBigDisplayCompassData() {
  if(compassCalibration) {
    displayBig.setCursor(0, 1);
    displayBig.setTextSize(1);

    if(compassCalibrationDone) {
      displayBig.print(offX);
      displayBig.print(" :: ");
      displayBig.print(offY);
    } else {
      displayBig.print("Calibration:");
      displayBig.print(int((compass_calibration_time - millis())/1000));
    }
  } else {
    int shiftedHead = curr_head;

    // if(shiftedHead >= 360) {
    //   shiftedHead -= 360;
    // } else if(shiftedHead < 0) {
    //   shiftedHead += 360;
    // }

    int nearestSideIndex = int((float(shiftedHead) / 45.0) + 0.5);
    int normalizedIndex = nearestSideIndex % 8;
    

    displayBig.setCursor(0, 1);
    displayBig.setTextSize(2);

    // int sideIndex = map(shiftedHead, 0, 359, 0, 8);

    displayBig.print(compass_queue_new[nearestSideIndex]);

    int16_t x1, y1;
    uint16_t w, h;
    displayBig.setTextSize(2);
    displayBig.getTextBounds(String(curr_head), 0, 0, &x1, &y1, &w, &h);

    displayBig.setCursor(63 - w, 1);
    displayBig.print(curr_head);
  }
}


void updateBigDisplayData() {
  //Serial.print("P(R)="); Serial.print(curr_pitch_rad); Serial.print(" R(R)="); Serial.print(curr_roll_rad);
  //Serial.print(" P(A)="); Serial.print(curr_pitch_angle); Serial.print(" R(A)="); Serial.println(curr_roll_angle);

  displayBig.clearDisplay();

  float cos_pitch_result = cos(curr_pitch_rad);
  float sin_pitch_result = sin(curr_pitch_rad);

  float tan_roll_result = atan(curr_roll_rad);

  // Serial.print("cos(");
  // Serial.print(curr_pitch_angle);
  // Serial.print("°) = ");
  // Serial.print(" (");
  // Serial.print(curr_pitch_rad);
  // Serial.print(") ");
  // Serial.print(cos_pitch_result);
  // Serial.print(" sin(");
  // Serial.print(curr_pitch_angle);
  // Serial.print("°) = ");
  // Serial.println(sin_pitch_result);

  int pitch_radius = 25;
  int pitch_start_x = 95;  //displayBig.width()*0.75;
  int pitch_start_y = displayBig.height() / 2 + 5;

  int diff_pitch_x = pitch_radius * cos_pitch_result;
  int diff_pitch_y = pitch_radius * sin_pitch_result;

  int roll_width = 35;
  int roll_start_x = 5;
  int roll_start_y = pitch_start_y + 10;
  int roll_arrow_height = 20;
  int roll_arrow_gap = 10;
  float roll_multi = 1.2;

  int diff_roll_y = abs(roll_width * tan_roll_result) * roll_multi;

  displayBig.setTextSize(1);

  // draw pitch
  displayBig.drawCircle(pitch_start_x, pitch_start_y, 5, SSD1306_WHITE);
  displayBig.drawLine(pitch_start_x, pitch_start_y, pitch_start_x + diff_pitch_x, pitch_start_y + diff_pitch_y, SSD1306_WHITE);
  displayBig.drawLine(pitch_start_x, pitch_start_y, pitch_start_x - diff_pitch_x, pitch_start_y - diff_pitch_y, SSD1306_WHITE);
  displayBig.drawFastVLine(pitch_start_x + diff_pitch_x, pitch_start_y + diff_pitch_y, 10, WHITE);
  displayBig.drawFastVLine(pitch_start_x - diff_pitch_x, pitch_start_y - diff_pitch_y, 10, WHITE);

  // disp corners
  // displayBig.drawFastHLine(0, 63, 10, WHITE);
  // displayBig.drawFastHLine(117, 0, 10, WHITE);
  // displayBig.drawFastVLine(0, 53, 10, WHITE);
  // displayBig.drawFastVLine(127, 0, 10, WHITE);

  displayBig.drawFastVLine(63, 20, 64, WHITE);
  displayBig.drawFastVLine(64, 20, 64, WHITE);

  // draw roll
  displayBig.drawFastHLine(roll_start_x, roll_start_y, roll_width, WHITE);
  if (curr_roll_angle > 0) {
    displayBig.drawLine(roll_start_x, roll_start_y, roll_start_x + roll_width, roll_start_y - diff_roll_y, SSD1306_WHITE);
  } else {
    displayBig.drawLine(roll_start_x + roll_width, roll_start_y, roll_start_x, roll_start_y - diff_roll_y, SSD1306_WHITE);
  }

  // draw roll arrow
  if (abs(curr_roll_angle) > 1) {
    displayBig.drawFastVLine(roll_start_x + roll_width + roll_arrow_gap, roll_start_y - roll_arrow_height, roll_arrow_height, SSD1306_WHITE);
    if (curr_roll_angle > 0) {
      displayBig.fillTriangle(
        roll_start_x + roll_width + roll_arrow_gap,
        roll_start_y - roll_arrow_height,
        roll_start_x + roll_width + roll_arrow_gap - 5,
        roll_start_y - roll_arrow_height + 5,
        roll_start_x + roll_width + roll_arrow_gap + 5,
        roll_start_y - roll_arrow_height + 5,
        WHITE);
    } else {
      displayBig.fillTriangle(
        roll_start_x + roll_width + roll_arrow_gap,
        roll_start_y, roll_start_x + roll_width + roll_arrow_gap - 5,
        roll_start_y - 5,
        roll_start_x + roll_width + roll_arrow_gap + 5,
        roll_start_y - 5,
        WHITE);
    }
  }


  displayBig.setTextColor(SSD1306_WHITE);

  displayBig.setTextSize(1.5);

  displayBig.setCursor(0, 0);
  displayBig.print(F(" R:"));
  displayBig.print(curr_roll_angle);

  displayBig.setCursor(64, 0);
  displayBig.print(F("P:"));
  displayBig.print(curr_pitch_angle);

  displayBig.display();
}

void updateBigDisplayElevationChart() {
  int bottomYPosition = 78;
  int maxHeight = 15;
  int xDiff = (64 - chartWidth) / 2;

  float realMaxChartValue = max((float)10, max(abs(maxChartValue), abs(minChartValue)));
  float realMaxChartValueMapHigh = realMaxChartValue * 1000;
  float realMaxChartValueMapLow = -1 * realMaxChartValue * 1000;
  float maxHeightMapHigh = maxHeight * 1000;
  float maxHeightMapLow = -1 * maxHeight * 1000;
  
  int barIndex = 0;
  int barValue = 0;

  if (chartBarCycle) {
    for (int i = chartBarsPointer; i < chartWidth; i++) {
      barValue = int(map(chartBars[i]*1000, realMaxChartValueMapLow, realMaxChartValueMapHigh, maxHeightMapLow, maxHeightMapHigh)/1000 + 0.5);
      
      displayBig.drawLine(barIndex + xDiff, bottomYPosition, barIndex + xDiff, bottomYPosition - barValue, WHITE);
      
      barIndex++;
      //Serial.print(F(","));
    }
  }

  for (int i = 0; i < chartBarsPointer; i++) {
    barValue = int(map(chartBars[i]*1000, realMaxChartValueMapLow, realMaxChartValueMapHigh, maxHeightMapLow, maxHeightMapHigh)/1000 + 0.5);

    displayBig.drawLine(barIndex + xDiff, bottomYPosition, barIndex + xDiff, bottomYPosition - barValue, WHITE);
    
    //Serial.print(F(","));
    barIndex++;
  }

  //displayBig.drawLine(0, bottomYPosition - maxHeight, 63, bottomYPosition - maxHeight, WHITE);
  displayBig.drawLine(0, bottomYPosition, 63, bottomYPosition, WHITE);
}

void updateBigDisplayGyroData() {
  //Serial.print("P(R)="); Serial.print(curr_pitch_rad); Serial.print(" R(R)="); Serial.print(curr_roll_rad);
  //Serial.print(" P(A)="); Serial.print(curr_pitch_angle); Serial.print(" R(A)="); Serial.println(curr_roll_angle);

  int roll_arrow_x = 56;
  int roll_arrow_y = 58;
  int roll_arrow_height = 28;

  displayBig.setTextColor(SSD1306_WHITE);

  // draw horizontal line
  displayBig.drawFastHLine(0, 25, 64, WHITE);

  displayBig.setCursor(0, 39);
  displayBig.setTextSize(2);

  int curr_roll_angle_int = int(curr_roll_angle_smooth + 0.5);

  // draw number sign and black box
  if (curr_roll_angle_int > 0) {
    displayBig.fillRect(2, 39, 6, 14, SSD1306_BLACK);
    displayBig.fillRect(0, 42, 12, 8, SSD1306_BLACK);
    displayBig.print(F("+"));
  } else if (curr_roll_angle_int < 0) {
    displayBig.fillRect(0, 42, 12, 8, SSD1306_BLACK);
    displayBig.print(F("-"));
  }

  // draw black box behind numbers
  if(abs(curr_roll_angle_int) >= 10) {
    displayBig.fillRoundRect(10, 30, 37, 31, 3, SSD1306_BLACK);
  } else {
    displayBig.fillRoundRect(10, 30, 20, 31, 3, SSD1306_BLACK);
  }

  displayBig.setTextSize(3);
  displayBig.setCursor(13, 35);
  displayBig.print(abs(curr_roll_angle_int));

  // draw roll arrow
  if (abs(curr_roll_angle_int) > 0) {
    displayBig.fillRect(roll_arrow_x - 2, roll_arrow_y - roll_arrow_height  + 10, 5, roll_arrow_height - 8, SSD1306_BLACK);
    displayBig.drawFastVLine(roll_arrow_x, roll_arrow_y - roll_arrow_height, roll_arrow_height, SSD1306_WHITE);
    if (curr_roll_angle_int > 0) {
      displayBig.fillTriangle(
        roll_arrow_x,
        roll_arrow_y - roll_arrow_height,
        roll_arrow_x - 6,
        roll_arrow_y - roll_arrow_height + 9,
        roll_arrow_x + 6,
        roll_arrow_y - roll_arrow_height + 9,
        WHITE);
    } else {
      displayBig.fillTriangle(
        roll_arrow_x,
        roll_arrow_y + 2,
        roll_arrow_x - 8,
        roll_arrow_y - 11,
        roll_arrow_x + 8,
        roll_arrow_y - 11,
        SSD1306_BLACK);
      displayBig.fillTriangle(
        roll_arrow_x,
        roll_arrow_y,
        roll_arrow_x - 6,
        roll_arrow_y - 9,
        roll_arrow_x + 6,
        roll_arrow_y - 9,
        WHITE);
    }
  }

  // draw horizontal dash line
  // displayBig.drawFastHLine(0, 78, 64, SSD1306_WHITE);
  // bool middle = false;
  // int leftFront = 0;
  // int rightFront = 63;
  // int dashSize = 3;
  // int dashSpace = 3;
  // while (!middle) {
  //   rightFront -= dashSize;

  //   displayBig.drawFastHLine(leftFront, 78, dashSize, WHITE);
  //   displayBig.drawFastHLine(rightFront, 78, dashSize, WHITE);

  //   leftFront += dashSize;

  //   leftFront += dashSpace;
  //   rightFront -= dashSpace;

  //   if (leftFront >= 31) {
  //     middle = true;
  //   }
  // }

  // draw pitch
  int arrowWidth = 40;

  displayBig.setTextSize(3);
  int curr_pitch_angle_int = max(min(int(curr_pitch_angle + 0.5), 99), -99);

  String pitchSign;
  if (curr_pitch_angle_int > 0) {
    pitchSign = '+';
  } else if (curr_pitch_angle_int < 0) {
    pitchSign = '-';
  } else {
    pitchSign = String("");
  }

  String pitchValue = pitchSign + String(abs(curr_pitch_angle_int));

  int16_t x1, y1;
  uint16_t w, h;
  displayBig.getTextBounds(pitchValue, 0, 0, &x1, &y1, &w, &h);

  if (curr_pitch_angle_int > 0) {
    displayBig.setCursor(63 - w, 107);
  } else if (curr_pitch_angle_int < 0) {
    displayBig.setCursor(0, 107);
  } else {
    displayBig.setCursor(32 - w / 2, 100);
  }

  displayBig.print(pitchValue);

  // draw pitch arrow
  if (abs(curr_pitch_angle_int) > 0) {
    int pitchArrowY = 110;
    int topY = 98;
    if (curr_pitch_angle > 0) {
      displayBig.fillRect(pitchArrowY - topY - 3, topY - 3, 24, 7, SSD1306_BLACK);
      
      displayBig.drawLine(0, pitchArrowY, pitchArrowY - topY, topY, SSD1306_WHITE);
      displayBig.drawFastHLine(pitchArrowY - topY, topY, 20, SSD1306_WHITE);
      displayBig.fillTriangle(
        0,
        pitchArrowY,
        2,
        pitchArrowY - 10,
        10,
        pitchArrowY - 2,
        WHITE);
    } else {
      displayBig.fillRect(63 - (pitchArrowY - topY) - 20 - 3, topY - 3, 24, 7, SSD1306_BLACK);

      displayBig.drawLine(63, pitchArrowY, 63 - (pitchArrowY - topY), topY, SSD1306_WHITE);
      displayBig.drawFastHLine(63 - (pitchArrowY - topY) - 20, topY, 20, SSD1306_WHITE);
      displayBig.fillTriangle(
        63,
        pitchArrowY,
        63 - 2,
        pitchArrowY - 10,
        63 - 10,
        pitchArrowY - 2,
        WHITE);
    }
  }




  // draw display
  displayBig.display();
}

void configDisplays() {
  // displaySmall.setTextSize(1);
  // displaySmall.setRotation(0);

  displayBig.setRotation(3);
  displayBig.setTextColor(SSD1306_WHITE);
}


void showStartDisplayMessage(String message) {
  displayBig.clearDisplay();

  displayBig.setTextSize(1);
  displayBig.setTextColor(SSD1306_WHITE);

  displayBig.setCursor(0, 10);
  displayBig.print(message);

  displayBig.display();
}
