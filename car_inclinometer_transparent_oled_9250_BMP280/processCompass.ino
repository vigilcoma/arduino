//Magnetometer Registers
//#define AK8963_ADDRESS    0x0C  // Address of AK8963 (MPU9250) magnetometer

#define WHO_AM_I_AK8963   0x00  // should return 0x48
#define INFO              0x01
#define AK8963_ST1        0x02  // data ready status bit 0
#define AK8963_XOUT_L	    0x03  // data
#define AK8963_XOUT_H	    0x04
#define AK8963_YOUT_L	    0x05
#define AK8963_YOUT_H	    0x06
#define AK8963_ZOUT_L	    0x07
#define AK8963_ZOUT_H	    0x08
#define AK8963_ST2        0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL       0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
//#define AK8963_ASTC       0x0C  // Self test control
//#define AK8963_I2CDIS     0x0F  // I2C disable
//#define AK8963_ASAX       0x10  // Fuse ROM x-axis sensitivity adjustment value
//#define AK8963_ASAY       0x11  // Fuse ROM y-axis sensitivity adjustment value
//#define AK8963_ASAZ       0x12  // Fuse ROM z-axis sensitivity adjustment value

#define  MFS_14BITS  0 // 0.6 mG per LSB
#define  MFS_16BITS  1    // 0.15 mG per LSB

float Declination = 8 + 22/60;
float setupAngle = -90 * DEG_TO_RAD;
float headOutput = 0.0;

float Mag_x_hor, Mag_y_hor;
float Mag_pitch, Mag_roll;
float Mag_x_dampened, Mag_y_dampened;

bool useCompassCompensation = false;

uint8_t MPU9250Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t MPU9250Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float MPU9250mRes;
float mx, my, mz;

int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float MPU9250magBias[3] = {148.09, 507.56, 151.69};

void MPU9250getMres() {
  switch (MPU9250Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          MPU9250mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          MPU9250mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void initAK8963()
{
  MPU9250getMres();

  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  magCalibration[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  magCalibration[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  magCalibration[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode, 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, MPU9250Mscale << 4 | MPU9250Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void magcalMPU9250() 
{
  uint16_t ii = 0, sample_count = 0, current_step = 0, progressStep = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(2000);
  Serial.println("Go!");
  
  sample_count = 5000;
  progressStep = sample_count/10;

  while (current_step < sample_count) {
    // Read the mag data
    if(MPU9250readMagData(mag_temp)) {
      if(current_step%progressStep == 0) {
        Serial.println((float)current_step/sample_count, 2);
      }

      for (int jj = 0; jj < 3; jj++) {
        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }

      current_step++;
    }

    delay(10);
    //delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
  }

  //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
  MPU9250magBias[0] = (float) mag_bias[0]*MPU9250mRes*magCalibration[0];  // save mag biases in G for main program
  MPU9250magBias[1] = (float) mag_bias[1]*MPU9250mRes*magCalibration[1];   
  MPU9250magBias[2] = (float) mag_bias[2]*MPU9250mRes*magCalibration[2];          

  Serial.println("Mag Calibration done!");
  Serial.println("AK8963 mag biases (mG)"); Serial.println(MPU9250magBias[0]); Serial.println(MPU9250magBias[1]); Serial.println(MPU9250magBias[2]);
  delay(10000);
}

bool MPU9250readMagData(int16_t * destination)
{
  bool res = false;

  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];

      res = true;
    }
  }

  return res;
}

void MPU9250ReadMag() {
  if(MPU9250readMagData(magCount)) {
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*MPU9250mRes*magCalibration[0] - MPU9250magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*MPU9250mRes*magCalibration[1] - MPU9250magBias[1];  
    mz = (float)magCount[2]*MPU9250mRes*magCalibration[2] - MPU9250magBias[2];
  }
}

void calcHeading() {
  MPU9250ReadMag();

  // ----- Fix the pitch, roll, & signs
  /*
     MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other
     which means that Mag_pitch equates to the Gyro_roll and Mag_roll equates to the Gryro_pitch

     The MPU-9520 and AK8963 Z axes point in opposite directions
     which means that the sign for Mag_pitch must be negative to compensate.
  */
  // Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  // Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  if(useCompassCompensation) {
    Mag_pitch = -rollOutput * DEG_TO_RAD;
    Mag_roll = pitchOutput * DEG_TO_RAD;

    // ----- Apply the standard tilt formulas
    Mag_x_hor = mx * cos(Mag_pitch) + my * sin(Mag_roll) * sin(Mag_pitch) - mz * cos(Mag_roll) * sin(Mag_pitch);
    Mag_y_hor = my * cos(Mag_roll) + mz * sin(Mag_roll);
  } else {
    Mag_x_hor = mx;
    Mag_y_hor = my;
  }

  Mag_x_dampened = Mag_x_dampened * 0.8 + Mag_x_hor * 0.2;
  Mag_y_dampened = Mag_y_dampened * 0.8 + Mag_y_hor * 0.2;

  float head = atan2(Mag_y_dampened, Mag_x_dampened) + Declination + setupAngle;

  //Correct for when signs are reversed.
  if (head < 0)
    head += 2 * M_PI;

  // Check for wrap due to addition of declination.
  if (head >= 2 * M_PI)
    head -= 2 * M_PI;

  // Convert radians to degrees for readability.
  float headDegrees = head * 180 / M_PI;

  if (headDegrees >= 360) {
    headDegrees -= 360;
  }

  headOutput = 0.9*headOutput + 0.1*map(headDegrees, 0, 360, 360, 0);
}

xyzFloat getMagValues() {
  xyzFloat magVal = {mx, my, mz};

  return magVal;
}

void updateBigDisplayCompassData() {
  // if(shiftedHead >= 360) {
  //   shiftedHead -= 360;
  // } else if(shiftedHead < 0) {
  //   shiftedHead += 360;
  // }

  int nearestSideIndex = int((headOutput / 45.0) + 0.5);
  int normalizedIndex = nearestSideIndex % 8;

  String head = compass_queue_new[nearestSideIndex];
  String headNum = String(int(headOutput + 0.5));
  String comp = head + ' ' + headNum;

  Paint_DrawString(41, 2, comp, &Font16, 0.5);

  if(useCompassCompensation) {
    Paint_DrawString(80, 17, "c", &Font8, 1, 1);
  }
}