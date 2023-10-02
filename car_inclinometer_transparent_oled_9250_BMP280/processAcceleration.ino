Quaternion quat;      // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector

float accMagnitude;
bool highAcc = false;

// --- Acceleration vars
xyzFloat magValue, corrAccRaw, corrGyrRaw, gValue;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float GyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

int accelerationFilter = 0;
String accelerationFilterAlias;

double deltaT{0.};
uint32_t newTime{0}, oldTime{0};

#define Kp 30 // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float lin_ax_smooth = 0;
float lin_ay_smooth = 0;
float lin_az_smooth = 0;

// Define filter constants
float alpha = 0.98; // Complementary filter factor

float smoothAlpha = 0.85; // Complementary filter factor

float accelX, accelY, accelZ; // Accelerometer data in m/s^2

// Initialize orientation variables
float accRoll = 0.0; // Roll angle in radians
float accPitch = 0.0; // Pitch angle in radians
float accYaw = 0.0; // Yaw angle in radians

float targetAnimAccel = 0.0, nextTargetAnimAccel = 0.0, currAnimAccel = 0.0, accelAnimStep = 0.001;

unsigned long last_filter_accel_time;
int filter_accel_interval = 3000;

int maxYAcc, minYAcc;

int currCounter = 0;

void calcAccelerationSimple() {
  corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  corrAccRaw = myMPU9250.getCorrectedAccRawValues();

  float gyroRoll = corrGyrRaw.x * deltaT;
  float gyroPitch = corrGyrRaw.y * deltaT;

  // Normalize accelerometer data
  accelX = corrAccRaw.x;
  accelY = corrAccRaw.y;
  accelZ = corrAccRaw.z;

  float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  accelX /= accelMagnitude;
  accelY /= accelMagnitude;
  accelZ /= accelMagnitude;

  // Calculate accelerometer-based orientation change
  float accelRoll = atan2(accelY, accelZ);
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

  // Apply complementary filter
  accRoll = alpha * gyroRoll + (1 - alpha) * accelRoll;
  accPitch = alpha * gyroPitch + (1 - alpha) * accelPitch;

  // Output linear acceleration (excluding gravity)
  float linearAccelX = accelX * 9.81 * cos(accPitch) * cos(accRoll);
  float linearAccelY = accelY * 9.81 * cos(accPitch) * sin(accRoll);
  float linearAccelZ = (accelZ - 1.0) * 9.81; // Subtract 1g to remove gravity

  lin_ax_smooth = smoothAlpha * lin_ax_smooth + (1 - smoothAlpha) * linearAccelX;
  lin_ay_smooth = smoothAlpha * lin_ay_smooth + (1 - smoothAlpha) * linearAccelY;
}

void nextAccelerationFilter() {
  accelerationFilter++;

  if(accelerationFilter > 2) {
    accelerationFilter = 0;
  }

  switch(accelerationFilter) {
    case 0:
      accelerationFilterAlias = "mg";
      break;
    case 1:
      accelerationFilterAlias = "mh";
      break;
    default:
      accelerationFilterAlias = "no";
      break;
  }

  last_filter_accel_time = millis();
}

void calcAcceleration() {
  newTime = micros();
  deltaT = newTime - oldTime;
  oldTime = newTime;
  deltaT = fabs(deltaT * 0.001 * 0.001);

  // calcAccelerationSimple();
  
  calcAccelerationQuater();
  
  smoothAcceleration();

  // if(currCounter == 0)
  // {
  //   currCounter = 20;
    
  //   Serial.println();
  //   Serial.print("AX:");Serial.print(lin_ax_smooth);Serial.print(", AY:");Serial.print(lin_ay_smooth);Serial.print(", currAnimAccel:");Serial.print(currAnimAccel);
  // } else {
  //   currCounter--;
  // }
}

void smoothAcceleration() {
  if(abs(lin_ay_smooth) > 0.1) {
    nextTargetAnimAccel = lin_ay_smooth;
    
    if(abs(lin_ay_smooth) > 0.8) {
      highAcc = true;
    }
  } else {
    nextTargetAnimAccel = 0.0;
  }

  if(abs(targetAnimAccel - currAnimAccel) < 0.1) {
    currAnimAccel = targetAnimAccel;
    
    targetAnimAccel = nextTargetAnimAccel;
    if(abs(targetAnimAccel) > 4) {
      accelAnimStep = 0.1;
    } else if(abs(targetAnimAccel) > 2) {
      accelAnimStep = 0.01;
    } else if(abs(targetAnimAccel) > 1) {
      accelAnimStep = 0.005;
    } else {
      accelAnimStep = 0.001;
    }
  } else {
    if(currAnimAccel > targetAnimAccel) {
      currAnimAccel -= accelAnimStep;
    } else if(currAnimAccel < targetAnimAccel) {
      currAnimAccel += accelAnimStep;
    }
  }
}

void calcAccelerationQuater() {
  corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  corrAccRaw = myMPU9250.getCorrectedAccRawValues();
  magValue = getMagValues();

  switch(accelerationFilter) {
    case 0:
      MadgwickQuaternionUpdate(-corrAccRaw.x, corrAccRaw.y, corrAccRaw.z, corrGyrRaw.x*M_PI/180.0f, -corrGyrRaw.y*M_PI/180.0f, -corrGyrRaw.z*M_PI/180.0f,  magValue.y, -magValue.x, magValue.z);
      break;
    case 1:
      mahony(-corrAccRaw.x, corrAccRaw.y, corrAccRaw.z, corrGyrRaw.x*M_PI/180.0f, -corrGyrRaw.y*M_PI/180.0f, -corrGyrRaw.z*M_PI/180.0f,  magValue.y, -magValue.x, magValue.z);
      break;
    default:
      no_filter(-corrAccRaw.x, corrAccRaw.y, corrAccRaw.z, corrGyrRaw.x*M_PI/180.0f, -corrGyrRaw.y*M_PI/180.0f, -corrGyrRaw.z*M_PI/180.0f,  magValue.y, -magValue.x, magValue.z);
      break;
  }

  a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
  a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  lin_ax = corrAccRaw.x + a31;
  lin_ay = corrAccRaw.y + a32;
  lin_az = corrAccRaw.z - a33;

  gValue = myMPU9250.getGValues();

  Quaternion quat((float)q[0], (float)q[1], (float)q[2], (float)q[3]);
  // VectorInt16 aa(corrAccRaw.x, corrAccRaw.y, corrAccRaw.z);
  VectorFloat gravity(gValue.x, gValue.y, gValue.z);

  // getLinearAccel(&aaReal, &aa, &gravity);
  VectorInt16 aaReal(lin_ax, lin_ay, lin_az);
  getLinearAccelInWorld(&aaWorld, &aaReal, &quat);

  // float magnitude = sqrtf(aaWorld.x*aaWorld.x + aaWorld.y*aaWorld.y + aaWorld.z*aaWorld.z);
  
  // accMagnitude = 0.8*accMagnitude + 0.2*magnitude;
  
  // if(accMagnitude > 10000) {
  //   highAcc = true;
  // }

  lin_ax_smooth = smoothAlpha * lin_ax_smooth + (1 - smoothAlpha) * aaWorld.x;
  lin_ay_smooth = smoothAlpha * lin_ay_smooth + (1 - smoothAlpha) * aaWorld.y;
  lin_az_smooth = smoothAlpha * lin_az_smooth + (1 - smoothAlpha) * aaWorld.z;

  lin_ax_smooth /= 1000;
  lin_ay_smooth /= 1000;
  lin_az_smooth /= 1000;
}

xyzFloat getAccValues() {
  xyzFloat accVal = {lin_ax, lin_ay, lin_az};

  return accVal;
}

xyzFloat getAccSmoothValues() {
  xyzFloat accVal = {lin_ax_smooth, lin_ay_smooth, lin_az_smooth};

  return accVal;
}

uint8_t getLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
  v -> x = vRaw -> x - gravity -> x*16384;
  v -> y = vRaw -> y - gravity -> y*16384;
  v -> z = vRaw -> z - gravity -> z*16384;

  return 0;
}

uint8_t getLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
  // rotate measured 3D acceleration vector into original state
  // frame of reference based on orientation quaternion
  memcpy(v, vReal, sizeof(VectorInt16));
  v -> rotate(q);

  return 0;
}

void updateBigDisplayAccelerationData() {
  ///// show accel value
  String curAcceleration = String(targetAnimAccel, 2);

  if(millis() - last_filter_accel_time < filter_accel_interval) {
    curAcceleration += "(" + accelerationFilterAlias + ")";
  }

  if(highAcc) {
    curAcceleration = ">> " + curAcceleration + " <<";
    highAcc = false;
  }

  int maxWidth = chartAccW/2;
  int initX = chartAccX + chartAccW/2;
  int maxHeight = 10;
  int totalSectors = 7;
  int hGap = 2;
  int sectorWidth = maxWidth/totalSectors;

  int modifiedAcc = abs(currAnimAccel) * 1000;

  int p;
  if(modifiedAcc <= 500) {
    p = map(modifiedAcc, 0, 500, 0, maxWidth/3);
  } else if(modifiedAcc <= 2000) {
    p = map(modifiedAcc, 0, 2000, maxWidth/3, maxWidth*2/3);
  } else {
    p = map(min(modifiedAcc, 5000), 0, 5000, maxWidth*2/3, maxWidth);
  }
   
  int h = maxHeight * (p/maxWidth);
  float sectors = float(p)/float(sectorWidth);
  int maxSectors = int(sectors + 0.5);

  int direction = 1;

  for (int i = 0; i < maxSectors; i++) {
    int width = sectorWidth;
    int height = maxHeight * (i + 1)/totalSectors;
    if(i == maxSectors - 1) {
      width = int(p % sectorWidth);
    }

    width = min(sectorWidth - hGap, width);

    if(currAnimAccel > 0) {
      direction = -1;
    }

    Paint_DrawRectangle(initX + (i * sectorWidth * direction), chartAccY + chartAccH - height, initX + (i*sectorWidth + width) * direction, chartAccY + chartAccH - 1, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  } 

  Paint_DrawString(chartAccX + chartAccW/2, 24, curAcceleration, &Font8, 0.5, 0.5);
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // short name local variable for readability
  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double hx, hy;
  double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Normalise accelerometer measurement
  double a_norm = ax * ax + ay * ay + az * az;
  if (a_norm == 0.) return;  // handle NaN
  recipNorm = 1.0 / sqrt(a_norm);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Normalise magnetometer measurement
  double m_norm = mx * mx + my * my + mz * mz;
  if (m_norm == 0.) return;  // handle NaN
  recipNorm = 1.0 / sqrt(m_norm);
  mx *= recipNorm;
  my *= recipNorm;
  mz *= recipNorm;

  // Auxiliary variables to avoid repeated arithmetic
  _2q0mx = 2.0f * q0 * mx;
  _2q0my = 2.0f * q0 * my;
  _2q0mz = 2.0f * q0 * mz;
  _2q1mx = 2.0f * q1 * mx;
  _2q0 = 2.0f * q0;
  _2q1 = 2.0f * q1;
  _2q2 = 2.0f * q2;
  _2q3 = 2.0f * q3;
  _2q0q2 = 2.0f * q0 * q2;
  _2q2q3 = 2.0f * q2 * q3;
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  // Reference direction of Earth's magnetic field
  hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
  hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  // Apply feedback step
  qDot1 -= beta * s0;
  qDot2 -= beta * s1;
  qDot3 -= beta * s2;
  qDot4 -= beta * s3;

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltaT;
  q1 += qDot2 * deltaT;
  q2 += qDot3 * deltaT;
  q3 += qDot4 * deltaT;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

void mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0) {
      // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
      recipNorm = 1.0 / sqrt(tmp);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Estimated direction of gravity in the body frame (factor of two divided out)
      vx = q[1] * q[3] - q[0] * q[2];
      vy = q[0] * q[1] + q[2] * q[3];
      vz = q[0] * q[0] - 0.5f + q[3] * q[3];

      // Error is cross product between estimated and measured direction of gravity in body frame
      // (half the actual magnitude)
      ex = (ay * vz - az * vy);
      ey = (az * vx - ax * vz);
      ez = (ax * vy - ay * vx);

      // Compute and apply to gyro term the integral feedback, if enabled
      if (Ki > 0.0f) {
          ix += Ki * ex * deltaT;  // integral error scaled by Ki
          iy += Ki * ey * deltaT;
          iz += Ki * ez * deltaT;
          gx += ix;  // apply integral feedback
          gy += iy;
          gz += iz;
      }

      // Apply proportional feedback to gyro term
      gx += Kp * ex;
      gy += Kp * ey;
      gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltaT = 0.5 * deltaT;
  gx *= deltaT;  // pre-multiply common factors
  gy *= deltaT;
  gz *= deltaT;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
  q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * deltaT;
  q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * deltaT;
  q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * deltaT;
  q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * deltaT;
  float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}