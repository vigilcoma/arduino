// ---------- Gyro CAlibration
xyzFloat gyrOffsetVal{62.696,-221.812,-21.949};
xyzFloat accOffsetVal{-11587.396,1765.650,-5552.863};
float accXMin, accXMax, accYMin, accYMax, accZMin, accZMax;

xyzFloat gyr;
float pitch, roll;

// ---------- Kalman Filter
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

float KalmanSpeed = 0.02; //default 0.004;

void calibrateGyro() {
  int started = millis();
  int iterations = 10000;
  xyzFloat gyroOffsetAccumulator{0.f, 0.f, 0.f};
  xyzFloat accelerationOffsetAccumulator{0.f, 0.f, 0.f};

  Serial.println("gyro callibration started, don't move");

  delay(1000);

  for(int i=0; i<iterations; i++){
    if(i % 100 == 0) {
      Serial.println((float)i/iterations, 2);
    }

    // acceleration
    accelerationOffsetAccumulator += myMPU9250.getAccRawValues();

    // gyro
    gyroOffsetAccumulator += myMPU9250.getGyrRawValues();
    delay(1);
  }

  // acceleration
  accelerationOffsetAccumulator /= iterations;
  accelerationOffsetAccumulator.z -= 16384.0f;
  accOffsetVal = accelerationOffsetAccumulator;

  // gyro
  gyrOffsetVal = gyroOffsetAccumulator / iterations;

  Serial.println();
  Serial.println("Callibration ended. Time elapsed: ");
  Serial.println(millis() - started);

  Serial.print(gyrOffsetVal.x, 3);Serial.print(",");Serial.print(gyrOffsetVal.y, 3);Serial.print(",");Serial.println(gyrOffsetVal.z, 3);
  Serial.print(accOffsetVal.x, 3);Serial.print(",");Serial.print(accOffsetVal.y, 3);Serial.print(",");Serial.println(accOffsetVal.z, 3);

  delay(10000);
}

void applyGyroOffsets() {
  Serial.print("Apply gyro: ");
  Serial.print(gyrOffsetVal.x, 3);Serial.print(",");Serial.print(gyrOffsetVal.y, 3);Serial.print(",");Serial.println(gyrOffsetVal.z, 3);
  myMPU9250.setGyrOffsets(gyrOffsetVal.x, gyrOffsetVal.y, gyrOffsetVal.z);
}

void applyAccOffsets() {
  Serial.print("Apply acc: ");
  Serial.print(accOffsetVal.x, 3);Serial.print(",");Serial.print(accOffsetVal.y, 3);Serial.print(",");Serial.println(accOffsetVal.z, 3);
  myMPU9250.setAccOffsets(accOffsetVal.x, accOffsetVal.x, accOffsetVal.y, accOffsetVal.y, accOffsetVal.z, accOffsetVal.z);
}

void readGyroSensor() {
  gyr = myMPU9250.getGyrValues();
  pitch = myMPU9250.getPitch();
  roll  = myMPU9250.getRoll();
}

void calcPitchRoll() {
  readGyroSensor();

  rollOutput = roll;
  pitchOutput = pitch;

  applyGyroKalman();
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + KalmanSpeed*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + KalmanSpeed * KalmanSpeed * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void applyGyroKalman() {
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, gyr.x, roll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, gyr.y, pitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  rollOutput = KalmanAngleRoll;
  pitchOutput = KalmanAnglePitch;
}