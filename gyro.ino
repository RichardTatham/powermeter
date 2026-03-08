/**
 * MPU6050/gyroscope specific code. Initialize, helpers to do angular math.
 */

#define CALIBRATION_SAMPLES 40


/**
 *  Calibrate and initialize the gyroscope
 */
void gyroSetup() {
  // "gyro" is defined in main, Arduino implicitly smashes these files together
  // to compile, so it's in scope.

  gyro.initialize();
  // Set to +/- 1000dps.
  gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

#ifdef DEBUG
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(gyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif // DEBUG

// TODO make a calibration mode. For now, test manually.
// 5 tests sitting at the kitchen table, offsets were   -39 -39 -39 -38 -38
#ifdef CALIBRATE
  // Calibrate the gyro
  gyro.setZGyroOffset(0);
  float sumZ = 0;
  int16_t maxSample = -32768;
  int16_t minSample = 32767;
  // Read n-samples
  for (uint8_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    delay(5);
    int16_t reading = gyro.getRotationZ();
    if (reading > maxSample) maxSample = reading;
    if (reading < minSample) minSample = reading;
    sumZ += reading;
  }

  // Throw out the two outliers
  sumZ -= minSample;
  sumZ -= maxSample;

  // Two fewer than the calibration samples because we took out two outliers.
  float deltaZ = sumZ / (CALIBRATION_SAMPLES - 2);
  deltaZ = -1 * deltaZ;
#ifdef DEBUG
  Serial.printf("Discounting max (%d) and min (%d) samples.\n", maxSample, minSample);
  Serial.printf("Gyro calculated offset: %f\n", deltaZ); 
#endif // DEBUG
#endif // CALIBRATE

  // In lieu of being able to store results from a calibration mode...
#ifndef CALIBRATE
  float deltaZ = GYRO_OFFSET;
#endif // CALIBRATE

  // Set that calibration
  gyro.setZGyroOffset(deltaZ);

  // Set zero motion detection
  gyro.setIntZeroMotionEnabled(true);
  gyro.setZeroMotionDetectionThreshold(4);
  // 1 LSB = 64ms. So 30s = 
  gyro.setZeroMotionDetectionDuration(80);
  gyro.setInterruptLatchClear(true);

#ifdef DEBUG
  dumpSettings();
#endif
}

/**
 * This doesn't do anything but echo applied setting on the MPU.
 */
void dumpSettings() {
  Serial.println();
  Serial.printf(" * Gyroscope Sleep Mode: %d\n", gyro.getSleepEnabled() ? "Enabled" : "Disabled");
  Serial.printf(" * Gyroscope offset:     %d\n", gyro.getZGyroOffset());
}

/**
 * Gets a normalized averaged rotational velocity calculation. The MPU6050 library supports a
 * normalized gyroscope reading, which trims off outliers and scales the values to deg/s.
 *
 * An exponential average is applied to further smooth data, with weight of WEIGHT. I don't
 * love this, becaues it means no value is every entirely discarded, but exponential decay
 * probably makes it effectively the same. Maybe something to revisit.
 *
 * Returns a value for foot speed, in degrees/second.
 */
float getNormalAvgVelocity(const float & lastAvg) {
  const static double WEIGHT = 0.90;
  // At +/- 250 degrees/s, the LSB/deg/s is 131. Per the mpu6050 spec.
  /* FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  */
  const static float SENSITIVITY = 32.8;

  // Request new data from the MPU. The orientation obviously dictates
  // which x/y/z value we're interested in, but right now Z.
  // Use the absolute value. Cause who knows if the chip is just backwards.
  float rotz = abs(gyro.getRotationZ() / SENSITIVITY);
  if (rotz < 90) {
    // Magic number here, but less than 90 dps is less than 1 crank rotation 
    // in 4 seconds (15 RPM), just assume it's noise from the road bumps.
    rotz = 0.f;
  }
  // Return a rolling average, including the last reading.
  // e.g. if weight is 0.90, it's 10% what it used to be, 90% this new reading.
  float newavg = (rotz * WEIGHT) + (lastAvg * (1 - WEIGHT));

  return newavg;
}

/**
 * Provide the average rate, in degrees/second.
 *
 * Returns the circular velocity of the rider's foot. Takes in the crank's averaged rotational
 * velocity, converts it to radians, multiplies by the crank radius, and returns the converted
 * value.
 *
 * Value returned is in meters/second
 */
float getCircularVelocity(float dps) {
  // 2 * PI * radians = 360 degrees  -- by definition
  // dps degrees/second * (PI / 180) rad/degree = rad/second
  // (rad/sec) / (rad/circumference) = (rad/sec) / 2 * PI = ratio of a circumference traveled, still per second
  // 2 * PI * CRANK_RADIUS = circumference  -- by definition, that's a circle
  // ratio of circumference traveled * circumference = meters/second

  // It all comes out to:
  // m/s = ((dps * (PI / 180)) / (2 * PI)) * (2 * PI * CRANK_RADIUS);
  // And simplifies to:
  return (dps * PI * CRANK_RADIUS) / 180;
}


