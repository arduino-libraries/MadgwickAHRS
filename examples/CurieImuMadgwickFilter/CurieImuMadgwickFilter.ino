/*
  ===============================================
  Example sketch for CurieImu library for Intel(R) Curie(TM) devices.
  Copyright (c) 2015 Intel Corporation.  All rights reserved.

  Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
  class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

  ===============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

/* Based on sketch by Helena Bisby <support@arduino.cc>
   This example code is in the public domain
   http://arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser

   Modified 02.21.16 by Michael J Smorto to implement Madgwick
   filter using actual sampling frequency and allowing user
   to change Beta factor from within the sketch.

   Addition code elements extracted from:
    (1) FreeIMU library by Fabio Veresano that allows floats to be passed
    over serial (See Communictions.h)
    (2) Timer function extracted from erikyo in post
    https://forum.arduino.cc/index.php?topic=378779.0
    on the Arduino forum.
*/

#include "CurieImu.h"
#include <MadgwickAHRS.h>
#include "CommunicationUtils.h"

//Setup Madgwick Filter
Madgwick filter; // initialise Madgwick object

/*
 * Full Scale Range   | LSB Sensitivity
 * -------------------+----------------
 * +/- 125  degrees/s | 262.4 LSB/deg/s
 * +/- 250  degrees/s | 131.2 LSB/deg/s
 * +/- 500  degrees/s | 65.5  LSB/deg/s
 * +/- 1000 degrees/s | 32.8  LSB/deg/s
 * +/- 2000 degrees/s | 16.4  LSB/deg/s : use 20.5
 */
#define gyro_sensitivity 20.5f

#define C_PI 3.14159265358979323846
#define rad2deg 180.0f/C_PI 
#define deg2rad C_PI/180.0f

int16_t ax, ay, az;
int16_t gx, gy, gz;
float yaw, pitch, roll;
float val[14], ypr[3], val_cal[6];

unsigned long lastUpdate = 0;
unsigned long now1;       // sample period expressed in milliseconds


// Repeat part of the code every X miliseconds
#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))
//Set up a timer Variable
uint32_t timer;

const int ledPin = 13;      // activity LED pin

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);     // wait for the serial port to open

  lastUpdate = 0;
  now1 = 0;
  
  // Initialize IMU
  Serial.println("Initializing IMU device...");
  CurieImu.initialize();
  CurieImu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
  CurieImu.setAccelRate(BMI160_ACCEL_RATE_100HZ);
  
  CurieImu.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
  CurieImu.setGyroRate(BMI160_GYRO_RATE_100HZ);


  delay(100);

  filter.beta = 0.07;  //use 0.05 for gyro range of 500
  filter.instability_fix = 1;

  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(1000);
  
  // The board must be resting in a horizontal position for 
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieImu.autoCalibrateGyroOffset();
  Serial.println(" Done");
  
  Serial.print("Starting Acceleration calibration...");
  CurieImu.autoCalibrateXAccelOffset(0);
  CurieImu.autoCalibrateYAccelOffset(0);
  CurieImu.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");
  
  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieImu.setGyroOffsetEnabled(true);
  CurieImu.setAccelOffsetEnabled(true);

  
  // verify connection to IMU
  Serial.println("Testing device connections...");
  if (CurieImu.testConnection()) {
    Serial.println("CurieImu connection successful");
  } else {
    Serial.println("CurieImu connection failed");
  }
  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);

  timer = micros();       // Initialize timer
  
}

void loop() {

 runEvery(10){         // Exetutes this part of the code every 10 miliseconds -> 100Hz 
  timer = micros();    // Reset the timer
  val_cal[0] = CurieImu.getAccelerationX();
  val_cal[1] = CurieImu.getAccelerationY();
  val_cal[2] = CurieImu.getAccelerationZ();
  val_cal[3] = CurieImu.getRotationX()/gyro_sensitivity;
  val_cal[4] = CurieImu.getRotationY()/gyro_sensitivity;
  val_cal[5] = CurieImu.getRotationZ()/gyro_sensitivity;

  now1 = micros();
  filter.sampleFreq = 1.0f / ((now1 - lastUpdate) / 1000000.0f);
  lastUpdate = now1;
  
  // use function from MagdwickAHRS.h to return quaternions
  //filter.updateIMU(val_cal[3], val_cal[4] , val_cal[5] , val_cal[0], val_cal[1], val_cal[2]);
  filter.updateIMU(val_cal[3]*deg2rad, val_cal[4]*deg2rad, val_cal[5]*deg2rad, val_cal[0], val_cal[1], val_cal[2]);

  // functions to find yaw roll and pitch from quaternions
  val[6] = filter.getYaw()*rad2deg;
  val[7] = filter.getRoll()*rad2deg;
  val[8] = filter.getPitch()*rad2deg;
  val[0] = val_cal[0];
  val[1] = val_cal[1];
  val[2] = val_cal[2];
  val[3] = val_cal[3]/gyro_sensitivity;
  val[5] = val_cal[4]/gyro_sensitivity;
  val[5] = val_cal[5]/gyro_sensitivity;
  val[9] = 0.0034 * CurieImu.getTemperature() + 73.4;

  val[10] = filter.q[0];  // quaternion outputs
  val[11] = filter.q[1];
  val[12] = filter.q[2];
  val[13] = filter.q[3];

  //Serial.print(val[0]); Serial.print("\t"); 
  //Serial.print(val[1]); Serial.print("\t"); 
  //Serial.println(val[2]);

  // display tab-separated accel/gyro x/y/z values
  /*Serial.print("a/g:\t");
   Serial.print(val_cal[0]);  Serial.print("\t"); Serial.print(val_cal[1]);
   Serial.print("\t"); Serial.print(val_cal[2]); Serial.print("\t"); 
   Serial.print(val_cal[3]); Serial.print("\t"); Serial.print(val_cal[4]); 
   Serial.print("\t"); Serial.println(val_cal[5]);
  */
  
   serialPrintFloatArr(val,14);
   Serial.print('\n');
 }
}

