#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
float accelScale, gyroScale;

void setup() {
  Serial.begin(9600);

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;

  // check if it's time to read data and update the filter
  if (CurieIMU.dataReady()) {

    // read scaled data from CurieIMU
    CurieIMU.readMotionSensorScaled(ax, ay, az, gx, gy, gz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
  }
}
