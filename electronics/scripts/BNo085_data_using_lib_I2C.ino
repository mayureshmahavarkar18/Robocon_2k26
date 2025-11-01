#include <Wire.h>
#include <Adafruit_BNO08x.h>

// BNO085 I2C address (default is 0x4A, can also be 0x4B)
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Configure which reports you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  
  // Enable rotation vector report (quaternion)
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  
  // Enable accelerometer
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  
  // Enable gyroscope
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  
  // Enable magnetometer
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetometer");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("BNO085 ESP32 Test");
  
  // Initialize I2C with custom pins (optional)
  // Wire.begin(SDA_PIN, SCL_PIN); // Uncomment and set your pins if needed
  Wire.begin(); // Use default ESP32 I2C pins (GPIO 21 = SDA, GPIO 22 = SCL)
  
  // Try to initialize the BNO085
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("BNO08x Found!");
  
  setReports();
  
  Serial.println("Reading events");
  delay(100);
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset ");
    setReports();
  }
  
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  switch (sensorValue.sensorId) {
    
    case SH2_ACCELEROMETER:
      Serial.print("Accel - x: ");
      Serial.print(sensorValue.un.accelerometer.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.accelerometer.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.accelerometer.z);
      break;
      
    case SH2_GYROSCOPE_CALIBRATED:
      Serial.print("Gyro - x: ");
      Serial.print(sensorValue.un.gyroscope.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.gyroscope.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.gyroscope.z);
      break;
      
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      Serial.print("Mag - x: ");
      Serial.print(sensorValue.un.magneticField.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.magneticField.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.magneticField.z);
      break;
      
    case SH2_ROTATION_VECTOR:
      Serial.print("Rotation Vector - r: ");
      Serial.print(sensorValue.un.rotationVector.real);
      Serial.print(" i: ");
      Serial.print(sensorValue.un.rotationVector.i);
      Serial.print(" j: ");
      Serial.print(sensorValue.un.rotationVector.j);
      Serial.print(" k: ");
      Serial.println(sensorValue.un.rotationVector.k);
      
      // Convert quaternion to Euler angles (optional)
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;
      
      // Roll (x-axis rotation)
      float roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
      // Pitch (y-axis rotation)
      float pitch = asin(2.0 * (qw * qy - qz * qx));
      // Yaw (z-axis rotation)
      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      
      // Convert to degrees
      roll = roll * 180.0 / PI;
      pitch = pitch * 180.0 / PI;
      yaw = yaw * 180.0 / PI;
      
      Serial.print("Euler Angles - Roll: ");
      Serial.print(roll);
      Serial.print(" Pitch: ");
      Serial.print(pitch);
      Serial.print(" Yaw: ");
      Serial.println(yaw);
      break;
  }
  
  delay(50); // Adjust delay as needed
}
