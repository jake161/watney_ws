#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//#define DEBUG

// Create the sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Define a struct to hold the IMU data
struct IMUData {
  float orientation_w;
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  int8_t temp;
};

void setup() {
  // Start the serial communication
  Serial.begin(9600);
  while (!Serial) {
    // Wait for serial port to connect. Needed for native USB
  }

  // Initialize the sensor
  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
}

void loop() {
  // Get quaternion orientation data
  imu::Quaternion quat = bno.getQuat();
  IMUData imu_data;

  imu_data.orientation_w = quat.w();
  imu_data.orientation_x = quat.x();
  imu_data.orientation_y = quat.y();
  imu_data.orientation_z = quat.z();

  // Get acceleration data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_data.accel_x = accel.x();
  imu_data.accel_y = accel.y();
  imu_data.accel_z = accel.z();

  // Get gyroscope data
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_data.gyro_x = gyro.x();
  imu_data.gyro_y = gyro.y();
  imu_data.gyro_z = gyro.z();

  // Get magnetometer data
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu_data.mag_x = mag.x();
  imu_data.mag_y = mag.y();
  imu_data.mag_z = mag.z();

  // Get temperature data
  imu_data.temp = bno.getTemp();

#ifdef DEBUG
  // Human-readable output
  Serial.print("Orientation: ");
  Serial.print("W: ");
  Serial.print(imu_data.orientation_w);
  Serial.print(" X: ");
  Serial.print(imu_data.orientation_x);
  Serial.print(" Y: ");
  Serial.print(imu_data.orientation_y);
  Serial.print(" Z: ");
  Serial.println(imu_data.orientation_z);

  Serial.print("Accel: ");
  Serial.print("X: ");
  Serial.print(imu_data.accel_x);
  Serial.print(" Y: ");
  Serial.print(imu_data.accel_y);
  Serial.print(" Z: ");
  Serial.println(imu_data.accel_z);

  Serial.print("Gyro: ");
  Serial.print("X: ");
  Serial.print(imu_data.gyro_x);
  Serial.print(" Y: ");
  Serial.print(imu_data.gyro_y);
  Serial.print(" Z: ");
  Serial.println(imu_data.gyro_z);

  Serial.print("Mag: ");
  Serial.print("X: ");
  Serial.print(imu_data.mag_x);
  Serial.print(" Y: ");
  Serial.print(imu_data.mag_y);
  Serial.print(" Z: ");
  Serial.println(imu_data.mag_z);

  Serial.print("Temperature: ");
  Serial.print(imu_data.temp);
  Serial.println(" C");
#else
  // Raw data output using struct
  Serial.write((uint8_t*)&imu_data, sizeof(imu_data));
#endif

  // Delay for a bit before reading again
  delay(200);
}