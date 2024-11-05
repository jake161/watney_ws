#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//#define DEBUG

// Create the sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
  sensors_event_t event;
  bno.getEvent(&event);

#ifdef DEBUG
  // Human-readable output
  Serial.print("Orientation: ");
  Serial.print("X: ");
  Serial.print(event.orientation.x);
  Serial.print(" Y: ");
  Serial.print(event.orientation.y);
  Serial.print(" Z: ");
  Serial.println(event.orientation.z);

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("Accel: ");
  Serial.print("X: ");
  Serial.print(accel.x());
  Serial.print(" Y: ");
  Serial.print(accel.y());
  Serial.print(" Z: ");
  Serial.println(accel.z());

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("Gyro: ");
  Serial.print("X: ");
  Serial.print(gyro.x());
  Serial.print(" Y: ");
  Serial.print(gyro.y());
  Serial.print(" Z: ");
  Serial.println(gyro.z());

  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print("Mag: ");
  Serial.print("X: ");
  Serial.print(mag.x());
  Serial.print(" Y: ");
  Serial.print(mag.y());
  Serial.print(" Z: ");
  Serial.println(mag.z());

  int8_t temp = bno.getTemp();
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
#else
  // CSV format output
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = bno.getTemp();

  Serial.print(event.orientation.x);
  Serial.print(",");
  Serial.print(event.orientation.y);
  Serial.print(",");
  Serial.print(event.orientation.z);
  Serial.print(",");
  Serial.print(accel.x());
  Serial.print(",");
  Serial.print(accel.y());
  Serial.print(",");
  Serial.print(accel.z());
  Serial.print(",");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print(gyro.z());
  Serial.print(",");
  Serial.print(mag.x());
  Serial.print(",");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.print(mag.z());
  Serial.print(",");
  Serial.print(temp);
  Serial.println();
#endif

  // Delay for a bit before reading again
  delay(1000);
}