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
  // Raw data output
  Serial.write((uint8_t*)&event.orientation.x, sizeof(event.orientation.x));
  Serial.write((uint8_t*)&event.orientation.y, sizeof(event.orientation.y));
  Serial.write((uint8_t*)&event.orientation.z, sizeof(event.orientation.z));

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.write((uint8_t*)&accel.x(), sizeof(accel.x()));
  Serial.write((uint8_t*)&accel.y(), sizeof(accel.y()));
  Serial.write((uint8_t*)&accel.z(), sizeof(accel.z()));

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.write((uint8_t*)&gyro.x(), sizeof(gyro.x()));
  Serial.write((uint8_t*)&gyro.y(), sizeof(gyro.y()));
  Serial.write((uint8_t*)&gyro.z(), sizeof(gyro.z()));

  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.write((uint8_t*)&mag.x(), sizeof(mag.x()));
  Serial.write((uint8_t*)&mag.y(), sizeof(mag.y()));
  Serial.write((uint8_t*)&mag.z(), sizeof(mag.z()));

  int8_t temp = bno.getTemp();
  Serial.write((uint8_t*)&temp, sizeof(temp));
#endif

  // Delay for a bit before reading again
  delay(1000);
}