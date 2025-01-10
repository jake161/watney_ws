#include <Arduino.h>

//#define DEBUG

void setup() {
  // Start the serial communication
  Serial.begin(9600);
  while (!Serial) {
    // Wait for serial port to connect. Needed for native USB
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming JSON message
    String message = Serial.readStringUntil('\n');
    
    // Echo the message back
    Serial.println(message);
  }

  // Delay for a bit before reading again
  delay(200);
}