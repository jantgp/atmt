#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(57600);
  Wire.begin();

  Serial.println("Scanning for I2C devices...");

  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("Scan complete");
  }
}

void loop() {
  // Nothing to do here
}
