#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Scanning I2C devices...");
  for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at 0x");
      Serial.println(addr, HEX);
    }
  }
  
}

void loop() {
  for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(1000);

}