#include <Wire.h>

uint8_t Register[3]={0,0,0};

void setup() {
  // put your setup code here, to run once:
  //pinMode(SDA,INPUT_PULLUP);
  //pinMode(SCL,INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  int a=0;
  Wire.beginTransmission(0x06);
  Wire.write(0x29);
  Wire.endTransmission(false);
  Wire.requestFrom(0x06,3);
  while(Wire.available()){
    a=(uint8_t)Wire.read();
    Serial.print("a=");
    Serial.println(a);
  }
  Serial.println('\n');
  delay(1000);
}
