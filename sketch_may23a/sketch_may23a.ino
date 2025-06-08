#include <Wire.h>

uint8_t Register[3]={0,0,0};
uint8_t I2C_ReceiveFlag=0;

void setup() {
  // put your setup code here, to run once:
  //pinMode(SDA,INPUT_PULLUP);
  //pinMode(SCL,INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  Wire.onReceive(ReadTask);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(0x06);
  Wire.write(0x29);
  int answer=Wire.endTransmission(false);
  Serial.print("answer is ");
  Serial.println(answer);
  Wire.requestFrom(0x06,3);

  if(I2C_ReceiveFlag==1){
    noInterrupts();
    I2C_ReceiveFlag=0;
    for(uint8_t i=0;i<3;i++){
      Serial.print("register");
      Serial.print(i);
      Serial.print("=");
      Serial.println(Register[i]);
      Serial.print('\n');
      Wire.endTransmission(true);
    }
    interrupts();
  }
  delay(1000);
}

void ReadTask(int howMany)
{
  for(uint8_t i=0;Wire.available()&&i<3;i++){
    Register[i]=(uint8_t)Wire.read();
  }
  I2C_ReceiveFlag=1;

}
