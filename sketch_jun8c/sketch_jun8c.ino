#include <Wire.h>

volatile uint8_t Register[3] = {0, 0, 0};
volatile uint8_t I2C_ReceiveFlag = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    //Wire.onReceive(ReadTask); // 注册接收回调
}

void loop() {
    // 发送请求到从设备 0x06
    Wire.beginTransmission(0x06);
    Wire.write(0x29); // 发送命令或寄存器地址
    Wire.endTransmission(); // 默认释放总线

    // 请求读取 3 字节
    Wire.requestFrom(0x06, 3);

    for(uint8_t i=0; i<3 && i<3; i++) {
        Register[i] = (uint8_t)Wire.read();
    }
    I2C_ReceiveFlag = 1;


    // 打印接收到的数据（通过中断回调填充）
    if(I2C_ReceiveFlag == 1) {
        noInterrupts();
        for(uint8_t i=0; i<3; i++) {
            Serial.print("register");
            Serial.print(i);
            Serial.print("=");
            Serial.println(Register[i]);
            
        }
        Serial.println();
        I2C_ReceiveFlag = 0;
        interrupts();
    }
    delay(1000);
}

/*
// 从机响应时的回调函数
void ReadTask(int howMany) {
    for(uint8_t i=0; i<howMany && i<3; i++) {
        Register[i] = (uint8_t)Wire.read();
    }
    I2C_ReceiveFlag = 1;
}
*/