#include <Wire.h>
#include "MPU6886.h"
#define GYROSCALE    0x10    //0x00: 0x01 0x10 0x11

  
float ax, ay, az;


MPU6886 IMU;

void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println("start");

  unsigned char buf;
  unsigned char regdata;

  regdata = 0x00;                                         //次行で設定するレジストリ設定値
  IMU.I2C_Write_NBytes(MPU6886_ADDRESS,0x19,1,&regdata);

  
  regdata = 0x07;                                         //次行で設定するレジストリ設定値
  IMU.I2C_Write_NBytes(MPU6886_ADDRESS,0x1A,1,&regdata);      //CONFIG,デフォルトは 0x01

  IMU.I2C_Read_NBytes(MPU6886_ADDRESS,0x19,1,&buf);
  Serial.print("0x19 value = ");
  Serial.println(buf);

  IMU.I2C_Read_NBytes(MPU6886_ADDRESS,0x1A,1,&buf);
  Serial.print("0x1A value = ");
  Serial.println(buf);


  IMU.Init();
}



void loop() {
  IMU.getAccelData(&ax,&ay,&az);
  Serial.print(ax * 1000);
  Serial.print(" , ");
  Serial.print(ay * 1000);
  Serial.print(" , ");
  Serial.println(az *1000);
//  delay(200);
}
