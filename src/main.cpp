#include <Arduino.h>
#include "MPU6050.h"

MPU6050 imu;
void setup() {
    // put your setup code here, to run once:
    Serial.begin(384000);
    if(imu.begin() < 0){
      Serial.println("IMU init Fail!");
      while(true);
    }
    Serial.println("IMU init Success!");
    delay(500);
    //imu.resetDevice();
    //Serial.println("Reset complete!");
}
//MPU6050_Raw_Data imuData;
MPU6050_Data imuData;
long timeStart = 0;
void loop() {
    timeStart = millis();
    imu.getData(&imuData);
    //imu.getAllData(&imuData);
    Serial.println("Ax:" + String(imuData.accel.x) + " Ay:"+ String(imuData.accel.y) + " Az:"+ String(imuData.accel.z) + " Gx:" + String(imuData.gyro.x) + " Gy:" + String(imuData.gyro.y) + " Gz:" + String(imuData.gyro.z) + " Temp:" + String(imuData.temp));
    Serial.println(String(millis() - timeStart));
    delay(5);
    // put your main code here, to run repeatedly:
}
