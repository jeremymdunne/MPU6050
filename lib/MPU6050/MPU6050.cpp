#include "MPU6050.h"


MPU6050_Raw_Data MPU6050::getRawData(){
  MPU6050_Raw_Data data;
  return data;
}

void MPU6050::getGyroData(Vector3f *vec){

}

void MPU6050::getAccelData(Vector3f *vec){

}

void MPU6050::getTempData(int *temp){

}

void MPU6050::getAllData(MPU6050_Raw_Data *rawData){
  //Serial.println(read16(MPU6050_ACCEL_XOUT_H));
  write8(MPU6050_ACCEL_XOUT_H);
  Wire.requestFrom(mpuAddr,14,true);
  rawData->rawAccel.x = Wire.read()<<8|Wire.read();
  rawData->rawAccel.y = Wire.read() <<8|Wire.read();
  rawData->rawAccel.z = Wire.read() <<8|Wire.read();
  rawData->temp = Wire.read() <<8|Wire.read();
  rawData->rawGyro.x = Wire.read() <<8|Wire.read();
  rawData->rawGyro.y = Wire.read() <<8|Wire.read();
  rawData->rawGyro.z = Wire.read() <<8|Wire.read();
}

int MPU6050::begin(){
  Wire.begin();
  if(!checkMPU()){
    return -1;
  }
  //write8(MPU6050_PWR_MGMT_1,MPU6050_CLOCK_SOURCE_X_GYRO);
  setClockSource(MPU6050_CLOCK_SOURCE_X_GYRO);
  /*
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);
  Wire.endTransmission(true);
  */
  //resetDevice();
  //setAccelRange(MPU6050_ACCEL_RANGE_2_GPS);
  //setGyroRange(MPU6050_GYRO_RANGE_250_DPS);
  //setClockSource(MPU6050_CLOCK_SOURCE_8_MHZ);

  return 0;
}

void MPU6050::setClockSource(mpu6050_clock_source source){
  write8(MPU6050_PWR_MGMT_1,source);
}

void MPU6050::setAccelRange(mpu6050_accel_range range){
    int value = read8(MPU6050_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= range << 3;
    write8(MPU6050_ACCEL_CONFIG,value);
}

void MPU6050::setGyroRange(mpu6050_gyro_range range){
  int value = read8(MPU6050_GYRO_CONFIG);
  value &= 0b11100111;
  value |= range << 3;
  write8(MPU6050_GYRO_CONFIG,value);
}



void MPU6050::resetDevice(){
  int value = read8(MPU6050_PWR_MGMT_1);
  Serial.println(value, BIN);
  value &= 0b01111111;
  value |= 1 << 7;
  write8(MPU6050_PWR_MGMT_1,value);
  delay(10);
  while(read8(MPU6050_PWR_MGMT_1) >> 7 != 0) delay(10);
  value = read8(MPU6050_PWR_MGMT_1);
  Serial.println(value, BIN);
}


bool MPU6050::checkMPU(){
  if(read8(MPU6050_WHO_AM_I) != MPU6050_WHO_AM_I_RESPONSE){
    return false;
  }
  return true;
}

void MPU6050::write8(int reg, int value){
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU6050::write8(int reg){
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.endTransmission();
}

int MPU6050::read8(int reg){
  write8(reg);
  Wire.requestFrom(mpuAddr,1);
  if(Wire.available() >= 1){
    return Wire.read();
  }
  return -1;
}

int MPU6050::read16(int regLow){
  write8(regLow);
  Wire.requestFrom(mpuAddr,2);
  int value = -1;
  if(Wire.available() >= 2){
    value = Wire.read();
    value = value << 8;
    value |= Wire.read();
  }
  return value;
}
