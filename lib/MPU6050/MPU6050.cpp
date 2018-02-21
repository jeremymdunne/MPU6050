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

void scaleData(){

}


float MPU6050::scaleTemp(int temp){
  return temp/340.0 + 36.53;
}



float MPU6050::calculateAccelScale(int scale){
  switch (scale) {
    case MPU6050_ACCEL_RANGE_2_GPS:
      return 16384.0;
      break;
    case MPU6050_ACCEL_RANGE_4_GPS:
      return 8192.0;
      break;
    case MPU6050_ACCEL_RANGE_8_GPS:
      return 4096.0;
      break;
    case MPU6050_ACCEL_RANGE_16_GPS:
      return 2048.0;
      break;
  }
  return -1;
}

float MPU6050::calculateGyroScale(int scale){
  switch (scale) {
    case MPU6050_GYRO_RANGE_250_DPS:
      return 131.0;
      break;
    case MPU6050_GYRO_RANGE_500_DPS:
      return 65.5;
      break;
    case MPU6050_GYRO_RANGE_1000_DPS:
      return 32.8;
      break;
    case MPU6050_GYRO_RANGE_2000_DPS:
      return 16.4;
      break;
  }
  return -1;
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
  //setClockSource(MPU6050_CLOCK_SOURCE_X_GYRO);
  /*
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);
  Wire.endTransmission(true);
  */
  resetDevice();
  setAccelRange(MPU6050_ACCEL_RANGE_2_GPS);
  setGyroRange(MPU6050_GYRO_RANGE_250_DPS);
  setClockSource(MPU6050_CLOCK_SOURCE_X_GYRO);

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
    accelScale = calculateAccelScale(range);
}

void MPU6050::setGyroRange(mpu6050_gyro_range range){
  int value = read8(MPU6050_GYRO_CONFIG);
  value &= 0b11100111;
  value |= range << 3;
  write8(MPU6050_GYRO_CONFIG,value);
  gyroScale = calculateGyroScale(range);
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
