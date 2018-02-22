#include "MPU6050.h"


void MPU6050::getRawData(MPU6050_Raw_Data * data){
  getAllData(data);
}

void MPU6050::zero(){
  long startMillis = millis();
  for(int i = 0; i < 100; i ++){
    update();
    delay(5);
  }
  long endMillis = millis();
  dX_Offset = runningData.gyro.x / ((endMillis - startMillis)/1000.0);
  dY_Offset = runningData.gyro.y / ((endMillis - startMillis)/1000.0);
  dZ_Offset = runningData.gyro.z / ((endMillis - startMillis)/1000.0);
  //clear previous values

  //startMillis = millis();
  for(int i = 0; i < 50; i ++){
    update();
  }
  //endMillis = millis();
  xOffset = runningData.orientation.x;
  yOffset = runningData.orientation.y;
  //z can't be fixed
}


void MPU6050::getData(MPU6050_Data *data){
    if(timeAtLastRead == 0){
      update();
      delay(10);
    }
    update();
    *data = runningData;
    data->orientation.x -= xOffset;
    data->orientation.y -= yOffset;
    data->orientation.z -= zOffset;
}


void MPU6050::applyFilter(MPU6050_Scaled_Data *scaleData){
  xAcc = atan2f(scaleData->accel.y, scaleData->accel.z) *180.0/M_PI;
  yAcc = atan2f(scaleData->accel.x, scaleData->accel.z) * 180.0/M_PI;
  runningData.orientation.x = (runningData.orientation.x + scaleData->gyro.x)*COMPLEMENTARY_FILTER_KP + (1.0 - COMPLEMENTARY_FILTER_KP)*xAcc;
  runningData.orientation.y = (runningData.orientation.y + scaleData->gyro.y)*COMPLEMENTARY_FILTER_KP + (1.0 - COMPLEMENTARY_FILTER_KP)*yAcc;
  runningData.orientation.z = runningData.gyro.z;
}

void MPU6050::update(){
  tempTime = micros();
  getAllData(&rawData);
  scaleData(&rawData,&scaledData);
  normalizeGyro(&scaledData, tempTime - timeAtLastRead);
  runningData.accel = scaledData.accel;
  runningData.temp = scaledData.temp;
  runningData.gyro.x += scaledData.gyro.x;
  runningData.gyro.y += scaledData.gyro.y;
  runningData.gyro.z += scaledData.gyro.z;
  applyFilter(&scaledData);
  timeAtLastRead = tempTime;
}

void MPU6050::normalizeGyro(MPU6050_Scaled_Data *data, long deltaMicros){
  data->gyro.x *= (float)(deltaMicros/1000000.0);
  data->gyro.y *= (float)(deltaMicros/1000000.0);
  data->gyro.z *= (float)(deltaMicros/1000000.0);
}


float MPU6050::scaleTemp(int temp){
  return temp/340.0 + 36.53;
}

void MPU6050::getScaledData(MPU6050_Scaled_Data *data){
  getAllData(&rawData);
  scaleData(&rawData, data);
}

void MPU6050::scaleData(MPU6050_Raw_Data *raw, MPU6050_Scaled_Data *data){
  data->gyro.x = raw->gyro.x / gyroScale - dX_Offset;
  data->gyro.y = -1.0*raw->gyro.y / gyroScale - dY_Offset;
  data->gyro.z = raw->gyro.z / gyroScale - dZ_Offset;
  data->accel.x = raw->accel.x / accelScale;
  data->accel.y = raw->accel.y / accelScale;
  data->accel.z = raw->accel.z / accelScale;
  data->temp = scaleTemp(raw->temp);
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


void MPU6050::getAllData(MPU6050_Raw_Data *raw){
  //Serial.println(read16(MPU6050_ACCEL_XOUT_H));
  write8(MPU6050_ACCEL_XOUT_H);
  Wire.requestFrom(mpuAddr,14,true);
  raw->accel.x = Wire.read()<<8|Wire.read();
  raw->accel.y = Wire.read() <<8|Wire.read();
  raw->accel.z = Wire.read() <<8|Wire.read();
  raw->temp = Wire.read() <<8|Wire.read();
  raw->gyro.x = Wire.read() <<8|Wire.read();
  raw->gyro.y = Wire.read() <<8|Wire.read();
  raw->gyro.z = Wire.read() <<8|Wire.read();
}

int MPU6050::begin(){
  Wire.begin();
  if(!checkMPU()){
    return -1;
  }
  resetDevice();
  setClockSource(MPU6050_CLOCK_SOURCE_X_GYRO);
  delay(100);
  setAccelRange(MPU6050_ACCEL_RANGE_2_GPS);
  setGyroRange(MPU6050_GYRO_RANGE_250_DPS);
  Serial.println(read8(MPU6050_CONFIG),BIN);

  return 0;
}

int MPU6050::begin(mpu6050_gyro_range gyroRange, mpu6050_accel_range accelRange){
  Wire.begin();
  if(!checkMPU()){
    return -1;
  }
  resetDevice();
  setClockSource(MPU6050_CLOCK_SOURCE_X_GYRO);
  delay(100);
  setAccelRange(accelRange);
  setGyroRange(gyroRange);
  //Serial.println(read8(MPU6050_CONFIG),BIN);
  return 0;
}

void MPU6050::setClockSource(mpu6050_clock_source source){
  write8(MPU6050_PWR_MGMT_1,source);
}

void MPU6050::setAccelRange(mpu6050_accel_range range){
    int value = read8(MPU6050_ACCEL_CONFIG);
    //write8(MPU6050_ACCEL_CONFIG, value &=0b00011111);
    //write8(MPU6050_ACCEL_CONFIG, value &=0b11100111);
    value &= 0b11100111;
    value |= range << 3;
    //Serial.println(value|(range  << 3),BIN);
    write8(MPU6050_ACCEL_CONFIG, value);
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
  //Serial.println(value, BIN);
  value &= 0b01111111;
  value |= 1 << 7;
  write8(MPU6050_PWR_MGMT_1,value);
  delay(10);
  while(read8(MPU6050_PWR_MGMT_1) >> 7 != 0) delay(10);
  value = read8(MPU6050_PWR_MGMT_1);
  //Serial.println(value, BIN);
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
