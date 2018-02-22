#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR_0 0x68
#define MPU6050_ADDR_1 0x69
#define MPU6050_SELF_TEST_X 0x0D
#define MPU6050_SELF_TEST_Y 0x0E
#define MPU6050_SELF_TEST_Z 0x0F
#define MPU6050_SELF_TEST_A 0x10
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FIFO_ENABLE 0x23
#define MPU6050_I2C_MST_CTRL 0x24
#define MPU6050_I2C_SLV0_ADDR 0x25
#define MPU6050_I2C_SLV0_REG 0x26
#define MPU6050_I2C_SLV0_CTRL 0x27
#define MPU6050_I2C_MST_STATUS 0x36
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_WHO_AM_I 0x75

#define MPU6050_WHO_AM_I_RESPONSE 0x68

#define COMPLEMENTARY_FILTER_KP .95


enum mpu6050_gyro_range{
  MPU6050_GYRO_RANGE_250_DPS = 0x00,
  MPU6050_GYRO_RANGE_500_DPS = 0x01,
  MPU6050_GYRO_RANGE_1000_DPS = 0x02,
  MPU6050_GYRO_RANGE_2000_DPS = 0x03
};

enum mpu6050_accel_range{
  MPU6050_ACCEL_RANGE_2_GPS = 0x00,
  MPU6050_ACCEL_RANGE_4_GPS = 0x01,
  MPU6050_ACCEL_RANGE_8_GPS = 0x02,
  MPU6050_ACCEL_RANGE_16_GPS = 0x03
};

enum mpu6050_clock_source{
  MPU6050_CLOCK_SOURCE_8_MHZ = 0x00,
  MPU6050_CLOCK_SOURCE_X_GYRO = 0x01,
  MPU6050_CLOCK_SOURCE_Y_GYRO = 0x02,
  MPU6050_CLOCK_SOURCE_Z_GYRO = 0x03
};

struct Vector3f{
  float x = 0;
  float y = 0;
  float z = 0;
};

struct Vector3i{
  int x = 0;
  int y = 0;
  int z = 0;
};

struct MPU6050_Raw_Data{
  Vector3i gyro, accel;
  int temp;
};

struct MPU6050_Scaled_Data{
  Vector3f gyro,accel;
  float temp;
};

struct MPU6050_Data{
  Vector3f gyro, accel, orientation;
  float temp;
};


class MPU6050{
public:
  int begin();
  void resetDevice();
  void setClockSource(mpu6050_clock_source source);
  void setAccelRange(mpu6050_accel_range range);
  void setGyroRange(mpu6050_gyro_range range);
  void getAllData(MPU6050_Raw_Data *rawData);
  void getTempData(int *temp);
  void getAccelData(Vector3f *vec);
  void getGyroData(Vector3f *vec);
  MPU6050_Raw_Data getRawData();
  void getScaledData(MPU6050_Scaled_Data *data);
  void getData(MPU6050_Data *data);
  void zero();

private:
  float xOffset = 0, yOffset = 0, zOffset = 0;
  float dX_Offset = 0, dY_Offset = 0, dZ_Offset = 0;
  long timeAtLastRead = 0;
  MPU6050_Raw_Data rawData;
  MPU6050_Scaled_Data scaledData;
  MPU6050_Data runningData;
  float xAcc, yAcc;
  long tempTime;
  int mpuAddr = MPU6050_ADDR_0;
  void update();

  void applyFilter(MPU6050_Scaled_Data *scaleData);
  void scaleData(MPU6050_Raw_Data *raw, MPU6050_Scaled_Data *scaledData);
  void normalizeGyro(MPU6050_Scaled_Data *data, long deltaMicros);
  float gyroScale = calculateGyroScale(MPU6050_GYRO_RANGE_250_DPS);
  float accelScale = calculateAccelScale(MPU6050_ACCEL_RANGE_2_GPS);
  float scaleTemp(int temp);
  bool checkMPU();
  float calculateGyroScale(int scale);
  float calculateAccelScale(int scale);
  void write8(int reg, int value);
  void write8(int reg);
  int read8(int reg);
  int read16(int regLow);
};

#endif
