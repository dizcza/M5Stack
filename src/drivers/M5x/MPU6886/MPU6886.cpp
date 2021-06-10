#include "MPU6886.h"
#include <math.h>
#include <Arduino.h>

MPU6886::MPU6886(const uint8_t addr){
  this->addr = addr;
}

uint8_t MPU6886::begin(Ascale acScale /* = AFS_2G */, Gscale gyScale /* = GFS_2000DPS */) {
  this->imuId = mpu6886(MPU6886_WHOAMI);
  delay(1);
  mpu6886(MPU6886_PWR_MGMT_1, (0x01<<7)); //reset - done first in old lib
  delay(100);
  //mpu6886(MPU6886_PWR_MGMT_1, (0x01<<0)); //CLKSEL[2:0] set to 001 to achieve full gyroscope performance
  mpu6886(MPU6886_PWR_MGMT_1, 0x00);      //wakeup
  delay(10);
  mpu6886(MPU6886_ACCEL_CONFIG2, 0x40);   // set FIFO 1K as in MPU6886
  delay(1);
  mpu6886(MPU6886_GYRO_CONFIG, 0x18);     // +- 2000 dps
  delay(1);
  mpu6886(MPU6886_ACCEL_CONFIG, 0x00);    // +- 2g
  delay(1);
  mpu6886(MPU6886_CONFIG, 0x03);          // LPF 42Hz @ 1khz output
  delay(1);
  mpu6886(MPU6886_SMPLRT_DIV, 0x14);      // 14 div, FIFO 50hz out
  delay(1);
  mpu6886(MPU6886_INT_PIN_CFG, 0x02);     // INT - pulse TODO
  delay(1);
  mpu6886(MPU6886_INT_ENABLE, 0x00);      // No INT
  delay(1);
  mpu6886(MPU6886_FIFO_EN,  0x00);        // No sensor push to FIFO
  delay(1);
  mpu6886(MPU6886_USER_CTRL, 0x00);       // No DMP/FIFO
  delay(1);
  //mpu6886(MPU6886_INT_ENABLE, 0x01);
  //delay(10);
  setGyroFsr(gyScale);
  setAccelFsr(acScale);
  return this->imuId;
}

int MPU6886::Init(void){
  begin();
  if(this->imuId != 0x19) return -1;
  return 0;
}

uint8_t MPU6886::mpu6886(uint8_t reg){
	Wire1.beginTransmission(this->addr);
	Wire1.write(reg);
	Wire1.endTransmission();
	Wire1.requestFrom(this->addr, uint8_t(1));
	return Wire1.read();
}

void MPU6886::mpu6886(uint8_t reg, uint8_t val){
  Wire1.beginTransmission(this->addr);
  Wire1.write(reg);
  Wire1.write((uint8_t)val);
  Wire1.endTransmission();
}

// Reading size bytes into data
void MPU6886::mpu6886(uint8_t reg, uint8_t size, uint8_t* data){
  Wire1.beginTransmission(this->addr);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(this->addr, size);
  for (uint8_t i = 0; i < size; i++) data[i] = Wire1.read();
}

void MPU6886::getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az){
  uint8_t buf[6];
  mpu6886(MPU6886_ACCEL_XOUT_H, 6, buf);
  *ax=((int16_t)buf[0]<<8)|buf[1];
  *ay=((int16_t)buf[2]<<8)|buf[3];
  *az=((int16_t)buf[4]<<8)|buf[5];
}
void MPU6886::getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz){
  uint8_t buf[6];
  mpu6886(MPU6886_GYRO_XOUT_H, 6, buf);
  *gx=((uint16_t)buf[0]<<8)|buf[1];
  *gy=((uint16_t)buf[2]<<8)|buf[3];
  *gz=((uint16_t)buf[4]<<8)|buf[5];
}

void MPU6886::getTempAdc(int16_t *t){
  uint8_t buf[2];
  mpu6886(MPU6886_TEMP_OUT_H, 2, buf);
  *t=((uint16_t)buf[0]<<8)|buf[1];
}

//!俯仰，航向，横滚：pitch，yaw，roll，指三维空间中飞行器的旋转状态。
void MPU6886::getAhrsData(float *pitch, float *roll, float *yaw){
  float accX = 0;
  float accY = 0;
  float accZ = 0;
  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;
  getGyroData(&gyroX, &gyroY, &gyroZ);
  getAccelData(&accX, &accY, &accZ);
  MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, pitch, roll, yaw);
}

void MPU6886::getGres(){
  switch (Gyscale) {
  // Possible gyro scales (and their register bit settings) are:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU6886::getAres(){
  switch (Acscale) {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void MPU6886::setGyroFsr(Gscale scale){
  //return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);//设置陀螺仪满量程范围
  unsigned char regdata;
  regdata = (scale<<3);
  mpu6886(MPU6886_GYRO_CONFIG, regdata);
  delay(10);
  Gyscale = scale;
  getGres();
}

void MPU6886::setAccelFsr(Ascale scale){
  unsigned char regdata;
  regdata = (scale<<3);
  mpu6886(MPU6886_ACCEL_CONFIG, regdata);
  delay(10);
  Acscale = scale;
  getAres();
}

void MPU6886::getAccelData(float* ax, float* ay, float* az){
  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  getAccelAdc(&accX, &accY, &accZ);
  *ax = (float)accX * aRes;
  *ay = (float)accY * aRes;
  *az = (float)accZ * aRes;
}

void MPU6886::getGyroData(float* gx, float* gy, float* gz){
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  getGyroAdc(&gyroX, &gyroY, &gyroZ);
  *gx = (float)gyroX * gRes;
  *gy = (float)gyroY * gRes;
  *gz = (float)gyroZ * gRes;
}

void MPU6886::getTempData(float *t){
  int16_t temp = 0;
  getTempAdc(&temp);
  *t = (float)temp / 326.8 + 25.0;
}

void MPU6886::setWakeOnMotion(unsigned short thresh, unsigned short lpa_freq) {
  if (lpa_freq > 500)
    /* At this point, the chip has not been re-configured, so the
      * function can safely exit.
      */
    return;

  uint8_t buf[3];
  uint8_t thresh_hw;

  /* 1LSb = 4mg. */
  if (thresh > 1020)
      thresh_hw = 255;
  else if (thresh < 4)
      thresh_hw = 1;
  else
      thresh_hw = thresh >> 2;

  // interrupt pulse’s width is 50us, active low, 
  // cleared if any read operation is performed
  mpu6886(MPU6886_INT_PIN_CFG, 0x12); //Reserved bit is 1 as in MPU6886 lib
  delay(1);

  //set CYCLE = 0, SLEEP = 0, and GYRO_STANDBY = 0
  buf[0] = mpu6886(MPU6886_PWR_MGMT_1);
  buf[0] &= 0x8F;
  mpu6886(MPU6886_PWR_MGMT_1, buf[0]);
  delay(10);

  //set STBY_XA = STBY_YA = STBY_ZA = 0, and STBY_XG = STBY_YG = STBY_ZG = 1
  buf[0] = mpu6886(MPU6886_PWR_MGMT_2);
  buf[0] &= 0xC7;
  buf[0] |= 0x07;
  mpu6886(MPU6886_PWR_MGMT_2, buf[0]);
  delay(10);

  //set ACCEL_FCHOICE_B = 0 and A_DLPF_CFG[2:0] = 1 (b001)
  buf[0] = mpu6886(MPU6886_ACCEL_CONFIG2);
  buf[0] &= 0xF1;
  buf[0] |= 0x01;
  mpu6886(MPU6886_ACCEL_CONFIG2, buf[0]);
  delay(1);

  //set WOM_INT_EN = 111 to enable motion interrupt
  buf[0] = mpu6886(MPU6886_INT_ENABLE);
  buf[0] |= 0x70;
  mpu6886(MPU6886_INT_ENABLE, buf[0]);
  delay(1);

  // Set motion threshold
  buf[0] = thresh_hw;
  buf[1] = thresh_hw;
  buf[2] = thresh_hw;
  mpu6886(MPU6886_ACCEL_WOM_THR, 3, buf);

  // set ACCEL_INTEL_EN = ACCEL_INTEL_MODE = 1; Ensure that bit 0 is set to 0 - 
  // Enable Accelerometer Hardware Intelligence
  buf[0] = mpu6886(MPU6886_ACCEL_INTEL_CTRL);
  buf[0] &= 0xFE;
  buf[0] |= 0xC0;
  mpu6886(MPU6886_ACCEL_INTEL_CTRL, buf[0]);
  delay(1);

  // Set Frequency of Wake-Up - 2 div, FIFO 500hz out
  if (lpa_freq <= 3)
      buf[0] = 255;
  else if (lpa_freq <= 7)
      buf[0] = 127;
  else if (lpa_freq <= 15)
      buf[0] = 63;
  else if (lpa_freq <= 31)
      buf[0] = 31;
  else if (lpa_freq <= 62)
      buf[0] = 15;
  else if (lpa_freq <= 125)
      buf[0] = 7;
  else if (lpa_freq <= 250)
      buf[0] = 3;
  else
      buf[0] = 1;
  mpu6886(MPU6886_SMPLRT_DIV, buf[0]);
  delay(1);

  // set CYCLE = 1 - Enable Cycle Mode (Accelerometer Low-Power Mode)
  buf[0] = mpu6886(MPU6886_PWR_MGMT_1);
  buf[0] |= 0x20;
  mpu6886(MPU6886_PWR_MGMT_1, buf[0]);

  return;
}