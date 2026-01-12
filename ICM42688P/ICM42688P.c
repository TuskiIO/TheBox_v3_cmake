#include "ICM42688P.h"

float Gyro_Scale_Factor;
float Accel_Scale_Factor;
ICM42688_Data_t ICM42688_Data;
extern SPI_HandleTypeDef hspi3;

static uint8_t IMU_SPI_TRX(uint8_t txData){
  uint8_t rxData;
  
  HAL_SPI_TransmitReceive(&hspi3, &txData, &rxData, 1, SPI3_TIMEOUT);
  return rxData;
}

void IMU_SPI_ReadReg(uint8_t reg_addr, uint8_t *rxData, uint16_t len){
  IMU_SPI_NSS_L();
  IMU_SPI_TRX(reg_addr | ICM42688_READ);
  for(uint16_t i = 0; i<len; i++){
    rxData[i] = IMU_SPI_TRX(0x00);
  }
  IMU_SPI_NSS_H();
//  for(uint16_t i=0; i<len;i++){
//    rxData[i] = 1;
//  }
}

void IMU_SPI_WriteReg(uint8_t reg_addr, uint8_t value){
  IMU_SPI_NSS_L();
  IMU_SPI_TRX(reg_addr | ICM42688_WRITE);
  IMU_SPI_TRX(value);
  IMU_SPI_NSS_H();
}


// 定义ICM42688P的初始化函数
void ICM42688_Init(void)
{
  // 定义一个配置结构体变量，并赋值为默认配置
  ICM42688_ConfigTypeDef icm_config;
  icm_config.accel_mode = ICM42688_MODE_LOW_NOISE;
  icm_config.gyro_mode  = ICM42688_MODE_LOW_NOISE;

  icm_config.gyro_fs    = ICM42688_GYRO_FS_2000;   
  icm_config.gyro_odr   = ICM42688_ODR_1KHZ;
  Gyro_Scale_Factor     = ICM42688_GYRO_SENS_2000;

  icm_config.accel_fs   = ICM42688_ACCEL_FS_16G;
  icm_config.accel_odr  = ICM42688_ODR_1KHZ;
  Accel_Scale_Factor    = ICM42688_ACCEL_SENS_16G;

  icm_config.accel_ui_filt_bw   = 0x01; //BW=max(400Hz, ODR)/4
  icm_config.gyro_ui_filt_bw    = 0x01; //BW=max(400Hz, ODR)/4
  // 定义一个临时变量，用于存储寄存器的值
  uint8_t tmp;

  // 读取ICM42688P的WHO_AM_I寄存器，检查设备ID是否正确
  IMU_SPI_ReadReg(ICM42688_WHO_AM_I,&tmp,1);
  
  // ID错误，停止初始化
  if (tmp != ICM42688_ID)
    return;

  ICM42688_SENSOR_ENABLE(0);
  
  HAL_Delay(10);
  
  // 写入ICM42688P的PWR_MGMT0寄存器，设置加速度计和陀螺仪的工作模式
  tmp = (icm_config.accel_mode << 2) | (icm_config.gyro_mode);
  IMU_SPI_WriteReg(ICM42688_PWR_MGMT0,tmp);

  // 写入ICM42688P的GYRO_CONFIG0寄存器，设置陀螺仪的量程和ODR
  tmp = (icm_config.gyro_fs << 5) | icm_config.gyro_odr;
  IMU_SPI_WriteReg(ICM42688_GYRO_CONFIG0,tmp);
  
  // 写入ICM42688P的ACCEL_CONFIG0寄存器，设置加速度计的量程和ODR
  tmp = (icm_config.accel_fs << 5) | icm_config.accel_odr;
  IMU_SPI_WriteReg(ICM42688_ACCEL_CONFIG0,tmp);
  
  // 写入ICM42688P的GYRO_ACCEL_CONFIG0寄存器，设置滤波器
  tmp = (icm_config.accel_ui_filt_bw << 4) | icm_config.gyro_ui_filt_bw;
  IMU_SPI_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0,tmp);
  
  HAL_Delay(10);

  ICM42688_SENSOR_ENABLE(1);

  HAL_Delay(10);
  
}

void ICM42688_GetTemper(void){
  uint8_t rxData[2];
  int16_t rawData;
  
  IMU_SPI_ReadReg(ICM42688_TEMP_DATA1,rxData,2);

  rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
  ICM42688_Data.tempData = (float)((rawData/ICM42688_TEMP_SENS) + ICM42688_TEMP_OFFSET);
}


void ICM42688_GetAccel(void){
  uint8_t rxData[6];
  int16_t rawData;
  
  IMU_SPI_ReadReg(ICM42688_ACCEL_DATA_X1,rxData,6);
  for(uint8_t i=0;i<3;i++){
    rawData = (int16_t)((rxData[2*i] << 8) | rxData[2*i+1]);
    ICM42688_Data.accelData[i] = (float)((float)(ICM42688_ACCEL_G * rawData) / Accel_Scale_Factor); 
  }
  

//  // 读取X轴数据
//  txData[0] = ICM42688_ACCEL_DATA_X1 | ICM42688_READ;
//  tx = ICM42688_ACCEL_DATA_X1 | ICM42688_READ;
//  
//  rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
//  accelData[0] = (int16_t)(rawData/Accel_Scale_Factor);
//
//  // 读取Y轴数据
//  txData[0] = ICM42688_ACCEL_DATA_Y1 | ICM42688_READ;
//  HAL_SPI_Transmit(&hspi2, txData, 1, SPI_TIMEOUT);
//  HAL_SPI_Receive(&hspi2, rxData, 2, SPI_TIMEOUT);
//  rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
//  accelData[1] = (int16_t)(rawData/Accel_Scale_Factor);
//
//  // 读取Z轴数据
//  txData[0] = ICM42688_ACCEL_DATA_Z1 | ICM42688_READ;
//  HAL_SPI_Transmit(&hspi2, txData, 1, SPI_TIMEOUT);
//  HAL_SPI_Receive(&hspi2, rxData, 2, SPI_TIMEOUT);
//  rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
//  accelData[2] = (int16_t)(rawData/Accel_Scale_Factor);
    
}

void ICM42688_GetGyro(void){
  uint8_t rxData[6];
  int16_t rawData;
  
  IMU_SPI_ReadReg(ICM42688_GYRO_DATA_X1,rxData,6);
  for(uint8_t i=0;i<3;i++){
    rawData = (int16_t)((rxData[2*i] << 8) | rxData[2*i+1]);
    ICM42688_Data.gyroData[i] = (float)(rawData/Gyro_Scale_Factor); 
  }
    
    
//    // 读取X轴陀螺仪数据
//    txData[0] = ICM42688_GYRO_DATA_X1 | ICM42688_READ;
//    HAL_SPI_Transmit(&hspi2, txData, 1, SPI_TIMEOUT);
//    HAL_SPI_Receive(&hspi2, rxData, 2, SPI_TIMEOUT);
//    rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
//    gyroData[0] = (int16_t)(rawData/Gyro_Scale_Factor);
//
//    // 读取Y轴陀螺仪数据
//    txData[0] = ICM42688_GYRO_DATA_Y1 | ICM42688_READ;
//    HAL_SPI_Transmit(&hspi2, txData, 1, SPI_TIMEOUT);
//    HAL_SPI_Receive(&hspi2, rxData, 2, SPI_TIMEOUT);
//    rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
//    gyroData[1] = (int16_t)(rawData/Gyro_Scale_Factor);
//
//    // 读取Z轴陀螺仪数据
//    txData[0] = ICM42688_GYRO_DATA_Z1 | ICM42688_READ;
//    HAL_SPI_Transmit(&hspi2, txData, 1, SPI_TIMEOUT);
//    HAL_SPI_Receive(&hspi2, rxData, 2, SPI_TIMEOUT);
//    rawData = (int16_t)((rxData[0] << 8) | rxData[1]);
//    gyroData[2] = (int16_t)(rawData/Gyro_Scale_Factor);
}



void ICM42688_SENSOR_ENABLE(uint8_t state){
  uint8_t tmp;
  
  //选择bank1
  IMU_SPI_WriteReg(ICM42688_REG_BANK_SEL,ICM42688_BANK1);
  
  HAL_Delay(1);
  
  //选择ENABLE/DISABLE
  if(state)
    tmp = (ICM42688_GYRO_ENABLE << 3) | ICM42688_ACCEL_ENABLE;
  else
    tmp = (ICM42688_GYRO_DISABLE << 3) | ICM42688_ACCEL_DISABLE;

  IMU_SPI_WriteReg(ICM42688_SENSOR_CONFIG0,tmp);
  
  HAL_Delay(1);

  //选择BANK0
  IMU_SPI_WriteReg(ICM42688_REG_BANK_SEL,ICM42688_BANK0);
  
  HAL_Delay(1);
}