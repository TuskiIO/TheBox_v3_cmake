#ifndef __ICM42688P_H
#define __ICM42688P_H

#include "stm32f7xx_hal.h"

#define SPI3_TIMEOUT 100
#define SPI3_USE_SOFT_NSS   1
#if SPI3_USE_SOFT_NSS
#define IMU_SPI_NSS_H()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define IMU_SPI_NSS_L()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#else //HARDWARE_NSS cannot use
#define IMU_SPI_NSS_H()    __NOP()
#define IMU_SPI_NSS_L()    __NOP()
#endif

typedef struct{
  float       accelData[3];
  float       gyroData[3];
  float       tempData;
}ICM42688_Data_t;

extern ICM42688_Data_t ICM42688_Data;

void IMU_SPI_ReadReg(uint8_t reg_addr, uint8_t *rxData, uint16_t len);
void IMU_SPI_WriteReg(uint8_t reg_addr, uint8_t value);

void ICM42688_Init(void);
void ICM42688_GetAccel(void);
void ICM42688_GetGyro(void);
void ICM42688_GetTemper(void);
void ICM42688_SENSOR_ENABLE(uint8_t state);



// 定义ICM42688P的寄存器地址
#define ICM42688_WHO_AM_I               0x75  //用于读取设备ID，应为0x42
#define ICM42688_REG_BANK_SEL           0x76  //bank地址选择

//bank 0
#define ICM42688_DRIVE_CONFIG           0x13  //配置SPI slew rate
#define ICM42688_INT_CONFIG             0x14  //用于配置中断信号的极性和驱动方式
#define ICM42688_FIFO_CONFIG            0x16  //配置FIFO模式
#define ICM42688_PWR_MGMT0              0x4E  //配置加速度计和陀螺仪的工作模式

#define ICM42688_GYRO_CONFIG0           0x4F  //配置陀螺仪的量程和采样率
#define ICM42688_ACCEL_CONFIG0          0x50  //配置加速度计的量程和采样率
#define ICM42688_GYRO_ACCEL_CONFIG0     0x52  //配置陀螺仪和加速度计的滤波器

#define ICM42688_TEMP_DATA1             0x1D // 用于读取温度数据的高字节
#define ICM42688_TEMP_DATA0             0x1E // 用于读取温度数据的低字节
#define ICM42688_ACCEL_DATA_X1          0x1F // 用于读取加速度计X轴数据的高字节
#define ICM42688_ACCEL_DATA_X0          0x20 // 用于读取加速度计X轴数据的低字节
#define ICM42688_ACCEL_DATA_Y1          0x21 // 用于读取加速度计Y轴数据的高字节
#define ICM42688_ACCEL_DATA_Y0          0x22 // 用于读取加速度计Y轴数据的低字节
#define ICM42688_ACCEL_DATA_Z1          0x23 // 用于读取加速度计Z轴数据的高字节
#define ICM42688_ACCEL_DATA_Z0          0x24 // 用于读取加速度计Z轴数据的低字节
#define ICM42688_GYRO_DATA_X1           0x25 // 用于读取陀螺仪X轴数据的高字节
#define ICM42688_GYRO_DATA_X0           0x26 // 用于读取陀螺仪X轴数据的低字节
#define ICM42688_GYRO_DATA_Y1           0x27 // 用于读取陀螺仪Y轴数据的高字节
#define ICM42688_GYRO_DATA_Y0           0x28 // 用于读取陀螺仪Y轴数据的低字节
#define ICM42688_GYRO_DATA_Z1           0x29 // 用于读取陀螺仪Z轴数据的高字节
#define ICM42688_GYRO_DATA_Z0           0x2A // 用于读取陀螺仪Z轴数据的低字节


// 定义ICM42688P的读写命令
#define ICM42688_READ                   0x80u // 读命令，寄存器地址的最高位为1
#define ICM42688_WRITE                  0x00u // 写命令，寄存器地址的最高位为0

//0x75 定义ICM42688P的设备ID
#define ICM42688_ID                     0x47 // 设备ID，应与WHO_AM_I寄存器的值相同

//0x76 选择bank
#define ICM42688_BANK0                  0x00u
#define ICM42688_BANK1                  0x01u
#define ICM42688_BANK2                  0x02
#define ICM42688_BANK3                  0x03
#define ICM42688_BANK4                  0x04

//0x13  选择SPI slew rate
#define SPI_SLEW_RATE_2NS               0x05  //<2ns (default)
#define SPI_SLEW_RATE_6NS               0x04  //2ns~6ns


//0x4E 定义ICM42688P的加速度计和陀螺仪的工作模式
#define ICM42688_MODE_OFF               0x00  // 关闭模式，PWR_MGMT0寄存器的第0-1位或第2-3位为00(default)
#define ICM42688_MODE_STANDBY           0x01  // 待机模式，PWR_MGMT0寄存器的第0-1位或第2-3位为01
#define ICM42688_MODE_LOW_POWER         0x02  // 低功耗模式，PWR_MGMT0寄存器的第0-1位或第2-3位为10
#define ICM42688_MODE_LOW_NOISE         0x03  // 低噪声模式，PWR_MGMT0寄存器的第0-1位或第2-3位为11


//0x4F[7:5] 定义ICM42688P的GYRO的量程
#define ICM42688_GYRO_FS_2000           0x00  //±2000dps(default)
#define ICM42688_GYRO_FS_1000           0x01
#define ICM42688_GYRO_FS_500            0x02
#define ICM42688_GYRO_FS_250            0x03
#define ICM42688_GYRO_FS_125            0x04
#define ICM42688_GYRO_FS_62_5           0x05
#define ICM42688_GYRO_FS_32_25          0x06
#define ICM42688_GYRO_FS_15_625         0x07

//0x50[7:5] 定义ICM42688P的ACCEL的量程
#define ICM42688_ACCEL_FS_16G           0x00  //±16g(default)
#define ICM42688_ACCEL_FS_8G            0x01
#define ICM42688_ACCEL_FS_4G            0x02
#define ICM42688_ACCEL_FS_2G            0x03


//0x4F[3:0] GYRO 0x50[3:0] ACCEL 定义ICM42688P的加速度计和陀螺仪的采样频率
#define ICM42688_ODR_32KHZ              0x01  //LN
#define ICM42688_ODR_16KHZ              0x02  //LN
#define ICM42688_ODR_8KHZ               0x03  //LN
#define ICM42688_ODR_4KHZ               0x04  //LN
#define ICM42688_ODR_2KHZ               0x05  //LN
#define ICM42688_ODR_1KHZ               0x06  //LN  (default)
#define ICM42688_ODR_200HZ              0x07  //LN or LP
#define ICM42688_ODR_100HZ              0x08  //LN or LP
#define ICM42688_ODR_50HZ               0x09  //LN or LP
#define ICM42688_ODR_25HZ               0x0A  //LN or LP
#define ICM42688_ODR_12_5HZ             0x0B  //LN or LP
#define ICM42688_ODR_6_25HZ             0x0C  //LP only ACCEL
#define ICM42688_ODR_3_125HZ            0x0D  //LP only ACCEL
#define ICM42688_ODR_1_5625HZ           0x0E  //LP only ACCEL
#define ICM42688_ODR_500HZ              0x0F  //LN or LP



// 定义ICM42688P的温度数据转换系数
#define ICM42688_TEMP_SENS              132.48  // 温度数据的灵敏度，单位为LSB/℃
#define ICM42688_TEMP_OFFSET            25.0    // 温度数据的偏移量，单位为℃

// 定义ICM42688P的加速度计数据转换系数
#define ICM42688_ACCEL_SENS_16G         2048  //±16g量程下的加速度数据的灵敏度，单位为LSB/g
#define ICM42688_ACCEL_SENS_8G          4096
#define ICM42688_ACCEL_SENS_4G          8192
#define ICM42688_ACCEL_SENS_2G          16384

#define ICM42688_ACCEL_G                9.8     // m/s^2/g


// 定义ICM42688P的陀螺仪数据转换系数
#define ICM42688_GYRO_SENS_2000           16.4 // ±2000dps量程下的陀螺仪数据的灵敏度，单位为LSB/dps
#define ICM42688_GYRO_SENS_1000           32.8
#define ICM42688_GYRO_SENS_500            65.5
#define ICM42688_GYRO_SENS_250            131.0
#define ICM42688_GYRO_SENS_125            262.0
#define ICM42688_GYRO_SENS_62_5           524.3
#define ICM42688_GYRO_SENS_32_25          1048.6
#define ICM42688_GYRO_SENS_15_625         2097.2

//bank1
#define ICM42688_SENSOR_CONFIG0         0x03  //sensor disable

//bank1 0x03 SENSOR_CONFIG0
#define ICM42688_GYRO_ENABLE            0x00  //gyro on
#define ICM42688_GYRO_DISABLE           0x03  //gyro off
#define ICM42688_ACCEL_ENABLE           0x00  //accel on
#define ICM42688_ACCEL_DISABLE          0x03  //accel off


// 定义ICM42688P的配置结构体
typedef struct
{
  uint8_t accel_mode;           // 加速度计工作模式，关闭，待机，低功耗或低噪声
  uint8_t gyro_mode;            // 陀螺仪工作模式，关闭，待机，低功耗或低噪声

  uint8_t gyro_fs;              // 陀螺仪量程，±16000dps，±8000dps，±4000dps或±2000dps
  uint8_t gyro_odr;             // 陀螺仪采样频率，1kHz，500Hz，200Hz，100Hz，50Hz，25Hz，12.5Hz，6.25Hz，3.125Hz，1.5625Hz或500Hz（低功耗模式）
  
  uint8_t accel_fs;             // 加速度计量程，±16g，±8g，±4g或±2g
  uint8_t accel_odr;            // 加速度计采样频率，1kHz，500Hz，200Hz，100Hz，50Hz，25Hz，12.5Hz，6.25Hz，3.125Hz，1.5625Hz或500Hz（低功耗模式）

  uint8_t accel_ui_filt_bw;
  uint8_t gyro_ui_filt_bw;
} ICM42688_ConfigTypeDef;


#endif