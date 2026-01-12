#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32f7xx_hal.h"

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 256
#define RX_TIMEOUT_MS  5   //10ms  //max 65ms
#define RX485_TX_USE_DMA            1
#define RS485_RX_USE_RTOS_SEMAPHORE 0

#define USB_TO_RS485_MODE           0

#define MB_Broadcast_ID 0x00
#define MB_Temp_ID 0xF7
#define MB_MAX_ID 0xF7      //range:0xE1~0XFF(225~255)

#define REPORT_UID_DELAY_FACTOR 100
#define MAX_SENSOR_NUM 246  //0xF6
#define SET_ID_RETRY_TIMES  2

extern uint8_t rx_buf[];
extern uint8_t tx_buf[];
extern uint16_t sensor_0xF7_cnt;
extern uint8_t *sensor_UID[];

// #pragma pack(push, 1)
// typedef struct {
//     uint8_t slaveID;
//     uint32_t UID;
//     uint8_t reserved[3];  // 填充字节保证8字节对齐
// } SensorID_t;
// #pragma pack(pop)
// /* 数据结构声明 */
// extern SensorID_t Sensor_ID[];

extern UART_HandleTypeDef huart4;  // UART4句柄 (F722替代LPUART1)
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern CRC_HandleTypeDef hcrc;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

void delay_us(uint16_t us);

/**
 * @brief 通用函数：发送请求帧并通过DMA接收响应帧，广播帧不回复，其他帧一发一收
 */
HAL_StatusTypeDef Modbus_Master_SendReceive(uint8_t *txFrame, uint16_t txLen, uint8_t *rxFrame);

/**
 * @brief  CMD80 (0x50)：读取从机特定地址处的字节数据
 * @param  slaveId: 从机地址
 * @param  start_reg: 起始地址（1字节）
 * @param  data_length: 读取字节数
 * @param  pData: 读出数据指针
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x50 | start_reg | data_length | CRC低 | CRC高 |
 * 响应帧格式: | slaveId | 0x50 | data_length | data1 ... dataN | CRC低 | CRC高 |
 */
HAL_StatusTypeDef Modbus_CMD50_ReadBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData);

/**
 * @brief  CMD81 (0x51)：写从机特定地址处的字节数据（写操作后回显）
 * @param  slaveId: 从机地址
 * @param  start_reg: 起始地址（1字节）
 * @param  data_length: 写入数据字节数
 * @param  pData: 待写入的数据缓冲区
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x51 | start_reg | data_length | data1 ... dataN | CRC低 | CRC高 | 
 * 响应帧格式: 同请求帧（回显）
 */
HAL_StatusTypeDef Modbus_CMD51_WriteBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData);

/**
 * @brief  CMD96 (0x60)：触发一次测量
 * @param  slaveId: 从机地址
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x60 | CRC低 | CRC高 |
 * 响应帧格式: 与请求帧一致
 */
HAL_StatusTypeDef Modbus_CMD60_TriggerMeasurement(uint8_t slaveId);

/**
 * @brief  CMD97 (0x61)：请求从机回报UID，向MB_Temp_ID广播，存储sensor_num与UID, *sensor_UID[];
 * @param  UID8_lower: 8位UID
 * @param  UID8_upper: 8位UID最大值
 * @param  delay_max: 最高延时
 * @param  return_UID_length: 要求返回的UID长度（单位字节）
 * @retval HAL_OK       接收到至少一个slave_ID=0xF7的传感器返回帧
 * @retval HAL_ERROR    未收到来自0xF7的有效帧, sensor_num=0
 *
 * 请求帧格式: | SlaveId | 0x61 | 8bit_UID_Lower | 8bit_UID_Upper | MB_ID_Lower | MB_ID_Upper | Delay_Lower | Delay_Upper | UID_Length | CRC_L | CRC_H
 * 响应帧格式: | slaveId | 0x61 | UID_Length | UID数据（长度=return_UID_length） | CRC低 | CRC高 |
 */
HAL_StatusTypeDef Modbus_CMD61_BroadcastReportUID(uint8_t UID8_lower, uint8_t UID8_upper, uint8_t delay_max, uint8_t UID_length);

/**
 * @brief  CMD98 (0x62)：根据UID设置从机地址
 * @param  UID_length: UID的字节数（1、2、4或12）
 * @param  pUID: UID数据缓冲区（按照从低到高存放）
 * @param  new_slave_id: 新的从机地址（合法范围1~247）
 * @retval HAL状态
 *
 * 请求帧格式: | slaveId | 0x62 | UID_length | UID数据（高字节先） | new_slave_id | CRC低 | CRC高 |
 * 响应帧格式: 同请求帧（回显）
 */
HAL_StatusTypeDef Modbus_CMD62_BroadcastSetSlaveID(uint8_t UID_length, uint8_t *pUID, uint8_t new_slave_id);

#endif /* MODBUS_RTU_H */