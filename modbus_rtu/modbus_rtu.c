#include "modbus_rtu.h"
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"

// STM32F722: 移除FreeRTOS依赖，使用裸机实现
// 使用32位对齐，确保DMA访问正确
__attribute__((aligned(32))) uint8_t rx_buf[RX_BUF_SIZE];
__attribute__((aligned(32))) uint8_t tx_buf[TX_BUF_SIZE];
uint16_t rx_size = 0;
uint16_t sensor_0xF7_cnt = 0;
uint8_t *sensor_UID[MAX_SENSOR_NUM];
uint8_t txFrame[256] = {0};
uint8_t rxFrame[256] = {0};
volatile uint32_t RS485_RX_flag;
extern TIM_HandleTypeDef htim2;

// 微秒级延时，适配216MHz时钟 (STM32F722)
void delay_us(uint16_t us) {
    uint16_t start = TIM4->CNT;
    uint16_t ticks = us; // 需要根据TIM4实际配置调整

    while ((TIM4->CNT - start) < ticks) {
        __NOP();
    }
}

/**
 * @brief 堵塞等待RS485_RX_flag置1
 * @param timeout_ms        堵塞等待时间
 * @retval 0    接收到RS485_RX_flag == 1
 * @retval -1   timeout_ms时间内未收到RS485_RX_flag == 1
 */
static int RS485_RX_flag_Acquire(uint32_t timeout_ms){
    RS485_RX_flag = 0;
    uint32_t start = HAL_GetTick();
    while (RS485_RX_flag == 0) {
        if((uint32_t)(HAL_GetTick() - start) > timeout_ms){
            return -1;
        }
        __NOP();
    }
    return 0;
}

/**
 * @brief UART接收完成回调函数（中断方式）
 * @param huart UART句柄
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == huart4.Instance)
    {
        rx_size = Size;
        // STM32F722没有D-Cache，不需要刷新
        #if USB_TO_RS485_MODE
        CDC_Transmit_HS(rx_buf, Size);
        #endif
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);	// 手动关闭DMA_IT_HT中断
        RS485_RX_flag = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart4.Instance){
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, RX_BUF_SIZE); // 接收发生错误后重启
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);	// 手动关闭DMA_IT_HT中断
        RS485_RX_flag = 1;
    }
}

/**
 * @brief 通用函数：发送请求帧并通过DMA接收响应帧
 */
HAL_StatusTypeDef Modbus_Master_SendReceive(uint8_t *tx_frame, uint16_t txLen, uint8_t *rx_frame)
{   
    // 停止当前接收，准备发送
    HAL_UART_AbortReceive(&huart4);
    memcpy(tx_buf, tx_frame, txLen);

    // DMA 发送
    if(HAL_UART_Transmit_DMA(&huart4, tx_buf, txLen) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 等待发送完成 (TX Complete)
    // 只有等到 TC=1，才能保证数据发完，可以安全切回 RX
    while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET);

    // 立即开启 IDLE 接收 (覆盖盲区)
    // 此时从机刚收到数据，正在处理，我们已经准备好接收了
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT); // 关掉半传输中断干扰

    /* 广播帧处理 */
    if(tx_frame[0] == MB_Broadcast_ID){
         memcpy(rx_frame, tx_frame, txLen);
         return HAL_OK;
    }

    // 9. 等待接收完成 (由 ReceiveToIdle 中断触发)
    if (RS485_RX_flag_Acquire(RX_TIMEOUT_MS) != 0){
        HAL_UART_AbortReceive(&huart4); // 超时停止接收
        return HAL_TIMEOUT;
    }

    if(rx_size > RX_BUF_SIZE){
        return HAL_ERROR;
    }
    memcpy(rx_frame, rx_buf, rx_size);
    return HAL_OK;
}


/* CMD0x50：读字节 */
HAL_StatusTypeDef Modbus_CMD50_ReadBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData)
{
    uint16_t txLen = 6;
    /* 构造请求帧: [slaveId, 0x50, start_reg, data_length, CRC低, CRC高] */
    txFrame[0] = slaveId;
    txFrame[1] = 0x50;
    txFrame[2] = start_reg;
    txFrame[3] = data_length;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 4);
    txFrame[4] = (uint8_t)(crc & 0xFF);
    txFrame[5] = (uint8_t)(crc >> 8);

    /* 计算预期响应帧长度: slaveId + func + data_length字节计数 + 数据(data_length) + CRC(2) */
    //uint16_t rxLen = 1 + 1 + 1 + data_length + 2;

    if(Modbus_Master_SendReceive(txFrame, txLen, rxFrame) == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }

    /* 校验响应：检查slaveId、功能码和数据字节计数 */
    if (rxFrame[0] != slaveId || rxFrame[1] != 0x50 || rxFrame[2] != data_length){
        return HAL_ERROR;
    }

    crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rx_size - 2);
    uint16_t recvCRC = rxFrame[rx_size - 2] | (rxFrame[rx_size - 1] << 8);
    if (crc != recvCRC){
        return HAL_ERROR;
    }

    memcpy(pData, &rxFrame[3], data_length);
    return HAL_OK;
}

/* CMD0x51：写字节（写操作回显） */
HAL_StatusTypeDef Modbus_CMD51_WriteBytes(uint8_t slaveId, uint8_t start_reg, uint8_t data_length, uint8_t *pData)
{
    /* 构造请求帧: [slaveId, 0x51, start_reg, data_length, data..., CRC低, CRC高] */
    uint16_t txLen = 4 + data_length;

    txFrame[0] = slaveId;
    txFrame[1] = 0x51;
    txFrame[2] = start_reg;
    txFrame[3] = data_length;
    memcpy(&txFrame[4], pData, data_length);
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, txLen);
    txFrame[txLen++] = (uint8_t)(crc & 0xFF);
    txFrame[txLen++] = (uint8_t)(crc >> 8);

    /* 预期响应帧与请求帧一致 */
    if(Modbus_Master_SendReceive(txFrame, txLen, rxFrame) == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }

    /* 简单对比响应与请求是否一致 */
    if (memcmp(txFrame, rxFrame, txLen) != 0){
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* CMD0x60：触发测量 */
HAL_StatusTypeDef Modbus_CMD60_TriggerMeasurement(uint8_t slaveId)
{
    /* 构造请求帧: [slaveId, 0x60, CRC低, CRC高] */
    uint16_t txLen = 4;

    txFrame[0] = slaveId;
    txFrame[1] = 0x60;
    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 2);
    txFrame[2] = (uint8_t)(crc & 0xFF);
    txFrame[3] = (uint8_t)(crc >> 8);
        
    /* 预期响应帧与请求帧一致 */
    if(Modbus_Master_SendReceive(txFrame, txLen, rxFrame) == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }
    // delay_us(50);

    if (memcmp(txFrame, rxFrame, txLen) != 0){
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* CMD0x61：请求回报UID */
HAL_StatusTypeDef Modbus_CMD61_BroadcastReportUID(uint8_t UID8_lower, uint8_t UID8_upper, uint8_t delay_max, uint8_t UID_length)
{
    /* 构造请求帧: [slaveId, 0x61, 8bit_UID_Lower, 8bit_UID_Upper, delay_min, delay_max, return_UID_length, CRC低, CRC高] */
    uint16_t txLen = 11;

    txFrame[0] = MB_Temp_ID;
    txFrame[1] = 0x61;
    txFrame[2] = UID8_lower;
    txFrame[3] = UID8_upper;
    txFrame[4] = 0x00;  //MBID_lower;
    txFrame[5] = 0xFF;  //MBID_upper;
    txFrame[6] = 0x00;  //delay_min;
    txFrame[7] = delay_max;
    txFrame[8] = UID_length;

    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, 9);
    txFrame[9] = (uint8_t)(crc & 0xFF);
    txFrame[10] = (uint8_t)(crc >> 8);

    //清空UID记录
    sensor_0xF7_cnt = 0;
    for (uint8_t i = 0; i < MAX_SENSOR_NUM; i++) {
        if (sensor_UID[i] != NULL) {
            free(sensor_UID[i]);
            sensor_UID[i] = NULL;
        }
    }
    #if RX485_TX_USE_DMA
    // 停止当前接收，准备发送
    HAL_UART_AbortReceive(&huart4);
    memcpy(tx_buf, txFrame, txLen);
    // DMA 发送
    HAL_UART_Transmit_DMA(&huart4, tx_buf, txLen);
    // 等待发送完成
    while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET);
    // 立即重启DMA接收，准备接收响应帧
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    #else
    HAL_UART_Transmit(&huart4, txFrame, txLen, 100);
    #endif

    /*等待delay_max*FACTOR(100ms)时间接收帧，每接收一个刷新时间*/
    while(RS485_RX_flag_Acquire(delay_max*REPORT_UID_DELAY_FACTOR) == 0){
        memcpy(rxFrame, rx_buf, rx_size);
        /* 预期响应帧长度: slaveId + func + Return_UID_Length + UID数据(UID_length) + CRC(2) */
        if (rx_size != 1+1+1+UID_length+2){
            continue;
        }
        crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)rxFrame, rx_size - 2);
        uint16_t recvCRC = rxFrame[rx_size - 2] | (rxFrame[rx_size - 1] << 8);
        if (crc != recvCRC){
            continue;
        }

        sensor_UID[sensor_0xF7_cnt] = (uint8_t *)malloc(UID_length);
        memcpy(sensor_UID[sensor_0xF7_cnt],&rxFrame[3],UID_length);
        sensor_0xF7_cnt++;
    }
    
    if(sensor_0xF7_cnt != 0){
        return HAL_OK;
    }
    return HAL_TIMEOUT;
}

/* CMD0x62：根据UID设置从机地址 */
HAL_StatusTypeDef Modbus_CMD62_BroadcastSetSlaveID(uint8_t UID_length, uint8_t *pUID, uint8_t new_slave_id)
{
    /* 构造请求帧: [BoardcastId, 0x62, UID_length, UID数据（高字节先），new_slave_id, CRC低, CRC高] */
    uint16_t txLen = 3 + UID_length + 1;

    txFrame[0] = MB_Temp_ID;
    txFrame[1] = 0x62;
    txFrame[2] = UID_length;
    memcpy(txFrame+3, pUID, UID_length);
    txFrame[txLen-1] = new_slave_id;

    uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)txFrame, txLen);
    txFrame[txLen++] = (uint8_t)(crc & 0xFF);
    txFrame[txLen++] = (uint8_t)(crc >> 8);

    /* 预期响应帧为回显请求帧 */
    uint16_t rxLen = txLen;
    Modbus_Master_SendReceive(txFrame, txLen, rxFrame);
    if (memcmp(txFrame, rxFrame, rxLen) != 0){
        return HAL_ERROR;
    }

    return HAL_OK;
}



