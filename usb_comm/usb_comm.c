/**
 * @file    usb_comm.c
 * @brief   USB通信封装层实现
 */

#include "usb_comm.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

/* 外部变量和函数 */
extern int host_port_open;  // 来自usbd_cdc_if.c
extern CRC_HandleTypeDef hcrc;  // CRC句柄

/* 私有变量 */
static usb_recv_callback_t recv_callbacks[MAX_RECV_CALLBACKS];
static int registered_callback_cnt = 0;
static uint8_t response_buffer[1024];  // 响应数据缓冲区


/**
 * @brief 初始化USB通信模块
 */
void usb_comm_init(void)
{
    // 初始化回调数组
    for (int i = 0; i < MAX_RECV_CALLBACKS; i++) {
        recv_callbacks[i].recv_fn = NULL;
        recv_callbacks[i].args = NULL;
    }
    registered_callback_cnt = 0;
}

/**
 * @brief 注册接收回调函数
 */
int usb_comm_register_recv(usb_recv_function recv_fn, void *args)
{
    if (registered_callback_cnt >= MAX_RECV_CALLBACKS) {
        return -1;  // 回调已满
    }

    recv_callbacks[registered_callback_cnt].recv_fn = recv_fn;
    recv_callbacks[registered_callback_cnt].args = args;
    registered_callback_cnt++;

    return 0;
}

/**
 * @brief 通过USB发送数据
 */
int usb_comm_send(void *data, uint32_t data_len)
{
    if (data == NULL || data_len == 0) {
        return -1;
    }

    // 检查USB是否已连接
    if (!host_port_open) {
        return -2;
    }

    // 调用USB CDC发送函数
    #if USE_USB_HS
        if (CDC_Transmit_HS((uint8_t*)data, (uint16_t)data_len) == USBD_OK) {
    #else
        if (CDC_Transmit_FS((uint8_t*)data, (uint16_t)data_len) == USBD_OK) {
    #endif
        return (int)data_len;
    }

    return -3;
}

/**
 * @brief USB接收处理函数
 */
void usb_comm_receive_handler(uint8_t *data, uint32_t data_len)
{
    // 首先解析命令
    usb_comm_parse_command(data, data_len);

    // 调用所有已注册的回调函数（兼容原有机制）
    for (int i = 0; i < registered_callback_cnt; i++) {
        if (recv_callbacks[i].recv_fn != NULL) {
            recv_callbacks[i].recv_fn(recv_callbacks[i].args, data, data_len);
        }
    }
}

/**
 * @brief USB命令解析处理函数
 * @note  解析命令帧头和命令类型，设置相应的标志位
 */
void usb_comm_parse_command(uint8_t *data, uint32_t data_len)
{
    // 检查最小帧长度：帧头(4) + 命令类型(1) + 数据长度(1) + CRC(2) = 8字节
    if (data_len < 8) {
        return;
    }

    // 检查命令帧头: 0xAA 0x55 0x00 0xFF
    if (data[0] != FRAME_HEADER_CMD_0 ||
        data[1] != FRAME_HEADER_CMD_1 ||
        data[2] != FRAME_HEADER_CMD_2 ||
        data[3] != FRAME_HEADER_CMD_3) {
        return;  // 帧头不匹配，忽略
    }

    // 提取命令类型和数据长度
    uint8_t cmd_type = data[4];
    uint8_t cmd_data_len = data[5];

    // 验证CRC16
    uint16_t calc_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, data_len - 2);
    uint16_t recv_crc = data[data_len - 2] | (data[data_len - 1] << 8);
    if (calc_crc != recv_crc) {
        return;  // CRC校验失败，忽略
    }

    // 处理不同命令类型
    switch (cmd_type) {
        case USB_CMD_GET_DATA:
            // 数据采集控制命令: data[0] = 1(开始), 0(停止)
            if (cmd_data_len >= 1) {
                usb_get_sensor_data_flag = data[6];
            }
            break;

        case USB_CMD_SET_CFG:
            // 设置传感器配置命令
            if (cmd_data_len >= 1 && cmd_data_len <= USB_TO_485_BUF_SIZE) {
                memcpy(usb_to_485_buf, &data[6], cmd_data_len);
                usb_set_sensor_cfg_flag = 1;
            }
            break;

        case USB_CMD_GET_CFG:
            // 读取传感器配置命令
            if (cmd_data_len >= 1) {
                memcpy(usb_to_485_buf, &data[6], cmd_data_len);
                usb_get_sensor_cfg_flag = 1;
            }
            break;

        default:
            // 未知命令
            break;
    }
}

/**
 * @brief 打包并发送响应数据
 * @param data      响应数据内容
 * @param data_len  数据长度
 * @retval 0 成功，<0 失败
 * @note  响应帧使用3字节帧头: 0x55 0xAA 0xFF
 */
int usb_comm_send_response(uint8_t *data, uint16_t data_len)
{
    // 检查参数
    if (data == NULL || data_len == 0) {
        return -1;
    }

    // 检查缓冲区大小 (帧头3字节 + 数据 + CRC2字节)
    if (data_len + 5 > sizeof(response_buffer)) {
        return -2;  // 数据过长
    }

    // 打包响应帧: | 0x55 0xAA 0xFF | DATA[] | CRC16_L | CRC16_H |
    uint16_t ptr = 0;

    // 帧头 (3字节)
    response_buffer[ptr++] = FRAME_HEADER_RESP_0;
    response_buffer[ptr++] = FRAME_HEADER_RESP_1;
    response_buffer[ptr++] = FRAME_HEADER_RESP_2;

    // 数据内容
    memcpy(&response_buffer[ptr], data, data_len);
    ptr += data_len;

    // 计算并添加CRC16
    uint16_t crc16 = HAL_CRC_Calculate(&hcrc, (uint32_t*)response_buffer, ptr);
    response_buffer[ptr++] = crc16 & 0xFF;
    response_buffer[ptr++] = (crc16 >> 8) & 0xFF;

    // 发送响应
    return usb_comm_send(response_buffer, ptr);
}

// usb_printf函数已在usbd_cdc_if.c中实现，此处不重复定义
