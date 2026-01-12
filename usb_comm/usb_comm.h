/**
 * @file    usb_comm.h
 * @brief   USB通信封装层，替代原UDP通信层
 *          为STM32F722移植项目提供USB CDC通信接口
 * @author 移植自MagLocV2_STM32H7
 */

#ifndef __USB_COMM_H__
#define __USB_COMM_H__

#include "stm32f7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 定义 */
#define USB_COMM_RX_BUF_SIZE    2048
#define USB_COMM_TX_BUF_SIZE    2048
#define MAX_RECV_CALLBACKS      10

/* 类型定义 */
typedef void (*usb_recv_function)(void *arg, void* data, uint32_t recv_len);

/* 回调注册结构体 */
typedef struct {
    usb_recv_function  recv_fn;
    void              *args;
} usb_recv_callback_t;

/**
 * @brief 初始化USB通信模块
 */
void usb_comm_init(void);

/**
 * @brief 注册接收回调函数
 * @param recv_fn 接收回调函数
 * @param args    回调函数参数
 * @retval 0 成功，-1 失败（回调已满）
 */
int usb_comm_register_recv(usb_recv_function recv_fn, void *args);

/**
 * @brief 通过USB发送数据
 * @param data      要发送的数据
 * @param data_len  数据长度
 * @retval 发送的字节数，<0表示失败
 */
int usb_comm_send(void *data, uint32_t data_len);

/**
 * @brief USB接收处理函数（在中断或主循环中调用）
 * @param data      接收到的数据
 * @param data_len  数据长度
 */
void usb_comm_receive_handler(uint8_t *data, uint32_t data_len);

/**
 * @brief USB printf函数（类似标准printf，通过USB发送）
 * @param format    格式化字符串
 * @param ...      可变参数
 */
void usb_printf(const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif /* __USB_COMM_H__ */
