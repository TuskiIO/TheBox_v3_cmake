/**
 * @file    usb_comm.c
 * @brief   USB通信封装层实现
 */

#include "usb_comm.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* 私有变量 */
static usb_recv_callback_t recv_callbacks[MAX_RECV_CALLBACKS];
static int registered_callback_cnt = 0;

/* 外部变量 */
extern int host_port_open;  // 来自usbd_cdc_if.c

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
    if (CDC_Transmit_HS((uint8_t*)data, (uint16_t)data_len) == USBD_OK) {
        return (int)data_len;
    }

    return -3;
}

/**
 * @brief USB接收处理函数
 */
void usb_comm_receive_handler(uint8_t *data, uint32_t data_len)
{
    // 调用所有已注册的回调函数
    for (int i = 0; i < registered_callback_cnt; i++) {
        if (recv_callbacks[i].recv_fn != NULL) {
            recv_callbacks[i].recv_fn(recv_callbacks[i].args, data, data_len);
        }
    }
}

/**
 * @brief USB printf函数
 */
void usb_printf(const char* format, ...)
{
    static char char_buf[256];  // 静态缓冲区用于printf输出

    if (!host_port_open) {
        return;
    }

    va_list args;
    va_start(args, format);
    int len = vsnprintf(char_buf, sizeof(char_buf), format, args);
    va_end(args);

    if (len > 0) {
        // 等待直到USB可以发送
        while (CDC_Transmit_HS((uint8_t*)char_buf, (uint16_t)len) == USBD_BUSY) {
            // 短暂延时
            HAL_Delay(1);
        }
    }
}
