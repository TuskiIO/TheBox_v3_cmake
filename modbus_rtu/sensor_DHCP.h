#ifndef _SENSOR_DHCP_H_
#define _SENSOR_DHCP_H_

#include "stm32h7xx_hal.h"
#include "modbus_rtu.h"
#include <stdbool.h>

#define FLASH_START_ADDR  0x081E0000  // Bank1 Sector7
#define FLASH_SECTOR      FLASH_SECTOR_7

typedef enum {
    DHCP_OK_EXISTING,  // UID已存在
    DHCP_OK_NEW,       // 分配新ID成功
    DHCP_ERR_FULL,     // ID已耗尽
    DHCP_FLASH_ERR     // FLASH操作失败
} DHCP_Status;

#pragma pack(push, 1)
typedef struct {
    uint8_t slaveID;
    uint32_t UID;
} SensorEntry;

typedef struct {
    uint32_t crc32;
    SensorEntry entries[MAX_SENSOR_NUM];
} FlashStorage;
#pragma pack(pop)

typedef struct {
    FlashStorage storage;
    uint8_t id_bitmap[(MAX_SENSOR_NUM+7)/8];
} DHCP_Manager;

// 初始化函数
DHCP_Status DHCP_Init(DHCP_Manager *mgr);
// ID分配函数
DHCP_Status DHCP_AssignID(DHCP_Manager *mgr, uint32_t uid, uint8_t *assigned_id);
// 冲突检测
bool DHCP_CheckConflict(DHCP_Manager *mgr, uint8_t slave_id, uint32_t uid);
// 冲突处理
void DHCP_HandleConflict(DHCP_Manager *mgr, uint8_t slave_id);


#endif
