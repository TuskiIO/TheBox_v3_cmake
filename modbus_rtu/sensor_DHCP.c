#include "sensor_DHCP.h"
#include "stm32h7xx_hal_flash.h"
#include <string.h>


// FLASH操作函数
static DHCP_Status Flash_EraseSector(void);
static DHCP_Status Flash_WriteData(uint32_t address, void *data, size_t size);
static DHCP_Status Flash_ReadData(uint32_t address, void *data, size_t size);

extern CRC_HandleTypeDef hcrc;
static uint32_t Calculate_CRC32(const uint8_t *data, size_t length) {
    return HAL_CRC_Calculate(&hcrc, (uint32_t*)data, length);
}

DHCP_Status DHCP_Init(DHCP_Manager *mgr) {
    memset(mgr, 0, sizeof(DHCP_Manager));
    
    // 从FLASH加载数据
    if(Flash_ReadData(FLASH_START_ADDR, &mgr->storage, sizeof(FlashStorage)) != DHCP_OK_NEW)
        return DHCP_FLASH_ERR;
    
    // CRC校验
    uint32_t crc = Calculate_CRC32((uint8_t*)&mgr->storage, sizeof(FlashStorage)-4);
    if(crc != mgr->storage.crc32) {
        memset(&mgr->storage, 0, sizeof(FlashStorage));
        return DHCP_FLASH_ERR;
    }
    
    // 构建位图
    for(int i=0; i<MAX_SENSOR_NUM; i++) {
        if(mgr->storage.entries[i].UID != 0) {
            uint8_t id = mgr->storage.entries[i].slaveID;
            if(id < MAX_SENSOR_NUM) {
                mgr->id_bitmap[id/8] |= (1 << (id%8));
            }
        }
    }
    return DHCP_OK_EXISTING;
}

DHCP_Status DHCP_AssignID(DHCP_Manager *mgr, uint32_t uid, uint8_t *assigned_id) {
    // 检查现有UID
    for(int i=0; i<MAX_SENSOR_NUM; i++) {
        if(mgr->storage.entries[i].UID == uid) {
            *assigned_id = mgr->storage.entries[i].slaveID;
            return DHCP_OK_EXISTING;
        }
    }
    
    // 寻找空闲ID
    for(uint8_t id=0; id<MAX_SENSOR_NUM; id++) {
        if(!(mgr->id_bitmap[id/8] & (1 << (id%8)))) {
            // 找到空闲ID
            mgr->storage.entries[id].slaveID = id;
            mgr->storage.entries[id].UID = uid;
            mgr->id_bitmap[id/8] |= (1 << (id%8));
            
            // 更新CRC并保存
            mgr->storage.crc32 = Calculate_CRC32((uint8_t*)&mgr->storage, sizeof(FlashStorage)-4);
            if(Flash_EraseSector() != DHCP_OK_NEW)
                return DHCP_FLASH_ERR;
            if(Flash_WriteData(FLASH_START_ADDR, &mgr->storage, sizeof(FlashStorage)) != DHCP_OK_NEW)
                return DHCP_FLASH_ERR;
            
            *assigned_id = id;
            return DHCP_OK_NEW;
        }
    }
    return DHCP_ERR_FULL;
}

bool DHCP_CheckConflict(DHCP_Manager *mgr, uint8_t slave_id, uint32_t uid) {
    return (slave_id < MAX_SENSOR_NUM) && 
           (mgr->storage.entries[slave_id].UID != uid) &&
           (mgr->storage.entries[slave_id].UID != 0);
}

void DHCP_HandleConflict(DHCP_Manager *mgr, uint8_t slave_id) {
    if(slave_id >= MAX_SENSOR_NUM) return;
    
    mgr->id_bitmap[slave_id/8] &= ~(1 << (slave_id%8));
    memset(&mgr->storage.entries[slave_id], 0, sizeof(SensorEntry));
    
    // 更新存储
    mgr->storage.crc32 = Calculate_CRC32((uint8_t*)&mgr->storage, sizeof(FlashStorage)-4);
    Flash_EraseSector();
    Flash_WriteData(FLASH_START_ADDR, &mgr->storage, sizeof(FlashStorage));
}

/* FLASH底层操作 */
static DHCP_Status Flash_EraseSector(void) {
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_1,
        .Sector = FLASH_SECTOR,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    
    uint32_t sectorError;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sectorError);
    HAL_FLASH_Lock();
    
    return (status == HAL_OK) ? DHCP_OK_NEW : DHCP_FLASH_ERR;
}

static DHCP_Status Flash_WriteData(uint32_t address, void *data, size_t size) {
    HAL_FLASH_Unlock();
    
    uint32_t *pData = (uint32_t*)data;
    size_t words = (size + 3) / 4;  // 转换为32位字数
    
    for(size_t i=0; i<words; i+=8) {  // 每次写入8个字（256位）
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, 
                           address + i*4, 
                           (uint32_t)(pData + i)) != HAL_OK) {
            HAL_FLASH_Lock();
            return DHCP_FLASH_ERR;
        }
    }
    
    HAL_FLASH_Lock();
    return DHCP_OK_NEW;
}

static DHCP_Status Flash_ReadData(uint32_t address, void *data, size_t size) {
    memcpy(data, (void*)address, size);
    return DHCP_OK_NEW;
}