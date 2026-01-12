#include "mag_sensor.h"
#include "cmsis_gcc.h"
#include "modbus_rtu.h"
#include "usbd_cdc_if.h"
// STM32F722: 移除FreeRTOS依赖

MY_SENSOR_module_t mag_sensor[MAX_SENSOR_NUM];
volatile double mcu_timestamp;
volatile uint32_t TIM2_time_s;
uint8_t sensor_num = 0;
uint8_t slaveID_tba;       //slaveID to be allocated
uint32_t slaveID_map[8] = {0};     //bit map of used slaveID
uint32_t sensor_pkg_cnt;
uint32_t sensor_err_pkg_cnt;

void Init_SlaveID_Map(void) {
    memset((void*)slaveID_map, 0, sizeof(slaveID_map));
    //保留MB_Broadcast_ID与MB_MAX_ID~0xFF的ID
    SET_SLAVEID_MAP(MB_Broadcast_ID);
    slaveID_map[7] = 0xFFFFFFFF << (MB_MAX_ID % 32);
    slaveID_tba = 0x01;
}

uint8_t Find_Free_SlaveID(void) {
    for(uint8_t i=0; i<8; i++){
        if(slaveID_map[i] != 0xFFFFFFFF){
            for(uint8_t b=0; b<32; b++){
                if(!(slaveID_map[i] & (1UL << b))){
                    return (i<<5) + b;
                }
            }
        }
    }
    return 0xFF;
}

HAL_StatusTypeDef Get_MagSensors_Plugged(void){
    uint8_t UID_length, error_mark=0;
    uint8_t delay_max;
    uint8_t retry_times;

    //确认slaveID_map中的空闲id
    if(slaveID_tba == 0xFF){
        return HAL_ERROR;
    }

    if(sensor_num == 0)
        delay_max = INITIAL_GETUID_DELAY_TIME;
    else
        delay_max = ADD_GETUID_DELAY_TIME;
    //return uid length = 2
    //return uid length = 4    //红色形态
    UID_length = 4;
    if(Modbus_CMD61_BroadcastReportUID(0x00, 0xFF, delay_max, UID_length) == HAL_OK){
        for(uint8_t i=0; i<sensor_0xF7_cnt; i++){
            //分配ID
            slaveID_tba = Find_Free_SlaveID();
            if(slaveID_tba == 0xFF){
                return HAL_ERROR;
            }
            //设置对应UID未收到正确回复->可能有重复冲突或者干扰
            //重试SET_ID_RETRY_TIMES次
            for(retry_times = SET_ID_RETRY_TIMES; retry_times>0; retry_times--){
                HAL_StatusTypeDef state = Modbus_CMD62_BroadcastSetSlaveID(UID_length, sensor_UID[i], slaveID_tba);
                if(state == HAL_OK){
                    break;
                }
            }
            free(sensor_UID[i]);
            sensor_UID[i] = NULL;
            //分配失败.
            if(retry_times == 0){
                error_mark++;
                continue;
            }
            //分配成功
            mag_sensor[sensor_num].sensor_pub_cfg.mb_slave_id = slaveID_tba;
            #if USE_USB_PRINTF
            usb_printf("Sensor index: %d; SlaveID: %x\n", sensor_num, slaveID_tba);
            #endif
            sensor_num++;   //分配成功的sensor才会被加入sensor_num里

            //处理slaveID_map
            SET_SLAVEID_MAP(slaveID_tba);
            HAL_Delay(10);
        }
    }
    else{
        //没有来自未分配地址的回复
        return HAL_TIMEOUT;
    }
   
    //return uid length = 12    //究极红色形态

    //error_mark表明有无法分配ID的传感器
    if(error_mark == 0)
        return HAL_OK;
    else{
        #if USE_USB_PRINTF
        usb_printf("Set SlaveID Error, error_num=%d\n", error_mark);
        #endif
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef Get_MagSensor_Config(MY_SENSOR_module_t *sensor){
    //get config
    if(Modbus_CMD50_ReadBytes(sensor->sensor_pub_cfg.mb_slave_id, MAG_SENSOR_CONFIG_OFFSET, MAG_SENSOR_FULL_CONFIG_LENGTH + sizeof(SENSOR_Data_t), (uint8_t*)&sensor->sensor_pub_cfg) != HAL_OK){
        //Handle error 
        return HAL_ERROR;
    }
    return HAL_OK;
}


HAL_StatusTypeDef Set_MagSensor_Config(uint8_t *RS485_buf){
    HAL_StatusTypeDef state;
    if(RS485_buf[0] == MB_Broadcast_ID){
        //if private config is included
        if(RS485_buf[1]+RS485_buf[2] > MAG_SENSOR_PUBLIC_CONFIG_LENGTH){
            return HAL_ERROR;
        }
        //state = Modbus_CMD51_WriteBytes(MB_Broadcast_ID, MAG_SENSOR_CONFIG_OFFSET + 1, MAG_SENSOR_CONFIG_LENGTH - 1, ((uint8_t*)new_full_cfg)+1);
    }
    state = Modbus_CMD51_WriteBytes(RS485_buf[0], RS485_buf[1], RS485_buf[2], &RS485_buf[3]);

    HAL_Delay(500);

    if(state != HAL_OK){
        //Handle error
        return HAL_ERROR;
    }
    return HAL_OK;
}


HAL_StatusTypeDef Get_MagSensors_Data(void){
    #if USE_MAG_SENSOR_DRDY// 等待mag_sensor_DRDY，多个传感器任意一个DRDY后即开始读数据
    uint8_t mag_sensor_rdy = 0;
    do{
        HAL_Delay(1);
        for(uint8_t i=0; i<sensor_num; i++){
            Modbus_CMD50_ReadBytes(mag_sensor[i].sensor_cfg.mb_slave_id, offsetof(MAG_SENSOR_module_t, mag_sensor_DRDY), 0x01, &mag_sensor_rdy);
            if(mag_sensor_rdy == 0x01){
                break;
            }
        }
    }while(mag_sensor_rdy != 0x01);
    #endif

    for(uint8_t i=0; i<sensor_num; i++){
        #if ONLY_GET_SENSOR_MAGVAL  //only get magVal[3]
        HAL_StatusTypeDef state = Modbus_CMD50_ReadBytes(
            mag_sensor[i].sensor_pub_cfg.mb_slave_id,
            offsetof(MY_SENSOR_module_t, mag_data) + offsetof(MAG_Data_t, magVal),
            12,
            (uint8_t *)&mag_sensor[i].mag_data.magVal
        );
        #else                       //get all data
        HAL_StatusTypeDef state = Modbus_CMD50_ReadBytes(mag_sensor[i].sensor_cfg.mb_slave_id, MAG_SENSOR_DATA_OFFSET, MAG_SENSOR_DATA_LENGTH, (uint8_t*)&mag_sensor[i]+MAG_SENSOR_DATA_OFFSET);
        #endif 
        if(state != HAL_OK){
            sensor_err_pkg_cnt++;
            __NOP();
            continue;
        }
        sensor_pkg_cnt++;
        delay_us(10);
    }
    return HAL_OK;
}

HAL_StatusTypeDef Check_MagSensors_SlaveID(void){
    uint8_t temp_slaveID = 0;

    //初始化sensor_num与slaveID_map
    sensor_num = 0;
    Init_SlaveID_Map();

    //这里改成问whoami
    //轮询确认已有的slaveID
    for(uint8_t i=1; i<MB_MAX_ID; i++){
        HAL_StatusTypeDef state=Modbus_CMD50_ReadBytes(i, 0x00, 0x01, &temp_slaveID);
        if(state == HAL_OK){
            //分配slaveID_map
            SET_SLAVEID_MAP(temp_slaveID);
            //记录slaveID
            mag_sensor[sensor_num].sensor_pub_cfg.mb_slave_id = temp_slaveID;
            sensor_num++;
            HAL_Delay(1);
            #if USE_USB_PRINTF
            usb_printf("Sensor index: %d; SlaveID: %x\n", sensor_num, temp_slaveID);
            #endif
        }
        else if(state == HAL_ERROR){
            //出现冲突，地址配置为0xF7
            RESET_SLAVEID_MAP(temp_slaveID);
            temp_slaveID = MB_Temp_ID;
            HAL_Delay(1);
            Modbus_CMD51_WriteBytes(i, 0x00, 0x01, &temp_slaveID);
            #if USE_USB_PRINTF
            usb_printf("SlaveID Conflict: %x\n", temp_slaveID);
            #endif
        }
        delay_us(10);
    }

    if(sensor_num == 0)
        return HAL_TIMEOUT;
    return HAL_OK;
}

double Update_TimeStamp(void) {
    mcu_timestamp = (double)TIM2_time_s + (double)(TIM2->CNT)/1000000.0f;
    return mcu_timestamp;
}

HAL_StatusTypeDef Get_DataReady(void){
    uint8_t sensor_1_datardy = 0;
    uint8_t sensor_index = 0;
    uint16_t datardy_timeout_cnt = 0;

    while(sensor_1_datardy != 1){
      //wait data ready
      HAL_StatusTypeDef state = Modbus_CMD50_ReadBytes(
        mag_sensor[sensor_index].sensor_pub_cfg.mb_slave_id, 
        offsetof(MY_SENSOR_module_t, sensor_data) + offsetof(SENSOR_Data_t,sensor_DRDY), 
        0x01, 
        &sensor_1_datardy
      );
      if(state == HAL_TIMEOUT){
        sensor_index++;
        if(sensor_index>=sensor_num)
            return HAL_ERROR;
      }
      datardy_timeout_cnt++;
      if(datardy_timeout_cnt>10000){
        return HAL_TIMEOUT;
      }
    }
    return HAL_OK;
}

uint8_t PC_Trans_Buff[1024] = {0};
uint16_t PC_TRANS_Assemble(double PC_TRANS_timestamp)
{
    // volatile uint32_t temp;
    uint16_t mag_idx = 0;
    uint16_t ptr = 0;

    // PC_Trans_Buff[ptr++] = 0x55;
    // PC_Trans_Buff[ptr++] = 0xaa;
    // PC_Trans_Buff[ptr++] = 0xff;
    // PC_Trans_Buff[ptr++] = (sensor_num) & 0xff;

    // memcpy((uint8_t *)&temp, (uint8_t *)&timestamp, 4);
    // PC_Trans_Buff[ptr++] = (temp) & 0xff;
    // PC_Trans_Buff[ptr++] = (temp >> 8) & 0xff;
    // PC_Trans_Buff[ptr++] = (temp >> 16) & 0xff;
    // PC_Trans_Buff[ptr++] = (temp >> 24) & 0xff;

    // float temp_sensor_error_pkg_percentage = ((10000.0*sensor_err_pkg_cnt)/sensor_pkg_cnt);
    // memcpy((uint8_t *)&temp, (uint8_t *)&temp_sensor_error_pkg_percentage, 4);
    // PC_Trans_Buff[ptr++] = (temp) & 0xff;
    // PC_Trans_Buff[ptr++] = (temp >> 8) & 0xff;
    // PC_Trans_Buff[ptr++] = (temp >> 16) & 0xff;
    // PC_Trans_Buff[ptr++] = (temp >> 24) & 0xff;

    // for (mag_idx = 0; mag_idx < sensor_num; mag_idx++){
    //     for(uint8_t i = 0; i<3; i++){
    //         //assemble float magVal[3]
    //         memcpy((uint8_t *)&temp, (uint8_t *)&mag_sensor[mag_idx].magVal[i], 4);
    //         PC_Trans_Buff[ptr++] = (temp) & 0xff;
    //         PC_Trans_Buff[ptr++] = (temp >> 8) & 0xff;
    //         PC_Trans_Buff[ptr++] = (temp >> 16) & 0xff;
    //         PC_Trans_Buff[ptr++] = (temp >> 24) & 0xff;
    //     }
    // }
    // return ptr;

    // frame header
    PC_Trans_Buff[ptr++] = 0x55;
    PC_Trans_Buff[ptr++] = 0xaa;
    PC_Trans_Buff[ptr++] = 0xff;
    // mag sensor num
    PC_Trans_Buff[ptr++] = sensor_num;
    // mag sensor timestamp
    memcpy((uint8_t *)(PC_Trans_Buff+ptr), (uint8_t *)&PC_TRANS_timestamp, 8);
    ptr += 8;

    // mag sensor error pkg percentage
    float temp_sensor_error_pkg_percentage = ((10000.0*sensor_err_pkg_cnt)/sensor_pkg_cnt);
    memcpy((uint8_t *)(PC_Trans_Buff+ptr), (uint8_t *)&temp_sensor_error_pkg_percentage, 4);
    ptr += 4;

    // mag sensor data
    for (mag_idx = 0; mag_idx < sensor_num; mag_idx++){
        // mag sensor slave id
        PC_Trans_Buff[ptr++] = mag_sensor[mag_idx].sensor_pub_cfg.mb_slave_id;
        // mag sensor UID32
        memcpy((uint8_t *)(PC_Trans_Buff+ptr), (uint8_t *)&mag_sensor[mag_idx].sensor_data.UID32, 4);
        ptr += 4;
        for(uint8_t i = 0; i<3; i++){
            //mag sensor float magVal[3]
            memcpy((uint8_t *)(PC_Trans_Buff+ptr), (uint8_t *)&mag_sensor[mag_idx].mag_data.magVal[i], 4);
            ptr += 4;
        }
    }
    // crc16
    uint16_t crc16 = HAL_CRC_Calculate(&hcrc, (uint32_t *)PC_Trans_Buff, ptr);
    PC_Trans_Buff[ptr++] = (crc16      ) & 0xff;
    PC_Trans_Buff[ptr++] = (crc16 >>  8) & 0xff;
    return ptr;

    // /*** old data format ***/
    // PC_Trans_Buff[ptr++] = 0x55;
    // PC_Trans_Buff[ptr++] = 0xaa;
    // PC_Trans_Buff[ptr++] = 0xff;
    // PC_Trans_Buff[ptr++] = (sensor_num) & 0xff;
    // PC_Trans_Buff[ptr++] = (sensor_num) & 0xff;

    // for (mag_idx = 0; mag_idx < sensor_num; mag_idx++){
    //     for(uint8_t i = 0; i<3; i++){
    //         //assemble float magVal[3]
    //         memcpy((uint8_t *)&temp, (uint8_t *)&mag_sensor[mag_idx].magVal[i], 4);
    //         PC_Trans_Buff[ptr++] = (temp >> 24) & 0xff;
    //         PC_Trans_Buff[ptr++] = (temp >> 16) & 0xff;
    //         PC_Trans_Buff[ptr++] = (temp >>  8) & 0xff;
    //         PC_Trans_Buff[ptr++] = (temp      ) & 0xff; 
    //     }
    // }
    // return ptr;
}