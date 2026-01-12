/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_rtu.h"
#include "mag_sensor.h"
#include "ICM42688P.h"
#include "usb_comm.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// USB端口选择配置：设置为1使用HS，设置为0使用FS（需与usbd_cdc_if.h中保持一致）
#define USE_USB_HS  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */
// CRC句柄（在stm32f7xx_hal_msp.c中初始化）
extern CRC_HandleTypeDef hcrc;
// TIM句柄（用于delay_us，需要在CubeMX中配置）
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// USB通信相关标志
volatile uint8_t usb_get_sensor_data_flag = 1;
volatile uint8_t usb_set_sensor_cfg_flag = 0;
volatile uint8_t usb_get_sensor_cfg_flag = 0;
uint8_t usb_to_485_buf[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI3_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_SPI3_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // 初始化USB设备
  MX_USB_DEVICE_Init();

  // 初始化USB通信层
  usb_comm_init();

  // 使能传感器电源
  HAL_GPIO_WritePin(SENS_PWR_EN_GPIO_Port, SENS_PWR_EN_Pin, GPIO_PIN_SET);

  // 等待传感器上电稳定
  HAL_Delay(1000);

  // 初始化时间戳
  mcu_timestamp = 0;
  TIM2_time_s = 0;
  sensor_pkg_cnt = 0;
  sensor_err_pkg_cnt = 0;

  // 检查已有的SlaveID，配置slaveID_map[8]
  HAL_StatusTypeDef state = Check_MagSensors_SlaveID();

  // 初始化新接入的传感器
  state = Get_MagSensors_Plugged();
  while(state == HAL_OK) {
    state = Get_MagSensors_Plugged();
  };

  // 更新配置
  for(uint8_t i = 0; i < sensor_num; i++) {
    Get_MagSensor_Config(&mag_sensor[i]);
    HAL_Delay(1);
  }

  // 初始化IMU
  ICM42688_Init();

  // 清空rx_buf，开启DMA空闲中断接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 传感器数据采集和发送
    if(usb_get_sensor_data_flag == 1) {
      // 触发测量
      Modbus_CMD60_TriggerMeasurement(MB_Broadcast_ID);
      Update_TimeStamp();
      HAL_Delay(5);

      // 获取传感器数据
      Get_MagSensors_Data();

      // 打包并发送到USB
      uint16_t PC_buf_size = PC_TRANS_Assemble(mcu_timestamp);
      #if USE_USB_HS
        CDC_Transmit_HS(PC_Trans_Buff, PC_buf_size);
      #else
        CDC_Transmit_FS(PC_Trans_Buff, PC_buf_size);
      #endif
    }

    // 处理配置设置命令
    if(usb_set_sensor_cfg_flag == 1) {
      PC_Trans_Buff[0] = 0x55;
      PC_Trans_Buff[1] = 0xaa;
      PC_Trans_Buff[2] = 0xff;
      if(Set_MagSensor_Config(usb_to_485_buf) == HAL_OK) {
        memcpy(&PC_Trans_Buff[3], usb_to_485_buf, usb_to_485_buf[2] + 6);
        uint16_t crc16 = HAL_CRC_Calculate(&hcrc, (uint32_t *)PC_Trans_Buff, usb_to_485_buf[2] + 6);
        PC_Trans_Buff[usb_to_485_buf[2] + 6] = (crc16      ) & 0xff;
        PC_Trans_Buff[usb_to_485_buf[2] + 7] = (crc16 >>  8) & 0xff;
        #if USE_USB_HS
          CDC_Transmit_HS(PC_Trans_Buff, usb_to_485_buf[2] + 8);
        #else
          CDC_Transmit_FS(PC_Trans_Buff, usb_to_485_buf[2] + 8);
        #endif
      } else {
        PC_Trans_Buff[3] = 0x00;
        PC_Trans_Buff[4] = 0x00;
        PC_Trans_Buff[5] = 0x00;
        uint16_t crc16 = HAL_CRC_Calculate(&hcrc, (uint32_t *)PC_Trans_Buff, 0x06);
        PC_Trans_Buff[6] = (crc16      ) & 0xff;
        PC_Trans_Buff[7] = (crc16 >>  8) & 0xff;
        #if USE_USB_HS
          CDC_Transmit_HS(PC_Trans_Buff, 8);
        #else
          CDC_Transmit_FS(PC_Trans_Buff, 8);
        #endif
      }
      usb_set_sensor_cfg_flag = 0;
    }

    // 处理配置读取命令
    if(usb_get_sensor_cfg_flag == 1) {
      for(uint8_t i = 0; i < sensor_num; i++) {
        if(usb_to_485_buf[0] == mag_sensor[i].sensor_pub_cfg.mb_slave_id ||
           usb_to_485_buf[0] == MB_Broadcast_ID) {
          PC_Trans_Buff[0] = 0x55;
          PC_Trans_Buff[1] = 0xaa;
          PC_Trans_Buff[2] = 0xff;
          if(Get_MagSensor_Config(&mag_sensor[i]) == HAL_OK) {
            memcpy(&PC_Trans_Buff[3], (uint8_t *)&mag_sensor[i].sensor_pub_cfg, sizeof(SENSOR_Public_Config_t));
            uint16_t crc16 = HAL_CRC_Calculate(&hcrc, (uint32_t *)PC_Trans_Buff, sizeof(SENSOR_Public_Config_t) + 3);
            PC_Trans_Buff[sizeof(SENSOR_Public_Config_t) + 3] = (crc16      ) & 0xff;
            PC_Trans_Buff[sizeof(SENSOR_Public_Config_t) + 4] = (crc16 >>  8) & 0xff;
            #if USE_USB_HS
              CDC_Transmit_HS(PC_Trans_Buff, sizeof(SENSOR_Public_Config_t) + 5);
            #else
              CDC_Transmit_FS(PC_Trans_Buff, sizeof(SENSOR_Public_Config_t) + 5);
            #endif
          } else {
            PC_Trans_Buff[3] = 0x00;
            PC_Trans_Buff[4] = 0x00;
            PC_Trans_Buff[5] = 0x00;
            uint16_t crc16 = HAL_CRC_Calculate(&hcrc, (uint32_t *)PC_Trans_Buff, 0x06);
            PC_Trans_Buff[6] = (crc16      ) & 0xff;
            PC_Trans_Buff[7] = (crc16 >>  8) & 0xff;
            #if USE_USB_HS
              CDC_Transmit_HS(PC_Trans_Buff, 8);
            #else
              CDC_Transmit_FS(PC_Trans_Buff, 8);
            #endif
          }
        }
      }
      usb_get_sensor_cfg_flag = 0;
    }

    // LED指示灯闪烁
    static uint16_t led_cnt = 0;
    led_cnt++;
    if(led_cnt > 100) {
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      led_cnt = 0;
    }

    HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 215;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 215;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_RESETB_GPIO_Port, USB_RESETB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENS_PWR_EN_GPIO_Port, SENS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_RESETB_Pin */
  GPIO_InitStruct.Pin = USB_RESETB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_RESETB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_PWR_ST_Pin */
  GPIO_InitStruct.Pin = INPUT_PWR_ST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_PWR_ST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = SENS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
