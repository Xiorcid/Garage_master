/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <dispcolor.h>
#include <gpio.h>
#include "info_disp.h"
#include "buttons.h"
#include "enc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_INIT    0
#define STATE_DISPLAY 1
#define STATE_800INIT 2

#define SCROLL_MODE_AUTO  0
#define SCROLL_MODE_HALT  1
#define SCROLL_MODE_CLICK 2

#define TRANSMIT_MODE_GET 0
#define TRANSMIT_MODE_SET 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
// DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
#define DEV0_UART &huart5

#define DEV_STATE_IN    1
#define DEV_STATE_UNIN  0


#define SIM800_UART &huart6
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double val = 10;
uint8_t tx_buff[]={84,66,67,68,69,70,71,72,73,74};
uint8_t palette = TEMP_PALETTE;
uint8_t device_type = TYPE_SET_TEL;

uint8_t state = STATE_INIT;
uint8_t dev_num = 0;

bool trySIMInit = 0;

uint8_t screen_scroll_mode = SCROLL_MODE_AUTO;

Button button = {ENC_KEY_GPIO_Port, ENC_KEY_Pin, TYPE_HIGH_PULL};

Encoder enc = {ENC_S1_GPIO_Port, ENC_S1_Pin, ENC_S2_GPIO_Port, ENC_S2_Pin};

// Device device_0 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_2 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_3 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_4 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};

Device deviceList[] = {{&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY}};

SIM sim = {SIM800_UART};

uint8_t rx_buff[10];

#define DEV_COUNT sizeof(deviceList)/sizeof(deviceList[0])

bool transmitMode = TRANSMIT_MODE_GET;

bool isSDInitialised = false;
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
	// gpio_SetGPIOmode_Out(GPIOB, GPIO_PIN_14);
  gpio_SetGPIOmode_Out(GPIOD, GPIO_PIN_2);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);


	// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	// HAL_Delay(1000);
	// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  // init(&button);
  // initEnc(&enc);
  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	dispcolor_Init(240, 240);

	uint32_t dev_init_timeout = HAL_GetTick();
  uint32_t screen_disp_time = HAL_GetTick();
  uint32_t hold_timeout = HAL_GetTick();

  uint32_t simtmr = HAL_GetTick();
  uint32_t txtmr = HAL_GetTick();
  
  uint8_t old_dev_num = 0;

  /* RTC */
  // GET RTC DATETIME
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
  
  if(gDate.Year == 0){
    Show_Message("SET UP RTC!", 1000);
    // RTC SETUP MENU
  }

  

  /* SD CARD */
  FATFS FatFs;
  FRESULT FR_Status;
  FIL Fil;
  UINT bytesWrote;

  FR_Status = f_mount(&FatFs, "", 1);
  if (FR_Status == FR_OK){
    isSDInitialised = true;
  }else{
    Show_Message("Problem with SD card!", 1000);
  }


  deviceList[0].tx_buff[0] = 84;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { // NEW CODE START
    if(state == STATE_INIT){
      Display_Init(dev_num, DEV_COUNT);
      HAL_UART_Transmit_DMA(deviceList[dev_num].uart, deviceList[dev_num].tx_buff, 10);
      HAL_UART_Receive_DMA(deviceList[dev_num].uart, deviceList[dev_num].rx_buff, 10);
    }
    // else if (state == STATE_800INIT){
    //   trySIMInit = 1;
    //   state = STATE_DISPLAY;
    //   //SIM_Init("TRY TO", "INIT SIM");
    //   HAL_UART_Transmit(sim.uart, "AT\r\n", 5, 1000); //UART8 NO DMA TX
    //   HAL_UART_Receive_DMA(sim.uart, rx_buff, 5);
    // }
    else{
      Dispaly_Data(&deviceList[dev_num]);
      if(!transmitMode){
        HAL_UART_Transmit_DMA(deviceList[dev_num].uart, deviceList[dev_num].tx_buff, 10);
      }else{
        char setValueTx[10];
        sprintf(setValueTx, "%d", deviceList[dev_num].setValue);
        HAL_UART_Transmit_DMA(deviceList[dev_num].uart, setValueTx, 10);
      }
    }
    // NEW CODE END

    // DEVICE INITIALIZATION START
    if (HAL_GetTick()-dev_init_timeout>1000 && state == STATE_INIT){
      if(dev_num == old_dev_num){
        dev_num++;
      }else{
        old_dev_num = dev_num;
      }
      dev_init_timeout = HAL_GetTick();
      screen_disp_time = HAL_GetTick();
    }

    // SIM INIT START
    // if(state == STATE_800INIT && HAL_GetTick() - simtmr > 1000){
    //   trySIMInit = 0;
    //   sim.initState = 0;
    //   state = STATE_DISPLAY;
    //   Show_Message("SIM INIT FAIL", 1000);
    // }
    // if(sim.initState = 1 && state == STATE_800INIT){
    //   state = STATE_DISPLAY;
    // }
    // SIM INIT END

    if(dev_num == DEV_COUNT && state == STATE_INIT){
      // state = STATE_800INIT;
      state = STATE_DISPLAY;
      simtmr = HAL_GetTick();
      dev_num = 0;
      
      // CREATE FILE "init_time_date.txt"
      char filename_buff[35];
      uint8_t buf_len;
      sprintf(filename_buff, "init_%02d-%02d-%02d_%02d-%02d-%2d.txt", gTime.Hours, gTime.Minutes, gTime.Seconds, gDate.Date, gDate.Month, gDate.Year);
      FR_Status = f_open(&Fil, filename_buff, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
      for (uint8_t i = 0; i < DEV_COUNT; i++){
        char init_file_buf[50];
        if(deviceList[i].initState == DEV_STATE_IN){
          buf_len = sprintf(init_file_buf, "DEV: %d\nMin/Max: %d/%d\nDev type: %d\n\n", i, deviceList[i].minValue, deviceList[i].maxValue, deviceList[i].deviceMode);
        }else{
          buf_len = sprintf(init_file_buf, "DEV: empty\n\n");
        }
        FR_Status = f_write(&Fil, init_file_buf, buf_len, &bytesWrote);
      }
      f_close(&Fil);
      // SAVE CONFIG
    }
    // DEVICE INITIALIZATION END

    // AUTO SCREEN SCROLL START
    if (HAL_GetTick()-screen_disp_time>5000 && screen_scroll_mode == SCROLL_MODE_AUTO){
      if(state != STATE_INIT){
        do{
          if(dev_num < DEV_COUNT-1){
            dev_num++;
          }else{
            dev_num=0;
          }
        }while (!deviceList[dev_num].initState);  
      }
      screen_disp_time = HAL_GetTick();
    }
    // AUTO SCREEN SCROLL END

    // GPIO READING START
    tick(&button);
    
    // GPIO READING END

    // SCROLL MODE CHANGE START
    if(isClicked(&button)){
      screen_disp_time = HAL_GetTick();
      if(screen_scroll_mode == SCROLL_MODE_AUTO){
        screen_scroll_mode = SCROLL_MODE_HALT;
      }else{
        screen_scroll_mode = SCROLL_MODE_AUTO;
      }
    }
    // SCROLL MODE CHANGE END

    // ENCODER START
    // TEST CODE START
    if (deviceList[dev_num].deviceDisplayMode == MODE_NORMAL){
    // TEST CODE
      // MANUAL SCREEN SROLL START
      // if(isRight(&enc)){
      //   if(dev_num < DEV_COUNT){
      //     dev_num++;
      //   }else{
      //     dev_num = 0;
      //   }
      //   screen_disp_time = HAL_GetTick();
      // }

      // if(isLeft(&enc)){
      //   if(dev_num > 0){
      //     dev_num--;
      //   }else{
      //     dev_num = DEV_COUNT;
      //   }
      //   screen_disp_time = HAL_GetTick();
      // }
      // MANUAL SCREEN SCROLL START
    }else{
      // VALUE EDIT START
      if(isRight(&enc)){
        if(deviceList[dev_num].setValue+5<deviceList[dev_num].maxValue){
          deviceList[dev_num].setValue +=5;
        }
      }
      if(isLeft(&enc)){
        if(deviceList[dev_num].setValue-5>deviceList[dev_num].minValue){
          deviceList[dev_num].setValue -=5;
        }
        else{
          for (uint8_t i = 0; i < 30; i++){
            Draw_Easter();
          }
        }
      }
      // VALUE EDIT END
    }
    // ENCODER END

    // EDIT VALUE START
    if(isHold(&button) && HAL_GetTick() - hold_timeout > 750){
      screen_disp_time = HAL_GetTick();
      hold_timeout = HAL_GetTick();
      if(deviceList[dev_num].deviceDisplayMode == MODE_NORMAL){
        deviceList[dev_num].setValue = deviceList[dev_num].currentValue;
        deviceList[dev_num].deviceDisplayMode = MODE_EDIT;
      }else{
        transmitMode = TRANSMIT_MODE_SET;
        deviceList[dev_num].deviceDisplayMode = MODE_NORMAL;
      }

      if(screen_scroll_mode == SCROLL_MODE_AUTO){
        screen_scroll_mode = SCROLL_MODE_HALT;
      }else{
        screen_scroll_mode = SCROLL_MODE_AUTO;
      }
    }
    // EDIT VALUE END
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  // sTime.Hours = 0x16;
  // sTime.Minutes = 0x34;
  // sTime.Seconds = 0x45;
  // sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  // sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  // if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  // sDate.Month = RTC_MONTH_OCTOBER;
  // sDate.Date = 0x14;
  // sDate.Year = 0x2024;

  // if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LED_Pin|SPI2_CS_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin SPI2_CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SPI2_CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_KEY_Pin ENC_S2_Pin */
  GPIO_InitStruct.Pin = ENC_KEY_Pin|ENC_S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_S1_Pin */
  GPIO_InitStruct.Pin = ENC_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_S1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*
  COMMUNICATION PROTOCOL
  Master    Slave
  T      -> palette(L/T), type(S/T/B), (value), X(value), (symbol) ... LB0X250Y%
  G      -> value ... 754
  val    -> OK
  ON     -> OK
  OFF    -> OK
  S      -> S(state(0/1)) ... S1
  */
  //uint8_t rx_buff[10];
 //(rx_buff, ""); 
  // HAL_UART_Receive_DMA(deviceList[dev_num].uart, rx_buff, 9);
  uint8_t rx_buff[10];
HAL_UART_Receive_DMA(&huart5, rx_buff, 10);
//HAL_Delay(10);
  // if(trySIMInit == 1){
  //   if(rx_buff[0] == 79){
  //     sim.initState = 1;
  //     trySIMInit = 0;
  //   }
  //   return;
  // }

    // NEW CODE START
    // IF FIRST LETTER IS L OR T
  if(rx_buff[0] == 76 || rx_buff[0] == 84){
    // IF L
    if (rx_buff[0] == 76){
      deviceList[dev_num].paletteType = LIGHT_PALETTE;
    }else{
      deviceList[dev_num].paletteType = TEMP_PALETTE;
    }
    // ENDIF

    // DEVICE MODE
    switch (rx_buff[1])
    {
    case 83:
      deviceList[dev_num].deviceMode = TYPE_SET_ONLY;
      break;
    case 84:
      deviceList[dev_num].deviceMode = TYPE_TEL_ONLY;
      break;
    case 66:
      deviceList[dev_num].deviceMode = TYPE_SET_TEL;
      break;
    }
    // ENDIF

    //GET MIN, MAX VALUE & SYMBOL START
    uint8_t minValueEndIndex;
    uint8_t maxValueEndIndex;

    for (int i = 2; i<10; i++){
      if(rx_buff[i] == 88){
        minValueEndIndex = i-1;
      }
      if(rx_buff[i] == 89){
        maxValueEndIndex = i-1;
        break;
      }
    }

    char minValueArr[minValueEndIndex-1];
    char maxValueArr[maxValueEndIndex-minValueEndIndex-1];

    memcpy(minValueArr, &rx_buff[2], (minValueEndIndex-1) * sizeof(char));

    memcpy(maxValueArr, &rx_buff[minValueEndIndex+2], (maxValueEndIndex-minValueEndIndex) * sizeof(char));

    deviceList[dev_num].minValue = atof(minValueArr);
    deviceList[dev_num].maxValue = atof(maxValueArr);    

    deviceList[dev_num].symbol = rx_buff[maxValueEndIndex+2];

    deviceList[dev_num].tx_buff[0] = 71;
    // GET MIN, MAX VALUE & SYMBOL END
    if(state == STATE_INIT){
      deviceList[dev_num].initState = DEV_STATE_IN;
      dev_num++;
    }
  }
  // else if (rx_buff[0] == 83){
  //   if(atoi(rx_buff[1]) == 1){
  //     deviceList[dev_num].isDevOn = 1;
  //   }else{
  //     deviceList[dev_num].isDevOn = 0;
  //   }
  // }
  else if(rx_buff[0] == 79){
    transmitMode = TRANSMIT_MODE_GET;
  }else{// ISSUE START
    deviceList[dev_num].currentValue = atof(rx_buff);
    // deviceList[dev_num].currentValue /= 10;
  }// ISSUE END
  // NEW CODE END



  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_9) {
    tickEnc(&enc);
  } else {
      __NOP();
  }
}
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
    Show_Message("PANIC!", 60000);
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
