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
#include "cst816s.h"
#include <string.h>
#include <stdio.h>
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


#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 128

#define RTC_BKP_CONSTANT  0x2345
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
// DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
#define DEV0_UART &huart4
#define DEV3_UART &huart5

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
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Set_Alarm(uint8_t minutes);
void Set_RTC();
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
void findNextInDev(bool dir);
bool checkSIMStatus();
bool sendSMS(char *message);
bool checkInitStatus();
bool enableGPRS();
void resetModule();
void syncTime();
void getTime();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double val = 10;
uint8_t tx_buff[]={84,66,67,68,69,70,71,72,73,74};
uint8_t palette = TEMP_PALETTE;
uint8_t device_type = TYPE_SET_TEL;

uint8_t state = STATE_INIT;
uint8_t dev_num = 0;
uint8_t act_dev_cnt = 0;

int8_t act_dev_list[]  = {-1, -1, -1, -1};

bool isDevPolledFirstTime = true;

uint8_t alarm_timeout = 1;

uint8_t screen_scroll_mode = SCROLL_MODE_AUTO;

//Device device_0 = {&huart4, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_2 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
//Device device_3 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};
// Device device_4 = {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY};

CST816S_TouchData TP;

Device deviceList[] = {{&huart4, DEV_STATE_UNIN, TYPE_SET_ONLY, DEV0_RST_GPIO_Port, DEV0_RST_Pin}, 
                       {&huart2, DEV_STATE_UNIN, TYPE_SET_ONLY, DEV1_RST_GPIO_Port, DEV1_RST_Pin}, 
                       {&huart3, DEV_STATE_UNIN, TYPE_SET_ONLY, DEV2_RST_GPIO_Port, DEV2_RST_Pin}, 
                       {&huart5, DEV_STATE_UNIN, TYPE_SET_ONLY, DEV3_RST_GPIO_Port, DEV3_RST_Pin}};
//Device deviceList[] = {device_0, dev};


SIM sim = {SIM800_UART};

uint8_t rx_buff[10];

uint8_t sim_rx_buff[30];

uint8_t SIMrxBuffer[RX_BUFFER_SIZE];
uint8_t SIMtxBuffer[TX_BUFFER_SIZE];

#define DEV_COUNT 4

bool transmitMode = TRANSMIT_MODE_GET;

bool isSDInitialised = false;

bool isSIMInitialised = false;
bool isSIMInserted = false;

bool allDeviceIsUnIn = true;

bool touchAvailable = false;

bool isRTCSetUp = false;

bool isRTCTimeSync = false;

bool longPressFlag = false;

bool showMainScreen = true;

const char *tele2_apn = "internet.tele2.lv";
const char *lmt_apn = "internet.lmt.lv";
const char *bite_apn = "internet";
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	dispcolor_Init(240, 240);

	uint32_t dev_init_timeout = HAL_GetTick();
  uint32_t screen_disp_time = HAL_GetTick();
  uint32_t touch_tmr = HAL_GetTick();
  uint32_t sim_tmr = HAL_GetTick();

  // uint32_t simtmr = HAL_GetTick();
  // uint32_t txtmr = HAL_GetTick();
  
  uint8_t old_dev_num = 0;

  /* RTC */
  // GET RTC DATETIME
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
  
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != RTC_BKP_CONSTANT){
    Show_RTC_Warning();
    // RTC SETUP MENU
    // Set_RTC();
    // NVIC_SystemReset();
  }else{
    isRTCSetUp = true;
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
    Show_SD_Warning();
  }

  /* SIM CARD */
  HAL_UART_Receive_DMA(&huart6, SIMrxBuffer, RX_BUFFER_SIZE);


  /* RESET */
  for (int i = 0; i < DEV_COUNT; i++){
    GPIO_TypeDef *curDevRSTport = deviceList[i].RSTport;
    uint16_t curDevRSTpin = deviceList[i].RSTpin;
    HAL_GPIO_WritePin(curDevRSTport, curDevRSTpin, GPIO_PIN_RESET);
	  HAL_Delay(100);
    HAL_GPIO_WritePin(curDevRSTport, curDevRSTpin, GPIO_PIN_SET);
    deviceList[i].tx_buff[0] = 84;
  }
  dev_init_timeout = HAL_GetTick();

  /* TOUCH */
  if (!CST816S_Init(&hi2c1)){
    HAL_NVIC_SystemReset();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){ 
    if(state == STATE_INIT){
      Display_Init(dev_num, DEV_COUNT);
      HAL_UART_Transmit_DMA(deviceList[dev_num].uart, deviceList[dev_num].tx_buff, 10);
      HAL_UART_Receive_DMA(deviceList[dev_num].uart, deviceList[dev_num].rx_buff, 10);
    }
    else{
      if (!allDeviceIsUnIn && !showMainScreen){
        Dispaly_Data(&deviceList[dev_num]);
        if(transmitMode == TRANSMIT_MODE_GET){
          if(deviceList[dev_num].deviceMode != TYPE_SET_ONLY){
            HAL_UART_Transmit_DMA(deviceList[dev_num].uart, deviceList[dev_num].tx_buff, 10);
            HAL_UART_Receive_DMA(deviceList[dev_num].uart, deviceList[dev_num].rx_buff, 10);
          }
        }else{
          char setValueTx[10];
          //sprintf(setValueTx, "%d", deviceList[dev_num].setValue);
          itoa(deviceList[dev_num].setValue, setValueTx, 10);
          HAL_UART_Transmit_DMA(deviceList[dev_num].uart, (uint8_t*)&setValueTx, 10);
          transmitMode = TRANSMIT_MODE_GET;
        }
        for(uint8_t act_dev = 0; act_dev < 4; act_dev++){
          if (act_dev_list[act_dev] == dev_num){
            Draw_NavBar(act_dev_cnt, act_dev);
            break;
          }
        }
        
      }else{
        Draw_Main_Screen();
        Draw_NavBar(act_dev_cnt, -1);
        //Draw_Easter();
      }
    }

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


    if(HAL_GetTick()-sim_tmr > 5000){
      if(!isSIMInserted){
        if(checkSIMStatus()){
          Add_Message("SIM detected");
          isSIMInserted = true;
        }
      }
      if(isSIMInserted && !isSIMInitialised){
        if(checkInitStatus()){
          Add_Message("GSM connected");
          if(!isRTCSetUp && !isRTCTimeSync){
            syncTime();
            resetModule();
            isRTCTimeSync = true;
          }else{
            isSIMInitialised = true;
          }
        }
      }
      if(isSIMInitialised && !isRTCSetUp){
        getTime();
        if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == RTC_BKP_CONSTANT){
          Add_Message("RTC is set up");
          isRTCSetUp = true;
        } 
      }
      sim_tmr = HAL_GetTick();
    }

    // TEST CODE START
    // if (HAL_GetTick() - sim_tmr > 10000 && isSIMInitialised && isSIMInserted){
    //   char msg[30];
    //   char b[4];
    //   gcvt(deviceList[dev_num].currentValue, 3, b);
    //   sprintf(msg,"TEMP: %s*C", b);
    //   if(sendSMS(msg)){
    //     // Show_Message("SMS OK", 500);
    //   }
    //   sim_tmr = HAL_GetTick();
    // }
    // TEST CODE END

    if(dev_num == DEV_COUNT && state == STATE_INIT){
      // state = STATE_800INIT;
      state = STATE_DISPLAY;
      //simtmr = HAL_GetTick();
      findNextInDev(true);
      screen_scroll_mode = SCROLL_MODE_AUTO;
      if(isSDInitialised){
        // CREATE FILE "init_time_date.txt"
       // Set_Alarm(alarm_timeout);
        char filename_buff[35];
        char reset_cause_buf[20];
        uint8_t buf_len = 0;
        // CREATE FILE
        sprintf(filename_buff, "init_%02d-%02d-%02d_%02d-%02d-%2d.txt", gTime.Hours, gTime.Minutes, gTime.Seconds, gDate.Date, gDate.Month, gDate.Year);
        FR_Status = f_open(&Fil, filename_buff, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
        // GET RESET REASON
        uint32_t reset_flags = RCC->CSR;
        if (reset_flags & RCC_CSR_IWDGRSTF) {
          buf_len = sprintf(reset_cause_buf, "Reset by IWDG\n");
        }else if (reset_flags & RCC_CSR_PINRSTF){
          buf_len = sprintf(reset_cause_buf, "Normal startup\n");
        }
        // SAVE
        FR_Status = f_write(&Fil, reset_cause_buf, buf_len, &bytesWrote);
        // GET DEVICE DATA
        for (uint8_t i = 0; i < DEV_COUNT; i++){
          char init_file_buf[75];
          // DEVISE INITIALISED
          if(deviceList[i].initState == DEV_STATE_IN){
            char dev_type_buf[15];
            // GET DEVICE TYPE
            switch (deviceList[i].deviceMode)
            {
            case TYPE_SET_ONLY:
              sprintf(dev_type_buf, "TYPE_SET_ONLY");
              break;
            case TYPE_SET_TEL:
              sprintf(dev_type_buf, "TYPE_SET_TEL");
              break;
            case TYPE_TEL_ONLY:
              sprintf(dev_type_buf, "TYPE_TEL_ONLY");
              break;
            }
            // GET DEVICE VARIABLES
            uint8_t max[5];
            uint8_t min[5];
            gcvt(deviceList[i].minValue, 5, min);
            gcvt(deviceList[i].maxValue, 5, max);
            // SAVE
            buf_len = sprintf(init_file_buf, "DEV: %d\nMin/Max: %s/%s\nDev type: %s\n\n", i, min, max, dev_type_buf);
          }else{ // DEVICE UNINITIALISED
            buf_len = sprintf(init_file_buf, "DEV: empty\n\n");
          }
          FR_Status = f_write(&Fil, init_file_buf, buf_len, &bytesWrote);
        }
        f_close(&Fil);
        // SAVE CONFIG
      }
    }
    // DEVICE INITIALIZATION END

    // AUTO SCREEN SCROLL START
    if (HAL_GetTick()-screen_disp_time>5000 && screen_scroll_mode == SCROLL_MODE_AUTO && !allDeviceIsUnIn && !showMainScreen){
      if(state != STATE_INIT){
        findNextInDev(true);
      }
      screen_disp_time = HAL_GetTick();
    }
    // AUTO SCREEN SCROLL END

    // TP READING START
    if(HAL_GetTick() - touch_tmr > 30 && touchAvailable){
      CST816S_ReadTouchData(&hi2c1, &TP);
      if (TP.finger_num != 0){
        // SOME ACTIONS
        
        // TP COORDS TO DISP COORDS
        uint8_t x = map(TP.x_coord, 0, 65535, 0, 240);
        uint8_t y = map(TP.y_coord, 0, 65535, 0, 240);
        
        // TEST
        if (x>78 && x < 162 && y > 78 && y<162 && TP.gesture_id == 0x00){
          deviceList[dev_num].isDevOn = !deviceList[dev_num].isDevOn;
        }
        if(TP.gesture_id == 0x00 && deviceList[dev_num].deviceDisplayMode ==MODE_EDIT){
          if (isTouchInsideArc(x, y, 120, 120, 55, 120, 136, 44)) {
              char buf[13];
              float tp_angle = getTouchAngle(x, y); // 46 314
              deviceList[dev_num].setValue = map(tp_angle, 46, 314, deviceList[dev_num].minValue, deviceList[dev_num].maxValue);
              sprintf(buf, "%d", map(tp_angle, 46, 314, 0, 100));
          }
        }
        // TEST
        if(TP.gesture_id == 0x0C && !longPressFlag && deviceList[dev_num].deviceMode != TYPE_TEL_ONLY && !showMainScreen){
          longPressFlag = true;
          // EDIT VALUE START

          screen_disp_time = HAL_GetTick();
          if(deviceList[dev_num].deviceDisplayMode == MODE_NORMAL){
            if (deviceList[dev_num].deviceMode != TYPE_SET_ONLY){
              deviceList[dev_num].setValue = deviceList[dev_num].currentValue;
            }
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
          // EDIT VALUE END
        }
        if(TP.gesture_id != 0x0C){
          longPressFlag = false;
        }

        // SCROLL START
        if(TP.gesture_id == 0x03 && deviceList[dev_num].deviceDisplayMode != MODE_EDIT){ // LEFT
          if (showMainScreen){showMainScreen=false; dev_num = act_dev_list[0];}
          else if (dev_num == act_dev_list[act_dev_cnt-1]){showMainScreen=true;}
          else{findNextInDev(true);}

          
          // Show_Message("LEFT", 500);
          screen_disp_time = HAL_GetTick();
        }
        if(TP.gesture_id == 0x04 && deviceList[dev_num].deviceDisplayMode != MODE_EDIT){ // RIGHT
          if (showMainScreen){showMainScreen=false; dev_num = act_dev_list[act_dev_cnt-1];}
          else if(dev_num == act_dev_list[0]){showMainScreen=true;}
          else{findNextInDev(false);}
          // Show_Message("RIGHT", 500);
          screen_disp_time = HAL_GetTick();
        }
        // SCROLL END
      }
      touchAvailable = false;
      touch_tmr = HAL_GetTick();
    }
    // TP READING END

    // IWDG RESET START
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK){
      Error_Handler();
    }
    // IWDG RESET END
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  RTC_AlarmTypeDef sAlarm = {0};

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
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == RTC_BKP_CONSTANT){return;}
  // // GET RTC DATETIME
  // RTC_DateTypeDef gDate;
  // RTC_TimeTypeDef gTime;
  // bool rtcBackup = false;

  // /* Get the RTC current Time */
  // HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  // /* Get the RTC current Date */
  // HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
  
  // if(gDate.Year != 0){
  //     rtcBackup = true;
  // }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  // if(rtcBackup){
  //   HAL_RTC_SetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  //   HAL_RTC_SetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
  // }
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
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|DEV0_RST_Pin|SPI2_CS_Pin|DC_Pin
                          |BL_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DEV1_RST_Pin|DEV2_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEV3_RST_GPIO_Port, DEV3_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 DEV0_RST_Pin SPI2_CS_Pin DC_Pin
                           BL_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|DEV0_RST_Pin|SPI2_CS_Pin|DC_Pin
                          |BL_EN_Pin;
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

  /*Configure GPIO pins : DEV1_RST_Pin DEV2_RST_Pin TP_RST_Pin */
  GPIO_InitStruct.Pin = DEV1_RST_Pin|DEV2_RST_Pin|TP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DEV3_RST_Pin */
  GPIO_InitStruct.Pin = DEV3_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEV3_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_INT_Pin */
  GPIO_InitStruct.Pin = TP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Set_Alarm(uint8_t minutes)
{   
  // RTC_DateTypeDef sDate;
  // RTC_TimeTypeDef sTime;
  // RTC_AlarmTypeDef sAlarm;
  // // Get the current RTC time and date
  // HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  // HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  // // Calculate the new alarm time
  // uint8_t newMinutes = (sTime.Minutes + minutes) % 60;
  // uint8_t minutesOverflow = (sTime.Minutes + minutes) / 60;

  // uint8_t newHours = (sTime.Hours + minutesOverflow) % 24;
  // uint8_t hoursOverflow = (sTime.Hours + minutesOverflow) / 24;

  // // Handle day, month, and year transitions
  // uint8_t newDay = sDate.Date + hoursOverflow;
  // uint8_t newMonth = sDate.Month;
  // uint16_t newYear = sDate.Year + 2000; // RTC returns year as offset from 2000

  // // Days in each month (account for leap year in February)
  // uint8_t daysInMonth[] = {31, (newYear % 4 == 0 && (newYear % 100 != 0 || newYear % 400 == 0)) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // // Adjust day and month
  // while (newDay > daysInMonth[newMonth - 1]) {
  //     newDay -= daysInMonth[newMonth - 1];
  //     newMonth++;
  //     if (newMonth > 12) {
  //         newMonth = 1;
  //         newYear++;
  //     }
  // }

  // // Set up the RTC alarm
  // sAlarm.AlarmTime.Hours = newHours;
  // sAlarm.AlarmTime.Minutes = newMinutes;
  // sAlarm.AlarmTime.Seconds = sTime.Seconds;
  // sAlarm.AlarmTime.TimeFormat = sTime.TimeFormat; // 24-hour format

  // sAlarm.AlarmDateWeekDay = newDay;
  // sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE; // Use specific date
  // sAlarm.AlarmMask = RTC_ALARMMASK_NONE; // Do not mask the date
  // sAlarm.Alarm = RTC_ALARM_A;

  // if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
  //     Error_Handler();
  // }
}

void Set_RTC(){
  // RTC_DateTypeDef gDate;
  // RTC_TimeTypeDef gTime;
  // while (1)
  // {
  //   Set_Time(&gDate, &gTime);
  //   // tick(&button);
  //   // if(isHold(&button)){return;}
  // }
}

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
  // if(trySIMInit){
  //     char rx_buff[10];
  // HAL_UART_Receive_DMA(deviceList[dev_num].uart, (uint8_t*)&rx_buff, 10);
  //   return;
  // }

  /* SIM COMMUNICATION */
  if (huart == SIM800_UART){
    SIMrxBuffer[RX_BUFFER_SIZE - 1] = '\0'; // Null-terminate the buffer for string operations
        // Restart the DMA reception
    HAL_UART_Receive_DMA(&huart6, SIMrxBuffer, RX_BUFFER_SIZE);
    return;
  }


  char rx_buff[10];
  HAL_UART_Receive_DMA(deviceList[dev_num].uart, (uint8_t*)&rx_buff, 10);


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
    uint8_t minValueEndIndex = 0;
    uint8_t maxValueEndIndex = 0;

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
      allDeviceIsUnIn = false;
      act_dev_cnt++;
      for (uint8_t i = 0; i< 4; i++){
        if(act_dev_list[i] == -1){
          act_dev_list[i] = dev_num;
          break;
        }
      }
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
    if(!isDevPolledFirstTime){
      deviceList[dev_num].currentValue = atof(rx_buff);
      deviceList[dev_num].currentValue /= 10;
    }else{
      isDevPolledFirstTime = false;
    }
  }// ISSUE END
  // NEW CODE END
}

// void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) 
// { 
//   // FATFS FatFs;
//   FRESULT FR_Status;
//   FIL Fil;
//   UINT bytesWrote;

//   /* RTC */
//   // GET RTC DATETIME
//   RTC_DateTypeDef gDate;
//   RTC_TimeTypeDef gTime;

//   /* Get the RTC current Time */
//   HAL_RTC_GetTime(hrtc, &gTime, RTC_FORMAT_BIN);
//   /* Get the RTC current Date */
//   HAL_RTC_GetDate(hrtc, &gDate, RTC_FORMAT_BIN);

//   char filename_buff[35];
//   uint8_t buf_len;
//   sprintf(filename_buff, "log_%02d-%02d-%02d_%02d-%02d-%2d.txt", gTime.Hours, gTime.Minutes, gTime.Seconds, gDate.Date, gDate.Month, gDate.Year);
//   FR_Status = f_open(&Fil, filename_buff, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
//   for (uint8_t i = 0; i < DEV_COUNT; i++){
//     char init_file_buf[75];
//     if(deviceList[i].initState == DEV_STATE_IN){
//       buf_len = sprintf(init_file_buf, "DEV: %d\nValue: %f\nSet value: %f\n", i, deviceList[i].currentValue, deviceList[i].setValue);
//       FR_Status = f_write(&Fil, init_file_buf, buf_len, &bytesWrote);
//       if(FR_Status != 0){break;}
//     }
//   }
//   f_close(&Fil);
//   // SAVE CONFIG
//   Set_Alarm(alarm_timeout);
// }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_9){
    touchAvailable = true;
	}else {
      __NOP();
  }
}

void findNextInDev(bool dir){
  if(!allDeviceIsUnIn){
    if(dir){
      do{
        if(dev_num < DEV_COUNT-1){
          dev_num++;
        }else{
          dev_num=0;
        }
      }while (!deviceList[dev_num].initState);  
    }else{
      do{
        if(dev_num > 0){
          dev_num--;
        }else{
          dev_num=DEV_COUNT-1;
        }
      }while (!deviceList[dev_num].initState);  
    }
    isDevPolledFirstTime = true;
  }
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SIM_sendCommand(const char *command){
    memset(SIMtxBuffer, 0, TX_BUFFER_SIZE);
    snprintf((char *)SIMtxBuffer, TX_BUFFER_SIZE, "%s\r\n", command);
    HAL_UART_Transmit_DMA(&huart6, SIMtxBuffer, strlen((char *)SIMtxBuffer));
}

bool checkSIMStatus(){
    SIM_sendCommand("AT+CSMINS?");
    HAL_Delay(100);
    if (strstr((char *)SIMrxBuffer, "+CSMINS: 0,1") != NULL) {
        return true; // SIM installed
    }
    return false; // SIM not installed
}

bool checkInitStatus(){
  SIM_sendCommand("AT+CREG?");
  if (strstr((char *)SIMrxBuffer, "+CREG: 0,1") != NULL) {
    return true;
  }
  return false;
}

bool sendSMS(char *message){
  SIM_sendCommand("AT+CMGF=1");
  HAL_Delay(50);
  SIM_sendCommand("AT+CMGS=\"+37122310302\"");
  HAL_Delay(50);
  SIM_sendCommand(message);
  HAL_Delay(50);
  uint8_t ctrlZ[] = {26};
  HAL_UART_Transmit_DMA(&huart6, ctrlZ, 1);
  if (strstr((char *)SIMrxBuffer, "ERROR") != NULL) {
    return false;
  }
  return true;
}

void resetModule(){
  SIM_sendCommand("AT+CFUN=1,1");
  HAL_Delay(50);
}

void syncTime(){
  SIM_sendCommand("AT+CLTS=1");
  HAL_Delay(50);
  SIM_sendCommand("AT&W");
  HAL_Delay(50);

}

void getTime(){
  char time[30];

  SIM_sendCommand("AT+CCLK?");
  HAL_Delay(50);

  int i, j, k;
  char *cclk = "+CCLK";
  for(i=0; SIMrxBuffer[i]!='\0'; i++) {
    for(j=i, k=0; cclk[k]!='\0' && SIMrxBuffer[j]==cclk[k]; j++, k++)
        ;
    if(k>0 && cclk[k]=='\0')
    {
        for (uint8_t l = 0; l < 30; l++){
          time[l] = SIMrxBuffer[i+l];
        }
        break;
    }
  }
  time[29] ='\0';
  //Show_Message(time, 1000);

  RTC_DateTypeDef nDate;
  RTC_TimeTypeDef nTime;
  // BUG!!!!!!
// Separate the date and time from the input string
  char date_part[9];  // For "23/12/31"
  char time_part[9];   // For "14:45:30"
  char offset_part[4]; // For "+05"
  
  // Use sscanf to extract date, time, and offset parts
  int result = sscanf(time, "+CCLK: \"%8s,%8s%3s\"", date_part, time_part, offset_part);

  if (result == 3) {
    // Now parse the date part (e.g., "23/12/31")
    uint8_t day, month, year;
    sscanf(date_part, "%2d/%2d/%2d", &year, &month, &day);

    // Fill the RTC_DateTypeDef structure
    nDate.Year = year;
    nDate.Month = month;
    nDate.Date = day;
    nDate.WeekDay = 5;

    if (HAL_RTC_SetDate(&hrtc, &nDate, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }

      // Parse the time part (e.g., "14:45:30")
    uint8_t hours, minutes, seconds;
    sscanf(time_part, "%2d:%2d:%2d", &hours, &minutes, &seconds);

    // Fill the RTC_TimeTypeDef structure
    nTime.Hours = hours;
    nTime.Minutes = minutes;
    nTime.Seconds = seconds;
    nTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    nTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &nTime, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_BKP_CONSTANT);
  }
}

bool enableGPRS(){
  //char apn[50];
  SIM_sendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  HAL_Delay(50);
  //sprintf(apn, "AT+SAPBR=3,1,\"APN\",\"%c\"", bite_apn);
  SIM_sendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"");
  HAL_Delay(50);
  SIM_sendCommand("AT+SAPBR=1,1");
  HAL_Delay(50);
  if (strstr((char *)SIMrxBuffer, "ERROR") != NULL) {
    return false;
  }
  return true;
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
  HAL_NVIC_SystemReset();
  //while (1){}
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
