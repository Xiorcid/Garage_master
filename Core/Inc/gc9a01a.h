#ifndef _GC9A01A_H
#define _GC9A01A_H


#define GC9A01_DIRECT_MODE		0
#define GC9A01_BUFFER_MODE		1

#define GC9A01_MODE 			GC9A01_BUFFER_MODE


#define GC9A01A_SPI_periph       SPI1
#define GC9A01A_RESET_Used       1
#define GC9A01A_CS_Used          0
#define GC9A01A_DC_Used          1
#define GC9A01A_BLK_PWM_Used     0

#if (GC9A01A_RESET_Used)
  #define GC9A01A_RESET_Port     GPIOA
  #define GC9A01A_RESET_Pin      GPIO_PIN_6
#endif

#if (GC9A01A_CS_Used)
  #define GC9A01A_CS_Port        GPIOB
  #define GC9A01A_CS_Pin         GPIO_PIN_10
#endif

#if (GC9A01A_DC_Used)
  #define GC9A01A_DC_Port        GPIOC
  #define GC9A01A_DC_Pin         GPIO_PIN_4
#endif

#if (GC9A01A_BLK_PWM_Used)

#else
  #define GC9A01A_BLK_Port       GPIOA
  #define GC9A01A_BLK_Pin        GPIO_PIN_1
#endif

uint16_t GC9A01A_GetWidth();
uint16_t GC9A01A_GetHeight();

void GC9A01A_Init();
void GC9A01A_Update();
void GC9A01A_SleepMode(uint8_t Mode);
void GC9A01A_DisplayPower(uint8_t On);
void GC9A01A_SetBL(uint8_t Value);
void GC9A01A_DrawPixel(int16_t x, int16_t y, uint16_t color);
uint16_t GC9A01A_GetPixel(int16_t x, int16_t y);
void GC9A01A_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void GC9A01A_DrawPartYX(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pBuff);
void GC9A01A_DrawPartXY(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pBuff);

#endif
