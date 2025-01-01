#ifndef _DISP
#define _DISP
#include "stdbool.h"
#include <stdint.h>

#define TEMP_PALETTE    0
#define LIGHT_PALETTE   1

#define TYPE_SET_TEL    0
#define TYPE_SET_ONLY   1
#define TYPE_TEL_ONLY   2

#define DEVICE_OFF      0
#define DEVICE_ON       1

#define MODE_EDIT       1
#define MODE_NORMAL     0

extern RTC_HandleTypeDef hrtc;
extern IWDG_HandleTypeDef hiwdg;
//void Dispaly_Data(double data, double set, bool isOn, double minV, double maxV, char symbol, uint8_t paletteType, uint8_t deviceType, bool displayMode);
void Dispaly_Data(Device *dev);

typedef struct{
    char *message;
    bool isActive;
    int8_t y;
    uint16_t tick;
} Message;

float getTouchAngle(int touchX, int touchY);
bool isTouchInsideArc(int touchX, int touchY, int centerX, int centerY, int innerRadius, int outerRadius, float startAngle, float endAngle);
void Set_Time(RTC_DateTypeDef *gDate, RTC_TimeTypeDef *gTime);
void Display_Init(int data, int maxV);
void SIM_Init(char *cmd, char *ack);
void Show_Message(char *msg, uint16_t time);
void Show_SD_Warning();
void Show_RTC_Warning();
void Draw_Easter();
void DisplayNoDev();
void Update_Messages();
void Draw_Main_Screen();
bool Add_Message(char *message);
void Draw_NavBar(uint8_t devs, int8_t cur_dev);
void DrawRoundedRect(uint8_t x, int8_t y, uint8_t width, uint8_t height, uint8_t radius, uint16_t color);

#endif /* SRC_DEMO_THERMOSTAT_H_ */