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

//void Dispaly_Data(double data, double set, bool isOn, double minV, double maxV, char symbol, uint8_t paletteType, uint8_t deviceType, bool displayMode);
void Dispaly_Data(Device *dev);

void Display_Init(int data, int maxV);
void SIM_Init(char *cmd, char *ack);
void Show_Message(char *msg, uint16_t time);
void Draw_Easter();

#endif /* SRC_DEMO_THERMOSTAT_H_ */