#include <stdint.h>
#include <dispcolor.h>
#include <font.h>
#include <math.h>
#include "main.h"
#include "info_disp.h"
#include "bitmaps.h"


#define PI 	3.14159265
#define xC	120
#define yC	120

#define MIN_VALUE		100
#define MAX_VALUE		450
#define MIN_ANGLE		-224
#define MAX_ANGLE		44
#define PALETTE_SIZE	(MAX_ANGLE - MIN_ANGLE) / 4
#define ERROR_PALETTE 2

static uint8_t PaletteReady = 0;
static uint8_t OldPalette;
static sRGB888 Palette[PALETTE_SIZE];
static uint32_t edit_tmr;
static bool diplayFlag;

static void GetBlueRedPalette(uint16_t steps, sRGB888 *pBuff, uint8_t type) {
	if (!pBuff)
		return;

	sRGB888 KeyColors[] = { { 0x00, 0x00, 0xFF }, { 0xFF, 0x00, 0x00 } };
	switch (type)
	{
	case LIGHT_PALETTE:
		KeyColors[0].r = 0x5F;
		KeyColors[0].g = 0x5F;
		KeyColors[0].b = 0x1C;  // LIGHT
		KeyColors[1].r = 0xFF;
		KeyColors[1].g = 0xFF;
		KeyColors[1].b = 0x55; 
		break;
	case ERROR_PALETTE:
		KeyColors[0].r = 0xF5;
		KeyColors[0].g = 0x75;
		KeyColors[0].b = 0x42;  // ERROR
		KeyColors[1].r = 0xFF;
		KeyColors[1].g = 0x42;
		KeyColors[1].b = 0x42; 
		break;
	}
	

	for (uint16_t step = 0; step < steps; step++) {
		float n = (float) step / (float) (steps - 1);

		pBuff->r = ((float) KeyColors[0].r) * (1.0f - n)
				+ ((float) KeyColors[1].r) * n;
		pBuff->g = ((float) KeyColors[0].g) * (1.0f - n)
				+ ((float) KeyColors[1].g) * n;
		pBuff->b = ((float) KeyColors[0].b) * (1.0f - n)
				+ ((float) KeyColors[1].b) * n;

		pBuff++;
	}
}

void Draw_BMP(int16_t x, int16_t y, const uint16_t *map, int8_t w, int8_t h, bool isPNG){
	for (int i = 0; i < w; i++){
		for (int j = 0; j < h; j++){
			if (isPNG && map[j*w+i] == 0x0000){
				continue;
			}
			dispcolor_DrawPixel(x+i, y+j, map[j*w+i]);
		}
	}
}

void Dispaly_Data(Device *dev) {
	/* Device types
		0 - Set value & telemetrty
		1 - Set value only
		2 - Telemetry only
	*/
	
	// FIX THIS:
	double data = dev->currentValue;
	double set = dev->setValue;
	bool isOn = dev->isDevOn;
	double minV = dev->minValue;
	double maxV = dev->maxValue;
	char symbol = dev->symbol;
	uint8_t paletteType = dev->paletteType;
	uint8_t deviceType = dev->deviceMode;
	bool displayMode = dev->deviceDisplayMode;
	int16_t drawData;
	// END

	if (!PaletteReady || paletteType != OldPalette) {
		GetBlueRedPalette(PALETTE_SIZE, Palette, paletteType);
		PaletteReady = 1;
		OldPalette = paletteType;
	}

	if (data>maxV||data<minV){
		GetBlueRedPalette(PALETTE_SIZE, Palette, ERROR_PALETTE);
		PaletteReady = 1;
		OldPalette = paletteType;
		if(data>maxV){
			drawData = maxV;
		}else{
			drawData = minV;
		}
	}else{
		PaletteReady = 0;
		drawData = data;
	}

	uint16_t bgColor, textColor;
    bgColor = BLACK;
    textColor = WHITE;

	HAL_Delay(30);
	dispcolor_FillScreen(bgColor);

	if (deviceType == TYPE_SET_ONLY || displayMode == MODE_EDIT){
		drawData = set; 
		data = set;
	}

	
	
	int16_t position = (drawData - minV) * (MAX_ANGLE - MIN_ANGLE) / (maxV - minV) + MIN_ANGLE;


	char buf[7];
	char b[4];
	if (deviceType != TYPE_SET_ONLY){
		gcvt(data, 3, b);
		//sprintf(b, "%f", data);
		sprintf(buf, "%s %c", b, symbol);
		
		if(displayMode == MODE_NORMAL){
			dispcolor_DrawString(120-(strlen(buf)*6), 160, FONTID_32F, buf, textColor);		
		}else{
			if(HAL_GetTick() - edit_tmr > 500 && HAL_GetTick() - edit_tmr < 1000){
				dispcolor_DrawString(120-(strlen(buf)*6), 160, FONTID_32F, buf, textColor);		
			}
			if (HAL_GetTick() - edit_tmr > 1000){edit_tmr = HAL_GetTick();}	
		}
	}

	if (deviceType != TYPE_TEL_ONLY){
		gcvt(set, 3, b);
		sprintf(buf, "%s %c", b, symbol);
		if (displayMode == MODE_NORMAL || HAL_GetTick() - edit_tmr < 500){
			if (deviceType == TYPE_SET_ONLY){
				dispcolor_DrawString(120-(strlen(buf)*6), 200, FONTID_32F, buf, textColor);
			}else{
				dispcolor_DrawString(120-(strlen(buf)*4), 200, FONTID_24F, buf, textColor);
			}	
		}else{
			if (HAL_GetTick() - edit_tmr > 750){
				edit_tmr = HAL_GetTick();
			}
		}
	}

	uint8_t mainRadius = 101;
	uint16_t idx = 0;
	for (int16_t angle = MIN_ANGLE; angle < position; idx++, angle += 4) {
		float angleRad = (float) angle * PI / 180;
		int xMain = cos(angleRad) * mainRadius + xC;
		int yMain = sin(angleRad) * mainRadius + yC;
		dispcolor_FillCircle(xMain, yMain, 20,
				RGB565(Palette[idx].r, Palette[idx].g, Palette[idx].b));
	}

	if (!isOn){
		Draw_BMP(78, 78, bitmap_off_84x84, 84, 84, 1);
		//Draw_BMP(78, 78, skynet_100x64, 100, 64, 1);
	} else {
		Draw_BMP(78, 78, bitmap_on_84x84, 84, 84, 1);
	}

	dispcolor_Update();
}


void Draw_Easter(){
	HAL_Delay(30);
	dispcolor_FillScreen(BLACK);
	Draw_BMP(70, 50, skynet_100x64, 100, 64, 1);
	dispcolor_DrawString(10, 125, FONTID_16F, "The machines rose from the \n\rashes of the nuclear fire.", WHITE);
	dispcolor_Update();
}

void Display_Init(int data, int maxV){
	HAL_Delay(30);
	dispcolor_FillScreen(BLACK);
	dispcolor_DrawString(85, 105, FONTID_16F, "Device init", WHITE);
	uint32_t id = HAL_GetDEVID();
	char buf[10];
	sprintf(buf, "DEV ID: %d", id);
	dispcolor_DrawString(80, 175, FONTID_16F, buf, WHITE);
	for (int j = 1; j < maxV+1; j++){
		if (j > data){break;}
		for (int i = 1; i < 10; i++){
			dispcolor_FillCircle((4*i)+(27*j), 150, 10, 0x00FF00);
		}
	}

	dispcolor_Update();
}

void SIM_Init(char *cmd, char *ack){
	HAL_Delay(30);
	dispcolor_FillScreen(BLACK);
	dispcolor_DrawString(85, 105, FONTID_16F, "SIM800L init", WHITE);
	char buf[10];
	sprintf(buf, "%s: %s", cmd, ack);
	dispcolor_DrawString(80, 175, FONTID_16F, buf, WHITE);
	dispcolor_Update();
}

void Show_Message(char *msg, uint16_t time){
	uint32_t tmr = HAL_GetTick();
	uint16_t max_angle_local = 44;
	int16_t min_angle_local = -244;
	while(HAL_GetTick() - tmr < time){
		HAL_Delay(30);
		dispcolor_FillScreen(BLACK);
		dispcolor_DrawString(120-(strlen(msg)*4), 120, FONTID_16F, msg, WHITE);
		
		int16_t position = (time - (HAL_GetTick() - tmr)) * (max_angle_local - min_angle_local) / time + min_angle_local;

		uint8_t mainRadius = 111;
		uint16_t idx = 0;
		for (int16_t angle = min_angle_local; angle < position; idx++, angle += 4) {
			float angleRad = (float) angle * PI / 180;
			int xMain = cos(angleRad) * mainRadius + xC;
			int yMain = sin(angleRad) * mainRadius + yC;
			dispcolor_FillCircle(xMain, yMain, 10, YELLOW);
		}
		dispcolor_Update();
	}
}