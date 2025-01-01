#include <stdint.h>
#include <dispcolor.h>
#include <font.h>
#include <math.h>
#include "main.h"
#include "info_disp.h"
#include "bitmaps.h"
#include <strings.h>


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

Message notifications[5];

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
	for (int16_t angle = MIN_ANGLE; angle < MAX_ANGLE; idx++, angle += 4) {
		float angleRad = (float) angle * PI / 180;
		int xMain = cos(angleRad) * mainRadius + xC;
		int yMain = sin(angleRad) * mainRadius + yC;
		dispcolor_FillCircle(xMain, yMain, 20, RGB565(17, 17, 17));
	}

	idx = 0;
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

	Update_Messages();
	dispcolor_Update();
}

void DisplayNoDev(){
	
}

void Update_Messages(){
	uint16_t max_tick = 50;
	for (uint8_t i =0; i < 5; i++){
		if(notifications[i].isActive){
			if(notifications[i].tick > max_tick){
				notifications[i].isActive = false;
				continue;
			}
			if(notifications[i].tick < max_tick/5){notifications[i].y += 5;}
			if(notifications[i].tick > max_tick-max_tick/5){notifications[i].y -= 5;}
			notifications[i].tick++;
			DrawRoundedRect(60, notifications[i].y, 120, 60, 10, WHITE);
			dispcolor_FillCircle(70, notifications[i].y+10, 3, ORANGE);
			if(strlen(notifications[i].message)<13){
				dispcolor_DrawString(120-(strlen(notifications[i].message)*4), notifications[i].y+22, FONTID_16F, notifications[i].message, WHITE);
			}else{
				char buff[14];
				for (uint8_t j = 0; j < 10; j++){
					buff[j] = notifications[i].message[j];
				}
				strcpy(buff + 10, "...");
				dispcolor_DrawString(120-(strlen(notifications[i].message)*4), notifications[i].y+22, FONTID_16F, buff, WHITE);
			}
		}
	}
}

bool Add_Message(char *message){
	for (uint8_t i =0; i < 5; i++){
		if (!notifications[i].isActive){
			notifications[i].isActive = true;
			notifications[i].message = message;
			notifications[i].y = -30;
			notifications[i].tick = 0;
			return true;
		}
	}
	return false;
}

void Draw_Easter(){
	HAL_Delay(30);
	dispcolor_FillScreen(BLACK);
	Draw_BMP(70, 50, skynet_100x64, 100, 64, 1);
	dispcolor_DrawString(10, 125, FONTID_16F, "The machines rose from the \n\rashes of the nuclear fire.", WHITE);
	Update_Messages();
	dispcolor_Update();
}

void Draw_Main_Screen(){
	HAL_Delay(30);
	dispcolor_FillScreen(BLACK);

	char time_buf[6];

  	RTC_TimeTypeDef gTime;
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	RTC_DateTypeDef gDate;
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	sprintf(time_buf, "%02d:%02d", gTime.Hours, gTime.Minutes);
	dispcolor_DrawString(85, 105, FONTID_32F, time_buf, WHITE);

	// Draw_BMP(140, 185, home_act_16x16, 16, 16, true);
	// Draw_BMP(100, 185, home_inact_16x16, 16, 16, true);

	Update_Messages();
	dispcolor_Update();
}

void Draw_NavBar(uint8_t devs, int8_t cur_dev){
	uint8_t y_pos = 190;
	uint8_t x_pos = 105;
	if(cur_dev != -1){
		Draw_BMP(x_pos, y_pos, home_inact_10x10, 10, 10, true);
		for(uint8_t dev = 0; dev < devs; dev++){
			if(dev == cur_dev){
				dispcolor_FillCircle(x_pos+15+10*dev, y_pos+5, 3, WHITE);
				continue;
			}
			dispcolor_FillCircle(x_pos+15+10*dev, y_pos+5, 3, GRAY);
		}
	}else{
		Draw_BMP(x_pos, y_pos, home_act_10x10, 10, 10, true);
		for(uint8_t dev = 0; dev < devs; dev++){
			dispcolor_FillCircle(x_pos+15+10*dev, y_pos+5, 3, GRAY);
		}
	}
}

void Display_Init(int data, int maxV){
	HAL_Delay(30);
	dispcolor_FillScreen(BLACK);
	dispcolor_DrawString(85, 105, FONTID_16F, "Device init", WHITE);
	uint32_t id = HAL_GetDEVID();
	char buf[10];
	sprintf(buf, "DEV ID: %d", id);
	dispcolor_DrawString(80, 175, FONTID_16F, buf, WHITE);
	for (int j = 1; j < maxV; j++){
		if (j > data){break;}
		for (int i = 1; i < 10; i++){
			dispcolor_FillCircle((4*i)+(27*j), 150, 10, 0x00FF00);
		}
	}
	Update_Messages();
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
	int8_t y = -30;
	while(HAL_GetTick() - tmr < 1000){
		HAL_Delay(30);
		dispcolor_FillScreen(BLACK);
		//dispcolor_DrawString(120-(strlen(msg)*4), y, FONTID_16F, msg, WHITE);
		
		// int16_t position = (time - (HAL_GetTick() - tmr)) * (max_angle_local - min_angle_local) / time + min_angle_local;

		// uint8_t mainRadius = 111;
		// uint16_t idx = 0;
		// for (int16_t angle = min_angle_local; angle < position; idx++, angle += 4) {
		// 	float angleRad = (float) angle * PI / 180;
		// 	int xMain = cos(angleRad) * mainRadius + xC;
		// 	int yMain = sin(angleRad) * mainRadius + yC;
		// 	dispcolor_FillCircle(xMain, yMain, 10, YELLOW);
		// }
		if(HAL_GetTick() - tmr < time/4){y+=5;}
		if(HAL_GetTick() - tmr > time*(3/4)){y-=5;}
		DrawRoundedRect(60, y, 120, 60, 10, WHITE);
		dispcolor_DrawString(120-(strlen(msg)*4), y, FONTID_16F, msg, WHITE);
		dispcolor_Update();
		// IWDG RESET START
		if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
		{
		Error_Handler();
		}
		// IWDG RESET END
	}
}

void Set_Time(RTC_DateTypeDef *gDate, RTC_TimeTypeDef *gTime){

}

void Show_SD_Warning(){
	uint32_t tmr = HAL_GetTick();
	while(HAL_GetTick() - tmr < 1000){
		HAL_Delay(30);
		dispcolor_FillScreen(BLACK);
		Draw_BMP(78, 78, sd_warn_84x84, 84, 84, 1);
		dispcolor_Update();
		// IWDG RESET START
		if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
		{
		Error_Handler();
		}
		// IWDG RESET END
	}
}

void Show_RTC_Warning(){
	uint32_t tmr = HAL_GetTick();
	while(HAL_GetTick() - tmr < 1000){
		HAL_Delay(30);
		dispcolor_FillScreen(BLACK);
		Draw_BMP(78, 78, rtc_warning_84x84, 84, 84, 1);
		dispcolor_Update();
		// IWDG RESET START
		if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
		{
		Error_Handler();
		}
		// IWDG RESET END
	}
}

void DrawRoundedRect(uint8_t x, int8_t y, uint8_t width, uint8_t height, uint8_t radius, uint16_t color) {
    int x1 = x + radius;                 // Left edge of the straight part
    int x2 = x + width - radius - 1;     // Right edge of the straight part
    int y1 = y + radius;                 // Top edge of the straight part
    int y2 = y + height - radius - 1;    // Bottom edge of the straight part

    // Draw the straight edges
    dispcolor_DrawLine(x1, y, x2, y, color);              // Top edge
    dispcolor_DrawLine(x1, y + height - 1, x2, y + height - 1, color); // Bottom edge
    dispcolor_DrawLine(x, y1, x, y2, color);              // Left edge
    dispcolor_DrawLine(x + width - 1, y1, x + width - 1, y2, color); // Right edge

	for (int j = x1; j <= x2; j++){
		dispcolor_DrawLine(j, y+1, j, y + height - 2, BLACK);              // Top edge
	}

    // Draw the corners as quarter circles
    for (int i = 0; i <= radius; i++) {
        int h = (int)sqrt(radius * radius - i * i); // Height of the quarter circle at this point

        // Top-left corner
        dispcolor_DrawPixel(x1 - i, y1 - h, color);
        dispcolor_DrawPixel(x1 - h, y1 - i, color);

        // Top-right corner
        dispcolor_DrawPixel(x2 + i, y1 - h, color);
        dispcolor_DrawPixel(x2 + h, y1 - i, color);

        // Bottom-left corner
        dispcolor_DrawPixel(x1 - i, y2 + h, color);
        dispcolor_DrawPixel(x1 - h, y2 + i, color);

        // Bottom-right corner
        dispcolor_DrawPixel(x2 + i, y2 + h, color);
        dispcolor_DrawPixel(x2 + h, y2 + i, color);
		if(i < radius){
			dispcolor_DrawLine(x1 - i, y1 - h+1, x1 - i, y2 + h-1, BLACK);
			dispcolor_DrawLine(x2 + i, y1-h+1, x2 + i, y2 + h -1, BLACK);
		}
    }
}

bool isTouchInsideArc(int touchX, int touchY, int centerX, int centerY, int innerRadius, int outerRadius, float startAngle, float endAngle) {
    // Convert degrees to radians for calculations
    const float DEG_TO_RAD = PI / 180.0;

    // Compute squared distance from touch point to center
    int dx = touchX - centerX;
    int dy = touchY - centerY;
    int distanceSquared = dx * dx + dy * dy;

    // Compute squared radii
    int innerRadiusSquared = innerRadius * innerRadius;
    int outerRadiusSquared = outerRadius * outerRadius;

    // Check if within the radii
    if (distanceSquared < innerRadiusSquared || distanceSquared > outerRadiusSquared) {
        return false; // Outside the ring
    }

    // Compute the angle of the touch point relative to the center
    float touchAngle = atan2(dy, dx) * (180.0 / PI); // Convert to degrees
    if (touchAngle < 0) {
        touchAngle += 360.0; // Normalize to [0, 360)
    }

    // Check if within the angular range
    if (startAngle <= endAngle) {
        // Simple case: start and end angles are in the correct order
        return touchAngle >= startAngle && touchAngle <= endAngle;
    } else {
        // Wraparound case: end angle is before start angle
        return touchAngle >= startAngle || touchAngle <= endAngle;
    }
}

float getTouchAngle(int touchX, int touchY) {
    // Calculate the differences in X and Y coordinates
    float dx = touchX - 120;
    float dy = touchY - 120;

    // Calculate the angle in radians using atan2
    float angleRad = atan2(dy, dx);

    // Convert radians to degrees
    float angleDeg = angleRad * (180.0 / PI);
	angleDeg -= 90;
    // Normalize the angle to the range [0, 360)
    if (angleDeg < 0) {
        angleDeg += 360.0;
    }

    return angleDeg;
}