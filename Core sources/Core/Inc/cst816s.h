// CST816S.h
#ifndef CST816S_H
#define CST816S_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// I2C address for CST816S touch IC
#define CST816S_I2C_ADDR 0x15

// CST816S register definitions
#define CST816S_REG_GESTURE_ID 0x01
#define CST816S_REG_FINGER_NUM 0x02
#define CST816S_REG_X_COORD_L  0x03
#define CST816S_REG_X_COORD_H  0x04
#define CST816S_REG_Y_COORD_L  0x05
#define CST816S_REG_Y_COORD_H  0x06

// GPIO pin definitions
#define CST816S_RST_PIN GPIO_PIN_8
#define CST816S_RST_PORT GPIOB
#define CST816S_INT_PIN GPIO_PIN_9
#define CST816S_INT_PORT GPIOB

#define X_MIN   3000
#define X_MAX   62000
#define Y_MIN   3000
#define Y_MAX   62000
#define X_CEN   32768
#define Y_CEN   32768

// Touch data structure
typedef struct {
    uint8_t gesture_id;
    uint8_t finger_num;
    uint16_t x_coord;
    uint16_t y_coord;
} CST816S_TouchData;

// Function prototypes
bool CST816S_Init(I2C_HandleTypeDef* hi2c);
bool CST816S_ReadTouchData(I2C_HandleTypeDef* hi2c, CST816S_TouchData* touch_data);
void CST816S_UpdateTouchData(CST816S_TouchData* touch_data);
#endif // CST816S_H
