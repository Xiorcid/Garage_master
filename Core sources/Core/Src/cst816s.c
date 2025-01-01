// CST816S.c
#include "cst816s.h"

// Initialize the CST816S touch IC
bool CST816S_Init(I2C_HandleTypeDef* hi2c) {
    // Reset the CST816S using GPIO
    HAL_GPIO_WritePin(CST816S_RST_PORT, CST816S_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CST816S_RST_PORT, CST816S_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(50);

    // Test communication with the CST816S by reading a register
    uint8_t reg = CST816S_REG_GESTURE_ID;
    if (HAL_I2C_Master_Transmit(hi2c, CST816S_I2C_ADDR << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false; // Communication failed
    }

    uint8_t data;
    if (HAL_I2C_Master_Receive(hi2c, CST816S_I2C_ADDR << 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false; // Communication failed
    }

    return true;
}

// Read touch data from the CST816S
bool CST816S_ReadTouchData(I2C_HandleTypeDef* hi2c, CST816S_TouchData* touch_data) {
    if (touch_data == NULL) {
        return false; // Invalid pointer
    }

    // uint8_t reg = CST816S_REG_GESTURE_ID;
    // if (HAL_I2C_Master_Transmit(hi2c, CST816S_I2C_ADDR << 1, &reg, 1, 100) != HAL_OK) {
    //     return false; // Communication failed
    // }

    uint8_t buffer[6];
    if (HAL_I2C_Master_Receive(hi2c, CST816S_I2C_ADDR << 1, buffer, 6, 100) != HAL_OK) {
        return false; // Communication failed
    }

    touch_data->gesture_id = buffer[0];
    touch_data->finger_num = buffer[1];
    touch_data->x_coord = (buffer[2] | (buffer[3] << 8));
    touch_data->y_coord = (buffer[4] | (buffer[5] << 8));

    return true;
}