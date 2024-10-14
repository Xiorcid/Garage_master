#ifndef _ENC
#define _ENC

#include "stdbool.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <gpio.h>

typedef struct _Encoder{
    GPIO_TypeDef* s1_port;
    uint16_t s1_pin;
    GPIO_TypeDef* s2_port;
    uint16_t s2_pin;
    bool s1LastState;
    bool isLeftTurn;
    bool isRightTurn;
}Encoder;

void initEnc(Encoder *enc);
void tickEnc(Encoder *enc);
bool isLeft(Encoder *enc);
bool isRight(Encoder *enc);

#endif