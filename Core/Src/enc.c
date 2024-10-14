#include "enc.h"

void initEnc(Encoder *enc){
    enc->s1LastState = !HAL_GPIO_ReadPin(enc->s1_port, enc->s1_pin);
}


void tickEnc(Encoder *enc){
    bool s1State = !HAL_GPIO_ReadPin(enc->s1_port, enc->s1_pin);
    if(s1State != enc -> s1LastState){
        if(HAL_GPIO_ReadPin(enc->s2_port, enc->s2_pin) != s1State){
            enc->isRightTurn = true;
        }else{
            enc->isLeftTurn = true;
        }
    }
}

bool isLeft(Encoder *enc){
    if(enc->isLeftTurn){
        enc->isLeftTurn = false;
        return true;
    } return false;
}

bool isRight(Encoder *enc){
    if(enc->isRightTurn){
        enc->isRightTurn = false;
        return true;
    } return false;
}