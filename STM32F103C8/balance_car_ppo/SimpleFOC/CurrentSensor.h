#ifndef CurrentSensor_H
#define CurrentSensor_H
#include "stm32f10x.h"
typedef struct 
{
    int pinA, pinB;
    int dir_a, dir_b;
    float ratio_a, ratio_b;
    float offset_ia, offset_ib;
	float ia, ib;
    float Iq, Id;
    uint8_t number;
} CurrentSensor;

void CurrentSensor_Init(CurrentSensor* CSx, uint8_t number, int pinA, int pinB, int dir_a, int dir_b, int gain, float resistor);
void getPhaseCurrents(CurrentSensor* CSx);
void cal_Iq_Id(CurrentSensor* CS, float ct, float st);

#endif
