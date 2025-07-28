#ifndef CurrentSensor_H
#define CurrentSensor_H
#include "stm32f10x.h"
typedef struct 
{
    int pinA;
    int pinB;
    int dir_a;
    int dir_b;
    float ratio_a;
    float ratio_b;
    float offset_ia;
    float offset_ib;
	float ia;
	float ib;
    uint8_t number;
} CurrentSensor;

void CurrentSensor_Init(CurrentSensor* CSx, uint8_t number, int pinA, int pinB, int dir_a, int dir_b, int gain, float resistor);
void getPhaseCurrents(CurrentSensor* CSx);

#endif
