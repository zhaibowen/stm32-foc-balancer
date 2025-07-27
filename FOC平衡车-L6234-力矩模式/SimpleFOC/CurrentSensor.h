#ifndef CurrentSensor_H
#define CurrentSensor_H

typedef struct 
{
    int pinA;
    int pinC;
    int dir_a;
    int dir_c;
    float ratio_a;
    float ratio_c;
    float offset_ia;
    float offset_ic;
	float ia;
    float ib;
	float ic;

} CurrentSensor;

void CurrentSensor_Init(CurrentSensor* CSx, int pinA, int pinC, int dir_a, int dir_c, int gain, float resistor);
void getPhaseCurrents(CurrentSensor* CSx);

#endif
