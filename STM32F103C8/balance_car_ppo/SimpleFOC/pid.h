#ifndef PI_H
#define PI_H

typedef struct 
{
    float Kp; //!< 比例增益(P环增益)
    float Ki; //!< 积分增益（I环增益）
    float proportional;
    float integral;
    float limit; 
} PIController;

void PI_Init(PIController* PIx, float Kp, float Ki, float limit);
float getPI(PIController* PIx, float error);

#endif
