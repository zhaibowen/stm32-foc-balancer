#include "PID.h"

void PI_Init(PIController* PIx, float Kp, float Ki, float limit) {
    PIx->Kp = Kp;
    PIx->Ki = Ki * 0.001f; // 固定周期，预乘时间
    PIx->limit = limit;
    PIx->proportional = 0;
    PIx->integral = 0;
}

// PI 控制器函数
float getPI(PIController* PIx, float error){
    PIx->proportional = PIx->Kp * error;
    PIx->integral += PIx->Ki * error;
    float output = PIx->proportional + PIx->integral;

    if (output > PIx->limit) {
        output = PIx->limit;
        PIx->integral = output - PIx->proportional; 
    } else if (output < -PIx->limit) {
        output = -PIx->limit;
        PIx->integral = output - PIx->proportional;
    }
    return output;
}
