#ifndef PID_H
#define PID_H

typedef struct 
{
    float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< Derivative gain 
    float output_ramp; 
    float limit; 
    float proportional;

    float error_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
    unsigned long time_prev;
} PIDController;

void PID_Init(PIDController* PIDx, float P, float I,  float D, float ramp, float limit);
float getPID(PIDController* PIDx, float error);

#endif
