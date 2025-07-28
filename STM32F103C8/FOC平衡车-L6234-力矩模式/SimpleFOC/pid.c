#include "stm32f10x.h"
#include "timer.h"
#include "usart.h"
#include "pid.h"
#include "foc_utils.h"

void PID_Init(PIDController* PIDx, float P, float I, float D, float ramp, float limit) {
    PIDx->P = P;
    PIDx->I = I;
    PIDx->D = D;
    PIDx->output_ramp = ramp;
    PIDx->limit = limit;
    PIDx->error_prev = 0;
    PIDx->output_prev = 0;
    PIDx->integral_prev = 0;
}

// PID 控制器函数
float getPID(PIDController* PIDx, float error){
	unsigned long now_us;
	float Ts;

	now_us = time_cntr;
	if (now_us < PIDx->time_prev) {
		Ts = (ulong_max_value - PIDx->time_prev + now_us) * 1e-5;
	} else {
		Ts = (now_us - PIDx->time_prev) * 1e-5;
	}
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;
    PIDx->time_prev = now_us;

    // P环
    float proportional = PIDx->P * error;
    // Tustin 散点积分（I环）
    float integral = PIDx->integral_prev + PIDx->I * Ts * 0.5f * (error + PIDx->error_prev);
    integral = _constrain(integral, -PIDx->limit, PIDx->limit);
    // D环（微分环节）
    float derivative = PIDx->D * (error - PIDx->error_prev) / Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -PIDx->limit, PIDx->limit);

    if(PIDx->output_ramp > 0) {
        // 对PID的变化速率进行限制
        float output_rate = (output - PIDx->output_prev) / Ts;
        if (output_rate > PIDx->output_ramp) {
            output = PIDx->output_prev + PIDx->output_ramp * Ts;
        } else if (output_rate < -PIDx->output_ramp) {
            output = PIDx->output_prev - PIDx->output_ramp * Ts;
        }
    }
    // 保存值（为了下一次循环）
    PIDx->proportional = proportional;
    PIDx->integral_prev = integral;
    PIDx->output_prev = output;
    PIDx->error_prev = error;
    return output;
}
