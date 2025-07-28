#include "stm32f10x.h"
#include "lowpass_filter.h"
#include "timer.h"

void LowPassFilter_init(LowPassFilter* LPFx) {
    // LPFx->Tf = time_constant;
    LPFx->y_prev = 0;
    // LPFx->timestamp_prev = time_cntr;
}

float getLowPassFilter(LowPassFilter* LPFx, float x) {
	// unsigned long now_us;
	// float Ts;

	// now_us = time_cntr;
	// if (now_us < LPFx->timestamp_prev) {
	// 	Ts = (ulong_max_value - LPFx->timestamp_prev + now_us) * 1e-5;
	// } else {
	// 	Ts = (now_us - LPFx->timestamp_prev) * 1e-5;
	// }
    // LPFx->timestamp_prev = now_us;
    // LPFx->dt = Ts;
	// if(Ts > 0.3)   //时间过长，大概是程序刚启动初始化，直接返回
	// {
	// 	LPFx->y_prev = x;
	// 	return x;
	// }

    // float alpha = LPFx->Tf / (LPFx->Tf + Ts);
    // float y = alpha * LPFx->y_prev + (1.0f - alpha) * x;
	float y = 0.9 * LPFx->y_prev + 0.1 * x;
    LPFx->y_prev = y;
    return y;
}
