#include "stm32f10x.h"
#include "lowpass_filter.h"
#include "timer.h"

void LowPassFilter_init(LowPassFilter* LPFx, float alpha) {
    LPFx->y_prev = 0;
    LPFx->alpha = alpha;
}

float getLowPassFilter(LowPassFilter* LPFx, float x) {
	float y = (1 - LPFx->alpha) * LPFx->y_prev + LPFx->alpha * x;
    LPFx->y_prev = y;
    return y;
}
