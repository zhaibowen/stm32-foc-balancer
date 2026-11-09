#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/**
  * 低通滤波器类
  */
  
typedef struct {
    float y_prev; //!< 上一个循环中的过滤后的值
    float alpha;
} LowPassFilter;

void LowPassFilter_init(LowPassFilter* LPFx, float alpha);
float getLowPassFilter(LowPassFilter* LPFx, float x);

#endif
