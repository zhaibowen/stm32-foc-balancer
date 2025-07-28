#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/**
  * 低通滤波器类
  */
  
typedef struct  
{
    // float Tf; //!< 低通滤波时间常数
    float y_prev; //!< 上一个循环中的过滤后的值
    // unsigned long timestamp_prev;
    // float dt;
} LowPassFilter;

void LowPassFilter_init(LowPassFilter* LPFx);
float getLowPassFilter(LowPassFilter* LPFx, float x);

#endif
