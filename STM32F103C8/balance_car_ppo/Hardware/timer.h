
#ifndef STM32_TIMER_H
#define STM32_TIMER_H

void TIM1_1kHz_Init(void);
void TIM2_PWM_Init(uint16_t PWM_Period);
void TIM3_PWM_Init(uint16_t PWM_Period);
void TIM4_1kHz_Init(void);
float get_time_interval(uint32_t *timestamp_prev);
void DWT_Init(void);

#endif
