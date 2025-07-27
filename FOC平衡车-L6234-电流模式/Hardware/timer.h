
#ifndef STM32_TIMER_H
#define STM32_TIMER_H
extern unsigned long time_cntr;
extern unsigned long ulong_max_value;

void TIM2_PWM_Init(uint16_t PWM_Period);
void TIM3_PWM_Init(uint16_t PWM_Period);
void TIM4_10us_Init(void);

#endif
