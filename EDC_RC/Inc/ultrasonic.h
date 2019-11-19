#ifndef __SEARCHPATH_H__
#define __SEARCHPATH_H__

#define US_BANK GPIOC
#define US_TRIG LL_GPIO_PIN_0
#define US_ECHO GPIO_PIN_1
#define USTIM TIM6

void Msr_dist_task(void);

extern uint32_t time_len;
extern uint8_t us_en;

__STATIC_INLINE void start_US_daemon(void)
{
	us_en = 1;
}

__STATIC_INLINE uint32_t Get_dist(void)
{
	return (time_len *170/1000);
}

#endif
