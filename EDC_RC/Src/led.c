#include "main.h"
#include "stm32f1xx_hal.h"
#include "led.h"

uint16_t led_times = 0;
uint32_t led_period = 100;

void Set_blink(uint16_t times,uint32_t period)//ms
{
	LL_GPIO_ResetOutputPin(LED_BANK, LED_PORT_LL);
	led_times = times*2;
	led_period = period/2;
}

void Led_task(void)
{
	uint32_t time_curr;
	static uint32_t time_next = 0;
	if(led_times != 0)
	{
		time_curr = HAL_GetTick();
		if(time_curr >= time_next)
		{
			HAL_GPIO_TogglePin(LED_BANK,LED_PORT);
			--led_times;
			time_next = time_curr + led_period;
		}
	}
}
