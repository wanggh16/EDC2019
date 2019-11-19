#ifndef __LED_H__
#define __LED_H__

#define LED_BANK GPIOD
#define LED_PORT GPIO_PIN_2
#define LED_PORT_LL LL_GPIO_PIN_2

void Set_blink(uint16_t times,uint32_t period);//ms
void Led_task(void);

#endif
