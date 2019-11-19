#ifndef __SEARCHPATH_H__
#define __SEARCHPATH_H__

#include "motor.h"

#define SENSER_BANK GPIOC
#define SENSER_L GPIO_PIN_3
#define SENSER_M GPIO_PIN_4
#define SENSER_R GPIO_PIN_10

#define BUFSIZE 8
#define BUFMASK 0X07;

void Searchpath(int16_t *speed,motortype *mt);

#endif
