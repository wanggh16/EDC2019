#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "motor.h"

#define HADC hadc1
#define BAT_PERIOD 200

uint8_t Check_volt(uint16_t *volt);

#endif
