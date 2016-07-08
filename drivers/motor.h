#ifndef __MOTOR
#define __MOTOR value

#include "conf.h"
#define MAX_PWM 3800
#define MIN_PWM 2000
#define PWM_RANGE 1800
#define STOP_PWM 1600

void Motor_init(void);
void Motor_Disabled(void);

#endif /* ifndef __MOTOR */
