#ifndef __TIM1_H
#define __TIM1_H 			   
#include "sys.h"

#include "STEP_MOTION.h"
void TIM1_GPIO_Config(void);
void TIM1_Mode_Config(void);
void PWM_TIM1(uint16_t arr,uint16_t psc);

#endif	


