#ifndef __STEP_MOTION_H
#define __STEP_MOTION_H 			   
#include "sys.h"

#include <math.h>

#define ACCELERATED_SPEED_LENGTH 3000   //定义加速度的点数，3000个细分步，调这个参数改变加速点
#define FRE_MIN 300 � 									//最低的运行频率，调这个参数调节最低运行速度
#define FRE_MAX 24000 									//最高的运行频率，调这个参数调节匀速时的最高速度

void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible);
void STEP_MOTION_IRQHandler(char tim_class);

#endif	




