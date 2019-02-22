#ifndef __STEP_MOTION_H
#define __STEP_MOTION_H 			   
#include "sys.h"

#include <math.h>

#define ACCELERATED_SPEED_LENGTH 3000   //������ٶȵĵ�����3000��ϸ�ֲ�������������ı���ٵ�
#define FRE_MIN 300 � 									//��͵�����Ƶ�ʣ����������������������ٶ�
#define FRE_MAX 24000 									//��ߵ�����Ƶ�ʣ������������������ʱ������ٶ�

void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible);
void STEP_MOTION_IRQHandler(char tim_class);

#endif	




