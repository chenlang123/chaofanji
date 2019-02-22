#ifndef __STEP_MOTION_H
#define __STEP_MOTION_H 			   
#include "sys.h"

#include <math.h>

#define ACCELERATED_SPEED_LENGTH 3000   //¶¨Òå¼ÓËÙ¶ÈµÄµãÊı£¬3000¸öÏ¸·Ö²½£¬µ÷Õâ¸ö²ÎÊı¸Ä±ä¼ÓËÙµã
#define FRE_MIN 300   									//×îµÍµÄÔËĞĞÆµÂÊ£¬µ÷Õâ¸ö²ÎÊıµ÷½Ú×îµÍÔËĞĞËÙ¶È
#define FRE_MAX 24000 									//×î¸ßµÄÔËĞĞÆµÂÊ£¬µ÷Õâ¸ö²ÎÊıµ÷½ÚÔÈËÙÊ±µÄ×î¸ßËÙ¶È

void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible);
void STEP_MOTION_IRQHandler(char tim_class);

#endif	




