#ifndef __TB6600_TIM_H__
#define __TB6600_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stm32f10x.h>

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define TB6600_TIMx                        TIM1
#define TB6600_TIM_APBxClock_FUN           RCC_APB2PeriphClockCmd
#define TB6600_TIM_CLK                     RCC_APB2Periph_TIM1

#define TB6600_TIM_GPIO_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define TB6600_TIM_GPIO_CLK                (RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO )
#define TB6600_TIM_CH1_PORT                GPIOA                 // 对应TB6600的PUL-，
#define TB6600_TIM_CH1_PIN                 GPIO_Pin_8            // 而PLU+直接接开发板的+5V(或者3.3V)

#define TB6600_DIR_GPIO_CLK                RCC_APB2Periph_GPIOB  // 电机旋转方向控制
#define TB6600_DIR_PORT                    GPIOB                 // 对应TB6600的DIR-，
#define TB6600_DIR_PIN                     GPIO_Pin_13           // 而DIR+直接接开发板的+5V(或者3.3V)

#define TB6600_ENA_GPIO_CLK                RCC_APB2Periph_GPIOB  // 电机使能控制，该引脚可不接悬空
#define TB6600_ENA_PORT                    GPIOB                 // 对应TB6600的ENA-，
#define TB6600_ENA_PIN                     GPIO_Pin_14           // 而ENA+直接接开发板的+5V(或者3.3V)

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void TB6600_TIMx_PWM_Init(void);

#endif	/* __TB6600_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
