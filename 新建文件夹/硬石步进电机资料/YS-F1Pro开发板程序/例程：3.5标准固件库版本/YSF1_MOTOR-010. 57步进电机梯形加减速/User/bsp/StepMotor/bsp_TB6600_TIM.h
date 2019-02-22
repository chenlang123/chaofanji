#ifndef __TB6600_TIM_H__
#define __TB6600_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stm32f10x.h>

/* 类型定义 ------------------------------------------------------------------*/
typedef enum {FALSE = 0, TRUE = !FALSE} bool;

enum // 回零状态
{
 	IDEL=0,       // 空闲状态
 	FASTSEEK,     // 快速寻找状态
	FASTSEEKSTOP, // 快速寻找停止状态
	FASTSEEKBACK, // 开始返回寻找状态
	SLOWSEEK,     // 慢速寻找
	SLOWSEEKSTOP, // 慢速寻找停止状态
	MOVETOZERO    // 已回零
};

typedef struct {
  uint8_t run_state ;//! What part of the speed ramp we are in. 旋转状态
  uint8_t dir ;//! Direction stepper motor should move. 旋转方向
  int32_t step_delay;//! Peroid of next timer delay. At start this value set the accelration rate. 
  uint32_t decel_start;//! What step_pos to start decelaration 减速启动
  int32_t decel_val;//! Sets deceleration rate. 设置减速速率
  int32_t min_delay;//! Minimum time delay (max speed)最小延时时间(对应最大速度)
  int32_t accel_count;//! Counter used when accelerateing/decelerateing to calculate step_delay.加减速阶段计数值
}speedRampData;

/* 宏定义 --------------------------------------------------------------------*/
#define AXIS_TIMX_CLOCKCMD           RCC_APB2PeriphClockCmd
#define AXIS_TIMx_RCC_CLK            RCC_APB2Periph_TIM1
#define AXIS_TIMx                    TIM1
#define AXIS_TIMx_IRQn               TIM1_CC_IRQn
#define AXIS_TIMx_IRQHandler         TIM1_CC_IRQHandler
#define AXIS_TIMx_OCnInit            TIM_OC1Init
#define AXIS_TIM_SetCompareN         TIM_SetCompare1

#define AXIS_PULSE_CLK               RCC_APB2Periph_GPIOA
#define AXIS_PULSE_1                 GPIO_Pin_8        // 脉冲输出引脚
#define AxisPulsePort                GPIOA

#define AXIS_DIR_CLK                 RCC_APB2Periph_GPIOB
#define AXIS_DIR_1                   GPIO_Pin_13        // 方向控制引脚
#define AxisDirPort                  GPIOB
#define Axis_SETDIR_CW()             GPIO_SetBits(AxisDirPort, AXIS_DIR_1)
#define Axis_SETDIR_CCW()            GPIO_ResetBits(AxisDirPort, AXIS_DIR_1)

#define AXIS_LMTPOS_CLK              RCC_APB2Periph_GPIOC
#define AXIS_LMTPOS_1                GPIO_Pin_2        // 正转限位输入引脚
#define AxisLmtPosPort               GPIOC

#define AXIS_LMTNEG_CLK              RCC_APB2Periph_GPIOC
#define AXIS_LMTNEG_1                GPIO_Pin_3        // 反转限位输入引脚
#define AxisLmtNegPort               GPIOC

#define AXIS_HOME_CLK                RCC_APB2Periph_GPIOC
#define AXIS_HOME_1                  GPIO_Pin_4        // 零点输入引脚
#define AxisHomePort                 GPIOC
#define AxisEXTI_PORTSource          GPIO_PortSourceGPIOC
#define AxisEXTIPinSource            GPIO_PinSource4
#define AxisEXTILine                 EXTI_Line4
#define AxisEXTI_IRQCHANNEL          EXTI4_IRQn
#define AxisEXTI_IRQHANDLER          EXTI4_IRQHandler

#define POSITIVE	                   1       // 方向：正方向
#define NEGATIVE	                   -1      // 方向：反方向

#define CW                           0       // 正转
#define CCW                          1       // 反转
#define T1_FREQ                      10000   // 频率

// Speed ramp states
#define STOP                         0      // 停止
#define ACCEL                        1      // 加速
#define DECEL                        2      // 减速
#define RUN                          3      // 正常运行

/* 扩展变量 ------------------------------------------------------------------*/
extern int32_t  ZeroDir;
extern uint32_t accel;         // 加速度
extern uint32_t decel;         // 减速度

/* 函数声明 ------------------------------------------------------------------*/
void StepMotor_TIMx_Init(void);
void AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void AxisMoveAbs(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
void LimitDetect(void);
void AxisHome(void);

#endif	/* __TB6600_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
