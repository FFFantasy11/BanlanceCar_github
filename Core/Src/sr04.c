/* ============================================================================
 * sr04.c —— 超声波测距（HC-SR04）
 *
 * 工作方式：先给 Trig 一个约 12 微秒的高电平脉冲，模块发出超声波；
 * 收到回波后 Echo 引脚会输出一个高电平脉冲，宽度正比于距离。
 * Echo 接在外部中断上（PA2，双边沿触发）：
 *   上升沿 → 把定时器 TIM3 清零并启动
 *   下降沿 → 停止定时器，读出计数值即为回波宽度（微秒）
 * TIM3 预分频设为 71，所以 1 个计数 = 1 微秒。
 * ==========================================================================*/

#include "sr04.h"
#include "pid.h"
uint16_t count;         /* 回波脉冲宽度（微秒） */
float distance;         /* 换算出的距离（厘米） */
extern TIM_HandleTypeDef htim3;

/* 微秒级粗略延时。用循环空转实现，不依赖定时器。 */
void RCCdelay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

/* 发出一次测距触发脉冲 */
void GET_Distance(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	RCCdelay_us(12);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

/* 外部中断回调。判断 Echo（PA2）的电平来分隔一次测距的起点和终点。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_2)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_SET)
		{
			/* 上升沿：回波开始，计时器清零并启动 */
			__HAL_TIM_SetCounter(&htim3,0);
			HAL_TIM_Base_Start(&htim3);
		}
		else
		{
			/* 下降沿：回波结束，读取宽度并换算距离。
			   声速约 340 m/s，一来一回共 2 倍路程，
			   故 距离(cm) = 微秒 × 0.017。 */
			HAL_TIM_Base_Stop(&htim3);
			count=__HAL_TIM_GetCounter(&htim3);
			distance=count*0.017;
		}
	}
}
