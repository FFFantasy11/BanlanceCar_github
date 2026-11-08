/* ============================================================================
 * encoder.c —— 编码器读数
 * ==========================================================================*/

#include "encoder.h"

/* 读出编码器定时器累计的脉冲数，读后清零，所以返回值代表
 * "上次读取到现在转过的角度"。有符号：正负对应两个转向。 */
int Read_Speed(TIM_HandleTypeDef *htim)
{
	int temp;
	temp=(short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}
