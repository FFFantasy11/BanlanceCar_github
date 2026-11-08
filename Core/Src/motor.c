/* ============================================================================
 * motor.c —— 电机输出层
 * 两件事：把 PWM 写到硬件上；做死区补偿。
 * ==========================================================================*/

#include "motor.h"
#include "math.h"

#define PWM_MAX 7200        /* 满占空比（定时器周期 7199） */
#define PWM_MIN -7200       /* 反方向满占空比 */

/* 摔倒保护阈值。倾角超过 FALL_ANGLE 锁存保护并切断电机，
 * 回到 FALL_ANGLE - FALL_HYST 以内才解除。这 10 度差值叫迟滞，
 * 用来防止在临界角度附近反复通断。 */
#define FALL_ANGLE 45	/* 度：超过则判定摔倒 */
#define FALL_HYST  10	/* 度：回到 35 度以内解除保护 */

/* ============================================================================
 * 死区补偿
 *
 * 死区：便宜的减速电机配 TB6612，PWM 太小时根本转不起来，本车实测约需 350。
 * 于是控制器给出很小的输出时电机毫无反应，车在接近直立时其实处于失控状态 ——
 * 这就是"推一下没反应，再推一点就猛地窜出去"的原因。
 *
 * 补偿办法：输出时叠加一个偏移量
 *     out = p + PWM_DEADZONE × tanh(p / PWM_DZ_SOFT)
 * 该函数过零点且平滑，指令穿越零时不会跳变，同时补回全部偏移。
 *
 * PWM_DEADZONE：要补偿的起步 PWM。0 表示关闭补偿。
 * PWM_DZ_SOFT ：拐点柔和度。越小拐得越急、死区越窄。
 *               注意小信号斜率 = 1 + PWM_DEADZONE/PWM_DZ_SOFT，
 *               相当于给环路额外加增益。本环路稳定余量有限，该斜率是硬上限：
 *               若加大补偿后车开始抖动，说明已超出余量。
 * ==========================================================================*/
#define PWM_DEADZONE 260
#define PWM_DZ_SOFT  100

extern TIM_HandleTypeDef htim1;
extern uint8_t stop;

/* 求绝对值。自己实现避免链接额外库代码。 */
int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}

/* 死区补偿。奇函数（f(-x) = -f(x)）所以不用处理正负号；
 * 输入 0 输出也是 0，保证电机能被干净关掉。 */
static int Comp(int p)
{
	float out;

	out=(float)p + (float)PWM_DEADZONE*(float)tanh((double)p/(double)PWM_DZ_SOFT);

	if(out> (float)PWM_MAX) out= (float)PWM_MAX;
	if(out< (float)PWM_MIN) out=(float)PWM_MIN;

	return (int)out;
}

/* 驱动两个电机。范围 -7200 ~ 7200，符号决定方向，数值决定转速。
 * 方向由两个 GPIO 电平组合决定，转速由 PWM 占空比决定。 */
void Load(int moto1,int moto2)			//-7200~7200
{
	if(moto1<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,Comp(abs(moto1)));
	if(moto2<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Comp(abs(moto2)));
}

/* 把两个输出限制在 ±PWM_MAX 内。要在死区补偿之前做，因为补偿还会额外叠加偏移。 */
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/* 摔倒保护（安全监视）。返回 1 表示已锁存，调用者必须切断电机。
 * 倾角回到 35 度以内时锁存自动解除。 */
uint8_t Stop(float *Med_Jiaodu,float *Jiaodu)
{
	int err=abs((int)(*Jiaodu-*Med_Jiaodu));

	if(stop==0)
	{
		if(err>FALL_ANGLE)				/* 倾斜过度 → 锁存 */
			stop=1;
	}
	else
	{
		if(err<(FALL_ANGLE-FALL_HYST))	/* 重新立起 → 解除 */
			stop=0;
	}
	return stop;
}
