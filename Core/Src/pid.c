/* ============================================================================
 * pid.c —— 平衡车控制核心
 *
 * 三个控制环，由外到内嵌套：
 *
 *     速度环 Velocity   管"车站在哪里"：被推走要自己走回来
 *          ↓ 输出角度偏移
 *     直立环 Vertical   管"车立得直不直"：把车拉回平衡角
 *          ↓ 输出 PWM
 *     转向环 Turn       管"车朝向哪边"：调节左右轮差速
 *
 * Control() 由 MPU6050 的 INT 引脚触发，实测每 8.9 ms 执行一次（约 112 Hz）。
 * ==========================================================================*/

#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "math.h"


/* ============================================================================
 * 传感器数据与中间变量
 * ==========================================================================*/

/* 传感器数据 */
int Encoder_Left,Encoder_Right;     /* 本周期内左右轮转过的脉冲数 */
float pitch,roll,yaw;               /* 姿态角。roll 是左右倾斜角，平衡车的关键角度 */
short gyrox,gyroy,gyroz;            /* 角速度原始值。gyrox 供直立环做阻尼，
                                       gyroz 供转向环做反馈 */
short	aacx,aacy,aacz;             /* 加速度计原始值，当前算法未使用 */

/* 控制中间变量 */
int Vertical_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Velocity_out;		/* 速度环输出，单位是"度"，加到直立环的角度目标上。
						   必须是浮点：改成整数会把角度目标量化成 1 度一跳，
						   经直立环 -159 倍放大后是 159 的 PWM 跳变，比电机死区
						   还大，车会一直小幅震荡。 */

float Med_Angle=2.20f;	/* 机械中心角（度）——车体重心正好在轮轴正上方时 roll 的读数。
						   实测值，取跑起来后 roll 的平均值即可（车的加速度正比于
						   tan(θ_b - roll)，轮速回到起点时积分为零，故平均值就是 θ_b）。
						   这个值准不准决定车会不会自己走偏。 */

/* ============================================================================
 * 控制参数
 * ==========================================================================*/

/* 直立环 PD：让车立直的主力气
 *   Vertical_Kp 比例项，偏离平衡角越多回正力越大，越大越"硬"
 *   Vertical_Kd 微分项，用角速度做阻尼防止冲过头，越大越"稳"
 * 太小车软、推一下就倒；太大会高频抖动甚至翻车。 */
float Vertical_Kp=-159,Vertical_Kd=-0.8;

/* 速度环 PI：让车"站回原位"而不是滑走
 *   Velocity_Kp 速度阻尼，让车能停下来
 *   Velocity_Ki 位置反馈，把车往回走，越大回得越快
 * 注意速度环是外环，其延迟直接吃掉稳定性余量。Kp 调到 -1.0 量级会让车以约
 * 1 Hz 自激振荡，所以增益必须保持较低。 */
float Velocity_Kp=-0.58,Velocity_Ki=-0.005;

/* 转向环 PI：闭环控制转向角速度
 * 输入是遥控下发的目标转速，反馈是陀螺仪实测转速，两者相减得到误差。
 * 用闭环是因为开环方案从不比较指令与实际，实际转速会随电池电压和地面摩擦变化。
 * 需要积分项：电机有死区，纯 P 环维持不了很小的转速；陀螺仪原始值也有零漂。 */
float Turn_Kp=0.8,Turn_Ki=0.02;

uint8_t stop;               /* 摔倒保护标志，1 = 已锁存，见 motor.c 的 Stop() */

/* 控制周期计数器，每执行一次 Control() 加一。主循环用它给串口发送计时，
 * 这样即使发送本身阻塞，采样间隔依然均匀。 */
volatile uint32_t Control_Tick=0;

/* 实测控制周期：算法必须知道两次计算之间真实过了多久。若写死 0.01 s 而中断实际
 * 只有 50 Hz，算法以为转过的角度只有真实值一半，车要歪几厘米才"感觉到"倾斜，
 * 然后过度修正 —— 表现就是迟钝但一修就过头。
 * 这里用 HAL_GetTick() 对 100 次调用取平均实测，并送串口显示。 */
float Control_dt=0.01f;			/* 两次 Control() 之间的秒数        */
float Control_dt_ms=10.0f;		/* 同上，单位毫秒，用于显示          */
static uint32_t Control_dt_t0=0;
static uint16_t Control_dt_n=0;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;			/* 超声波距离（厘米），由 sr04.c 中断写入 */
extern uint16_t count;			/* 回波脉冲宽度（微秒），同样由该中断设置 */
extern uint8_t Fore,Back,Left,Right;    /* 蓝牙遥控方向标志，见 stm32f1xx_it.c */
void GET_Distance(void);		/* 发出一次超声波触发脉冲，定义在 sr04.c */

/* ============================================================================
 * 超声波避障
 * 前方 30 cm 内有东西就不让继续前进；后退永不拦截，保证车能退出来。
 *
 * 坑：没收到回波时中断不执行，distance 会一直保留旧值，看起来正常其实已"瞎"。
 * 所以盯着回波宽度 count —— 只有真收到回波它才变。先记下 count，发射一次，
 * 等一小会儿再看它有没有变；没变就认为读数过期，跳过避障判断。
 * ==========================================================================*/
#define OBSTACLE_CM        30	/* 停车距离（厘米） */
#define SONAR_PERIOD_TICKS 12	/* 每 12 个控制周期（约 107 ms）发一次超声波 */
#define SONAR_WINDOW_TICKS  6	/* 发射后等 6 个周期（约 53 ms）收回波 */

/* 1 = 本次读数是真实回波可用于避障；0 = 无回波，跳过判断（屏幕显示 D:--） */
uint8_t distance_valid=0;

/* 遥控速度上限，单位"每周期编码器脉冲数"。
 * 编码器实测 8197 脉冲/米、周期 8.9 ms，故 12 ≈ 0.165 m/s，慢到电机 350 的死区
 * 占主导，速度误差平均不到零，积分项会一路累积。40 ≈ 0.55 m/s（正常步行速度）
 * 时控制环更容易稳住。Target_Speed 每周期只变 1，到 40 约需 355 ms，起步柔和。 */
#define SPEED_Y 	12

/* 转向指令上限，单位陀螺仪原始值。量程 ±2000 度/秒对应 16.4 原始值/(度/秒)，
 * 故 1640 = 100 度/秒。必须与 gyroz 同单位，因为转向环要把两者相减。 */
#define SPEED_Z 	1640

/* 按住转向键时，目标转速每周期增加多少。100 表示约 0.14 s 加到上限。 */
#define TURN_RAMP 	100

/* 速度环误差低通滤波系数，0.7 对应约 25 ms 时间常数。
 * 越大越平滑但车越迟钝；速度环是外环，延迟直接侵蚀稳定余量，不要往上调大。 */
#define VEL_LPF_A     0.7f

/* 速度环比例项（速度阻尼）最多把角度目标推动多少度 —— 这是"刹车能力"的来源。
 * 最坏情况 18 + 15 = 33 度，仍在 45 度摔倒保护线内。 */
#define VEL_P_LIMIT   18.0f

/* 速度环积分项（位置反馈）最多把角度目标推动多少度。
 * 它不只决定安全上限，还决定"车能被推多远仍保持线性"：
 *      可推距离 = VEL_INT_LIMIT / (|Ki| × 8197)  米
 * 积分项一旦饱和，位置环等于开环，车退化成开关控制，一推就冲过头。
 * 关键：P 项和 I 项必须分别限幅，绝不对两者之和做一次限幅 —— 否则积分项攒满后
 * 会挤占比例项空间，车在最需要刹车时恰恰失去刹车。 */
#define VEL_INT_LIMIT 15.0f

/* 转向环限幅，同理：积分项与输出各自独立限幅 */
#define TURN_INT_LIMIT 1500.0f	/* 转向积分项最多贡献多少 PWM */
#define TURN_OUT_LIMIT 2500.0f	/* 转向环最多要求多少差速 PWM */

/* 转向方向符号，只能是 +1 或 -1。反了阻尼就变正反馈，车会自己越转越快。
 *
 * 验证方法（把车拿在手里、两轮悬空，此时不可能跑掉）：
 *   1. 不要碰转向按键（目标转速 = 0）
 *   2. 用手把车身往左拧，看两个轮子
 *   3. 轮子应"抵抗"这个旋转，即试图把车拧回右边 —— 符号正确
 *   4. 若轮子反而帮着往左转，就把符号反过来
 *
 * 改动此值后必须重新做一次上面的验证，否则有失控风险。 */
#define TURN_SIGN  (+1)


/* ============================================================================
 * 三个控制环
 * ==========================================================================*/

/* 直立环 PD
 * 偏离目标多少就回正多少（P），再加角速度阻尼防止冲过头（D）。 */
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

/* 速度环滤波器状态。用浮点不用整数：截断成整数会量化输出，且误差小时滤波器会卡住。 */
static float Err_LowOut_last=0.0f;

/* 速度环积分器。累加的是速度误差，所以物理意义是"通电至今走了多远"（单位：脉冲）。
 * 暴露出来是为了实测编码器刻度：用手把车恰好推 1.00 米，看它变化多少，
 * 那个数就是真实的每米脉冲数。代码一直用的是推算值 8790，若编码器线数不是 11，
 * 这个常数会差 1.5~2 倍，足以把"刚好稳定"变成"发散"。 */
float Speed_Pos=0.0f;

/* 不受限幅的距离累加器，只用于标定，控制算法不用。
 * 上面的 Speed_Pos 被 VEL_INT_LIMIT 限幅后只能记录约 0.43 米，推 1.00 米会提前
 * 饱和导致测量失真。这个没有限幅。故意不被 Velocity_Reset() 清零，
 * 以免标定中途触发摔倒保护把正在读的数抹掉。 */
float Speed_Pos_Raw=0.0f;

/* 转向环积分器。累加 (目标转速 - 实测转速)，Turn_Ki × Turn_Int 即积分项贡献的 PWM。
 * 放在函数外是为了让 Velocity_Reset() 一并清零，避免停车后带着"旧账"启动。 */
static float Turn_Int=0.0f;

/* 清空速度环滤波器与积分器。摔倒保护锁存期间调用，避免解除瞬间车猛地前窜。 */
static void Velocity_Reset(void)
{
	Err_LowOut_last=0;
	Speed_Pos=0;
	Turn_Int=0;
}

/* 速度环 PI
 * 输入目标速度与左右轮脉冲数，输出角度偏移量（度）。
 * 原理：车被推走 → 产生速度误差 → 输出角度偏移 → 直立环让车反向倾斜 → 车走回来。 */
float Velocity(int Target,int encoder_L,int encoder_R)
{
	static float a=VEL_LPF_A;
	int Err;
	float Err_LowOut,Vp,Vi,temp;
	/* 1. 速度误差 = 实际 - 目标 */
	Err=(encoder_L+encoder_R)-Target;
	/* 2. 低通滤波 */
	Err_LowOut=(1.0f-a)*(float)Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	/* 3. 累积积分项 */
	Speed_Pos+=Err_LowOut;
	Speed_Pos_Raw+=Err_LowOut;		/* 不受限幅的副本，供编码器标定 */
	/* 4. 限制积分项的作用范围（不是限制它本身的数值）。
	       Velocity_Ki 是负数，用"从限幅值反推 Speed_Pos"的写法就与 Ki 的正负
	       和大小都无关。 */
	if(Velocity_Ki!=0.0f)
	{
		Vi=Velocity_Ki*Speed_Pos;
		if(Vi>VEL_INT_LIMIT)
		{
			Vi=VEL_INT_LIMIT;
			Speed_Pos=VEL_INT_LIMIT/Velocity_Ki;
		}
		else if(Vi<(-VEL_INT_LIMIT))
		{
			Vi=(-VEL_INT_LIMIT);
			Speed_Pos=(-VEL_INT_LIMIT)/Velocity_Ki;
		}
	}
	else
	{
		Speed_Pos=0.0f;
		Vi=0.0f;
	}
	/* 5. 比例项（速度阻尼），独立限幅。
	       P 项和 I 项分开限幅比具体数值更重要：P 项就是刹车能力，
	       一旦被积分项挤占，车就会在最需要刹车时失去刹车能力。 */
	Vp=Velocity_Kp*Err_LowOut;
	if(Vp>VEL_P_LIMIT)Vp=VEL_P_LIMIT;
	else if(Vp<(-VEL_P_LIMIT))Vp=(-VEL_P_LIMIT);
	/* 6. 两项相加。最坏情况是两者限幅之和，只要明显小于 45 度保护线即可。 */
	temp=Vp+Vi;
	return temp;
}


/* 转向环 PI（闭环角速度）
 * 输入实测与目标转速，输出左右轮差速 PWM。把指令与实测相减，
 * 所以实际转向速度不会随电池电压和地面摩擦变化。 */
int Turn(float gyro_Z,int Target_turn)
{
	float err,Ti,out;

	err=(float)Target_turn-gyro_Z;	/* 指令与实测之差 */

	Turn_Int+=err;
	Ti=Turn_Ki*Turn_Int;
	/* 限制积分项作用范围（同样用"从限幅值反推"的写法，与 Ki 正负无关） */
	if(Ti>TURN_INT_LIMIT)
	{
		Ti=TURN_INT_LIMIT;
		Turn_Int=TURN_INT_LIMIT/Turn_Ki;
	}
	else if(Ti<(-TURN_INT_LIMIT))
	{
		Ti=(-TURN_INT_LIMIT);
		Turn_Int=(-TURN_INT_LIMIT)/Turn_Ki;
	}

	out=Turn_Kp*err+Ti;
	if(out> TURN_OUT_LIMIT) out= TURN_OUT_LIMIT;
	if(out<-TURN_OUT_LIMIT) out=-TURN_OUT_LIMIT;

	return (int)(TURN_SIGN*out);
}



/* ============================================================================
 * 主控制函数：每约 8.9 ms 被 MPU6050 中断调用一次
 * 顺序：测控制周期 → 读传感器 → 摔倒保护 → 遥控与避障 → 三个环 → 输出电机
 * ==========================================================================*/
void Control(void)	/* called every 10 ms */
{
	int PWM_out;
	Control_Tick++;		/* 串口发送的计时基准，见 main.c */

	/* 0. 实测控制周期（对 100 次取平均） */
	if(Control_dt_n==0)Control_dt_t0=HAL_GetTick();
	Control_dt_n++;
	if(Control_dt_n>=100)
	{
		uint32_t now=HAL_GetTick();
		Control_dt=(float)(now-Control_dt_t0)*0.001f/(float)Control_dt_n;
		Control_dt_n=0;
		if(Control_dt<0.001f)Control_dt=0.001f;		/* 防止后面除零 */
		Control_dt_ms=Control_dt*1000.0f;
	}
	/* 1. 读编码器和姿态。先读陀螺仪以保证角速度最新。
	      加速度计的读取被注释掉：当前算法用不到，跳过它能省下约三分之一花在
	      软件 I2C 上的时间，那部分时间也是控制延迟的一部分。 */
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
//	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	mpu_dmp_get_data(&pitch,&roll,&yaw);


	/* 2. 摔倒保护：倾角超限就锁存、切断电机并直接返回，不再计算任何输出 */
	if(Stop(&Med_Angle,&roll))
	{
		Velocity_Reset();
		Load(0,0);
		return;
	}

	/* 3. 重设位置基准。
	      Speed_Pos 是速度误差的积分，同时代表"已经走了多远"。目标速度为 0 时，
	      环路要回到积分上次归零的位置（即出发点）。这对"推一下自己走回来"是对的，
	      但对遥控不对：往前开一段后按停车应该是"停在这里"而不是"开回出发点"。
	      所以只要"是否在行驶"这个状态发生变化就把积分器清零，当前位置成为新基准。
	      没有遥控指令时前后都是 0，状态不变，原有行为不受影响。 */
	{
		static uint8_t was_driving=0;
		uint8_t driving=(uint8_t)(Fore||Back);
		if(driving!=was_driving)
			Velocity_Reset();		/* 基准 := 当前位置 */
		was_driving=driving;
	}

	/* 4. 超声波测距：定时发射并判断读数是否有效（原理见上面的说明） */
	{
		static uint32_t sonar_fire_tick=0;
		static uint16_t sonar_count_seen=0;
		static uint8_t  sonar_waiting=0;

		if(sonar_waiting)
		{
			if((Control_Tick - sonar_fire_tick) >= SONAR_WINDOW_TICKS)
			{
				distance_valid=(uint8_t)(count!=sonar_count_seen);
				sonar_waiting=0;
			}
		}
		else if((Control_Tick - sonar_fire_tick) >= SONAR_PERIOD_TICKS)
		{
			sonar_fire_tick=Control_Tick;
			sonar_count_seen=count;
			GET_Distance();
			sonar_waiting=1;
		}
	}

	/* 5. 遥控指令。目标速度的物理含义是 (左轮 + 右轮) 脉冲数的期望值。
	      实测：车前进时这个和是负的（手推 1.00 米，Speed_Pos_Raw 从 +382 到 -7815），
	      所以前进要减小目标速度，后退要增大。 */
	if((Fore==0)&&(Back==0))Target_Speed=0;	/* 无指令 → 原地保持 */
	if(Fore==1)
	{
		if(distance_valid && (distance<(float)OBSTACLE_CM))
		{
			/* 前方有障碍：清掉 Fore 一次解决三件事 —— 停止前进、位置基准重设到
			   当前位置（Fore 属于行驶状态，状态变化会在上面第 3 步触发）、
			   障碍物还在时后续前进指令一律被拒绝。后退不受限，车总能倒出来。 */
			Fore=0;
			Target_Speed=0;
		}
		else
		{
			Target_Speed--;					/* 前进：目标为负 */
		}
	}
	if(Back==1)Target_Speed++;				/* 后退：目标为正，永不拦截 */
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);	/* 限幅 */

	/* 6. 转向。目标转速与 gyroz 同单位，转向环才能相减。
	      指令渐变给出也渐变收回：收回要渐变是因为转向可在行进中随时施加，
	      直接跳到零相当于给差速一个阶跃，车会猛地偏一下。 */
	if(Left==1)           Target_turn+=TURN_RAMP;	/* 左转 */
	else if(Right==1)     Target_turn-=TURN_RAMP;	/* 右转 */
	else
	{
		if(Target_turn> TURN_RAMP)      Target_turn-=TURN_RAMP;	/* 逐渐回零 */
		else if(Target_turn<-TURN_RAMP) Target_turn+=TURN_RAMP;
		else                            Target_turn=0;
	}
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);	/* 限幅 */

	/* 7. 运行三个控制环。直立环的目标角 = 机械中心角 + 速度环偏移量。
	      转向环目标为 0 时误差就是 -gyroz，所以它本身就会抑制自转。 */
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);

	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);

	Turn_out=Turn(gyroz,Target_turn);

	/* 直立环输出叠加上转向差速：一边减速、一边加速 */
	PWM_out=Vertical_out;
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}
