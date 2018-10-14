/**
 * @Description：本程序定义了电机的相关驱动程序的函数，方便后面的扩展
 * @FileName：motor.c
 * @Author：王非
 * @Date:2018.10.08
 */
#include "motor.h"
#include "timer.h"
#include "uart.h"

#ifdef AC_MOTOR

// 逆时针旋转命令
uint8_t const round_left_commd[7] = {0xff, 0x01, 0x00, 0x04, 0x01, 0x00, 0x06};
// 顺时针旋转命令
uint8_t const round_right_commd[7] = {0xff, 0x01, 0x00, 0x02, 0x01, 0x00, 0x04};
// 停止旋转命令
uint8_t const round_stop_commd[7] = {0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};

#else

// 作为脉冲计数的标志
uint32_t PLUS_CNT = 1;

#endif

void RoundLeft2Angle(uint16_t desAngle)
{
#ifdef AC_MOTOR
    Round_left();
    Hal_DelayXms((uint16_t)(desAngle / 0.0078));
    Round_stop();
#else
	Dir_pin = 1;	// 方向为正转
	Moto_RoundXangel(desAngle);	
#endif
}

void RoundRight2Angle(uint16_t desAngle)
{
#ifdef AC_MOTOR	
    Round_right();
    Hal_DelayXms((uint16_t)(desAngle / 0.0078));
    Round_stop();
#else
	Dir_pin = 0;	// 方向为反转
	Moto_RoundXangel(desAngle);
	Dir_pin = 1;	// 方向为回归至初始
#endif
}
#ifdef AC_MOTOR	
void Round_left()
{
	// 发送命令
    SendArrayHex(3, round_left_commd, 7);
}

void Round_right()
{
	// 发送命令
    SendArrayHex(3, round_right_commd, 7);
}

void Round_stop()
{
	// 发送命令
    SendArrayHex(3, round_stop_commd, 7);

}
#else // 采用步进电机，定义步进电机相关的函数
/*--------------------PCA底层控制的驱动函数--------------------------*/
void PCA_init(void)
{
	PLUS_CNT = 0;			// 脉冲计数器初始化
	Dir_pin = 0;			// 方向引脚初始化，默认正转
	Plus_pin = 0;			// Plus引脚初始化
	
	CCON = 0x00;
    CMOD = 0x00;            //PCA时钟为1/12系统时钟，静止PCA定时器溢出中断
    CL = 0x00;				// 清楚计数器的值
    CH = 0x00;
    CCAPM0 = 0x49;          //PCA模块0为16位定时器模式
//	EA = 1;
}

void PCA_Create_Plus(void)
{
	uint16_t value = PLUS_VALUE;
	CCAP0L = value;			// 更新比较器的值
    CCAP0H = value >> 8;
	CR = 1;					// PCA定时器开始工作	
}
void PCA_Stop_Plus(void)
{
	CR = 0;					// PCA定时器停止工作
	CL = 0x00;				// 清楚计数器的值
    CH = 0x00;
}
void Moto_RoundXangel(uint16_t desAngle)
{
	uint8_t angleTime = (desAngle / 3);
	PLUS_CNT = 0;
	
	PCA_Create_Plus();
	
	while(1)
	{
		if(PLUS_CNT == (angleTime * 400))
		{
			PLUS_CNT = 1;
			PCA_Stop_Plus();
			break;
		}
	}
}
#endif
/* --------------------------PCA中断处理函数-------------------- */
void PCA_Isr() interrupt 7 using 1
{	
	uint16_t value = PLUS_VALUE;
	
	CCF0 = 0;				// 清中断标志
	CL = 0x00;				// 清楚计数器的值
    CH = 0x00;
    CCAP0L = value;			// 更新比较器的值
    CCAP0H = value >> 8;
	
    Plus_pin = !Plus_pin;                                 //测试端口
	
	PLUS_CNT ++;
}
