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

#endif

void RoundLeft2Angle(uint16_t desAngle)
{
#ifdef AC_MOTOR
    Round_left();
    Hal_DelayXms((uint16_t)(desAngle / 0.0078));
    Round_stop();
#else
#endif
}

void RoundRight2Angle(uint16_t desAngle)
{
#ifdef AC_MOTOR	
    Round_right();
    Hal_DelayXms((uint16_t)(desAngle / 0.0078));
    Round_stop();
#else
#endif
}

void Round_left()
{
#ifdef AC_MOTOR	
	// 发送命令
    SendArrayHex(3, round_left_commd, 7);
#else
#endif
}

void Round_right()
{
#ifdef AC_MOTOR   
	// 发送命令
    SendArrayHex(3, round_right_commd, 7);
#else
#endif
}

void Round_stop()
{
#ifdef AC_MOTOR	  
	// 发送命令
    SendArrayHex(3, round_stop_commd, 7);
#else
#endif
}