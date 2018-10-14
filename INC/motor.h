/**
 * @Description：本程序定义了电机的相关驱动程序的函数声明
 * @FileName：motor.h
 * @Author：王非
 * @Date:2018.10.08
 */ 

#ifndef __MOTOR_H__
#define __MOTOR_H__

/* includes---------------------------------- */
#include "datatype.h"
#include "hal.h"

/* Pin defines------------------------------- */
// 采用原串口3的引脚
sbit Plus_pin = P0 ^ 0;			// 原标TXD3引脚
sbit Dir_pin = P0 ^ 1;			// 原标RXD3引脚

/* Relative defines-------------------------------------- */
#define PLUS_FREQ 1200					// 脉冲的频率
#define PLUS_VALUE (FOSC/12/PLUS_FREQ)	// PCA定时器比较器的值

/**
 * Function:控制电机向左转desAngle度
 * @param:
 *      desAngle:目标度数
 */ 
void RoundLeft2Angle(uint16_t desAngle);

/**
 * Function:控制电机向右转desAngle度
 * @param:
 *      desAngle:目标度数
 */ 
void RoundRight2Angle(uint16_t desAngle);

#ifdef AC_MOTOR	
/**
 * Function：控制电机开始左转，逆时针转动，度数增加
 */ 
void Round_left();

/**
 * Function: 控制电机开始右转，顺时针转动，度数减小
 */ 
void Round_right();

/**
 * Function: 使电机停止旋转 
 */ 
void Round_stop();

#else

/**
 * Function:PCA定时器初始化，初始化PCA硬件相关，以及电机相关的计数器和引脚
 */ 
void PCA_init(void);

/**
 * Function：脉冲产生引脚，产生固定600Hz的脉冲
 */ 
void PCA_Create_Plus(void);

/**
 * Function:脉冲停止，关闭PCA定时器
 */ 
void PCA_Stop_Plus(void);

/**
 * Function:电机转动X角度
 * @param:
 *      des:电机转动的目的角度
 */ 
void Moto_RoundXangel(uint16_t des);

#endif

#endif