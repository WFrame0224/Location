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

/**
 * Function：控制电机开始左转，顺时针转动，度数增加
 */ 
void Round_left();

/**
 * Function: 控制电机开始右转，逆时针转动，度数减小
 */ 
void Round_right();

/**
 * Function: 使电机停止旋转 
 */ 
void Round_stop();

#endif