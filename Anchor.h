/**
 * @Description: 本头文件是锚节点.c文件的函数以及定义声明
 * @Filename:  Anchor.h
 * @Author：王非
 * @Date：  2018.09.06
 */
#ifndef __ANCHOR_H__
#define __ANCHOR_H__

#include "datatype.h"

// 定义锚节点接收到433Mhz控制命令的格式
typedef enum
{
    NoneCommd = 0,			// 属于无效的命令
	InitCommd = 1,          // 属于启动帧命令
    ControlCommd = 2,       // 属于电机控制帧命令
	REST = 3				// 属于复位命令
}Commd_Type;

// 定义锚节点角色
typedef enum
{
	Anchor_1 = 1,
	Anchor_2 = 2,
	Anchor_3 = 3,
	Anchor_4 = 4,
	Anchor_None = 99
}Anchor_Number;

// 定义命令接收缓存区的类型
typedef struct
{
    uint8_t Commd[3];       // 命令的缓存存储区
    uint8_t Commd_Index;    // 缓存区地址指针
    Commd_Type Commd_Type;  // 命令类型
    bool Commd_In_Flag;
}CommdInfo;

// 解析出的角度类型
typedef struct
{
    uint8_t F;
    uint16_t ANGLE;    
}DesAngle;

// 数据包的状态
typedef enum
{
	NONE = 0,
	HEAD1 = 1,
	HEAD2 = 2,
	MSG = 3,
	TAIL1 = 4,
	TAIL2 = 5
}State;


/*==============================Functions============================*/
/*!
 * Function：从eeprom中获取锚节点的标号，默认地址是0x0000
 * Return:
 *      false: 不是锚节点
 *      true:  是锚节点    
 */ 
bool ReadAnchorNumber();

/*!
 * Function：返回锚节点标号
 * Return:
 *      锚节点标号   
 */ 
uint8_t GetAnchorNumber();

/*!
 * Function:接收433Mh信道经过无线串口发来的消息，解析出实际的电机启动、控制帧
 * Parameter:
 *      commdinfo:保存
 */ 
void getMsgAngle(uint8_t Msg);
void getMsgAngle1(uint8_t Msg);
/**
 * Function: 获取当前记录的角度
 * return：
 *      返回当前的角度
 */ 
int16_t getCurrentAngle();

/*!
 * Function：实现电机的初始角度置位
 * 注：电机的控制采用 串口3 发送控制命令
 */ 
void InitRound();

/*!
 * Function: 实现电机的持续调节，默认以6度的分辨率进行发送，且需要延时相应的时间，（具体时间根据锚节点标号进行延时）
 * ，每6度需要利用2.4G信道发送RSSI读取控制帧到待定位节点，待定位节点，且每个待定位节点发送10次读取控制帧
 */ 
void continueRound();

/*!
 * Function:发送RSSI读取控制帧指令，利用2.4G信道进行发送
 * Parameter:
 *     AngleDir：每一条RSSI读取指令对应的角度值
 */ 
void Send_GetRssiCommd(int16_t ActualAngle);

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

/**
 * Function：锚节点运行主程序，实现锚节点的所有流程：
 *      获取433中心站消息 --> 解析识别 --> 控制电机旋转 -->  发送RSSI读取命令给待测节点 --> 发送中心站over标志
 */ 
void Anchor_run();
#endif