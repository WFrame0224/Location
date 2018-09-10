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
    InitCommd = 0,          // 属于启动帧命令
    ControlCommd = 1        // 属于电机控制帧
}Commd_Type;

// 定义命令接收缓存区的类型
typedef struct
{
    uint8_t Commd[9];       // 命令的缓存存储区
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


/*==============================Functions============================*/
/*!
 * Function：从eeprom中获取锚节点的标号，默认地址是0x0000
 * Parameter:
 *      AnchorNumber：需要保存的标号指针
 * Return:
 *      false: 不是锚节点
 *      true:  是锚节点    
 */ 
bool getAnchorNumber(char *AnchorNumber);
/*!
 * Function:接收433Mh信道经过无线串口发来的消息，解析出实际的电机启动、控制帧
 * Parameter:
 *      commdinfo:保存
 */ 
void getAngle(uint8_t Msg);

/*!
 * Function：实现电机的初始角度置位
 * Parameter：
 *      desangle：需要置的初始角度总和
 * 注：电机的控制采用 串口4 发送控制命令
 */ 
void InitRound(DesAngle desangle);

/*!
 * Function: 实现电机的持续调节，默认以6度的分辨率进行发送，且需要延时相应的时间，（具体时间根据锚节点标号进行延时）
 * ，每6度需要利用2.4G信道发送RSSI读取控制帧到待定位节点，待定位节点，且每个待定位节点发送10次读取控制帧
 *  Parameter：
 *      desangle：需要置的初始角度总和
 */ 
void continueRound(DesAngle desangle);

/*!
 * Function:发送RSSI读取控制帧指令，利用2.4G信道进行发送
 * Parameter:
 *     AngleDir：每一条RSSI读取指令对应的角度值
 */ 
void Send_GetRssiCommd(DesAngle AngleDir);

/**
 * Function：控制电机开始左转
 */ 
void Round_left();

/**
 * Function: 控制电机开始右转
 */ 
void Round_right();

/**
 * Function: 使电机停止旋转 
 */ 
void Round_stop();

#endif