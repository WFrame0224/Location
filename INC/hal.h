#ifndef __HAL_H__
#define __HAL_H__

#include <STC8.H>
#include <intrins.h>

/* 定义单片机工作频率 */
#define FOSC 24000000L
//#define FOSC 11059200L

/* 定义外围设备的特性 */

//定义串口波特率
#define BAUD1 115200
#define BAUD2 115200
#define BAUD3 19200
#define BAUD4 115200

//定义定时器周期频率
#define Timer0Hz 1000 //每1ms进入一次定时器中断

/* 引脚定义 */


#endif