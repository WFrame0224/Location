#ifndef __TIMER_H__
#define __TIMER_H__

#include "main.h"

/**
 * Function: 定时器的初始化，使用定时器0
 */ 
void  InitTimer0();

/**
 * Function:硬件定时 X ms
 */ 
void Hal_DelayXms(uint16_t Xms);

#endif