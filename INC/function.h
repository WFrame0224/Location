#ifndef __FUNCTION_H__
#define __FUNCTION_H__

#include "main.h"
void REST_MCU();
void ArrayReset(unsigned char *Array,unsigned char Ch,char Head,char Length);
void Delay_1ms();
void Delay_100ms();
void Delay_Xms(unsigned int Xms);
void Delay_10us();
void Delay_Xus(unsigned int Xus);
#endif