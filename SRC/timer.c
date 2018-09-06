#include "timer.h"

extern uint xdata TimeIndex = 0;

void InitTimer0()
{
    AUXR |= 0x80;                         //定时器时钟1T模式
    TMOD &= 0xF0;                         //设置定时器
    TL0 = (65536 - FOSC / Timer0Hz);      //设置定时初值
    TH0 = (65536 - FOSC / Timer0Hz) >> 8; //设置定时初值
    TF0 = 0;                              //清楚TF0标志
    TR0 = 1;                              //定时器0开始计时
    ET0 = 1;
    EA = 1;
}

void Timer0Isr() interrupt 1
{
    TimeIndex = TimeIndex + 1;
}