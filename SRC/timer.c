#include "timer.h"

// 总的计时指针，只是在 time.c 文件中起作用
static uint idata TimeIndex = 0;

// 用于保证命令的连续性，和正确性
extern uint idata Msg_TimeIndex = 0;
// 消息的计数标志位
extern uint8_t idata Msg_Index = 0;

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

void Hal_DelayXms(uint16_t Xms)
{
    TimeIndex = 0;

	while (1)
    {
        if(TimeIndex == Xms)
		{
			break;
		}
    }

}

void Timer0Isr() interrupt 1
{
    TimeIndex = TimeIndex + 1;
	
	Msg_TimeIndex = Msg_TimeIndex + 1;
	
	if(Msg_TimeIndex >= 10)// 命令消息超时，消息计时器清零，消息指针清零
	{
		Msg_TimeIndex = 0;
		Msg_Index = 0;
	}
}
