#include "function.h"

/**清除数组中指定区域，
 * Array：待清除的数组
 * Ch：清除区域准备填充的数据
 * Head：待清除的起始地址
 * Length：待清除的数据长度
 */
 /*
void ArrayReset(unsigned char *Array, unsigned char Ch, char Head, char Length)
{
	char i = 0;
	for (i = Head; i <= Length; i++)
	{
		Array[i] = Ch;
	}
}
*/
/**
 * 单片机进行软件复位
 */
void REST_MCU()
{
	IAP_CONTR |= 0x60;      // 进行软件复位
}
#ifdef SOFT_DELAY	
/**定义了部分常用的软件延时函数
 * Delay_1ms():定义了1ms的延时
 * Delay_100ms():定义了100ms的延时
 * Delay_Xms():定义了Xms的延时，使用时最好小于1000，即秒级延时最好不要使用
 */
void Delay_1ms() //@24.000MHz
{
	unsigned char i, j;

	_nop_();
	i = 32;
	j = 40;
	do
	{
		while (--j)
			;
	} while (--i);
}
void Delay_100ms() //@24.000MHz
{
	unsigned char i, j, k;

	_nop_();
	_nop_();
	i = 13;
	j = 45;
	k = 214;
	do
	{
		do
		{
			while (--k)
				;
		} while (--j);
	} while (--i);
}
/**
 * 定义了Xms的延时
 */ 
void Delay_Xms(unsigned int Xms) //@24.000MHz
{
	unsigned int i;
	if (Xms < 100)
	{
		for (i = 0; i <= Xms; i++)
		{
			Delay_1ms();
		}
	}
	else
	{
		for (i = 1; i <= Xms / 100;i ++)
		{
			Delay_100ms();
		}
	}
}
/**
 * 定义了10us的延时
 */
void Delay_10us()		//@24.000MHz
{
	unsigned char i;

	i = 78;
	while (--i);
}
/**
 * 定义了大于10us，小于1000us的延时
 */ 
void Delay_Xus(unsigned int Xus)
{
	unsigned char i;
	for (i = 1; i <= Xus/10;i ++)
	{
		Delay_10us();
	}
}
#endif