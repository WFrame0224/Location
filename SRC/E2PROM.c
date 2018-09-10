#include "E2PROM.h"
#include "function.h"
#include "string.h"

/*
 * E2PROM 空闲模式，关闭各项功能
 */
void IapIdle()
{
    IAP_CONTR = 0;          //关闭 IAP 功能
    IAP_CMD = CMD_IDLE;     //清除命令寄存器
    IAP_TRIG = 0;           //清除触发寄存器
    IAP_ADDRH = 0x80;       //将地址设置到非 IAP 区域
    IAP_ADDRL = 0;
}
/*
 * E2PROM 读字符
 * uint16_t addrOrigin:起始地址
 * 返回读到的字符
 */
uint8_t IapReadByte(uint16_t addr)
{
    uint8_t dat;
	
    IAP_CONTR = WT_24M;    //使能 IAP
    IAP_CMD = CMD_READ;    //设置 IAP 读命令
    IAP_ADDRL = addr;      //设置 IAP 低地址
    IAP_ADDRH = addr >> 8; //设置 IAP 高地址
    IAP_TRIG = 0x5a;       //写触发命令(0x5a)
    IAP_TRIG = 0xa5;       //写触发命令(0xa5)
    _nop_();
    dat = IAP_DATA;        //读 IAP 数据
    IapIdle();             //关闭 IAP 功能
    return dat;
}
/*
 * E2PROM 读字符串
 * uint16_t addrOrigin:起始地址
 * uint8_t* ReadMsg：读到的数据
 */
void IapReadFlash(uint16_t addrOrigin,int Length,uint8_t* ReadMsg)
{
    int i;
	
    for(i = 0;i < Length;i++)
    {
        *ReadMsg++ = IapReadByte(addrOrigin + i);
        Delay_Xus(7); 
    }
	ReadMsg = ReadMsg - Length;
}
/*
 * E2PROM 写字符串
 * uint16_t addr：起始地址
 * uint8_t dat：要写入的字符
 */
void IapProgramByte(uint16_t addr, uint8_t dat)
{
    IAP_CONTR = WT_24M;    //使能 IAP
    IAP_CMD = CMD_PROGRAM; //设置 IAP 写命令
    IAP_ADDRL = addr;      //设置 IAP 低地址
    IAP_ADDRH = addr >> 8; //设置 IAP 高地址
    IAP_DATA = dat;        //写 IAP 数据
    IAP_TRIG = 0x5a;       //写触发命令(0x5a)
    IAP_TRIG = 0xa5;       //写触发命令(0xa5)
    _nop_();
    IapIdle(); //关闭 IAP 功能
}
/*
 * E2PROM 写字符串
 * uint16_t addrOrigin:起始地址
 * uint8_t* Msg：需要写入的数据
 * 返回：长度写入的字符串长度
 */
int IapProgramFlash(uint16_t addrOrigin,uint8_t* Msg)
{
    int Length,i;
    Length = strlen(Msg);
    
    for(i = 0;i < Length;i ++)
    {
        IapProgramByte(addrOrigin +i ,*Msg ++);
        Delay_Xus(7); 
    }
    return Length;
}
/*
 *  E2PROM 擦除
 */
void IapEraseSector(uint16_t addr)
{
    IAP_CONTR = WT_24M;    //使能 IAP
    IAP_CMD = CMD_ERASE;   //设置 IAP 擦除命令
    IAP_ADDRL = addr;      //设置 IAP 低地址
    IAP_ADDRH = addr >> 8; //设置 IAP 高地址
    IAP_TRIG = 0x5a;       //写触发命令(0x5a)
    IAP_TRIG = 0xa5;       //写触发命令(0xa5)
    _nop_();               //
    IapIdle();             //关闭 IAP 功能
}