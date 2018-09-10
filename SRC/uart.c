/**
 * @FileName:uart.c
 * @Description:
 *      本程序实现了四个串口的通信，其中根据项目PCB设定：
 *      串口1: 2.4G模块使用，作为RSSI信号采集端
 *      串口2: 433MHz模块使用，作为RSSI信号的数据收集端
 *      串口3: 预留出陀螺仪等导航器件
 */ 
#include "uart.h"
#include "Anchor.h"

bit idata busy1, busy2, busy3, busy4; //串口发送标志
const uchar idata _16_2_str[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/************************************************/

/**串口发送字符
 * dat：待发送的字符
 * Uart_Port：需要使用的串口号
 */
void SendData(unsigned char Uart_Port, unsigned char dat)
{
    switch (Uart_Port)
    {
    case 1:
        while (busy1)
            ;
        busy1 = 1;
        SBUF = dat;
        break;
    case 2:
        while (busy2)
            ;
        busy2 = 1;
        S2BUF = dat;
        break;
    case 3:
        while (busy3)
            ;
        busy3 = 1;
        S3BUF = dat;
        break;
    case 4:
        while (busy4)
            ;
        busy4 = 1;
        S4BUF = dat;
        break;
    default:
        break;
    }
}

/**串口发送字符串
 * Uart_Port：需要使用的串口号
 * s：待发送的字符串
 */
void SendString(unsigned char Uart_Port, unsigned char *s)
{
    while (*s)
    {
        SendData(Uart_Port, *s++);
    }
}

void SendArrayHex(unsigned char Uart_Port, char *s, uint n)
{
    while (n--)
    {
        SendData(Uart_Port, *s++);
    }
}
void SendArrayStr(unsigned char Uart_Port, char *p, uint n)
{
    while(n--)
    {
        send_16_2_str(Uart_Port, *p++);
    }
}
void send_16_2_str(unsigned char Uart_Port, uchar temp)
{
    uchar str[2] = {'0', '0'};
    uchar low;
    uchar high;
    low = temp & 0x0f;
    high = (temp & 0xf0) >> 4;
    str[0] = _16_2_str[high];
    str[1] = _16_2_str[low];
    SendData(Uart_Port, str[0]);
    SendData(Uart_Port, str[1]);
}
void SendHex2Ascills(unsigned char Uart_Port, unsigned char *p, uint n)
{
	int i = 0;
	for(i = 0; i < n; i++)
    {
        send_16_2_str(Uart_Port, *p++);
    }
}
/********************Initialization********************************/
void Init_Uart()
{
    Init_UART1();
    Init_UART2();
    Init_UART3();
    Init_UART4();

    EA = 1;
}
void Init_UART1()
{
//    P_SW1 = 0x40; // 将串口1切换至P3.6,P3.7

    SCON = 0x50;
    TL1 = (65536 - (FOSC / 4 / BAUD1));
    TH1 = (65536 - (FOSC / 4 / BAUD1)) >> 8;
    AUXR |= 0X40; //
//    AUXR = 0x40;  //定时器1为1T模式
    TMOD = 0x00;  //定时器1为模式0(16位自动重载)
    TR1 = 1;
    ES = 1; //使能串口1中断

    busy1 = 0;
}

void Init_UART2()
{
    S2CON = 0x50;
    T2L = (65536 - (FOSC / 4 / BAUD2));
    T2H = (65536 - (FOSC / 4 / BAUD2)) >> 8;
    AUXR |= 0X14; //T2为1T模式，并启动定时器2
    IE2 |= 0x01;  //使能串口2中断
    busy2 = 0;
}

void Init_UART3()
{
    S3CON = 0x50;
    T3L = (65536 - (FOSC / 4 / BAUD3));
    T3H = (65536 - (FOSC / 4 / BAUD3)) >> 8;
    T4T3M |= 0x0a;
    IE2 |= 0x08; //使能串口3中断
    busy3 = 0;
}

void Init_UART4()
{
    S4CON = 0x50;
    T4L = (65536 - (FOSC / 4 / BAUD4));
    T4H = (65536 - (FOSC / 4 / BAUD4)) >> 8;
    T4T3M |= 0xa0;
    IE2 |= 0x10; //使能串口4中断
    busy4 = 0;
}

/**************UART_Interrupt_Fuctions****************************/
void UART1_Isr() interrupt 4 using 1
{
    uint8_t idata dat = 0;
	if (TI)
    {
        TI = 0;    //清中断标志
        busy1 = 0; //清忙标志
    }
    if (RI)
    {
        RI = 0;
		dat = SBUF;
		S2BUF = dat;
    }
}

void UART2_Isr() interrupt 8
{
    uint8_t idata dat = 0;
	
	if (S2CON & 0x02)
    {
        S2CON &= ~0x02; //清中断标志
        busy2 = 0;      //清忙标志
    }
    if (S2CON & 0x01)
    {
        S2CON &= ~0x01; //清中断标志
		
		dat = S2BUF;
		SBUF = dat;
    }
}

void Uart3Isr() interrupt 17
{
    if (S3CON & 0x02)
    {
        S3CON &= ~0x02;
        busy3 = 0;
    }
    if (S3CON & 0x01)
    {
        S3CON &= ~0x01;
    }
}

void UART4_Isr() interrupt 18
{
    uchar idata dat = 0;
    if (S4CON & 0x02)
    {
        S4CON &= ~0x02; //清中断标志
        busy4 = 0;      //清忙标志
    }
    if (S4CON & 0x01)
    {
        S4CON &= ~0x01; //清中断标志

        dat = S4BUF; 
		getAngle(dat);
    }
}