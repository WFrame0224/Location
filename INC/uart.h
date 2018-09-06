#ifndef __UART_H__
#define __UART_H__

#include "datatype.h"
#include "hal.h"

void SendData(unsigned char Uart_Port, unsigned char dat);
void SendString(unsigned char Uart_Port, unsigned char *s);
void SendArrayHex(unsigned char Uart_Port,char *s,uint n);
void SendArrayStr(unsigned char Uart_Port, char *p,uint n);
void send_16_2_str(unsigned char Uart_Port,uchar temp);
void SendHex2Ascills(unsigned char Uart_Port, unsigned char *p, uint n);
void Init_Uart();
void Init_UART1();
void Init_UART2();
void Init_UART3();
void Init_UART4();

#endif