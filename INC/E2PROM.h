#ifndef __E2PROM_H__
#define __E2PROM_H__

#include "hal.h"
#include "datatype.h"

#define CMD_IDLE        0   //空闲模式
#define CMD_READ        1   //IAP字节读取命令
#define CMD_PROGRAM     2   //IAP字节编程命令
#define CMD_ERASE       3   //IAP扇区擦除命令

#define WT_30M      0x80    //不同时钟频率对应的IAP使能命令
#define WT_24M      0x81
#define WT_20M      0x82
#define WT_12M      0x83
#define WT_6M       0x84
#define WT_3M       0x85
#define WT_2M       0x86
#define WT_1M       0x87

#define IAP_ADDRESS 0x0400

//--------------------------------------------------
void IapIdle();
uint8_t IapReadByte(uint16_t addr);
void IapProgramByte(uint16_t addr, uint8_t dat);
void IapEraseSector(uint16_t addr);
int IapProgramFlash(uint16_t addrOrigin,uint8_t* Msg);
void IapReadFlash(uint16_t addrOrigin,int Length,uint8_t* ReadMsg);

#endif