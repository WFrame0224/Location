/**
 * @Description: 本程序实现了锚节节点(Anchor)的相关功能，以及数据处理
 * @FileName:   Anchor.c
 * @Author:     王非
 * @Date:       2018.09.06
 */
#include "Anchor.h"
#include "E2PROM.h"
#include "string.h"
#include "uart.h"
#include "2G4.h"
#include "timer.h"

// 存放中心站由433M信道传来的启动帧和电机控制帧
extern CommdInfo commdinfo = {{0x00}, 0, 0, false};

// 存放需求角度以及角度旋转的方向
extern DesAngle desangle = {'-', 0x0000};

// 记录当前的实际角度,不同于DesAngle,自带符号位
static int16_t CurrentAngle = 0x0000;

// 存放锚节点的标号
static uint8_t AnchorNumber = 0;

/*========================Functions================================*/

bool ReadAnchorNumber()
{
    uint16_t ADDR = 0x0000; // 锚节点信息存放的默认地址
    uint8_t Msg[2] = {0x00};

    IapReadFlash(ADDR, 2, Msg);
    if (Msg[0] != 'a') // 如果不是锚节点
    {
        return false;
    }

    AnchorNumber = Msg[1] - '0'; // 得到实际的标号
    return true;
}
uint8_t GetAnchorNumber()
{
    return AnchorNumber;
}

void getMsgAngle(uint8_t Msg)
{
    uint8_t Head[3] = {0x00, 0x00, 0x00};
    uint8_t Tail[3] = {0x00, 0x00, 0x00};

    uint8_t i;

    commdinfo.Commd[commdinfo.Commd_Index] = Msg;
    commdinfo.Commd_Index += 1;

    if (commdinfo.Commd_Index == 9)
    {
        commdinfo.Commd_Index = 0;
        for (i = 0; i < 3; i++)
        {
            Head[i] = commdinfo.Commd[i];
            Tail[i] = commdinfo.Commd[6 + i];
        }
        if ((!memcmp(Head, "ang", 3)) && (!memcmp(Tail, "ang", 3))) // 收到的是启动帧
        {
            commdinfo.Commd_In_Flag = true;
            commdinfo.Commd_Type = InitCommd;
        }
        else if ((!memcmp(Head, "des", 3)) && (!memcmp(Tail, "des", 3))) // 收到的是电机控制帧
        {
            commdinfo.Commd_In_Flag = true;
            commdinfo.Commd_Type = ControlCommd;
        }
        else
        {
            commdinfo.Commd_In_Flag = false;
            return;
        }
        desangle.F = commdinfo.Commd[3];                                 // 得到转动方向
        desangle.ANGLE = (commdinfo.Commd[4] << 8) | commdinfo.Commd[5]; // 得到转动的角度
    }
}

int16_t getCurrentAngle()
{
    return CurrentAngle;
}

void Send_GetRssiCommd(int16_t ActualAngle)
{
    uint8_t i = 0;
    // 读取控制帧                  帧头   标号  符号     角度     次数  帧尾
    uint8_t getrssicommd[9] = {'a', 'b', 0, 0x00, 0x00, 0x00, 0, 'a', 'b'};
    // 存入当前标号
    getrssicommd[2] = GetAnchorNumber();
    // 存入角度信息
    if (ActualAngle > 0)
    {
        getrssicommd[3] = '+';
        ActualAngle = ActualAngle;
    }
    else
    {
        getrssicommd[3] = '-';
        ActualAngle = -ActualAngle;
    }
    // 存放角度的绝对值
    getrssicommd[4] = (ActualAngle >> 8) & 0xff;
    getrssicommd[5] = ActualAngle & 0xff;

    for (i = 0; i < 10; i++) // 每个Anchor每次发送10个RSSI控制帧
    {
        getrssicommd[6] = i;
        Tx_Msg_2G4(getrssicommd, 9);
        // 每次延时50ms
        Hal_DelayXms(50);
    }
}

void InitRound(DesAngle desangle)
{
    /**
	 * 电机旋转 8度/s	 
	 */
    // 先根据锚节点标号，对应到实际的角度值
    switch (GetAnchorNumber())
    {
        case 0: // 锚节点0
            CurrentAngle += 180;
            break;

        case 1: // 锚节点1
            CurrentAngle += 270;
            break;

        case 2: // 锚节点2
            CurrentAngle += 0;
            break;

        case 3: // 锚节点3
            CurrentAngle += 90;
            break;

        default: // 如果不是锚节点
            return;
            break;
    }
    // 进行初始角度设置
    if (desangle.F == '+') // 逆时针旋转，向左转,度数增加
    {
        Round_left();
        Hal_DelayXms((uint16_t)(desangle.ANGLE / 0.008));
        Round_stop();

        CurrentAngle += desangle.ANGLE;
    }
    else if (desangle.F == '-') // 顺时针旋转，向右转，度数减小,最后不要采用顺时针
    {
        Round_right();
        Hal_DelayXms((uint16_t)(desangle.ANGLE / 0.008));
        Round_stop();
        if (CurrentAngle == 0)
        {
            CurrentAngle = 360;
        }
        CurrentAngle -= desangle.ANGLE;
    }
    else
    {
        return;
    }
}

void continueRound(DesAngle desangle)
{

    if (desangle.F == '+') // 逆时针旋转，向左转,度数增加
    {
        while (CurrentAngle <= desangle.ANGLE)
        {
            Round_left();
            Hal_DelayXms((uint16_t)(6 / 0.008)); // 以6度的分辨率进行
            Round_stop();
            CurrentAngle += 6;
            switch (GetAnchorNumber())
            {
                case 0:
                    break;
                case 1:
                    Hal_DelayXms(1 * 700);
                    break;
                case 2:
                    Hal_DelayXms(2 * 700);
                    break;
                case 3:
                    Hal_DelayXms(3 * 700);
                    break;
                default:
                    break;
            }
            Send_GetRssiCommd(CurrentAngle);
        }
    }
    else if (desangle.F == '-') // 顺时针旋转，向右转，度数减小
    {
        while (CurrentAngle >= desangle.ANGLE)
        {
            Round_right();
            Hal_DelayXms((uint16_t)(6 / 0.008)); // 以6度的分辨率进行
            Round_stop();
            CurrentAngle -= 6;
            switch (GetAnchorNumber())
            {
                case 0:
                    break;
                case 1:
                    Hal_DelayXms(1 * 700);
                    break;
                case 2:
                    Hal_DelayXms(2 * 700);
                    break;
                case 3:
                    Hal_DelayXms(3 * 700);
                    break;
                default:
                    break;
            }
            Send_GetRssiCommd(CurrentAngle);
        }
    }
    else
    {
        return;
    }
}

void Round_left()
{

    uint8_t round_left_commd[7] = {0xff, 0x01, 0x00, 0x04, 0x01, 0x00, 0x06};
    // 发送命令
    SendArrayHex(3, round_left_commd, 7);
}

void Round_right()
{
    uint8_t round_right_commd[7] = {0xff, 0x01, 0x00, 0x02, 0x01, 0x00, 0x04};
    // 发送命令
    SendArrayHex(3, round_right_commd, 7);
}

void Round_stop()
{
    uint8_t round_stop_commd[7] = {0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
    // 发送命令
    SendArrayHex(3, round_stop_commd, 7);
}