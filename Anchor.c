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

// 逆时针旋转命令
uint8_t const round_left_commd[7] = {0xff, 0x01, 0x00, 0x04, 0x01, 0x00, 0x06};
// 顺时针旋转命令
uint8_t const round_right_commd[7] = {0xff, 0x01, 0x00, 0x02, 0x01, 0x00, 0x04};
// 停止旋转命令
uint8_t const round_stop_commd[7] = {0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};

// RSSI读取命令结束
const uint8_t RSSI_OVER[8] = {'R', 'S', 'S', 'I', 'O', 'V', 'E', 'R'};

// 存放需求角度以及角度旋转的方向
DesAngle desangle = {'+', 0x0000};

extern uint16_t idata CurrentAngle;

// 存放锚节点的标号
Anchor_Number Anchor_Num = Anchor_None;

// 初始化设置计数器
uint8_t Init_time = 0;

/*========================Functions================================*/

void Anchor_run()
{
    while (1)
    {
        if (commdinfo.Commd_In_Flag == true)
        {
            commdinfo.Commd_In_Flag = false;
            switch (commdinfo.Commd_Type)
            {
				case InitCommd: // 如果收到的命令是启动帧控制

					if (Init_time == 0)
					{
						Init_time = Init_time + 1; // 为保证角度设置只是设置一次

						// 先根据锚节点标号，对应到实际的角度值
						switch (GetAnchorNumber())
						{
							case Anchor_1: // 锚节点1
								CurrentAngle = CurrentAngle + 180;
								break;

							case Anchor_2: // 锚节点2
								CurrentAngle = CurrentAngle + 270;
								break;

							case Anchor_3: // 锚节点3
								CurrentAngle = CurrentAngle + 0;
								break;

							case Anchor_4: // 锚节点4
								CurrentAngle = CurrentAngle + 90;
								break;

							default: // 如果不是锚节点
								return;
								break;
						}
					}

					// 控制电机转动至初始位置
					InitRound();

					break;

				case ControlCommd: // 如果收到的命令是电机控制帧命令
					
					// 控制电机逐渐转动，直到转至需求角度
					continueRound();
				
					// 电机全部转动完毕之后，四号锚节点给433中心站发送 RSSIOVER 命令
					switch (GetAnchorNumber())
					{
						case Anchor_4:
						// RSSI读取命令发送完毕
						SendArrayHex(4, RSSI_OVER, 8);
						break;
					}

					break;

				case NoneCommd:

				default:
					break;
            }
        }
    }
}

bool ReadAnchorNumber()
{
    uint16_t ADDR = 0x0000; // 锚节点信息存放的默认地址
    uint8_t Msg[2] = {0x00};

    IapReadFlash(ADDR, 2, Msg);
    if (Msg[0] != 'a') // 如果不是锚节点
    {
        return false;
    }

    Anchor_Num = Msg[1] - '0'; // 得到实际的标号
    return true;
}

uint8_t GetAnchorNumber()
{
    return Anchor_Num;
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
            commdinfo.Commd_Type = NoneCommd;
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
    // 读取控制帧                  帧头 标号  符号    角度   次数  帧尾
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

void InitRound()
{
#ifdef UART_1

    uint8_t angle[2] = {0x00};

#endif
    /**
	 * 电机旋转 8度/s	 
	 */
    // 进行初始角度设置
    if (desangle.F == '+') // 逆时针旋转，向左转,度数增加
    {
        Round_left();
        Hal_DelayXms((uint16_t)(desangle.ANGLE / 0.008));
        Round_stop();

        CurrentAngle = CurrentAngle + desangle.ANGLE;
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
        CurrentAngle = CurrentAngle - desangle.ANGLE;
    }
    else
    {
        return;
    }
	
#ifdef UART_1

    angle[0] = (CurrentAngle >> 8) & 0xff;
    angle[1] = CurrentAngle;
    SendString(1, "The CurrentAngle is:");
    SendHex2Ascills(1, angle, 2);

#endif
}

void continueRound()
{

#ifdef UART_1

    uint8_t angle[2] = {0x00};

#endif

    if (desangle.F == '+') // 逆时针旋转，向左转,度数增加
    {
        desangle.ANGLE += (CurrentAngle);
        while (CurrentAngle <= desangle.ANGLE)
        {       
			// 延时不同的时间进行RSSI读取控制命令帧的发送
            switch (GetAnchorNumber())
            {
                case Anchor_1:
                    break;
                case Anchor_2:
                    Hal_DelayXms((Anchor_2 - 1) * 700);
                    break;
                case Anchor_3:
                    Hal_DelayXms((Anchor_3 - 1) * 700);
                    break;
                case Anchor_4:
                    Hal_DelayXms((Anchor_4 -1 ) * 700);
                    break;
                default:
                    break;
            }

            Send_GetRssiCommd(CurrentAngle);

            switch (GetAnchorNumber())
            {
                case Anchor_1:
                    Hal_DelayXms((4 - Anchor_1) * 700);
                    break;
                case Anchor_2:
                    Hal_DelayXms((4 - Anchor_2) * 700);
                    break;
                case Anchor_3:
                    Hal_DelayXms((4 - Anchor_3) * 700);
                    break;
                case Anchor_4:
                    break;
                default:
                    break;
            }
			Round_left();
            Hal_DelayXms((uint16_t)(6 / 0.008)); // 以6度的分辨率进行
            Round_stop();
            CurrentAngle = CurrentAngle + 6;

#ifdef UART_1

            angle[0] = (CurrentAngle >> 8) & 0xff;
            angle[1] = CurrentAngle & 0xff;
            SendString(1, "The CurrentAngle is:");
            SendHex2Ascills(1, angle, 2);

#endif
        }
    }
    else if (desangle.F == '-') // 顺时针旋转，向右转，度数减小
    {
        desangle.ANGLE = (CurrentAngle)-desangle.ANGLE;
        while (CurrentAngle >= desangle.ANGLE)
        {
            switch (GetAnchorNumber())
            {
                case Anchor_1:
                    break;
                case Anchor_2:
                    Hal_DelayXms((Anchor_2 - 1) * 700);
                    break;
                case Anchor_3:
                    Hal_DelayXms((Anchor_3 - 1) * 700);
                    break;
                case Anchor_4:
                    Hal_DelayXms((Anchor_4 - 1) * 700);
                    break;
                default:
                    break;
            }

            Send_GetRssiCommd(CurrentAngle);

            switch (GetAnchorNumber())
            {
                case Anchor_1:
                    Hal_DelayXms((4 - Anchor_1) * 700);
                    break;
                case Anchor_2:
                    Hal_DelayXms((4 - Anchor_2) * 700);
                    break;
                case Anchor_3:
                    Hal_DelayXms((4 - Anchor_3) * 700);
                    break;
                case Anchor_4:
                    break;
                default:
                    break;
            }
			Round_right();
            Hal_DelayXms((uint16_t)(6 / 0.008)); // 以6度的分辨率进行
            Round_stop();
            CurrentAngle = CurrentAngle - 6;
			
#ifdef UART_1

            angle[0] = (CurrentAngle >> 8) & 0xff;
            angle[1] = CurrentAngle & 0xff;
            SendString(1, "The CurrentAngle is:");
            SendHex2Ascills(1, angle, 2);
#endif
        }
		
    }
//    else
//    {
//        return;
//    }
}

void Round_left()
{
    // 发送命令
    SendArrayHex(3, round_left_commd, 7);
}

void Round_right()
{
    // 发送命令
    SendArrayHex(3, round_right_commd, 7);
}

void Round_stop()
{
    // 发送命令
    SendArrayHex(3, round_stop_commd, 7);
}