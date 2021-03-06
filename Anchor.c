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
#include "function.h"
#include "motor.h"

// 存放中心站由433M信道传来的启动帧和电机控制帧
CommdInfo commdinfo = {{0x00}, 0, 0, false};

// RSSI读取命令结束
const uint8_t RSSI_OVER[8] = {'R', 'S', 'S', 'I', 'O', 'V', 'E', 'R'};

// 存放需求角度以及角度旋转的方向
DesAngle desangle = {'+', 0x0000};

extern uint16_t CurrentAngle;

// 存放锚节点的标号
Anchor_Number Anchor_Num = Anchor_None;

// 初始化设置计数器
uint8_t Init_time = 0;

extern uint8_t idata Msg_Index;

// 帧头类型的设计，用于串口解析函数1
uint8_t Commd_Head = 0;
// 协议解析状态机
State state_machine = NONE;


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
								CurrentAngle = CurrentAngle + 270;
								break;

							case Anchor_2: // 锚节点2
								CurrentAngle = CurrentAngle + 360;
								break;

							case Anchor_3: // 锚节点3
								CurrentAngle = CurrentAngle + 90;
								break;

							case Anchor_4: // 锚节点4
								CurrentAngle = CurrentAngle + 180;
								break;

							default: // 如果不是锚节点
								return;
								break;
						}
					}

					// 控制电机转动至初始位置
					InitRound();
                    commdinfo.Commd_Type = NoneCommd;
					break;

				case ControlCommd: // 如果收到的命令是电机控制帧命令
					
					// 控制电机逐渐转动，直到转至需求角度
					continueRound();
				
					// 电机全部转动完毕之后，四号锚节点给433中心站发送 RSSIOVER 命令
					if(Anchor_Num == Anchor_1)
					{
						// RSSI读取命令发送完毕
						SendArrayHex(4, RSSI_OVER, 8);
						return;
					}
                    commdinfo.Commd_Type = NoneCommd;
					break;
                case REST:
                    commdinfo.Commd_Type = NoneCommd;
                    REST_MCU();      // 进行软件复位
                    return;
				case NoneCommd:
					break;
				default:
                    commdinfo.Commd_Type = NoneCommd;
					break;
            }
			// 命令类型复位，等待下一次接收
			commdinfo.Commd_Type = NoneCommd;
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
    switch(state_machine)
    {
        case NONE:
            if(Msg == 'm')
            {
                state_machine = HEAD1;//第一帧帧头开始
            }
            else
            {
                state_machine = NONE;// 不是帧头开始，状态机复位
            }
            break;
        case HEAD1:
            if(Msg == 'm')
            {
                state_machine = HEAD2;
            }
            else
            {
                state_machine = NONE; // 帧头监测有误，状态机复位
            }  
            break;
        case HEAD2:
            switch(Msg)
            {
                case 'I':
                    state_machine = MSG;
                    commdinfo.Commd_Type = InitCommd;
                    break;
                case 'C':
                    state_machine = MSG;
                    commdinfo.Commd_Type = ControlCommd;
                    break;
                case 'R':
                    state_machine = MSG;
                    commdinfo.Commd_Type = REST;
                    break;
                default:
                    commdinfo.Commd_Type = NoneCommd;
                    state_machine = NONE; // 命令类型有误，状态机复位
                    break;
            }

            break;
        case MSG:
            commdinfo.Commd[commdinfo.Commd_Index] = Msg;
//	        Msg_Index += 1;
//            commdinfo.Commd_Index = Msg_Index;
			commdinfo.Commd_Index += 1;
            if(commdinfo.Commd_Index == 3) // 数据接收完毕
            {
                Msg_Index = 0;
                commdinfo.Commd_Index = Msg_Index;
                state_machine = TAIL1;     // 状态机切换至尾帧检测
                desangle.F = commdinfo.Commd[0];                                 // 得到转动方向
                desangle.ANGLE = (commdinfo.Commd[1] << 8) | commdinfo.Commd[2]; // 得到转动的角度
            }
            break;
        case TAIL1:
            if(Msg == 'm')
            {
                state_machine = TAIL2;
            }
            else
            {
                state_machine = NONE;         // 帧尾出现问题，状态机复位
                memset((commdinfo.Commd), 0x00, 3);
                commdinfo.Commd_Type = NoneCommd;
                desangle.F = 0;
                desangle.ANGLE = 0x0000;
            }
            break;
        case TAIL2:
            if(Msg == 'm')                   // 完整有效的数据包接收完毕
            {
                state_machine = NONE;       // 状态机复位，准备下一次数据接收
                commdinfo.Commd_In_Flag = true;
				if(commdinfo.Commd_Type == REST)
				{
					REST_MCU();      		// 进行软件复位,保证复位具有最高的优先级
				}
            }
            else
            {
                state_machine = NONE;         // 帧尾出现问题，状态机复位
                commdinfo.Commd_In_Flag = false;
                memset((commdinfo.Commd), 0x00, 3);
                commdinfo.Commd_Type = NoneCommd;
                desangle.F = 0;
                desangle.ANGLE = 0x0000;
            }
            break;
        default:
            state_machine = NONE;         // 状态机出现问题，状态机复位
            break;

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
        getrssicommd[6] = i + 1;
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
	 * 电机旋转 7.8度/s	 
	 */
    // 进行初始角度设置
    if (desangle.F == '+') // 逆时针旋转，向左转,度数增加
    {
		// 电机逆时针转动到目标角度
		RoundLeft2Angle(desangle.ANGLE);
		
        CurrentAngle = CurrentAngle + desangle.ANGLE;
    }
    else if (desangle.F == '-') // 顺时针旋转，向右转，度数减小,最后不要采用顺时针
    {
		// 电机顺时针转到目标角度
		RoundRight2Angle(desangle.ANGLE);
     
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
		
		Hal_DelayXms((GetAnchorNumber() - 1) * 700);
		Send_GetRssiCommd(CurrentAngle);
		Hal_DelayXms((4 - GetAnchorNumber()) * 700);
		
        while (CurrentAngle < desangle.ANGLE)
        {       
		
			// 以6度的分辨率进行旋转
			RoundLeft2Angle(6);
			CurrentAngle = CurrentAngle + 6;
			
			// 延时不同的时间进行RSSI读取控制命令帧的发送
            Hal_DelayXms((GetAnchorNumber() - 1) * 700);

            Send_GetRssiCommd(CurrentAngle);

			// 将时间差补回来
            Hal_DelayXms((4 - GetAnchorNumber()) * 700);
			
			
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
		
		Hal_DelayXms((GetAnchorNumber() - 1) * 700);
		Send_GetRssiCommd(CurrentAngle);
		Hal_DelayXms((4 - GetAnchorNumber()) * 700);
		
        while (CurrentAngle > desangle.ANGLE)
        {
            // 以6度的分辨率旋转
			RoundRight2Angle(6);
			CurrentAngle = CurrentAngle - 6;
			
			// 延时不同的时间进行RSSI读取控制命令帧的发送
            Hal_DelayXms((GetAnchorNumber() - 1) * 700);

            Send_GetRssiCommd(CurrentAngle);

            // 将时间差补回来
            Hal_DelayXms((4 - GetAnchorNumber()) * 700);
			
			
#ifdef UART_1

            angle[0] = (CurrentAngle >> 8) & 0xff;
            angle[1] = CurrentAngle & 0xff;
            SendString(1, "The CurrentAngle is:");
            SendHex2Ascills(1, angle, 2);
#endif
        }
    }
}

