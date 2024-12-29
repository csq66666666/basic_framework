/**
 * @file upper.c
 * @author Weedy 
 * @brief 工程机器人机械臂部分应用,负责接收robot_cmd的控制命令并根据命令进行正逆运动学解算,得到输出
 *        
 *
 * @version 0.0
 * @date 2024-12-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "robot_def.h"
#ifdef ROBOT_ENGINEER

#include "message_center.h"
#include "elec_switch.h"


static Publisher_t *upper_pub;                   // 机械臂应用消息发布者(云台反馈给cmd)
static Subscriber_t *upper_sub;                  // cmd控制消息订阅者
static Upper_Upload_Data_s upper_feedback_data; // 回传给cmd的机械臂状态信息
static Upper_Ctrl_Cmd_s upper_cmd_recv;         // 来自cmd的控制信息

static ElecSwitchInstance *elec_switch_valve1;	// 电磁阀1继电器模块
static ElecSwitchInstance *elec_switch_valve2;	// 电磁阀2继电器模块
static ElecSwitchInstance *elec_switch_valve3;	// 电磁阀3继电器模块
static ElecSwitchInstance *elec_switch_valve4;	// 电磁阀4继电器模块

static ElecSwitchInstance *elec_switch_pump1;	// 气泵1继电器模块
static ElecSwitchInstance *elec_switch_pump2;	// 气泵2继电器模块

void Upper_Init()
{
    ElecSwitch_Init_Config_s elecswitch_init_cofig = {
		.GPIOx = VALVE1_GPIO_Port,
		.GPIO_Pin = VALVE1_Pin,
		.trigger_level = HIGH,
	};
	elec_switch_valve1 = ElecSwitchInit(&elecswitch_init_cofig);

	elecswitch_init_cofig.GPIOx = VALVE2_GPIO_Port;
	elecswitch_init_cofig.GPIO_Pin = VALVE2_Pin;
	elecswitch_init_cofig.trigger_level = HIGH;
	elec_switch_valve2 = ElecSwitchInit(&elecswitch_init_cofig);

	elecswitch_init_cofig.GPIOx = VALVE3_GPIO_Port;
	elecswitch_init_cofig.GPIO_Pin = VALVE3_Pin;
	elecswitch_init_cofig.trigger_level = HIGH;
	elec_switch_valve3 = ElecSwitchInit(&elecswitch_init_cofig);

	elecswitch_init_cofig.GPIOx = VALVE4_GPIO_Port;
	elecswitch_init_cofig.GPIO_Pin = VALVE4_Pin;
	elecswitch_init_cofig.trigger_level = HIGH;
	elec_switch_valve4 = ElecSwitchInit(&elecswitch_init_cofig);

	elecswitch_init_cofig.GPIOx = PUMP_GPIO_Port;
	elecswitch_init_cofig.GPIO_Pin = PUMP1_Pin;
	elecswitch_init_cofig.trigger_level = HIGH;
	elec_switch_pump1 = ElecSwitchInit(&elecswitch_init_cofig);

	elecswitch_init_cofig.GPIOx = PUMP_GPIO_Port;
	elecswitch_init_cofig.GPIO_Pin = PUMP2_Pin;
	elecswitch_init_cofig.trigger_level = HIGH;
	elec_switch_pump2 = ElecSwitchInit(&elecswitch_init_cofig);

	upper_pub = PubRegister("upper_feed", sizeof(Upper_Upload_Data_s));
    upper_sub = SubRegister("upper_cmd", sizeof(Upper_Ctrl_Cmd_s));
}

void Valve_Ctrl()
{
	if(upper_cmd_recv.valve_mode & valve1_on)
		ElecSwitchSet(elec_switch_valve1);
	else
		ElecSwitchReset(elec_switch_valve1);

	if(upper_cmd_recv.valve_mode & valve2_on)
		ElecSwitchSet(elec_switch_valve2);
	else
		ElecSwitchReset(elec_switch_valve2);

	if(upper_cmd_recv.valve_mode & valve3_on)
		ElecSwitchSet(elec_switch_valve3);
	else
		ElecSwitchReset(elec_switch_valve3);

	if(upper_cmd_recv.valve_mode & valve4_on)
		ElecSwitchSet(elec_switch_valve4);
	else
		ElecSwitchReset(elec_switch_valve4);
}

void Pump_Ctrl()
{
	if(upper_cmd_recv.pump_mode == PUMP_ON)
	{
		ElecSwitchSet(elec_switch_pump1);
		ElecSwitchSet(elec_switch_pump2);
	}
	else
	{
		ElecSwitchReset(elec_switch_pump1);
		ElecSwitchReset(elec_switch_pump2);
	}
}

void Upper_Task()
{
	//获取机械臂控制数据
	SubGetMessage(upper_sub, &upper_cmd_recv);

    switch (upper_cmd_recv.upper_mode)
    {
	case UPPER_TEST:
		Valve_Ctrl();
		Pump_Ctrl();
		break;

	case UPPER_ZERO_FORCE:
		break;
	}

	PubPushMessage(upper_pub, (void *)&upper_feedback_data);
}
#endif  // ROBOT_ENGINEER
