// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "upper.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *upper_cmd_pub;           // 上层机构控制消息发布者
static Subscriber_t *upper_feed_sub;         // 上层机构反馈信息订阅者
static Upper_Ctrl_Cmd_s upper_cmd_send;      // 传递给上层机构的控制信息
static Upper_Upload_Data_s upper_fetch_data; // 从上层机构获取的反馈信息

static TickType_t dial_time_start; // 计时开始时的时间
static TickType_t dial_time_now;   // 当前时刻时间
static uint8_t dial_press_flag;    // 拨轮按下标志位
static upper_mode_e upper_last_mode;

static Robot_Status_e robot_state; // 机器人整体工作状态

static uint8_t Ore_Storage_Flag1; // 矿仓1矿石存放标志位
static uint8_t Ore_Storage_Flag2; // 矿仓2矿石存放标志位

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{

    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    upper_cmd_pub = PubRegister("upper_cmd", sizeof(Upper_Ctrl_Cmd_s));
    upper_feed_sub = SubRegister("upper_feed", sizeof(Upper_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
    gimbal_cmd_send.yaw_fixed_angle = 0;
    gimbal_cmd_send.yaw_free_angle = 220;
    gimbal_cmd_send.pitch_free_angle = 90;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 将当前位姿更新到发送端，使后续操作在当前位姿上执行
 *
 */
static void CmdRecvUpdate()
{
    upper_cmd_send.joint_data.yaw1 = upper_fetch_data.joint_data.yaw1;
    upper_cmd_send.joint_data.yaw2 = upper_fetch_data.joint_data.yaw2;
    upper_cmd_send.joint_data.yaw3 = upper_fetch_data.joint_data.yaw3;
    upper_cmd_send.joint_data.roll_differ = upper_fetch_data.joint_data.roll_differ;
    upper_cmd_send.joint_data.pitch_differ = upper_fetch_data.joint_data.pitch_differ;
    upper_cmd_send.joint_data.lift_dist = upper_fetch_data.joint_data.lift_dist;
}

/**
 * @brief 云台控制数据更新
 *
 */
static void GimbalSendUpdate()
{
    gimbal_cmd_send.yaw1_angle = -upper_cmd_send.joint_data.yaw1;
    gimbal_cmd_send.yaw_free_angle = gimbal_fetch_data.yaw_free_angle_upload;
    gimbal_cmd_send.yaw_fixed_angle = gimbal_fetch_data.yaw_fixed_angle_upload;
}
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    static uint8_t test_flag = 1;
    if (upper_cmd_send.upper_mode != UPPER_CALI)
    {
        if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[上]
        {
            // 控制底盘运行模式
            if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],底盘正常行进
            {
                chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            }
            chassis_cmd_send.vx = 40.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
            chassis_cmd_send.vy = 40.0f * (float)rc_data[TEMP].rc.rocker_r1; // |竖直方向
            chassis_cmd_send.wz = -5.0f * (float)rc_data[TEMP].rc.rocker_l_; // ↺旋转方向 遥控器摇杆从左往右值增大,与旋转方向相反，所以取相反数
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中]
        {
            upper_cmd_send.upper_mode = UPPER_SINGLE_MOTOR;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;

            if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上] ，抬升+yaw1+yaw2
            {
                upper_cmd_send.joint_data.yaw1 += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
                upper_cmd_send.joint_data.lift_dist += 0.002f * (float)rc_data[TEMP].rc.rocker_l1;
                upper_cmd_send.joint_data.yaw2 += 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
            }
            else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中] ，yaw3+差速器
            {
                upper_cmd_send.joint_data.yaw3 += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
                upper_cmd_send.joint_data.roll_differ += 0.001f * (float)rc_data[TEMP].rc.rocker_r_; // 待改动
                upper_cmd_send.joint_data.pitch_differ += 0.001f * (float)rc_data[TEMP].rc.rocker_r1;
            }
        }
        else if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[下]
        {
            if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上] ，自定义控制器数据传输
            {
                upper_cmd_send.ctrl_data.pitch = chassis_fetch_data.ctrl_data.pitch;
                upper_cmd_send.ctrl_data.yaw = chassis_fetch_data.ctrl_data.yaw;
                upper_cmd_send.ctrl_data.roll = chassis_fetch_data.ctrl_data.roll;
                upper_cmd_send.ctrl_data.push_dist = chassis_fetch_data.ctrl_data.push_dist;
                upper_cmd_send.ctrl_data.traverse_dist = chassis_fetch_data.ctrl_data.traverse_dist;
            }
        }
        // 真空泵控制,拨轮向上打为负,向下为正
        if (rc_data[TEMP].rc.dial < -100) // 向上打开/关闭真空泵
        {
            chassis_cmd_send.pump_mode = VALVE_ALL_OPEN;
        }
        else if (rc_data[TEMP].rc.dial > 100)
        {
            chassis_cmd_send.pump_mode = VALVE_ALL_CLOSE;
        }
        // 拨轮下拨2s机械臂初始化
        if (rc_data[TEMP].rc.dial > 100) // 拨轮下拨
        {
            if (!dial_press_flag)
            {
                dial_press_flag = 1;
                dial_time_start = xTaskGetTickCount();
            }

            if (dial_press_flag == 1)
            {
                dial_time_now = xTaskGetTickCount();

                if ((dial_time_now - dial_time_start) > 2000)
                {
                    upper_cmd_send.upper_mode = UPPER_CALI;
                }
            }
        }
        else
        {
            dial_press_flag = 0;
        }
    }
    else
    {
        if (upper_fetch_data.action_step == 0)
        {
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    /***********************************************   此处为基本动作控制   ***********************************************/
    if (!(rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
    //  W/S  //  A/D  //  Q/E  //
    //  前后 //  左右  // 旋转  //
    {
        if (chassis_cmd_send.chassis_mode == CHASSIS_NORMAL)
        {
            chassis_cmd_send.vx = 7000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
            chassis_cmd_send.vy = 7000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
            chassis_cmd_send.wz = 1000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].q - (float)rc_data[TEMP].key[KEY_PRESS].e); // ↺自旋
        }
        else if (chassis_cmd_send.chassis_mode == CHASSIS_CHARGE)
        {
            chassis_cmd_send.vx = 4000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
            chassis_cmd_send.vy = 4000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
            chassis_cmd_send.wz = 1000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].q - (float)rc_data[TEMP].key[KEY_PRESS].e); // ↺自旋
        }
    }
    else
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
    }

    // 机械臂键鼠控制
    // shift + //  W/S  //  Q/E  //  A/D 
    //         //  抬升 //  yaw1 //  yaw2
    if ((rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        upper_cmd_send.joint_data.lift_dist += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s);
        upper_cmd_send.joint_data.yaw1 += 0.5f *(1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e);
        upper_cmd_send.joint_data.yaw2 += 0.5f *(1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d);
    }

    //  ctrl + //   W/S  //   A/D  //   Q/E 
    //           小pitch //  yaw3  //   roll 
    if ((!rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        upper_cmd_send.joint_data.roll_differ += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q);
        upper_cmd_send.joint_data.pitch_differ += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
        upper_cmd_send.joint_data.yaw3 += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
    }

    if (rc_data[TEMP].key_count[KEY_PRESS][15] % 2) // 最后一个键为b键
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
    }
    else
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    }
    // 小云台键鼠控制
    //  ctrl + shift  //    W/S     //    A/D
    //                   云台pitch  //   云台yaw
    if ((rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    {
        if (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE)
        {
            gimbal_cmd_send.pitch_free_angle += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
            gimbal_fetch_data.yaw_free_angle_upload += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
        }
        else if (gimbal_cmd_send.gimbal_mode == GIMBAL_FIX_ANGLE_MODE)
        {
            gimbal_cmd_send.pitch_free_angle += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
            gimbal_fetch_data.yaw_fixed_angle_upload += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
        }
    }

    // 这里由于优先级原因泵控制必须要在模式控制之前以强制覆盖
    // shift + R 关闭真空泵
    if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].r)
        chassis_cmd_send.pump_mode = VALVE_ALL_CLOSE;
    // 单击 R 键打开真空泵
    else if (rc_data[TEMP].key[KEY_PRESS].r)
        chassis_cmd_send.pump_mode |= VALVE_ALL_OPEN;

    /**************************************************   此处为动作组   **************************************************/
    if (upper_cmd_send.upper_mode < UPPER_SLIVER_MINING)
    {
        upper_cmd_send.upper_mode = UPPER_SINGLE_MOTOR;

        // ctrl + shift + F 键进入存矿仓矿石模式
        if (rc_data[TEMP].key[KEY_PRESS].f && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].shift)
        {
            if (Ore_Storage_Flag2 == 1) // 先存2
            {
                upper_cmd_send.upper_mode = UPPER_STORAGE_ORE_1;
            }
            else
            {
                upper_cmd_send.upper_mode = UPPER_STORAGE_ORE_2;
            }
        }
        // shift + F 键进入取矿仓矿石模式
        else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].f)
        {
            if (Ore_Storage_Flag1 != 0) // 先取1
            {
                upper_cmd_send.upper_mode = UPPER_FETCH_ORE_1;
            }
            else
            {
                upper_cmd_send.upper_mode = UPPER_FETCH_ORE_2;
            }
        }
        // 单击 F 键进入小资源岛一位双矿模式
        else if (rc_data[TEMP].key[KEY_PRESS].f)
        {
            upper_cmd_send.upper_mode = UPPER_TWO_SLIVER_MINING;
        }

        // 单击 G 键进入大资源岛取矿模式
        if (rc_data[TEMP].key[KEY_PRESS].g)
        {
            upper_cmd_send.upper_mode = UPPER_GLOD_MINING;
        }

        // 单击 C 键进入取单银矿，地面矿模式
        if (rc_data[TEMP].key[KEY_PRESS].c)
        {
            upper_cmd_send.upper_mode = UPPER_SLIVER_MINING;
        }

        // 单击 V 键进入自定义控制器兑矿模式
        if (rc_data[TEMP].key[KEY_PRESS].v)
        {
            upper_cmd_send.upper_mode = UPPER_EXCHANGE;
            upper_cmd_send.joint_data.lift_dist = upper_fetch_data.joint_data.lift_dist; // 视自定义控制器抬升设计情况而决定是否保留此句
        }
    }

    /**************************************************   此处为模式控制任务   **************************************************/

    if (upper_cmd_send.upper_mode == UPPER_SLIVER_MINING) // 单银矿石，地矿
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        chassis_cmd_send.pump_mode = VALVE_ARM1;
        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].c && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
            upper_cmd_send.cfm_flag = 1;

        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 3)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_TWO_SLIVER_MINING) // 一位双矿
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GET_TWO_SILVER_MODE;
        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
        {
            upper_cmd_send.cfm_flag = 1;
            chassis_cmd_send.pump_mode = VALVE_ALL_OPEN;
        }
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 6) // 再次单击 F 键继续执行
        {
            upper_cmd_send.cfm_flag = 2;
        }
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 7) // 再次单击 F 键继续执行
        {
            chassis_cmd_send.pump_mode = VALVE_T_ALL_OPEN; // 先关闭臂上气路再抬起
            upper_cmd_send.cfm_flag = 3;
        }

        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 3 && upper_fetch_data.action_step != 6 && upper_fetch_data.action_step != 7)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
            Ore_Storage_Flag1 = 1;
            Ore_Storage_Flag2 = 1;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_FETCH_ORE_1) // 取矿仓1矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T_ALL_OPEN;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
        {
            chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T2;
            upper_cmd_send.cfm_flag = 1;
        }

        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 3)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            Ore_Storage_Flag1 = 0;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_FETCH_ORE_2) // 取矿仓2矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T2;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
        {
            chassis_cmd_send.pump_mode = VALVE_ARM1;
            upper_cmd_send.cfm_flag = 1;
        }

        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 3)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
            Ore_Storage_Flag2 = 0;
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_STORAGE_ORE_1) // 存矿仓1矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T_ALL_OPEN;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 4) // 再次单击 F 键继续执行
        {
            chassis_cmd_send.pump_mode = VALVE_T_ALL_OPEN;
            upper_cmd_send.cfm_flag = 1;
        }

        // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
        if (upper_fetch_data.action_step != 4)
            upper_cmd_send.cfm_flag = 0;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
            Ore_Storage_Flag1 = 1;
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_STORAGE_ORE_2) // 存矿仓2矿石
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T2;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
            upper_cmd_send.stop_flag = 1;
        else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 4) // 再次单击 F 键继续执行
        {
            chassis_cmd_send.pump_mode = VALVE_T2;
            upper_cmd_send.cfm_flag = 1;
        }
        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
            Ore_Storage_Flag2 = 1;
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_GLOD_MINING) // 取金矿模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GET_GOLD_MODE;
        chassis_cmd_send.pump_mode = VALVE_ARM1;

        // 任务执行结束
        if (upper_fetch_data.action_step == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            upper_cmd_send.stop_flag = 0;
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
            CmdRecvUpdate();
        }
    }
    else if (upper_cmd_send.upper_mode == UPPER_EXCHANGE) // 控制器兑换
    {
        chassis_cmd_send.chassis_mode = CHASSIS_MINING;
        // upper_cmd_send.ctrlr_data.pitch = chassis_fetch_data.ctrlr_data.pitch;
        // upper_cmd_send.ctrlr_data.yaw = chassis_fetch_data.ctrlr_data.yaw;
        // upper_cmd_send.ctrlr_data.roll = chassis_fetch_data.ctrlr_data.roll;
        // upper_cmd_send.ctrlr_data.push_dist = chassis_fetch_data.ctrlr_data.push_dist;
        // upper_cmd_send.ctrlr_data.traverse_dist = chassis_fetch_data.ctrlr_data.traverse_dist;

        if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
        {
            upper_cmd_send.upper_mode = UPPER_NO_MOVE;
            CmdRecvUpdate();
        }
    }
    if (Ore_Storage_Flag1) // 锁定矿仓气路防止矿石掉落
    {
        chassis_cmd_send.pump_mode |= VALVE_T1;
    }
    if (Ore_Storage_Flag2)
    {
        chassis_cmd_send.pump_mode |= VALVE_T2;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_NOMOVE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        upper_cmd_send.upper_mode = UPPER_ZERO_FORCE;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(upper_feed_sub, &upper_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right))
        MouseKeySet(); // 键鼠控制
    else
        RemoteControlSet(); // 遥控器控制

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    UpperJointConstrain(&upper_cmd_send.joint_data);

    GimbalSendUpdate();
    
    if (upper_last_mode != upper_cmd_send.upper_mode)
    {
        CmdRecvUpdate();
    }
    upper_last_mode = upper_cmd_send.upper_mode;
    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(upper_cmd_pub, (void *)&upper_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
