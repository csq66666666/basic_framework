#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "servo_motor.h"

static ServoInstance *servo_yaw_motor, *servo_pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void GimbalInit()
{   
    Servo_Init_Config_s servo_config = {
        .Servo_type = Servo270,
        .Servo_Angle_Type = Free_Angle_mode,
        .htim = &htim1,
        .Channel = TIM_CHANNEL_1
    };
    servo_yaw_motor = ServoInit(&servo_config);

    servo_config.Servo_type = Servo180;
    servo_config.Channel = TIM_CHANNEL_2;
    servo_pitch_motor = ServoInit(&servo_config);

    Servo_Motor_FreeAngle_Set(servo_yaw_motor, 220);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, 90);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 云台自由运动模式
 *
 */
static void GimbalFreeMode()
{
    gimbal_feedback_data.yaw_free_angle_upload = gimbal_cmd_recv.yaw_free_angle;
    gimbal_feedback_data.yaw_fixed_angle_upload = -YAW1_VERTICAL_ANGLE + gimbal_cmd_recv.yaw_free_angle + gimbal_cmd_recv.yaw1_angle - 220;
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, gimbal_cmd_recv.pitch_free_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, gimbal_feedback_data.yaw_free_angle_upload);
}

/**
 * @brief 云台固定角度模式
 *
 */
static void GimbalFixAngleMode()
{
    gimbal_feedback_data.yaw_free_angle_upload = YAW1_VERTICAL_ANGLE + gimbal_cmd_recv.yaw_fixed_angle - gimbal_cmd_recv.yaw1_angle + 220;
    gimbal_feedback_data.yaw_fixed_angle_upload = gimbal_cmd_recv.yaw_fixed_angle;
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, gimbal_cmd_recv.pitch_free_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, gimbal_feedback_data.yaw_free_angle_upload);
}

/**
 * @brief 云台一位双矿模式
 *
 */
static void GimbalGetTwoSilverMode()
{

}

/**
 * @brief 云台取金矿模式
 *
 */
static void GimbalGetGoldMode()
{

}

/**
 * @brief 根据模式选择控制方式
 *
 */
static void GimbalModeControl()
{
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_NOMOVE:
        break;
    case GIMBAL_FREE_MODE:
        GimbalFreeMode();
        break;
    case GIMBAL_FIX_ANGLE_MODE:
        GimbalFixAngleMode();
        break;
    case GIMBAL_GET_TWO_SILVER_MODE: 
        GimbalGetTwoSilverMode();
        break;
    case GIMBAL_GET_GOLD_MODE:
        GimbalGetGoldMode();
        break;
    default:
        break;
    }
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    GimbalModeControl();
    ServoMotorControl(); // 驱动舵机转动
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}