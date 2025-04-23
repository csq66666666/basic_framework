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

static float pitch_angle = 90, yaw_angle = 220, yaw_fixed_angle = 0;

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
 * @brief 云台角度限幅
 *
 */
static void GimbalAngleConstrain(float *yaw, float *pitch)
{
    if (*yaw >= YAW_MAX_ANGLE)
        *yaw = YAW_MAX_ANGLE;
    else if (*yaw <= YAW_MIN_ANGLE)
        *yaw = YAW_MIN_ANGLE;

    if (*pitch >= PITCH_MAX_ANGLE)
        *pitch = PITCH_MAX_ANGLE;
    else if (*pitch <= PITCH_MIN_ANGLE)
        *pitch = PITCH_MIN_ANGLE;
}

/**
 * @brief 云台自由运动模式
 *
 */
static void GimbalFreeMode()
{
    pitch_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_angle += gimbal_cmd_recv.yaw_add_angle;
    yaw_fixed_angle = -YAW1_VERTICAL_ANGLE + yaw_angle + gimbal_cmd_recv.yaw1_angle - 220;
    GimbalAngleConstrain(&yaw_angle, &pitch_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (int16_t)pitch_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (int16_t)yaw_angle);
}

/**
 * @brief 云台固定角度模式
 *
 */
static void GimbalFixAngleMode()
{
    pitch_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_fixed_angle += gimbal_cmd_recv.yaw_add_angle;
    yaw_angle = YAW1_VERTICAL_ANGLE + yaw_fixed_angle - gimbal_cmd_recv.yaw1_angle + 220;
    GimbalAngleConstrain(&yaw_angle, &pitch_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (int16_t)pitch_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (int16_t)yaw_angle);
}

/**
 * @brief 云台一位单银模式
 *
 */
static void GimbalSliverMiningMode()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台一位双银模式1，看小资源岛
 *
 */
static void GimbalTwoSliverMiningMode1()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台一位双银模式2，看矿仓
 *
 */
static void GimbalTwoSliverMiningMode2()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台存矿仓矿石模式1
 *
 */
static void GimbalStorageOreMode1()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台存矿仓矿石模式2
 *
 */
static void GimbalStorageOreMode2()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台取矿仓矿石模式1
 *
 */
static void GimbalFetchOreMode1()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台取矿仓矿石模式2
 *
 */
static void GimbalFetchOreMode2()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
}

/**
 * @brief 云台取金矿模式
 *
 */
static void GimbalGlodMiningMode()
{
    static float pitch_temp_angle = 0, yaw_temp_angle = 0;
    pitch_temp_angle += gimbal_cmd_recv.pitch_add_angle;
    yaw_temp_angle += gimbal_cmd_recv.yaw_add_angle;
    GimbalAngleConstrain(&yaw_temp_angle, &pitch_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_yaw_motor, (uint16_t)yaw_temp_angle);
    Servo_Motor_FreeAngle_Set(servo_pitch_motor, (uint16_t)pitch_temp_angle);
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
    case GIMBAL_SLIVER_MINGING_MODE: 
        GimbalSliverMiningMode();
        break;
    case GIMBAL_TWO_SLIVER_MINGING_MODE1: 
        GimbalTwoSliverMiningMode1();
        break;
    case GIMBAL_STORAGE_ORE_MODE1: 
        GimbalStorageOreMode1();
        break;
    case GIMBAL_STORAGE_ORE_MODE2: 
        GimbalStorageOreMode2();
        break;
    case GIMBAL_FETCH_ORE_MODE1: 
        GimbalFetchOreMode1();
        break;
    case GIMBAL_FETCH_ORE_MODE2: 
        GimbalFetchOreMode2();
        break;
    case GIMBAL_GOLD_MINING_MODE:
        GimbalGlodMiningMode();
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