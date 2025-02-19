/**
 * @file chassis.c
 * @author csq666666 Weedy
 * @brief 机械臂应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意确定所有电机的正反转
 *
 * @version 0.1
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "upper.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "user_lib.h"
// #include "vofa.h"

// 上层机构所有电机定义  大YAW（3508）      同步带YAW（3508）  控制差速器YAW（2006） 左差速（2006）    右差速（2006）           升降（3508）
static DJIMotorInstance *upper_yaw1_motor, *upper_yaw2_motor, *upper_yaw3_motor, *upper_differ_motor_l, *upper_differ_motor_r, *upper_lift_motor;
static float upper_yaw1_op, upper_yaw2_op, upper_yaw3_op, upper_differ_l_op, upper_differ_r_op; // 机械臂电机输出数据,用于设定电机参考值
static float upper_lift_op;                                                                     // 抬升电机输出数据,用于设定电机参考值
static upper_mode_e upper_last_mode;                                                            // 上一次的模式
static Upper_Joint_Data_s upper_solve;                                                          // 上层机构所有关节数据,由cmd发送或程序自行修改
// static Upper_Kine_s Upper_Kine;                                                                     // 运动学数据
static uint8_t action_finish_flag;
static uint8_t action_step = 0;
static uint8_t action_flag = 0;

static Publisher_t *upper_pub;                  // 机械臂应用消息发布者(机械臂反馈给cmd)
static Subscriber_t *upper_sub;                 // cmd控制消息订阅者
static Upper_Upload_Data_s upper_feedback_data; // 回传给cmd的机械臂状态信息
static Upper_Ctrl_Cmd_s upper_cmd_recv;         // 来自cmd的控制信息

attitude_t *Upper_IMU_data;

/**
 * @brief 通过电机速度来判断动作是否完成
 * @param 校准的步骤 完成一次 加一
 */
#define ActionFinishJudge(step, cmd_time, dirt, step_time)          \
    {                                                               \
        if (fabsf(upper_yaw1_motor->measure.speed_aps) < 100 &&      \
            fabsf(upper_yaw2_motor->measure.speed_aps) < 100 &&     \
            fabsf(upper_yaw3_motor->measure.speed_aps) < 100 &&    \
            fabsf(upper_differ_motor_l->measure.speed_aps) < 100 && \
            fabsf(upper_differ_motor_r->measure.speed_aps) < 100 && \
            fabsf(upper_lift_motor->measure.speed_aps) < 100 &&   \
            )                                                      \
        {                                                           \
            (cmd_time)++;                                           \
            if ((cmd_time) > (step_time))                           \
            {                                                       \
                (cmd_time) = 0;                                     \
                if (dirt)                                           \
                    (step)++;                                       \
                else                                                \
                    (step)--;                                       \
            }                                                       \
        }                                                           \
    }

/**
 * @brief 机械臂初始化
 *
 */
void UpperInit()
{
    Upper_IMU_data = INS_Init(); // 云台IMU初始化
    // 抬升电机
    Motor_Init_Config_s upper_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000,
                .MaxOut = 15000,
            },
            .speed_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    upper_lift_motor = DJIMotorInit(&upper_motor_config);
    upper_lift_motor->measure.init_flag = 1;

    // 大YAW（3508）
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 2;

    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 15000;

    upper_motor_config.controller_param_init_config.speed_PID.Kp = 2;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;

    upper_yaw1_motor = DJIMotorInit(&upper_motor_config);
    upper_yaw1_motor->measure.init_flag = 1;

    // 同步带YAW（3508）
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 3;

    upper_motor_config.controller_param_init_config.angle_PID.Kp = 15;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 10000;

    upper_motor_config.controller_param_init_config.speed_PID.Kp = 4;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;

    upper_motor_config.motor_type = M3508;
    upper_yaw2_motor = DJIMotorInit(&upper_motor_config);
    upper_yaw2_motor->measure.init_flag = 1;

    // 控制差速器YAW（2006）
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 4;

    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 20000;

    upper_motor_config.controller_param_init_config.speed_PID.Kp = 2.1;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0.21;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 3000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;

    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    upper_motor_config.motor_type = M2006;
    upper_yaw3_motor = DJIMotorInit(&upper_motor_config);
    upper_yaw3_motor->measure.init_flag = 1;

    // 差速器左侧电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 5;

    upper_motor_config.controller_param_init_config.angle_PID.Kp = 10;
    upper_motor_config.controller_param_init_config.angle_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.angle_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.angle_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.angle_PID.MaxOut = 10000;

    upper_motor_config.controller_param_init_config.speed_PID.Kp = 3;
    upper_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    upper_motor_config.controller_param_init_config.speed_PID.Kd = 0;
    upper_motor_config.controller_param_init_config.speed_PID.IntegralLimit = 2000;
    upper_motor_config.controller_param_init_config.speed_PID.MaxOut = 15000;

    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    upper_motor_config.motor_type = M2006;
    upper_differ_motor_l = DJIMotorInit(&upper_motor_config);
    upper_differ_motor_l->measure.init_flag = 1;

    // 差速器右侧电机
    upper_motor_config.can_init_config.can_handle = &hcan1;
    upper_motor_config.can_init_config.tx_id = 6;
    upper_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    upper_differ_motor_r = DJIMotorInit(&upper_motor_config);
    upper_differ_motor_r->measure.init_flag = 1;

    upper_pub = PubRegister("upper_feed", sizeof(Upper_Upload_Data_s));
    upper_sub = SubRegister("upper_cmd", sizeof(Upper_Ctrl_Cmd_s));
}

/**
 * @brief 将各关节当前状态反馈给cmd
 *
 */
static void UpperFeedUpdata()
{
    upper_feedback_data.joint_data.yaw1 = (upper_yaw1_motor->measure.total_angle - upper_yaw1_motor->measure.init_angle) * ROTOR_2_SHAFT_YAW1;
    upper_feedback_data.joint_data.yaw2 = (upper_yaw2_motor->measure.total_angle - upper_yaw2_motor->measure.init_angle) * ROTOR_2_SHAFT_YAW2;
    upper_feedback_data.joint_data.yaw3 = (upper_yaw3_motor->measure.total_angle - upper_yaw3_motor->measure.init_angle) * ROTOR_2_SHAFT_YAW3;
    upper_feedback_data.joint_data.roll_differ = (((upper_differ_motor_r->measure.total_angle - upper_differ_motor_r->measure.init_angle) - (-1.0f * upper_differ_motor_l->measure.total_angle + upper_differ_motor_l->measure.init_angle)) / 2.0f * GEAR_RATION_DIFFER) * ROTOR_2_SHAFT_ROLL_DIFFER;
    upper_feedback_data.joint_data.pitch_differ = (((upper_differ_motor_r->measure.total_angle - upper_differ_motor_r->measure.init_angle) + (-1.0f * upper_differ_motor_l->measure.total_angle + upper_differ_motor_l->measure.init_angle)) / 2.0f) * ROTOR_2_SHAFT_PITCH_DIFFER; // 电机反装，测量值需要取反

    upper_feedback_data.joint_data.lift_dist = ((upper_lift_motor->measure.total_angle - upper_lift_motor->measure.init_angle)) / LIFT_DIST_2_ANGLE;

    upper_feedback_data.action_step = action_step;

    upper_last_mode = upper_cmd_recv.upper_mode;
}

/**
 * @brief 上层机构电机使能
 *
 */
static void UpperIMotorEnable()
{
    DJIMotorEnable(upper_yaw1_motor);
    DJIMotorEnable(upper_yaw2_motor);
    DJIMotorEnable(upper_yaw3_motor);
    DJIMotorEnable(upper_differ_motor_l);
    DJIMotorEnable(upper_differ_motor_r);
    DJIMotorEnable(upper_lift_motor);
}

/**
 * @brief 上层机构无力模式
 *
 */
static void UpperZeroForceMode()
{
    DJIMotorStop(upper_yaw1_motor);
    DJIMotorStop(upper_yaw2_motor);
    DJIMotorStop(upper_yaw3_motor);
    DJIMotorStop(upper_differ_motor_l);
    DJIMotorStop(upper_differ_motor_r);
    DJIMotorStop(upper_lift_motor);
    upper_lift_motor->measure.init_flag = 1;
}

/**
 * @brief 将关节数据换算成电机输出数据
 *
 */
static void UpperCalculate()
{
    upper_yaw1_op = upper_solve.yaw1 * SHAFT_2_ROTOR_YAW1 + upper_yaw1_motor->measure.init_angle;
    upper_yaw2_op = upper_solve.yaw2 * SHAFT_2_ROTOR_YAW2 + upper_yaw2_motor->measure.init_angle;
    upper_yaw3_op = upper_solve.yaw3 * SHAFT_2_ROTOR_YAW3 + upper_yaw3_motor->measure.init_angle;
    upper_differ_l_op = (upper_solve.pitch_differ - upper_solve.roll_differ / GEAR_RATION_DIFFER) * SHAFT_2_ROTOR_ROLL_DIFFER - upper_differ_motor_l->measure.init_angle; // 电机反装，测量的初始值需要取反
    upper_differ_r_op = (upper_solve.pitch_differ + upper_solve.roll_differ / GEAR_RATION_DIFFER) * SHAFT_2_ROTOR_ROLL_DIFFER + upper_differ_motor_r->measure.init_angle;
    upper_lift_op = upper_solve.lift_dist * LIFT_DIST_2_ANGLE - upper_lift_motor->measure.init_angle; // 电机反装，测量的初始值需要取反
}

/**
 * @brief 上层机构校准模式
 *
 */
static void UpperCaliMode()
{
    static uint16_t cali_time = 0;
    static const float speed_maxout_differ = 15000;
    static const float speed_maxout_yaw = 15000;
    static const float angle_maxout_yaw = 10000;

    static const float speed_maxout_lift = 15000;
    static float pitch_test_max;
    static float pitch_test_min;

    if (action_step == 5)
    {
        // calibrate pitch_differ max
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculate();
            DJIMotorOuterLoop(upper_differ_motor_l, SPEED_LOOP);
            DJIMotorOuterLoop(upper_differ_motor_r, SPEED_LOOP);
            upper_differ_motor_l->motor_controller.speed_PID.MaxOut = 4000;
            upper_differ_motor_r->motor_controller.speed_PID.MaxOut = 4000;
            upper_differ_l_op = 15000;
            upper_differ_r_op = 15000;
        }
        else if (action_finish_flag == 0 && action_flag == 2)
        {
            action_flag = 3;
            upper_differ_l_op = -15000;
            upper_differ_r_op = -15000;
        }
        else if (action_flag == 0)
        {
            action_flag = 1;

            DJIMotorOuterLoop(upper_differ_motor_l, ANGLE_LOOP);
            DJIMotorOuterLoop(upper_differ_motor_r, ANGLE_LOOP);
            upper_solve.pitch_differ = upper_feedback_data.joint_data.pitch_differ + (pitch_test_max - pitch_test_min) / 2.0f - 10.0f;
            UpperCalculate();
        }

        if (fabsf(upper_differ_motor_l->measure.speed_aps) < 1.0f && fabsf(upper_differ_motor_r->measure.speed_aps) < 1.0f)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)
                {
                    cali_time = 0;
                    if (action_flag == 1)
                    {
                        action_flag = 2;
                        UpperCalculate();
                        pitch_test_max = upper_feedback_data.joint_data.pitch_differ;
                    }
                    else if (action_flag == 3)
                    {
                        action_flag = 0;
                        action_finish_flag = 1;
                        UpperCalculate();
                        pitch_test_min = upper_feedback_data.joint_data.pitch_differ;
                    }
                }
                else
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_differ_motor_l->measure.init_flag = 1;
                    upper_differ_motor_r->measure.init_flag = 1;
                    upper_solve.pitch_differ = 0;
                }
            }
        }
    }
    else if (action_step == 2)
    {
        // yaw1
        if (action_finish_flag == 0 && action_flag == 0) // step 1
        {
            action_flag = 1;
            UpperCalculate();
            DJIMotorOuterLoop(upper_yaw1_motor, SPEED_LOOP);
            upper_yaw1_motor->motor_controller.speed_PID.MaxOut = 3000;
            upper_yaw1_op = 5000;
        }
        else if (action_flag == 0) // step 3
        {
            action_flag = 1;
            DJIMotorOuterLoop(upper_yaw1_motor, ANGLE_LOOP);
            upper_yaw1_motor->motor_controller.angle_PID.MaxOut = 4000; // 防止回来过程中yaw运动太快
            upper_solve.yaw1 = upper_feedback_data.joint_data.yaw1 - 20;
            UpperCalculate();
        }

        if (fabsf(upper_yaw1_motor->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)  // step 2
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 1;
                }
                else  // step 4
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_yaw1_motor->measure.init_flag = 1;
                    upper_solve.yaw1 = 0;
                    // UpperCalculate(); // 此时init_flag还未被置位，调用UpperCalculate函数会使大yaw轴回到未校准时的位置
                }
            }
        }
    }
    else if (action_step == 3)
    {
        // yaw2
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculate();
            DJIMotorOuterLoop(upper_yaw2_motor, SPEED_LOOP);
            upper_yaw2_motor->motor_controller.speed_PID.MaxOut = 3000;
            upper_yaw2_op = 15000;
        }
        else if (action_flag == 0)
        {
            action_flag = 1;
            DJIMotorOuterLoop(upper_yaw2_motor, ANGLE_LOOP);
            upper_yaw2_motor->motor_controller.angle_PID.MaxOut = 4000;
            upper_solve.yaw2 = upper_feedback_data.joint_data.yaw2 - 120;
            UpperCalculate();
        }

        if (fabsf(upper_yaw2_motor->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 1;
                }
                else
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_yaw2_motor->measure.init_flag = 1;
                    upper_solve.yaw2 = 0;
                    // UpperCalculate();
                }
            }
        }
    }
    else if (action_step == 4)
    { // yaw3
        if (action_finish_flag == 0 && action_flag == 0)
        {
            action_flag = 1;
            UpperCalculate();
            DJIMotorOuterLoop(upper_yaw3_motor, SPEED_LOOP);
            upper_yaw3_motor->motor_controller.speed_PID.MaxOut = 3000;
            upper_yaw3_op = 15000;
        }
        else if (action_flag == 0)
        {
            action_flag = 1;
            DJIMotorOuterLoop(upper_yaw3_motor, ANGLE_LOOP);
            upper_yaw3_motor->motor_controller.angle_PID.MaxOut = 4000;
            upper_solve.yaw3 = upper_feedback_data.joint_data.yaw3 - 120;
            UpperCalculate();
        }

        if (fabsf(upper_yaw3_motor->measure.speed_aps) < EPS)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                if (action_finish_flag == 0)
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 1;
                }
                else
                {
                    cali_time = 0;
                    action_flag = 0;
                    action_finish_flag = 0;
                    action_step++;
                    upper_yaw3_motor->measure.init_flag = 1;
                    upper_solve.yaw3 = 0;
                    // UpperCalculate();
                }
            }
        }
    }
    else if (action_step == 1)
    {
        // 抬升
        if (action_finish_flag == 0 && action_flag == 0) // step 1
        {
            action_flag = 1;
            UpperCalculate();
            DJIMotorOuterLoop(upper_lift_motor, SPEED_LOOP);
            upper_lift_motor->motor_controller.speed_PID.MaxOut = 1500;
            upper_lift_op = -15000;
        }
        else if (action_flag == 0) // step 3
        {
            action_flag = 1;
            DJIMotorOuterLoop(upper_lift_motor, ANGLE_LOOP);
            upper_lift_motor->motor_controller.angle_PID.MaxOut = 4000; // 防止回来过程中yaw运动太快
            upper_solve.lift_dist = upper_feedback_data.joint_data.lift_dist - 200;
            UpperCalculate();
        }

        if (fabsf(upper_lift_motor->measure.speed_aps) < 100)
        {
            cali_time++;

            if (cali_time > CALI_STEP_TIME)
            {
                cali_time = 0;
                action_flag = 0;
                action_step++;
                DJIMotorStop(upper_lift_motor);
                DWT_Delay(1);
                upper_lift_motor->measure.init_flag = 1;
                DJIMotorOuterLoop(upper_lift_motor, ANGLE_LOOP);
                upper_solve.lift_dist = 0;
            }
        }
    }
    else if (action_step == 6)
    {
        action_step = 0;
        upper_differ_motor_l->motor_controller.speed_PID.MaxOut = speed_maxout_differ;
        upper_differ_motor_r->motor_controller.speed_PID.MaxOut = speed_maxout_differ;

        upper_yaw1_motor->motor_controller.speed_PID.MaxOut = speed_maxout_yaw;
        upper_yaw1_motor->motor_controller.angle_PID.MaxOut = angle_maxout_yaw;

        upper_yaw2_motor->motor_controller.speed_PID.MaxOut = speed_maxout_yaw;
        upper_yaw2_motor->motor_controller.angle_PID.MaxOut = angle_maxout_yaw;

        upper_yaw3_motor->motor_controller.speed_PID.MaxOut = speed_maxout_yaw;
        upper_yaw3_motor->motor_controller.angle_PID.MaxOut = angle_maxout_yaw;

        upper_lift_motor->motor_controller.speed_PID.MaxOut = speed_maxout_lift;
        UpperCalculate();
    }
}

/**
 * @brief 设置电机参考值
 *
 */
static void UpperOutput()
{
    DJIMotorSetRef(upper_yaw1_motor, upper_yaw1_op);
    DJIMotorSetRef(upper_yaw2_motor, upper_yaw2_op);
    DJIMotorSetRef(upper_yaw3_motor, upper_yaw3_op);
    DJIMotorSetRef(upper_differ_motor_l, upper_differ_l_op);
    DJIMotorSetRef(upper_differ_motor_r, upper_differ_r_op);
    DJIMotorSetRef(upper_lift_motor, upper_lift_op);
}

/**
 * @brief 单轴控制模式
 *
 */
static void UpperSingleMode()
{
    upper_solve.yaw1 = upper_cmd_recv.joint_data.yaw1;
    upper_solve.yaw2 = upper_cmd_recv.joint_data.yaw2;
    upper_solve.yaw3 = upper_cmd_recv.joint_data.yaw3;
    upper_solve.pitch_differ = upper_cmd_recv.joint_data.pitch_differ;
    upper_solve.roll_differ = upper_cmd_recv.joint_data.roll_differ;
    upper_solve.lift_dist = upper_cmd_recv.joint_data.lift_dist;
}

/**
 * @brief 根据模式选择控制方式
 *
 */
static void UpperModeControl()
{
    UpperIMotorEnable();
    if (upper_cmd_recv.upper_mode != upper_last_mode)
        action_step = 1;

    switch (upper_cmd_recv.upper_mode)
    {
    case UPPER_ZERO_FORCE:
        UpperZeroForceMode();
        break;
    case UPPER_NO_MOVE:
        break;
    case UPPER_CALI:
        UpperCaliMode();
        break;
    case UPPER_SINGLE_MOTOR:
        UpperSingleMode();
        break;
    default:
        break;
    }
}

void UpperTask()
{
    // 获取上层机构控制数据
    SubGetMessage(upper_sub, &upper_cmd_recv);

    // 根据控制模式设定上层机构工作方式
    UpperModeControl();

    // 将关节数据换算成电机输出数据
    if (upper_cmd_recv.upper_mode != UPPER_CALI)
        UpperCalculate();

    // 电机输出
    UpperOutput();

    // 更新反馈数据
    UpperFeedUpdata();

    // lift_l = (upper_lift_motor_l->measure.total_angle - upper_lift_motor_l->measure.init_angle) / LIFT_DIST_2_ANGLE;
    // lift_r = -(upper_lift_motor_r->measure.total_angle - upper_lift_motor_r->measure.init_angle) / LIFT_DIST_2_ANGLE;
    // delta_lift = lift_l - lift_r;
    // 反馈上层机构数据
    PubPushMessage(upper_pub, (void *)&upper_feedback_data);
}

void UpperJointConstrain(Upper_Joint_Data_s *joint_data)
{
    joint_data->yaw1 = float_constrain(joint_data->yaw1, yaw1_MIN, yaw1_MAX);
    joint_data->yaw2 = float_constrain(joint_data->yaw2, yaw2_MIN, yaw2_MIN);
    joint_data->pitch_differ = float_constrain(joint_data->pitch_differ, PITCH_DIFFER_MIN, PITCH_DIFFER_MAX);
    joint_data->lift_dist = float_constrain(joint_data->lift_dist, 0, LIFT_MAX_SAFE_DIST);
    joint_data->yaw3 = float_constrain(joint_data->yaw3, yaw3_MIN, yaw3_MAX);
}