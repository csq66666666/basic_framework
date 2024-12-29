#ifndef GIMBAL_H
#define GIMBAL_H

#if defined(ROBOT_BALANCE_INFANTRY) || defined(ROBOT_INFANTRY) || defined(ROBOT_SENTRY) || defined(ROBOT_HERO)

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H

#endif