#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "RemoteTask.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "GimbalTask.h"

void chassis_task(void const * argument);

typedef enum 
{
    CHASSIS_NO_MOVE,
    CHASSIS_INIT,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_FOLLOW_CHASSIS,

}chassis_mode_e;


typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
}chassis_motor_t;


typedef struct
{
    const RC_ctrl_t *chassis_rc_ctrl;
    const gimbal_motor_t *chassis_yaw_motor; 
    chassis_mode_e chassis_mode;
    chassis_mode_e chassis_last_mode;
    chassis_motor_t chassis_motor[4];
    pid_type_def speed_pid[4];
    pid_type_def follow_yaw_angle_pid;
    fp32 chassis_relative_angle_set;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    
}chassis_move_t;

#define M2006_MOTOR_SPEED_PID_KP 15000.0f
#define M2006_MOTOR_SPEED_PID_KI 0.0f
#define M2006_MOTOR_SPEED_PID_KD 200.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT 16000.0
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


#define CHASSIS_FOLLOW_GIMBAL_PID_KP 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 350.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f


#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.000415809748903494517209f
#define CHASSIS_CONTROL_FREQUENCE 500.0f
#define CHASSIS_VX_RC_SEN 0.006f
#define CHASSIS_VY_RC_SEN 0.005f

#define CHASSIS_WZ_SET_SCALE -1.0f
#define MOTOR_DISTANCE_TO_CENTER 0.1f

#endif

