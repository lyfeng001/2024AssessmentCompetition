#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "CAN_Receive.h"
#include "RemoteTask.h"
#include "pid.h"


void gimbal_task(void const * argument);
extern const gimbal_motor_t *get_yaw_motor_point(void);



typedef enum 
{
    GIMBAL_INIT,
    GIMBAL_ENCODER,

}gimbal_mode_e;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_mode_e gimbal_mode;
    gimbal_mode_e gimbal_last_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle;
    fp32 min_relative_angle;
    pid_type_def gimbal_motor_speed_pid;
    gimbal_PID_t gimbal_motor_angle_pid;

}gimbal_motor_t;




typedef struct 
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const gimbal_motor_t gimbal_yaw_motor;
    

}gimbal_move_t;




#define YAW_SPEED_PID_KP  3600.0f
#define YAW_SPEED_PID_KI  0.0f
#define YAW_SPEED_PID_KD  0.0f

#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f


#define YAW_ANGLE_PID_KP  8.0f
#define YAW_ANGLE_PID_KI  0.0f
#define YAW_ANGLE_PID_KD  0.0f

#define YAW_ANGLE_PID_MAX_OUT 10.0f
#define YAW_ANGLE_PID_MAX_IOUT 0.0f








#endif

