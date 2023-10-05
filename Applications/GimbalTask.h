#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "CAN_Receive.h"
#include "RemoteTask.h"
#include "pid.h"


void gimbal_task(void const * argument);




typedef enum 
{
    GIMBAL_INIT,
    GIMBAL_ZERO_FORCE,
    GIMBAL_ENCODER,

}gimbal_mode_e;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_mode_e gimbal_mode;
    gimbal_mode_e gimbal_last_mode;
    uint16_t offset_ecd;
    fp32 relative_angle;
    fp32 relative_angle_set;
    fp32 max_relative_angle;
    fp32 min_relative_angle;
    pid_type_def gimbal_motor_speed_pid;
    gimbal_PID_t gimbal_motor_angle_pid;
    fp32 gimbal_motor_speed_set;
    int16_t give_current;

}gimbal_motor_t;




typedef struct 
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    gimbal_motor_t gimbal_yaw_motor;


}gimbal_move_t;


#ifndef PI
#define PI					3.14159265358979f
#endif

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //2*PI/8192
#endif

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

#define YAW_RC_COEFF 1.57/660.0f

#define MIDDLE_YAW 1000

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8192


extern const gimbal_motor_t *get_yaw_motor_point(void);



#endif

