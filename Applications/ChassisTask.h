#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "RemoteTask.h"
#include "pid.h"
#include "CAN_Receive.h"

void chassis_task(void const * argument);

typedef enum 
{
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
    RC_ctrl_t *chassis_rc_ctrl;
    chassis_mode_e chassis_mode;
    chassis_mode_e chassis_last_mode;
    chassis_motor_t chassis_motor[4];
    pid_type_def speed_pid;
    pid_type_def follow_yaw_angle_pid;

    
}chassis_move_t;




#endif

