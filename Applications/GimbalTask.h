#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "CAN_Receive.h"
#include "RemoteTask.h"
#include "pid.h"


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

}gimbal_motor_t;




typedef struct 
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const gimbal_motor_t *gimbal_yaw_motor;
    

}gimbal_move_t;





void gimbal_task(void const * argument);
extern const gimbal_motor_t *get_yaw_motor_point(void);


#endif

