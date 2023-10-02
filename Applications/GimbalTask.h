#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

void gimbal_task(void const * argument);

typedef struct 
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    


}gimbal_move_t;




extern const gimbal_motor_t *get_yaw_motor_point(void);


#endif

