#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"
#include "pid.h"


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



chassis_move_t chassis_move_data;
void chassis_init(chassis_move_t* chassis_move_data);
void chassis_mode_change_control_transit(chassis_move_t* chassis_move_data);
void chassis_mode_set(chassis_move_t* chassis_move_data);
void chassis_feedback_update(chassis_move_t* chassis_move_data);
void chassis_control_loop(chassis_move_t* chassis_move_data);

void chassis_zero_force_control(chassis_move_t* chassis_move_data);
void chassis_no_move_control(chassis_move_t* chassis_move_data);
void chassis_follow_yaw_control(chassis_move_t* chassis_move_data);



void chassis_task(void const * argument)
{
	chassis_init(&chassis_move_data);

	while(1)
	{	
		chassis_mode_change_control_transit(&chassis_move_data);
		chassis_mode_set(&chassis_move_data);
		chassis_control_loop(&chassis_move_data);	
//		CAN_cmd_chassis();
		vTaskDelay(2);
	}
}

void chassis_init(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data == NULL)
	{
		return;
	}
	const static fp32 motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};
	const static fp32 chassis_follow_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	chassis_move_data->chassis_mode = CHASSIS_INIT;
	chassis_move_data->chassis_rc_ctrl = get_remote_control_point();

	for (int i=0;i<4;i++)
	{
		chassis_move_data->chassis_motor[i].chassis_motor_measure = get_chassis_motor_meature_point(i); 
		PID_init(&chassis_move_data->speed_pid[i], PID_POSITION, motor_speed_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT);	
	}
	PID_init(&chassis_move_data->follow_yaw_angle_pid, PID_POSITION, chassis_follow_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	chassis_feedback_update(chassis_move_data);


}

void chassis_mode_change_control_transit(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data == NULL)
	{
		return;
	}
	if(chassis_move_data->chassis_last_mode == chassis_move_data->chassis_mode)
	{
		return;
	}

	if((chassis_move_data->chassis_last_mode != CHASSIS_FOLLOW_GIMBAL) && chassis_move_data->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
	{
		chassis_move_data->chassis_relative_angle_set = 0.0f;
	}
	chassis_move_data->chassis_last_mode = chassis_move_data->chassis_mode;

}


void chassis_feedback_update(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data == NULL)
	{
		return;
	}
	
	for (int i = 0; i < 4; i++)
    {
        chassis_move_data->chassis_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_data->chassis_motor[i].chassis_motor_measure->speed_rpm;
        chassis_move_data->chassis_motor[i].accel = chassis_move_data->speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;

    }

}

void chassis_mode_set(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data == NULL)
	{
		return;
	}
	if (switch_is_down(chassis_move_data->chassis_rc_ctrl->rc.s[0]))
    {
        chassis_move_data->chassis_mode = CHASSIS_NO_MOVE;
    }
	else if (switch_is_up(chassis_move_data->chassis_rc_ctrl->rc.s[0]))
    {
        chassis_move_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
    }
}


void chassis_control_loop(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data->chassis_mode == CHASSIS_INIT)
	{
		chassis_zero_force_control(chassis_move_data);
	}
	else if(chassis_move_data->chassis_mode == CHASSIS_NO_MOVE)
	{
		chassis_no_move_control(chassis_move_data);
	}
	else if (chassis_move_data->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
	{
		chassis_follow_yaw_control(chassis_move_data);
	}
}


void chassis_zero_force_control(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data->vx_set == NULL || chassis_move_data->vy_set == NULL || chassis_move_data->wz_set == NULL)
	{
		return;
	}
	chassis_move_data->vx_set = 0.0;
	chassis_move_data->vy_set = 0.0;
	chassis_move_data->wz_set = 0.0;
	for (int i = 0;i<4;i++)
	{
		chassis_move_data->chassis_motor[i].give_current = 0;
	} 
}

void chassis_no_move_control(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data->vx_set == NULL || chassis_move_data->vy_set == NULL || chassis_move_data->wz_set == NULL)
	{
		return;
	}

	chassis_move_data->vx_set = 0.0;
	chassis_move_data->vy_set = 0.0;
	chassis_move_data->wz_set = 0.0;

}


void chassis_follow_yaw_control(chassis_move_t* chassis_move_data)
{
	if(chassis_move_data->vx_set == NULL || chassis_move_data->vy_set == NULL || chassis_move_data->wz_set == NULL)
	{
		return;
	}
	int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
	if (switch_is_up(chassis_move_data->chassis_rc_ctrl->rc.s[0]))
	{
		rc_deadband_limit(chassis_move_data->chassis_rc_ctrl->rc.ch[1], vx_channel, 10);
		rc_deadband_limit(chassis_move_data->chassis_rc_ctrl->rc.ch[0], vy_channel, 10);

		vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
		vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
	}
	else
	{	
		vx_set_channel = 0;
		vy_set_channel = 0;
	}
	//可添加滤波，使之更加平滑！
	chassis_move_data->vx_set = vx_set_channel;
	chassis_move_data->vy_set = vy_set_channel;

}




