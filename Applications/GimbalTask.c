#include "GimbalTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"

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

gimbal_move_t gimbal_move_data;
void gimbal_init(gimbal_move_t *gimbal_move_data);
void gimbal_feedback_update(gimbal_move_t *gimbal_move_data);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
void gimbal_mode_set(gimbal_move_t *gimbal_move_data);
void gimbal_mode_change_control_transit(gimbal_move_t *gimbal_move_data);
void gimbal_control_loop(gimbal_move_t *gimbal_move_data);

void gimbal_zero_force_control(gimbal_move_t *gimbal_move_data);
void gimbal_encoder_control(gimbal_move_t *gimbal_move_data);
void gimbal_init_control(gimbal_move_t *gimbal_move_data);

void cal_from_detla_to_current(gimbal_move_t *gimbal_move_data);



void gimbal_task(void const * argument)
{	
	gimbal_init(&gimbal_move_data);

	while(1)
	{
		taskENTER_CRITICAL();
		gimbal_mode_set(&gimbal_move_data);
		taskEXIT_CRITICAL();
		
		gimbal_mode_change_control_transit(&gimbal_move_data);
		gimbal_feedback_update(&gimbal_move_data);
		gimbal_control_loop(&gimbal_move_data);

		CAN_cmd_gimbal(gimbal_move_data.gimbal_yaw_motor.give_current);
		
		vTaskDelay(2);
	}
	
}


void gimbal_init(gimbal_move_t *gimbal_move_data)
{
	static const fp32 yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	gimbal_move_data->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();

	gimbal_move_data->gimbal_yaw_motor.offset_ecd = MIDDLE_YAW;	//设置中值

	gimbal_move_data->gimbal_yaw_motor.max_relative_angle = PI;
	gimbal_move_data->gimbal_yaw_motor.min_relative_angle = -PI;	//设置yaw轴单圈限位

	gimbal_move_data->gimbal_rc_ctrl = get_remote_control_point();	//获取遥控器指针

	gimbal_move_data->gimbal_yaw_motor.gimbal_mode = GIMBAL_ZERO_FORCE;
	gimbal_move_data->gimbal_yaw_motor.gimbal_last_mode = GIMBAL_ZERO_FORCE;

	gimbal_PID_init(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_ANGLE_PID_MAX_OUT, YAW_ANGLE_PID_MAX_IOUT, YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD);
	PID_init(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

	gimbal_PID_clear(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_angle_pid);
	PID_clear(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_pid);

	gimbal_feedback_update(gimbal_move_data);

	gimbal_move_data->gimbal_yaw_motor.relative_angle_set = gimbal_move_data->gimbal_yaw_motor.relative_angle;
	gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_set = gimbal_move_data->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;


}


void gimbal_feedback_update(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}

	gimbal_move_data->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_move_data->gimbal_yaw_motor.gimbal_motor_measure->ecd, 
																					gimbal_move_data->gimbal_yaw_motor.offset_ecd);
	
}



static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}




void gimbal_mode_set(gimbal_move_t *gimbal_move_data)
{
	if (gimbal_move_data == NULL)
	{
		return;
	}


	if(switch_is_up(gimbal_move_data->gimbal_rc_ctrl->rc.s[0]))
	{
		if(gimbal_move_data->gimbal_yaw_motor.gimbal_mode != GIMBAL_INIT)
		{
			gimbal_move_data->gimbal_yaw_motor.gimbal_mode = GIMBAL_ENCODER;
		}
	}
	else
	{
		gimbal_move_data->gimbal_yaw_motor.gimbal_mode = GIMBAL_ZERO_FORCE;
	}

	
	if(gimbal_move_data->gimbal_yaw_motor.gimbal_last_mode == GIMBAL_ZERO_FORCE && 
		gimbal_move_data->gimbal_yaw_motor.gimbal_mode != GIMBAL_ZERO_FORCE)
	
	{
		gimbal_move_data->gimbal_yaw_motor.gimbal_mode = GIMBAL_INIT;
	}

	
	
	if (gimbal_move_data->gimbal_yaw_motor.gimbal_mode == GIMBAL_INIT)
	{
		if (__fabs(gimbal_move_data->gimbal_yaw_motor.relative_angle - 0.0f) < 0.1f)
		{	
			gimbal_move_data->gimbal_yaw_motor.gimbal_mode = GIMBAL_ENCODER;

		}

	}
	gimbal_move_data->gimbal_yaw_motor.gimbal_last_mode = gimbal_move_data->gimbal_yaw_motor.gimbal_mode;
}


void gimbal_mode_change_control_transit(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}

	if(gimbal_move_data->gimbal_yaw_motor.gimbal_mode == GIMBAL_ENCODER && 
		gimbal_move_data->gimbal_yaw_motor.gimbal_last_mode != GIMBAL_ENCODER)
	{
		gimbal_move_data->gimbal_yaw_motor.relative_angle_set = gimbal_move_data->gimbal_yaw_motor.relative_angle;
	}
}



void gimbal_control_loop(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}
	if(gimbal_move_data->gimbal_yaw_motor.gimbal_mode == GIMBAL_ZERO_FORCE)
	{
		gimbal_zero_force_control(gimbal_move_data);
	}
	else if(gimbal_move_data->gimbal_yaw_motor.gimbal_mode == GIMBAL_ENCODER)
	{
		gimbal_encoder_control(gimbal_move_data);
	} 
	else if(gimbal_move_data->gimbal_yaw_motor.gimbal_mode == GIMBAL_INIT)
	{
		gimbal_init_control(gimbal_move_data);
	}
}



void gimbal_zero_force_control(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}

	gimbal_move_data->gimbal_yaw_motor.give_current = 0;
}



void gimbal_encoder_control(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}
	static int16_t yaw_channel = 0;

	if(switch_is_up(gimbal_move_data->gimbal_rc_ctrl->rc.s[0]))
	{
		rc_deadband_limit(gimbal_move_data->gimbal_rc_ctrl->rc.ch[2], yaw_channel, 10);
		gimbal_move_data->gimbal_yaw_motor.relative_angle_set = yaw_channel*YAW_RC_COEFF;
		
		/*	如需限位
		if(gimbal_move_data->gimbal_yaw_motor.relative_angle_set > gimbal_move_data->gimbal_yaw_motor.max_relative_angle)
		{
			gimbal_move_data->gimbal_yaw_motor.relative_angle_set = gimbal_move_data->gimbal_yaw_motor.max_relative_angle;
		}
		else if(gimbal_move_data->gimbal_yaw_motor.relative_angle_set < gimbal_move_data->gimbal_yaw_motor.min_relative_angle)
		{
			gimbal_move_data->gimbal_yaw_motor.relative_angle_set = gimbal_move_data->gimbal_yaw_motor.min_relative_angle;
		}
		*/

	}
	else
	{
		gimbal_move_data->gimbal_yaw_motor.relative_angle_set = 0.0f;
	}
	cal_from_detla_to_current(gimbal_move_data);
}



void gimbal_init_control(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}
	gimbal_move_data->gimbal_yaw_motor.relative_angle_set = 0.0f;
	cal_from_detla_to_current(gimbal_move_data);
}



void cal_from_detla_to_current(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data ==NULL)
	{
		return;
	}
	gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_set = gimbal_PID_calc(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_angle_pid,
				gimbal_move_data->gimbal_yaw_motor.relative_angle, gimbal_move_data->gimbal_yaw_motor.relative_angle_set,gimbal_move_data->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm);
	gimbal_move_data->gimbal_yaw_motor.give_current = PID_calc(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_pid, gimbal_move_data->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm, gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_set);
	
}




const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_move_data.gimbal_yaw_motor;
}











