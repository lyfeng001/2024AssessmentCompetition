#include "GimbalTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"



gimbal_move_t gimbal_move_data;

void gimbal_init(gimbal_move_t *gimbal_move_data);
void gimbal_feedback_update(gimbal_move_t *gimbal_move_data);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);





void gimbal_task(void const * argument)
{	
	gimbal_init(&gimbal_move_data);

	while(1)
	{
		
		// CAN_cmd_chassis(500,0,0,0);
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

	gimbal_move_data->gimbal_yaw_motor.gimbal_mode = GIMBAL_INIT;

	gimbal_PID_init(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_angle_pid, YAW_ANGLE_PID_MAX_OUT, YAW_ANGLE_PID_MAX_IOUT, YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD);
	PID_init(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

	gimbal_PID_clear(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_angle_pid);
	PID_clear(&gimbal_move_data->gimbal_yaw_motor.gimbal_motor_speed_pid);

	gimbal_feedback_update(gimbal_move_data);

}


void gimbal_feedback_update(gimbal_move_t *gimbal_move_data)
{
	if(gimbal_move_data == NULL)
	{
		return;
	}

	gimbal_move_data->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_move_data->gimbal_yaw_motor.gimbal_motor_measure->ecd, 
																					gimbal_move_data->gimbal_yaw_motor.offset_ecd)
	
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





const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_move_data.gimbal_yaw_motor;
}











