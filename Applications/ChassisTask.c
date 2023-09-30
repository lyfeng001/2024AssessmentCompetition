#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"
#include "pid.h"

chassis_move_t chassis_move_data;
void chassis_init(chassis_move_t* chassis_move_data);
void chassis_mode_set(chassis_move_t* chassis_move_data);
void chassis_feedback_update(chassis_move_t* chassis_move_data);
void chassis_control_loop();

void chassis_task(void const * argument)
{
	chassis_init(&chassis_move_data);

	while(1)
	{	
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
	
}
void chassis_control_loop()
{
	

}

