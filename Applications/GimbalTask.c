#include "GimbalTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"



gimbal_move_t gimbal_move_data;

void gimbal_init(gimbal_move_t *gimbal_move_data);







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
	
}








const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}











