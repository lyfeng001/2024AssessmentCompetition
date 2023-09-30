#include "GimbalTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"

void gimbal_task(void const * argument)
{
	while(1)
	{
		CAN_cmd_chassis(500,0,0,0);
		vTaskDelay(2);
	}
	
}


const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

