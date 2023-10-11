#include "ServoTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"
#include "RemoteTask.h"
#include "tim.h"
#include "stdint.h"


void servo_control(const RC_ctrl_t *servo_rc);
int ctrl;
const RC_ctrl_t *servo_rc;


void servo_task(void const * argument)
{
	ctrl = 40;
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,ctrl);
	while(1)
	{
		servo_rc = get_remote_control_point();
		servo_control(servo_rc);
		
		vTaskDelay(2);
	}
	
}
	

void servo_control(const RC_ctrl_t *servo_rc)
{
	if(switch_is_up(servo_rc->rc.s[0]))
	{

		ctrl = ((-(float)servo_rc->rc.ch[3]+660) / 1320.0f * 40 + 20);
		if(ctrl>=60)
		{
			ctrl = 60;
		}
		else if(ctrl<=20)
		{
			ctrl = 20;
		}
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,ctrl);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,40);
	}
}


