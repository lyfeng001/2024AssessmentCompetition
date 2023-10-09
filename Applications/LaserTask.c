#include "LaserTask.h"
#include "main.h"
#include "cmsis_os.h"
#include "RemoteTask.h"


void laser_control(const RC_ctrl_t *laser_rc);
void laser_flick(void);


void laser_task(void const * argument)
{

	while(1)
	{
		const RC_ctrl_t *laser_rc = get_remote_control_point();
		laser_control(laser_rc);
		vTaskDelay(2);
	}

}

void laser_control(const RC_ctrl_t *laser_rc)
{
	if(switch_is_up(laser_rc->rc.s[0]))
	{
		if(switch_is_up(laser_rc->rc.s[1]))
		{
			laser_flick();
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	}
}



void laser_flick()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	vTaskDelay(200);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	vTaskDelay(200);
}






