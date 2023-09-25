#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"

void chassis_control_loop();

void chassis_task(void const * argument)
{
	while(1)
	{	
		chassis_control_loop();
		
		CAN_cmd_chassis(500,0,0,0);
		vTaskDelay(2);
	}
}


void chassis_control_loop()
{
	

}