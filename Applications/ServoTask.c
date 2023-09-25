#include "ServoTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"

void servo_task(void const * argument)
{
	while(1)
	{
		CAN_cmd_chassis(500,0,0,0);
		vTaskDelay(2);
	}
	
}
	

