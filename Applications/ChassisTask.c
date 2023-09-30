#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"


chassis_move_t chassis_move_data;
void chassis_init(chassis_move_t* chassis_move_data);
void chassis_mode_set(chassis_move_t* chassis_move_data);

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
	

}
void chassis_mode_set(chassis_move_t* chassis_move_data)
{

}
void chassis_control_loop()
{
	

}

