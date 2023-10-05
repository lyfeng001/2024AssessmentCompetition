#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stdint.h"


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t last_ecd;
	int16_t ecd_count;
	int16_t given_current;
} motor_measure_t; 


typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
	CAN_GIMBAL_ALL_ID = 0x1FF,
	CAN_2006_M1_ID = 0x201,
    CAN_2006_M2_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
    CAN_2006_M4_ID = 0x204,
    CAN_YAW_MOTOR_ID = 0x205,


} can1_msg_id_e;

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_gimbal(int16_t yaw);
extern const motor_measure_t *get_chassis_motor_meature_point(uint8_t i);
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);


#endif


