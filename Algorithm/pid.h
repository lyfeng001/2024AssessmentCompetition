#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "struct_typedef.h"



//����pidģʽ ����ʽ��λ��ʽ
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

/***�������***/	
		fp32 dT;
		fp32 max_dout;
} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

extern void PID_clear(pid_type_def *pid);


typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;
		fp32 last_err;
	
    fp32 max_out;
    fp32 max_iout;
		fp32 max_dout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
		fp32 dT;
} gimbal_PID_t;

extern void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
extern fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

extern void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear);





#endif


