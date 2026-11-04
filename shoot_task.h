#ifndef _SHOOT_TASK_H
#define _SHOOT_TASK_H

#include "remote_control.h"
#include "main.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "pid.h"

#define WHOLE_ROUND_2006_ECD        8192
#define HALF_ROUND_2006_ECD         4096

#define MOTOR_2006_RATIO            36

#define FRIC_RADIUS							0.03f		//m
#define BULLET_RADIUS						0.008f		//m
#define BULLET_SPEED						22.0f		//m/s

#define FRIC_RATE							(BULLET_SPEED/(FRIC_RADIUS+BULLET_RADIUS))		//rad/s

#define TRIGGER_CELL                8

#define FRIC_ON_MS				500     //ms
#define SHOOT_FREQUENCY			3     //HZ

typedef enum
{
	FRIC_OFF =0,
	FRIC_ON,
}fric_state_e;

typedef enum
{
	TRIGGER_OFF =0,
	TRIGGER_GO,
	TRIGGER_WAIT,
	TRIGGER_LOCKED_ROTOR,
}trigger_state_e;

typedef struct
{
    const motor_measure_t	*fric_motor_point;

    fp32 speed;
    fp32 speed_set;
    int16_t give_current;

}fric_motor_t;

typedef struct
{
    const motor_measure_t	*trigger_motor_point;

    fp32 give_current;

    fp32 last_angle;
    fp32 angle;
    fp32 angle_set; 
    fp32 angle_accumulation;    

    fp32 rate;
    fp32 rate_set;

}trigger_motor_t;

typedef struct
{
	const RC_ctrl_t			*shoot_remote_point;
	
	fric_state_e fric_state;		// 0 1
	trigger_state_e trigger_state;	// 0 1 2

//	uint8_t aimbot_state;			//����״̬
//	uint8_t aimbot_mode;			//����ģʽ����/�ر�
	
	fp32 trigger_angle;
	
	fric_motor_t fric_motor_para[2];
	trigger_motor_t trigger_motor_para;
	
	pid_type_def fric_speed_pid[2];

	pid_type_def trigger_speed_pid;
	pid_type_def trigger_angle_pid;

}shoot_para_t;

void shoot_task(void const *argument);

#endif

