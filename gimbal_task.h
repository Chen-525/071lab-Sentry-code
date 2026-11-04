#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "can_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "INS_task.h"
#include "user_lib.h"
#include "arm_math.h"

#define GIMBAL_YAW_START_ECD		5436
#define GIMBAL_PITCH_START_ECD		7394

#define GIMBAL_YAW_START_ANGLE				0
#define GIMBAL_PITCH_START_ANGLE			GIMBAL_PITCH_START_ECD * 2.0f * PI / WHOLE_ROUND_6020_ECD

#define GIMBAL_PITCH_MAX_ANGLE				5.9687198f
#define GIMBAL_PITCH_MIN_ANGLE				5.43489408f

#define WHOLE_ROUND_6020_ECD        8192.0f
#define HALF_ROUND_6020_ECD         4096.0f
#define QUARTER_ROUND_6020_ECD      2048.0f


#define YAW_SENSITIVITY				0.015f
#define PITCH_SENSITIVITY			0.003f

#define OVERSHOOT_DEAD_ZONE     0.00001f  //rad

#define YAW_DIR               -1    //YAW转向方向
#define PITCH_DIR             -1     //PITCH转向方向

typedef enum
{
	GIMBAL_NO_MOVE = 0,
	GIMBAL_NORMAL_MOVE,
	GIMBAL_FOLLOW_YAW,
	GIMBAL_SPIN,
}gimbal_mode_e;

typedef struct
{	
	const motor_measure_t	*gimbal_motor_point;

	fp32 acc;
	fp32 rate;		//rad/s
	fp32 rate_set;

	fp32 angle;		//rad
	fp32 last_angle;
	fp32 angle_set;
	fp32 last_angle_set;

	fp32 angle_accumulation;	//rad 过零计算后的实际角度

	fp32 give_current;

	int16_t yaw_grear_ecd;      //YAW经过减速比后的虚拟编码器 （头朝向为零点，0-16384）
	int16_t last_yaw_grear_ecd;  //YAW经过减速比后的虚拟编码器上一次值
	int16_t yaw_ecd;			 //YAW6020初步处理
	int16_t last_yaw_ecd;		 //YAW6020初步处理上一次值

	fp32 imu_angle_set;	//rad
	fp32 last_imu_angle_set;
	fp32 imu_angle;		//rad
	fp32 last_imu_angle;	//rad
	fp32 imu_accumulation;	//rad 过零计算后的实际角度
}gimbal_motor_t;

typedef struct
{
	const RC_ctrl_t			*gimbal_remote_point;
	const fp32				*gimbal_ins_angle_point;

	gimbal_motor_t			gimbal_yaw_motor;
	gimbal_motor_t			gimbal_pitch_motor;
	gimbal_mode_e 			gimbal_mode;
	gimbal_mode_e			gimbal_last_mode;

	pid_type_def			gimbal_yaw_motor_ecd_speed_pid;
	pid_type_def			gimbal_yaw_motor_ecd_angle_pid;

	pid_type_def			gimbal_yaw_motor_imu_speed_pid;
	pid_type_def			gimbal_yaw_motor_imu_angle_pid;

	pid_type_def			gimbal_pitch_motor_speed_pid;
	pid_type_def			gimbal_pitch_motor_angle_pid;

}gimbal_para_t;

void gimbal_task(void const * argument);

fp32 gimbal_zero_crossing_operation(fp32 ref, fp32 last_ref, fp32 *angle_accumulation);
gimbal_para_t *get_gimbal_para_point(void);
uint8_t get_gimbal_mode(void);
fp32 get_gimbal_yaw_err_angle(void);

#endif