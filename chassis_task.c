#include "chassis_task.h"
#include "can_receive.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "math.h"
#include "arm_math.h"
#include "usart_receive.h"

#define M3508_MOTOR_SPEED_PID_KP 						15000.0f
#define M3508_MOTOR_SPEED_PID_KI 						0.00f
#define M3508_MOTOR_SPEED_PID_KD		  		  0.0f
#define M3508_MOTOR_SPEEED_PID_MAX_OUT 				25000.0f
#define M3508_MOTOR_SPEEED_PID_MAX_IOUT 		0.00  

#define    CHASSIS_FOLLOW_YAW_PID_KP				0.8f
#define    CHASSIS_FOLLOW_YAW_PID_KI				0.0f
#define    CHASSIS_FOLLOW_YAW_PID_KD				0.0f
#define    CHASSIS_FOLLOW_YAW_PID_MAX_OUT			40.0f
#define    CHASSIS_FOLLOW_YAW_PID_MAX_IOUT		0.0f


chassis_move_t chassis_move; 
kalman_1d_t kf_vx, kf_vy, kf_wz;

static void chassis_init(chassis_move_t *chassis_init);
static void chassis_feedback_update(chassis_move_t *chassis_feed_back);
static void chassis_speed_set(chassis_move_t *chassis_speed_set);
static void chassis_control_loop(chassis_move_t *chassis_control_loop);
static void chassis_mode_set(chassis_move_t *chassis_mode_set);
static void chassis_set_ctrl(chassis_move_t *chassis_set_ctrl);

void chassis_task(void const *argument)
{
		vTaskDelay(500);
		chassis_init(&chassis_move);
	while(1)
	{	
		
		chassis_feedback_update(&chassis_move);
		chassis_mode_set(&chassis_move);
		chassis_set_ctrl(&chassis_move);
		chassis_control_loop(&chassis_move);

		if(chassis_move.chassis_mode != CHASSIS_NO_MOVE)
		CAN_cmd_chassis_motor(chassis_move.motor_chassis[0].give_current,
													chassis_move.motor_chassis[1].give_current,
													chassis_move.motor_chassis[2].give_current,
													chassis_move.motor_chassis[3].give_current);
		else
		CAN_cmd_chassis_motor(0,
													0,
													0,
													0);
		vTaskDelay(1);
	}
	
}

static void chassis_init(chassis_move_t *chassis_init)
{
	uint8_t i ;
	chassis_init->chassis_RC = get_remote_control_point();
	chassis_init->chassis_navigation_data = get_navigation_point();
	chassis_init->gimbal_para_point = get_gimbal_para_point();

 	const fp32 chassis_motor_kp_ki_kd[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
  	const fp32 chassis_follow_yaw_kp_ki_kd[3] = {CHASSIS_FOLLOW_YAW_PID_KP, CHASSIS_FOLLOW_YAW_PID_KI, CHASSIS_FOLLOW_YAW_PID_KD};

	for(i = 0; i < 4;i++)
	{
		chassis_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_init->motor_speed_pid[i],PID_POSITION,chassis_motor_kp_ki_kd,M3508_MOTOR_SPEEED_PID_MAX_OUT,M3508_MOTOR_SPEEED_PID_MAX_IOUT);
	}
	PID_init(&chassis_init->chassis_follow_yaw_pid,PID_POSITION,chassis_follow_yaw_kp_ki_kd,CHASSIS_FOLLOW_YAW_PID_MAX_OUT,CHASSIS_FOLLOW_YAW_PID_MAX_IOUT);
}
static void chassis_mode_set(chassis_move_t *chassis_mode_set)
{
//	if(chassis_mode_set->chassis_RC->rc.s[0] == 2)
//	  chassis_mode_set->chassis_mode = CHASSIS_NO_MOVE;
//	else if(chassis_mode_set->chassis_RC->rc.s[0] == 3 && chassis_mode_set->chassis_RC->rc.s[1] == 2)
//		chassis_mode_set->chassis_mode = CHASSIS_MOVE_NORMAL;
//	else if(chassis_mode_set->chassis_RC->rc.s[0] == 3 && chassis_mode_set->chassis_RC->rc.s[1] == 3)
//		chassis_mode_set->chassis_mode = CHASSIS_NAVIGATION;
//		
	
	chassis_mode_set->chassis_mode = get_gimbal_mode();
	
}


static void chassis_feedback_update(chassis_move_t *chassis_feed_back)
{
	uint8_t i;
	for(i = 0; i < 4 ; i++)
	{
		chassis_feed_back->motor_chassis[i].speed = PI * OMNI_WHEEL_RADIOS * chassis_feed_back->motor_chassis[i].chassis_motor_measure->speed_rpm /60.0f / M3508_RATIO;
//				chassis_feed_back->motor_chassis[i].speed = chassis_feed_back->motor_chassis[i].chassis_motor_measure->speed_rpm;

		chassis_feed_back->motor_chassis[i].accel = chassis_feed_back->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

}	

//底盘普通模式运动解算
static void chassis_move_normal(chassis_move_t *chassis_move_normal)
{
	
	chassis_move_normal->vx = (fp32)chassis_move_normal->chassis_RC->rc.ch[2] / 660.0f * CHASSIS_SPEED_MAX;
	chassis_move_normal->vy = (fp32)chassis_move_normal->chassis_RC->rc.ch[3] / 660.0f * CHASSIS_SPEED_MAX;
//	chassis_move_normal->wz = (fp32)chassis_move_normal->chassis_RC->rc.ch[0] / 660.0f * CHASSIS_TURNING_RATE_MAX;
	chassis_move_normal->wz = 0;
}

fp32 speed_limit = 5000;
	
static void chassis_move_navigation(chassis_move_t *chassis_move_navigation)
{

	speed_limit += chassis_move_navigation->chassis_RC->rc.ch[1] * 0.0008;
	chassis_move_navigation->vx = chassis_move_navigation->chassis_navigation_data->vx * speed_limit;
		chassis_move_navigation->vy = -chassis_move_navigation->chassis_navigation_data->vy * speed_limit;
	chassis_move_navigation->wz = chassis_move_navigation->chassis_navigation_data->wz * speed_limit;
	
}

void chassis_set_mode_no_move(chassis_move_t *chassis_set_mode_no_move_point)
{
	chassis_set_mode_no_move_point->chassis_mode = CHASSIS_NO_MOVE;
}

void chassis_set_mode_follow_yaw(chassis_move_t *chassis_set_mode_follow_yaw_point)
{
	chassis_set_mode_follow_yaw_point->vx = (fp32)chassis_set_mode_follow_yaw_point->chassis_RC->rc.ch[2] / 660.0f * CHASSIS_SPEED_MAX;
	chassis_set_mode_follow_yaw_point->vy = (fp32)chassis_set_mode_follow_yaw_point->chassis_RC->rc.ch[3] / 660.0f * CHASSIS_SPEED_MAX;
	chassis_set_mode_follow_yaw_point->wz = PID_calc(&chassis_set_mode_follow_yaw_point->chassis_follow_yaw_pid, get_gimbal_yaw_err_angle(), 0);
}

void chassis_set_mode_spin(chassis_move_t *chassis_set_mode_spin_point)
{
	chassis_set_mode_spin_point->vx = (fp32)chassis_set_mode_spin_point->chassis_RC->rc.ch[2] / 660.0f * CHASSIS_SPEED_MAX;
	chassis_set_mode_spin_point->vy = (fp32)chassis_set_mode_spin_point->chassis_RC->rc.ch[3] / 660.0f * CHASSIS_SPEED_MAX;
	chassis_set_mode_spin_point->wz = CHASSIS_SPIN_RATE;
}

void chassis_set_mode_navigation(chassis_move_t *chassis_set_mode_navigation_point)
{
	chassis_set_mode_navigation_point->vx = chassis_set_mode_navigation_point->chassis_navigation_data->vx * CHASSIS_NAVIGATION_SPEED;
	chassis_set_mode_navigation_point->vy = -chassis_set_mode_navigation_point->chassis_navigation_data->vy * CHASSIS_NAVIGATION_SPEED;
	chassis_set_mode_navigation_point->wz = chassis_set_mode_navigation_point->chassis_navigation_data->wz * CHASSIS_TURNING_RATE_MAX;
}

static void chassis_set_ctrl(chassis_move_t *chassis_set_ctrl)
{
	switch(chassis_set_ctrl->chassis_mode)
	{
		case CHASSIS_NO_MOVE:
		chassis_set_mode_no_move(&chassis_move);
			break;		
		break;
		case CHASSIS_NORMAL_MOVE:
			chassis_move_normal(&chassis_move);
			break;
		case CHASSIS_FOLLOW_YAW:
			chassis_set_mode_follow_yaw(&chassis_move);
			break;
		case CHASSIS_SPIN:
			chassis_set_mode_spin(&chassis_move);
			break;
		case CHASSIS_NAVIGATION:
			chassis_move_navigation(&chassis_move);
			break;
		default:
			break;
	}
}

static void chassis_control_loop(chassis_move_t *chassis_control_loop)
{

		chassis_control_loop->vx_set = sqrt(pow(chassis_control_loop->vx, 2) + pow(chassis_control_loop->vy, 2)) * 
	cos(atan2f(chassis_control_loop->vy, chassis_control_loop->vx) - PI/2 + 
		chassis_control_loop->gimbal_para_point->gimbal_yaw_motor.angle);
	chassis_control_loop->vy_set = sqrt(pow(chassis_control_loop->vx, 2) + pow(chassis_control_loop->vy, 2)) *
	sin(atan2f(chassis_control_loop->vy, chassis_control_loop->vx) - PI/2 + 
		chassis_control_loop->gimbal_para_point->gimbal_yaw_motor.angle);
	chassis_control_loop->wz_set = chassis_control_loop->wz;

	chassis_control_loop->motor_chassis[3].speed_set = -(chassis_control_loop->vx_set * sin(PI/4) - chassis_control_loop->vy_set * cos(PI/4) - chassis_control_loop->wz_set);
	chassis_control_loop->motor_chassis[2].speed_set = (chassis_control_loop->vx_set * cos(PI/4) + chassis_control_loop->vy_set * sin(PI/4) + chassis_control_loop->wz_set);
	chassis_control_loop->motor_chassis[1].speed_set = (chassis_control_loop->vx_set * sin(PI/4) - chassis_control_loop->vy_set * cos(PI/4) + chassis_control_loop->wz_set);
	chassis_control_loop->motor_chassis[0].speed_set = -(chassis_control_loop->vx_set * cos(PI/4) + chassis_control_loop->vy_set * sin(PI/4) - chassis_control_loop->wz_set);

	uint8_t i;
	for(i = 0;i < 4;i++)
	{
		chassis_control_loop->motor_chassis[i].give_current = PID_calc(&chassis_control_loop->motor_speed_pid[i],
																		chassis_control_loop->motor_chassis[i].speed,
																		chassis_control_loop->motor_chassis[i].speed_set);
	}
}
