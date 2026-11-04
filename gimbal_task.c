#include "gimbal_task.h"

gimbal_para_t gimbal_para;

#define GIMBAL_YAW_MOTOR_ECD_SPEED_PID_KP				1500.0f
#define GIMBAL_YAW_MOTOR_ECD_SPEED_PID_KI				0.0f
#define GIMBAL_YAW_MOTOR_ECD_SPEED_PID_KD	 	     	0.0f
#define GIMBAL_YAW_MOTOR_ECD_SPEED_PID_MAX_OUT	    	25000.0f
#define GIMBAL_YAW_MOTOR_ECD_SPEED_PID_MAX_IOUT		0.0f

#define GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_KP				20.0f
#define GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_KI				0.0f
#define GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_KD	 	     	0.0f
#define GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_MAX_OUT	    	5000.0f
#define GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_MAX_IOUT		0.0f

#define GIMBAL_PITCH_MOTOR_SPEED_PID_KP				5000.0f
#define GIMBAL_PITCH_MOTOR_SPEED_PID_KI				0.0f
#define GIMBAL_PITCH_MOTOR_SPEED_PID_KD	 	     	0.0f
#define GIMBAL_PITCH_MOTOR_SPEED_PID_MAX_OUT	    	25000.0f
#define GIMBAL_PITCH_MOTOR_SPEED_PID_MAX_IOUT		000.0f

#define GIMBAL_PITCH_MOTOR_ANGLE_PID_KP				20.0f
#define GIMBAL_PITCH_MOTOR_ANGLE_PID_KI				0.0f	
#define GIMBAL_PITCH_MOTOR_ANGLE_PID_KD	 	     	0.0f
#define GIMBAL_PITCH_MOTOR_ANGLE_PID_MAX_OUT	    	5000.0f
#define GIMBAL_PITCH_MOTOR_ANGLE_PID_MAX_IOUT		0.0

#define GIMBAL_YAW_MOTOR_IMU_SPEED_PID_KP				1500.0f
#define GIMBAL_YAW_MOTOR_IMU_SPEED_PID_KI				0.0f
#define GIMBAL_YAW_MOTOR_IMU_SPEED_PID_KD	 	     	0.0f
#define GIMBAL_YAW_MOTOR_IMU_SPEED_PID_MAX_OUT	    	25000.0f
#define GIMBAL_YAW_MOTOR_IMU_SPEED_PID_MAX_IOUT			0.0f

#define GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_KP				20.0f
#define GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_KI				0.0f
#define GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_KD	 	     	0.0f
#define GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_MAX_OUT	    	500.0f
#define GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_MAX_IOUT		200.0f


static void gimbal_init(gimbal_para_t *gimbal_init_point);
static void gimbal_feed_back(gimbal_para_t *gimbal_feed_back_point);
static void gimbal_set_mode(gimbal_para_t *gimbal_set_mode_point);
static void gimbal_set_control(gimbal_para_t *gimbal_set_control_point);
static void gimbal_control_loop(gimbal_para_t *gimbal_control_loop_point);


static fp32 Limit(fp32 set, fp32 max, fp32 min)
{
	if(set>max)
		set = max;
	else if(set<min)
		set = min;
	return set;
}


void gimbal_task(void const *argument)
{
    vTaskDelay(500);
    gimbal_init(&gimbal_para);
    while(1)
    {
        gimbal_feed_back(&gimbal_para);
        gimbal_set_mode(&gimbal_para);
        gimbal_set_control(&gimbal_para);
        gimbal_control_loop(&gimbal_para);
        if(gimbal_para.gimbal_mode != GIMBAL_NO_MOVE)
        {
            CAN_cmd_gimbal_yaw(gimbal_para.gimbal_yaw_motor.give_current);
					CAN_cmd_gimbal_pitch(gimbal_para.gimbal_pitch_motor.give_current);
//				CAN_cmd_gimbal_pitch(0);
        }
        else
        {
            CAN_cmd_gimbal_yaw(0);
					
        }

        vTaskDelay(3);
    }
}

void gimbal_init(gimbal_para_t *gimbal_init_point)
{

	fp32 gimbal_yaw_motor_ecd_speed_kp_ki_kd[3] = {GIMBAL_YAW_MOTOR_ECD_SPEED_PID_KP, GIMBAL_YAW_MOTOR_ECD_SPEED_PID_KI, GIMBAL_YAW_MOTOR_ECD_SPEED_PID_KD};
	fp32 gimbal_yaw_motor_ecd_angle_kp_ki_kd[3] = {GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_KP, GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_KI, GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_KD};

	fp32 gimbal_pitch_motor_speed_kp_ki_kd[3] = {GIMBAL_PITCH_MOTOR_SPEED_PID_KP, GIMBAL_PITCH_MOTOR_SPEED_PID_KI, GIMBAL_PITCH_MOTOR_SPEED_PID_KD};
	fp32 gimbal_pitch_motor_angle_kp_ki_kd[3] = {GIMBAL_PITCH_MOTOR_ANGLE_PID_KP, GIMBAL_PITCH_MOTOR_ANGLE_PID_KI, GIMBAL_PITCH_MOTOR_ANGLE_PID_KD};

    fp32 gimbal_yaw_motor_imu_angle_kp_ki_kd[3] = {GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_KP, GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_KI, GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_KD};
	fp32 gimbal_yaw_motor_imu_speed_kp_ki_kd[3] = {GIMBAL_YAW_MOTOR_IMU_SPEED_PID_KP, GIMBAL_YAW_MOTOR_IMU_SPEED_PID_KI, GIMBAL_YAW_MOTOR_IMU_SPEED_PID_KD};

    gimbal_init_point->gimbal_remote_point = get_remote_control_point();
    gimbal_init_point->gimbal_ins_angle_point = get_INS_angle_point();

    gimbal_init_point->gimbal_yaw_motor.gimbal_motor_point = get_gimbal_motor_measure_point(0);
	gimbal_init_point->gimbal_pitch_motor.gimbal_motor_point = get_gimbal_motor_measure_point(1);

	PID_init(&gimbal_init_point->gimbal_yaw_motor_ecd_speed_pid, PID_POSITION, gimbal_yaw_motor_ecd_speed_kp_ki_kd, GIMBAL_YAW_MOTOR_ECD_SPEED_PID_MAX_OUT, GIMBAL_YAW_MOTOR_ECD_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_init_point->gimbal_yaw_motor_ecd_angle_pid, PID_POSITION, gimbal_yaw_motor_ecd_angle_kp_ki_kd, GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_MAX_OUT, GIMBAL_YAW_MOTOR_ECD_ANGLE_PID_MAX_IOUT);

	PID_init(&gimbal_init_point->gimbal_pitch_motor_speed_pid, PID_POSITION, gimbal_pitch_motor_speed_kp_ki_kd, GIMBAL_PITCH_MOTOR_SPEED_PID_MAX_OUT, GIMBAL_PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_init_point->gimbal_pitch_motor_angle_pid, PID_POSITION, gimbal_pitch_motor_angle_kp_ki_kd, GIMBAL_PITCH_MOTOR_ANGLE_PID_MAX_OUT, GIMBAL_PITCH_MOTOR_ANGLE_PID_MAX_IOUT);

	PID_init(&gimbal_init_point->gimbal_yaw_motor_imu_speed_pid, PID_POSITION, gimbal_yaw_motor_imu_speed_kp_ki_kd, GIMBAL_YAW_MOTOR_IMU_SPEED_PID_MAX_OUT, GIMBAL_YAW_MOTOR_IMU_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_init_point->gimbal_yaw_motor_imu_angle_pid, PID_POSITION, gimbal_yaw_motor_imu_angle_kp_ki_kd, GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_MAX_OUT, GIMBAL_YAW_MOTOR_IMU_ANGLE_PID_MAX_IOUT);

    gimbal_init_point->gimbal_yaw_motor.angle_set = GIMBAL_YAW_START_ANGLE;
	gimbal_init_point->gimbal_pitch_motor.angle_set = GIMBAL_PITCH_START_ANGLE;
	gimbal_init_point->gimbal_yaw_motor.angle_accumulation = 0;

}

//零点处理
fp32 gimbal_zero_crossing_operation(fp32 ref, fp32 last_ref, fp32 *angle_accumulation)
{
	//设圆上的数值顺时针递增
	if(ref-last_ref > PI) 				//逆时针过零
	{	
		ref = ref - 2.0f * PI;
	}
	else if(ref-last_ref < -PI)		//顺时针过零
	{
		ref = ref + 2.0f * PI;
	}
	*angle_accumulation += (ref - last_ref);
	return *angle_accumulation;
}

void get_yaw_ecd(gimbal_para_t *gimbal_get_yaw_real_ecd_point)
{
	gimbal_get_yaw_real_ecd_point->gimbal_yaw_motor.last_yaw_ecd = gimbal_get_yaw_real_ecd_point->gimbal_yaw_motor.yaw_ecd;
	gimbal_get_yaw_real_ecd_point->gimbal_yaw_motor.yaw_ecd = gimbal_get_yaw_real_ecd_point->gimbal_yaw_motor.gimbal_motor_point->ecd - GIMBAL_YAW_START_ECD;
	if(gimbal_get_yaw_real_ecd_point->gimbal_yaw_motor.yaw_ecd < 0)
	{
		gimbal_get_yaw_real_ecd_point->gimbal_yaw_motor.yaw_ecd += WHOLE_ROUND_6020_ECD;
	}
}


static void gimbal_feed_back(gimbal_para_t *gimbal_feed_back_point)
{
 	get_yaw_ecd(&gimbal_para);   
	gimbal_feed_back_point->gimbal_yaw_motor.angle = (fp32)gimbal_feed_back_point->gimbal_yaw_motor.yaw_ecd * 2.0f * PI / WHOLE_ROUND_6020_ECD;
    gimbal_feed_back_point->gimbal_yaw_motor.rate = (fp32)gimbal_feed_back_point->gimbal_yaw_motor.gimbal_motor_point->speed_rpm * 2.0f * PI / 60.0f;

		gimbal_feed_back_point->gimbal_pitch_motor.angle = (fp32)gimbal_feed_back_point->gimbal_pitch_motor.gimbal_motor_point->ecd * 2.0f * PI / WHOLE_ROUND_6020_ECD;
	gimbal_feed_back_point->gimbal_pitch_motor.rate = (fp32)gimbal_feed_back_point->gimbal_pitch_motor.gimbal_motor_point->speed_rpm * 2.0f * PI / 60.0f;

	
    gimbal_feed_back_point->gimbal_yaw_motor.imu_angle = *gimbal_feed_back_point->gimbal_ins_angle_point + PI;

    gimbal_feed_back_point->gimbal_yaw_motor.angle_accumulation = gimbal_zero_crossing_operation(gimbal_feed_back_point->gimbal_yaw_motor.angle, gimbal_feed_back_point->gimbal_yaw_motor.last_angle, &gimbal_feed_back_point->gimbal_yaw_motor.angle_accumulation);
	gimbal_feed_back_point->gimbal_yaw_motor.imu_accumulation = gimbal_zero_crossing_operation(gimbal_feed_back_point->gimbal_yaw_motor.imu_angle, gimbal_feed_back_point->gimbal_yaw_motor.last_imu_angle, &gimbal_feed_back_point->gimbal_yaw_motor.imu_accumulation);

	gimbal_feed_back_point->gimbal_yaw_motor.last_angle = gimbal_feed_back_point->gimbal_yaw_motor.angle;
	gimbal_feed_back_point->gimbal_yaw_motor.last_imu_angle = gimbal_feed_back_point->gimbal_yaw_motor.imu_angle;
	gimbal_feed_back_point->gimbal_yaw_motor.last_angle_set = gimbal_feed_back_point->gimbal_yaw_motor.angle_set;
}

static void gimbal_set_mode(gimbal_para_t *gimbal_set_mode_point)
{
	gimbal_set_mode_point->gimbal_last_mode = gimbal_set_mode_point->gimbal_mode;
	if(gimbal_set_mode_point->gimbal_remote_point->rc.s[0] == 2)
	{
		gimbal_set_mode_point->gimbal_mode = GIMBAL_NO_MOVE;
	}
	else if(gimbal_set_mode_point->gimbal_remote_point->rc.s[0] == 3 && gimbal_set_mode_point->gimbal_remote_point->rc.s[1] == 3)
	{
		gimbal_set_mode_point->gimbal_mode = GIMBAL_NORMAL_MOVE;
	}
	else if(gimbal_set_mode_point->gimbal_remote_point->rc.s[0] == 3 && gimbal_set_mode_point->gimbal_remote_point->rc.s[1] == 2)
	{
		gimbal_set_mode_point->gimbal_mode = GIMBAL_FOLLOW_YAW;
	}
	else if(gimbal_set_mode_point->gimbal_remote_point->rc.s[0] == 3 && gimbal_set_mode_point->gimbal_remote_point->rc.s[1] == 1)
	{
		gimbal_set_mode_point->gimbal_mode = GIMBAL_SPIN;
	}
	else if(gimbal_set_mode_point->gimbal_remote_point->rc.s[0] == 1 && gimbal_set_mode_point->gimbal_remote_point->rc.s[1] == 1)
	{
		gimbal_set_mode_point->gimbal_mode = GIMBAL_NAVIGATION;
	}
	else
	{
		gimbal_set_mode_point->gimbal_mode = GIMBAL_NO_MOVE;
	}

}

//过冲滤波
void over_shoot_solve(fp32 *angle_set_ouput,fp32 last_angle_set,fp32 angle_ref,fp32 last_angle_ref)
{
	if(*angle_set_ouput < last_angle_set && angle_ref - last_angle_ref >= OVERSHOOT_DEAD_ZONE)
		*angle_set_ouput = angle_ref;
	else if(*angle_set_ouput > last_angle_set && angle_ref - last_angle_ref <= -OVERSHOOT_DEAD_ZONE)
		*angle_set_ouput = angle_ref;
}

void gimbal_mode_change_reset(gimbal_para_t *gimbal_mode_change_reset_point)
{
	if(gimbal_mode_change_reset_point->gimbal_mode != gimbal_mode_change_reset_point->gimbal_last_mode)
	{
		gimbal_mode_change_reset_point->gimbal_yaw_motor.angle_accumulation = gimbal_mode_change_reset_point->gimbal_yaw_motor.angle;
		gimbal_mode_change_reset_point->gimbal_yaw_motor.imu_accumulation = gimbal_mode_change_reset_point->gimbal_yaw_motor.imu_angle;
		gimbal_mode_change_reset_point->gimbal_yaw_motor.angle_set = gimbal_mode_change_reset_point->gimbal_yaw_motor.angle_accumulation;
		gimbal_mode_change_reset_point->gimbal_yaw_motor.imu_angle_set = gimbal_mode_change_reset_point->gimbal_yaw_motor.imu_angle;
	}
}

void gimbal_set_mode_no_move(gimbal_para_t *gimbal_set_mode_no_move_point)
{
	gimbal_set_mode_no_move_point->gimbal_mode = GIMBAL_NO_MOVE;
}

void gimbal_set_mode_normal_move(gimbal_para_t *gimbal_set_mode_normal_move_point)
{
	gimbal_mode_change_reset(gimbal_set_mode_normal_move_point);
	gimbal_set_mode_normal_move_point->gimbal_yaw_motor.angle_set += YAW_DIR * (fp32)gimbal_set_mode_normal_move_point->gimbal_remote_point->rc.ch[0] / 660.0f * YAW_SENSITIVITY;
	gimbal_set_mode_normal_move_point->gimbal_pitch_motor.angle_set += PITCH_DIR * (fp32)gimbal_set_mode_normal_move_point->gimbal_remote_point->rc.ch[1] / 660.0f * PITCH_SENSITIVITY;
	gimbal_set_mode_normal_move_point->gimbal_pitch_motor.angle_set = Limit(gimbal_set_mode_normal_move_point->gimbal_pitch_motor.angle_set, GIMBAL_PITCH_MAX_ANGLE, GIMBAL_PITCH_MIN_ANGLE);

	over_shoot_solve(&gimbal_set_mode_normal_move_point->gimbal_yaw_motor.angle_set,
	gimbal_set_mode_normal_move_point->gimbal_yaw_motor.last_angle_set,
	gimbal_set_mode_normal_move_point->gimbal_yaw_motor.angle,
	gimbal_set_mode_normal_move_point->gimbal_yaw_motor.last_angle);
}

void gimbal_follow_yaw_control_loop(gimbal_para_t *gimbal_follow_yaw_control_loop_point)		//陀螺仪写法
{
	gimbal_mode_change_reset(gimbal_follow_yaw_control_loop_point);
	gimbal_follow_yaw_control_loop_point->gimbal_yaw_motor.imu_angle_set += YAW_DIR * (fp32)gimbal_follow_yaw_control_loop_point->gimbal_remote_point->rc.ch[0] / 660.0f * YAW_SENSITIVITY;
	gimbal_follow_yaw_control_loop_point->gimbal_pitch_motor.angle_set += PITCH_DIR * (fp32)gimbal_follow_yaw_control_loop_point->gimbal_remote_point->rc.ch[1] / 660.0f * PITCH_SENSITIVITY;
	gimbal_follow_yaw_control_loop_point->gimbal_pitch_motor.angle_set = Limit(gimbal_follow_yaw_control_loop_point->gimbal_pitch_motor.angle_set, GIMBAL_PITCH_MAX_ANGLE, GIMBAL_PITCH_MIN_ANGLE);

}

void gimbal_navigation_control_loop(gimbal_para_t *gimbal_navigation_control_loop_point)
{
	gimbal_mode_change_reset(gimbal_navigation_control_loop_point);
	gimbal_navigation_control_loop_point->gimbal_yaw_motor.imu_angle_set += YAW_DIR * (fp32)gimbal_navigation_control_loop_point->gimbal_remote_point->rc.ch[0] / 660.0f * YAW_SENSITIVITY;
	gimbal_navigation_control_loop_point->gimbal_pitch_motor.angle_set += PITCH_DIR * (fp32)gimbal_navigation_control_loop_point->gimbal_remote_point->rc.ch[1] / 660.0f * PITCH_SENSITIVITY;
	gimbal_navigation_control_loop_point->gimbal_pitch_motor.angle_set = Limit(gimbal_navigation_control_loop_point->gimbal_pitch_motor.angle_set, GIMBAL_PITCH_MAX_ANGLE, GIMBAL_PITCH_MIN_ANGLE);
}

static void gimbal_set_control(gimbal_para_t *gimbal_set_control_point)
{
	switch(gimbal_set_control_point->gimbal_mode)
	{
		case GIMBAL_NO_MOVE:
		gimbal_set_mode_no_move(&gimbal_para);
			break;
		case GIMBAL_NORMAL_MOVE:
			gimbal_set_mode_normal_move(&gimbal_para);
			break;
		case GIMBAL_FOLLOW_YAW:
			gimbal_follow_yaw_control_loop(&gimbal_para);
			break;
		case GIMBAL_SPIN:
			gimbal_follow_yaw_control_loop(&gimbal_para);
			break;
		case GIMBAL_NAVIGATION:
			gimbal_navigation_control_loop(&gimbal_para);
			break;
		default:
			break;
	}
}

static void gimbal_control_loop(gimbal_para_t *gimbal_control_loop_point)
{
	if(gimbal_control_loop_point->gimbal_mode == GIMBAL_NORMAL_MOVE)
	{
	gimbal_control_loop_point->gimbal_yaw_motor.rate_set = PID_calc(&gimbal_control_loop_point->gimbal_yaw_motor_ecd_angle_pid,
																	 gimbal_control_loop_point->gimbal_yaw_motor.angle_accumulation,
																 	 gimbal_control_loop_point->gimbal_yaw_motor.angle_set);
	gimbal_control_loop_point->gimbal_yaw_motor.give_current = PID_calc(&gimbal_control_loop_point->gimbal_yaw_motor_ecd_speed_pid,
																		 gimbal_control_loop_point->gimbal_yaw_motor.rate,
																		 gimbal_control_loop_point->gimbal_yaw_motor.rate_set);
		if(fabs(gimbal_control_loop_point->gimbal_yaw_motor.rate - gimbal_control_loop_point->gimbal_yaw_motor.rate_set) <= 1.3)
			gimbal_control_loop_point->gimbal_yaw_motor.give_current = 0;
	}
	else if(gimbal_control_loop_point->gimbal_mode == GIMBAL_FOLLOW_YAW || gimbal_control_loop_point->gimbal_mode == GIMBAL_SPIN)
	{
	gimbal_control_loop_point->gimbal_yaw_motor.rate_set = PID_calc(&gimbal_control_loop_point->gimbal_yaw_motor_imu_angle_pid,
																	 gimbal_control_loop_point->gimbal_yaw_motor.imu_accumulation,
																 	 gimbal_control_loop_point->gimbal_yaw_motor.imu_angle_set);
	gimbal_control_loop_point->gimbal_yaw_motor.give_current = PID_calc(&gimbal_control_loop_point->gimbal_yaw_motor_imu_speed_pid,
																		 gimbal_control_loop_point->gimbal_yaw_motor.rate,
																		 gimbal_control_loop_point->gimbal_yaw_motor.rate_set);
				if(fabs(gimbal_control_loop_point->gimbal_yaw_motor.rate - gimbal_control_loop_point->gimbal_yaw_motor.rate_set) <= 1.3)
			gimbal_control_loop_point->gimbal_yaw_motor.give_current = 0;
	}

	gimbal_control_loop_point->gimbal_pitch_motor.rate_set = PID_calc(&gimbal_control_loop_point->gimbal_pitch_motor_angle_pid,
																	 gimbal_control_loop_point->gimbal_pitch_motor.angle,
																 	 gimbal_control_loop_point->gimbal_pitch_motor.angle_set);

	gimbal_control_loop_point->gimbal_pitch_motor.give_current = PID_calc(&gimbal_control_loop_point->gimbal_pitch_motor_speed_pid,
																			 gimbal_control_loop_point->gimbal_pitch_motor.rate,
																			 gimbal_control_loop_point->gimbal_pitch_motor.rate_set);
}



fp32 get_gimbal_yaw_err_angle(void)
{
	return gimbal_para.gimbal_yaw_motor.angle_accumulation;
}

gimbal_para_t *get_gimbal_para_point(void)
{
	return &gimbal_para;
}

uint8_t get_gimbal_mode(void)
{
	return gimbal_para.gimbal_mode;
}

