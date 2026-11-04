#include "shoot_task.h"

shoot_para_t shoot_para;


#define GIMBAL_FRIC_MOTOR_SPEED_KP			250.0f
#define GIMBAL_FRIC_MOTOR_SPEED_KI			0.8f
#define GIMBAL_FRIC_MOTOR_SPEED_KD			0.0f
#define GIMBAL_FRIC_MOTOR_SPEED_MAX_OUT		30000.0f
#define GIMBAL_FRIC_MOTOR_SPEED_MAX_IOUT	20.0f

#define GIMBAL_TRIGGER_MOTOR_ANGLE_KP			50.0f
#define GIMBAL_TRIGGER_MOTOR_ANGLE_KI			0.0f
#define GIMBAL_TRIGGER_MOTOR_ANGLE_KD			0.0f
#define GIMBAL_TRIGGER_MOTOR_ANGLE_MAX_OUT		300.0f
#define GIMBAL_TRIGGER_MOTOR_ANGLE_MAX_IOUT		0.0f

#define GIMBAL_TRIGGER_MOTOR_SPEED_KP			20.0f
#define GIMBAL_TRIGGER_MOTOR_SPEED_KI			0.0f
#define GIMBAL_TRIGGER_MOTOR_SPEED_KD			0.0f
#define GIMBAL_TRIGGER_MOTOR_SPEED_MAX_OUT		20000.0f
#define GIMBAL_TRIGGER_MOTOR_SPEED_MAX_IOUT		0.0f

static void shoot_init(shoot_para_t *shoot_init_point);
static void shoot_feed_back(shoot_para_t *shoot_feed_back_point);
static void shoot_state(shoot_para_t *shoot_state_point);
static void shoot_set_control_loop(shoot_para_t *shoot_set_control_loop_point);


void shoot_task(void const *argument)
{
    vTaskDelay(500);
    shoot_init(&shoot_para);
    while(1)
    {
        shoot_feed_back(&shoot_para);
        shoot_state(&shoot_para);
        shoot_set_control_loop(&shoot_para);
        if(shoot_para.shoot_remote_point->rc.s[0] != 2)
        CAN_cmd_shoot(shoot_para.trigger_motor_para.give_current,
            shoot_para.fric_motor_para[0].give_current, 
            shoot_para.fric_motor_para[1].give_current
            );
        else
            CAN_cmd_shoot(0, 0, 0);
        vTaskDelay(3);
    }

}

static void shoot_init(shoot_para_t *shoot_init_point)
{
    const fp32 fric_motor_speed_kp_ki_kd[3] = {GIMBAL_FRIC_MOTOR_SPEED_KP, GIMBAL_FRIC_MOTOR_SPEED_KI, GIMBAL_FRIC_MOTOR_SPEED_KD};

    const fp32 trigger_motor_angle_kp_ki_kd[3] = {GIMBAL_TRIGGER_MOTOR_ANGLE_KP, GIMBAL_TRIGGER_MOTOR_ANGLE_KI, GIMBAL_TRIGGER_MOTOR_ANGLE_KD};
    const fp32 trigger_motor_speed_kp_ki_kd[3] = {GIMBAL_TRIGGER_MOTOR_SPEED_KP, GIMBAL_TRIGGER_MOTOR_SPEED_KI, GIMBAL_TRIGGER_MOTOR_SPEED_KD};
    
    shoot_init_point->shoot_remote_point = get_remote_control_point();
    
    shoot_init_point->trigger_motor_para.trigger_motor_point = get_shoot_motor_measure_point(0);
    shoot_init_point->fric_motor_para[0].fric_motor_point = get_shoot_motor_measure_point(1);
    shoot_init_point->fric_motor_para[1].fric_motor_point = get_shoot_motor_measure_point(2);

    PID_init(&shoot_init_point->fric_speed_pid[0], PID_POSITION, fric_motor_speed_kp_ki_kd, GIMBAL_FRIC_MOTOR_SPEED_MAX_OUT, GIMBAL_FRIC_MOTOR_SPEED_MAX_IOUT);
    PID_init(&shoot_init_point->fric_speed_pid[1], PID_POSITION, fric_motor_speed_kp_ki_kd, GIMBAL_FRIC_MOTOR_SPEED_MAX_OUT, GIMBAL_FRIC_MOTOR_SPEED_MAX_IOUT);
    PID_init(&shoot_init_point->trigger_angle_pid, PID_POSITION, trigger_motor_angle_kp_ki_kd, GIMBAL_TRIGGER_MOTOR_ANGLE_MAX_OUT, GIMBAL_TRIGGER_MOTOR_ANGLE_MAX_IOUT);
    PID_init(&shoot_init_point->trigger_speed_pid, PID_POSITION, trigger_motor_speed_kp_ki_kd, GIMBAL_TRIGGER_MOTOR_SPEED_MAX_OUT, GIMBAL_TRIGGER_MOTOR_SPEED_MAX_IOUT);
}

static void shoot_feed_back(shoot_para_t *shoot_feed_back_point)
{
    shoot_feed_back_point->trigger_motor_para.angle = (fp32)shoot_feed_back_point->trigger_motor_para.trigger_motor_point->ecd * 2.0f * PI / WHOLE_ROUND_2006_ECD;
    shoot_feed_back_point->trigger_motor_para.rate = (fp32)shoot_feed_back_point->trigger_motor_para.trigger_motor_point->speed_rpm * 2.0f * PI / 60.0f;

    shoot_feed_back_point->trigger_motor_para.angle_accumulation = gimbal_zero_crossing_operation(shoot_feed_back_point->trigger_motor_para.angle, 
																																		shoot_feed_back_point->trigger_motor_para.last_angle,
																																	&shoot_feed_back_point->trigger_motor_para.angle_accumulation);
    shoot_feed_back_point->trigger_motor_para.last_angle = shoot_feed_back_point->trigger_motor_para.angle;

    shoot_feed_back_point->fric_motor_para[0].speed = (fp32)shoot_feed_back_point->fric_motor_para[0].fric_motor_point->speed_rpm * 2.0f * PI / 60.0f;
    shoot_feed_back_point->fric_motor_para[1].speed = (fp32)shoot_feed_back_point->fric_motor_para[1].fric_motor_point->speed_rpm * 2.0f * PI / 60.0f;
}

uint32_t trigger_start_ms,fric_start_ms;


static void shoot_state(shoot_para_t *shoot_state_point)
{
    switch(shoot_state_point->fric_state)
    {
        case FRIC_OFF:
            shoot_state_point->fric_motor_para[0].speed_set = 0;
            shoot_state_point->fric_motor_para[1].speed_set = 0;
						shoot_state_point->trigger_motor_para.angle_set = shoot_state_point->trigger_motor_para.angle_accumulation;
            if(shoot_state_point->shoot_remote_point->rc.ch[4] >= 330.0f)
						{
							shoot_state_point->fric_state = FRIC_ON;
							shoot_state_point->trigger_state = TRIGGER_GO;
						}
							break;
        case FRIC_ON:
            shoot_state_point->fric_motor_para[0].speed_set = FRIC_RATE;
            shoot_state_point->fric_motor_para[1].speed_set = FRIC_RATE;
        switch (shoot_state_point->trigger_state)
        {
            case TRIGGER_GO:
            shoot_state_point->trigger_motor_para.angle_set += 2 * PI / TRIGGER_CELL * 36;
            trigger_start_ms = HAL_GetTick();
						shoot_state_point->trigger_state = TRIGGER_WAIT;
            break;
            case TRIGGER_WAIT:
            if(HAL_GetTick() - trigger_start_ms >= 1000 / SHOOT_FREQUENCY && shoot_state_point->shoot_remote_point->rc.ch[4] >= 550.0f)
            {
                shoot_state_point->trigger_state = TRIGGER_GO;
            }
            break;
             default:
            break;
        }
        if(shoot_state_point->shoot_remote_point->rc.ch[4] <= -300.0f)
        {
					shoot_state_point->fric_state = FRIC_OFF;
					shoot_state_point->trigger_state = TRIGGER_OFF;
				}
					break;
        default:
            break;
    }
}

static void shoot_set_control_loop(shoot_para_t *shoot_set_control_loop_point)
{
    shoot_set_control_loop_point->fric_motor_para[0].give_current = PID_calc(&shoot_set_control_loop_point->fric_speed_pid[0], shoot_set_control_loop_point->fric_motor_para[0].speed, shoot_set_control_loop_point->fric_motor_para[0].speed_set);
    shoot_set_control_loop_point->fric_motor_para[1].give_current = PID_calc(&shoot_set_control_loop_point->fric_speed_pid[1], shoot_set_control_loop_point->fric_motor_para[1].speed, shoot_set_control_loop_point->fric_motor_para[1].speed_set);

    shoot_set_control_loop_point->trigger_motor_para.rate_set = PID_calc(&shoot_set_control_loop_point->trigger_angle_pid, shoot_set_control_loop_point->trigger_motor_para.angle_accumulation, shoot_set_control_loop_point->trigger_motor_para.angle_set);
    shoot_set_control_loop_point->trigger_motor_para.give_current = PID_calc(&shoot_set_control_loop_point->trigger_speed_pid, shoot_set_control_loop_point->trigger_motor_para.rate, shoot_set_control_loop_point->trigger_motor_para.rate_set);
	
}