#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "usart_receive.h"
#include "gimbal_task.h"

//������ٱ�
#define M3508_RATIO 	 				 19	  //1:19
//ȫ���ְ뾶
#define OMNI_WHEEL_RADIOS      0.14 //m 

#define PI  									3.1415926535
//���̼��ٶȱ���
#define CHASSIS_CONTROL_FREQUENCE 500.0f

#define CHASSIS_SPEED_MAX		4.0f
#define CHASSIS_TURNING_RATE_MAX		2.0f

#define CHASSIS_SPIN_RATE				4.0f					//rad/s

typedef enum
{
	CHASSIS_NO_MOVE = 0,
	CHASSIS_NORMAL_MOVE,
	CHASSIS_FOLLOW_YAW,
	CHASSIS_SPIN,
	CHASSIS_NAVIGATION,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;


typedef struct
{
  const volatile RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
//  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
//  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid
  
	const volatile navigation_data_t *chassis_navigation_data;

	gimbal_para_t *gimbal_para_point;
	pid_type_def		chassis_follow_yaw_pid;

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�

} chassis_move_t;

void chassis_task(void const *argument);

#endif
