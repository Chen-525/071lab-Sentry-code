#ifndef _USART_RECEIVE_H
#define _USART_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"


  typedef struct {
      float x;     // ״̬����ֵ
      float P;     // �������Э����
      float Q;     // ��������Э���� (ģ�����ζ�)
      float R;     // ��������Э���� (���������ζ�)
      float K;     // ���������� (ÿ�ָ���)
  } kalman_1d_t;
	

void kalman_init(kalman_1d_t *kf, float Q, float R);
float kalman_update(kalman_1d_t *kf, float measurement);

	
typedef __packed struct
{
	uint8_t head;
 float vx;
	float vy;
	float wz;
	uint16_t checksum;
}navigation_data_t;

void MX_USART1_UART_Init(void);
void navigation_kalman_init(void);
const navigation_data_t *get_navigation_point(void);

#endif
