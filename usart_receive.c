#include "usart_receive.h"
#include "usart.h"
#include "crc8_crc16.h"
#include <string.h>

uint16_t RxCounter = 0;
uint8_t rxd_state;
//uint8_t navigation_tx_buffer[28];	
uint8_t navigation_rx_buffer[15];	
uint8_t rx_data;
UART_HandleTypeDef huart1;
navigation_data_t navigation_data;


void kalman_init(kalman_1d_t *kf, float Q, float R)
{
      kf->x = 0.0f;
      kf->P = 1.0f;      // ��ʼ��ȷ���ȣ�����һ�������������
      kf->Q = Q;
      kf->R = R;
      kf->K = 0.0f;
}
	
	
float kalman_update(kalman_1d_t *kf, float measurement)
{
      // ---- Ԥ�� ----
      // ����ģ��: x(k|k-1) = x(k-1|k-1)��P(k|k-1) = P(k-1|k-1) + Q
      kf->P += kf->Q;

      // ---- ���� ----
      // K = P / (P + R)
      // x = x + K * (z - x)
      // P = (1 - K) * P
      kf->K = kf->P / (kf->P + kf->R);
      kf->x += kf->K * (measurement - kf->x);
      kf->P *= (1.0f - kf->K);

      return kf->x;
}

	void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data, 1);//??????,??????
    switch(rxd_state)
    {
      case 0:
      {
        if(rx_data == 0xA5)   //??
        {
            RxCounter = 0; // ?? RxCounter ? 0
            rxd_state = 1;
            navigation_rx_buffer[RxCounter]=rx_data;
					  
            RxCounter++;
        }
        break;
      }
      case 1:
      {
        if(rx_data == 0xA5)   //??????0xA5,??????
        {
            rxd_state = 0;
//            memset(&aimbot.Vision_Rx_Data, 0, sizeof(aimbot.Vision_Rx_Data)); //??????
            break;
        }
        navigation_rx_buffer[RxCounter]=rx_data;
        RxCounter++;
        if(RxCounter==sizeof(navigation_data))  //??
            rxd_state = 2;
        break;
      }
      case 2:
      {
        if(verify_CRC16_check_sum((uint8_t *)&navigation_rx_buffer, sizeof(navigation_rx_buffer)) == 1)
        {
          memcpy(&navigation_data, navigation_rx_buffer, sizeof(navigation_rx_buffer));
//		  aimbot_mode_set(0);
        }
        rxd_state = 0;
        break;
      }
    }
		
	}
}

const navigation_data_t *get_navigation_point(void)
{
	return &navigation_data;
}
