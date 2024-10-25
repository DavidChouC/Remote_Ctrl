#include "bsp_usart.h"

extern UART_HandleTypeDef huart1;

uint8_t RxBuffer[1];//���ڽ��ջ���
uint16_t RxLine = 0;//ָ���
uint8_t DataBuff[200];//ָ������
uint8_t	cAlmStr[] = "!�������!\r\n";

/**
  *@breif      printf�ض���
  *@param      none
  *@retval     none
  */
int fputc(int ch, FILE *f)
{
 
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
}

/**
  *@breif      ���ڽ����жϻص�����
  *@param      none
  *@retval     none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)//����1�ж��������յ��Է��͵���Ϣ
	{
		if(RxLine >= 200)	//����ж�
		{
			RxLine = 0;				//��ս��ճ���
			memset(DataBuff,0,sizeof(DataBuff));	//��ջ�������
			HAL_UART_Transmit(&huart1, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);
		}
		else
		{
			DataBuff[RxLine++] = RxBuffer[0];	//�����յ������ݱ��浽��������
			if( (DataBuff[RxLine-1] == '\n') && (DataBuff[RxLine-2] == '\r'))//�жϽ�����־λ\r\n
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)&DataBuff, RxLine,0xFFFF); //���յ�����Ϣ���ͳ�ȥ
				memset(DataBuff,0,sizeof(DataBuff));  //��ջ�������
				RxLine = 0;	//��ս��ճ���
			}
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);	//ÿ����һ�����ݾʹ�һ���ж�
	}
}
