#include "bsp_usart.h"

extern UART_HandleTypeDef huart1;

uint8_t RxBuffer[1];//串口接收缓冲
uint16_t RxLine = 0;//指令长度
uint8_t DataBuff[200];//指令内容
uint8_t	cAlmStr[] = "!数据溢出!\r\n";

/**
  *@breif      printf重定向
  *@param      none
  *@retval     none
  */
int fputc(int ch, FILE *f)
{
 
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
}

/**
  *@breif      串口接收中断回调函数
  *@param      none
  *@retval     none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)//串口1中断用来接收电脑发送的消息
	{
		if(RxLine >= 200)	//溢出判断
		{
			RxLine = 0;				//清空接收长度
			memset(DataBuff,0,sizeof(DataBuff));	//清空缓存数组
			HAL_UART_Transmit(&huart1, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);
		}
		else
		{
			DataBuff[RxLine++] = RxBuffer[0];	//将接收到的数据保存到缓存数组
			if( (DataBuff[RxLine-1] == '\n') && (DataBuff[RxLine-2] == '\r'))//判断结束标志位\r\n
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)&DataBuff, RxLine,0xFFFF); //将收到的信息发送出去
				memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
				RxLine = 0;	//清空接收长度
			}
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);	//每接收一个数据就打开一次中断
	}
}
