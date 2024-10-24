#include "Bsp_Key.h"

/**
  *@breif      按键读取
  *@param      none
  *@retval     keynum:按键值
  */

uint8_t Key_Read(void)
{
	uint8_t keynum = 0;
	if(HAL_GPIO_ReadPin(L_UP_GPIO_Port,L_UP_Pin) == 0)//判断按下
	{
		keynum = L_UP;
	}
	if(HAL_GPIO_ReadPin(L_L_GPIO_Port,L_L_Pin) == 0)//判断按下
	{
		keynum = L_L;
	}
	if(HAL_GPIO_ReadPin(L_DOWN_GPIO_Port,L_DOWN_Pin) == 0)//判断按下
	{
		keynum = L_DOWN;
	}
	if(HAL_GPIO_ReadPin(L_R_GPIO_Port,L_R_Pin) == 0)//判断按下
	{
		keynum = L_R;
	}
	
	if(HAL_GPIO_ReadPin(R_UP_GPIO_Port,R_UP_Pin) == 0)//判断按下
	{
		keynum = R_UP;
	}
	if(HAL_GPIO_ReadPin(R_L_GPIO_Port,R_L_Pin) == 0)//判断按下
	{
		keynum = R_L;
	}
	if(HAL_GPIO_ReadPin(R_DOWN_GPIO_Port,R_DOWN_Pin) == 0)//判断按下
	{
		keynum = R_DOWN;
	}
	if(HAL_GPIO_ReadPin(R_R_GPIO_Port,R_R_Pin) == 0)//判断按下
	{
		keynum = R_R;
	}
	if(HAL_GPIO_ReadPin(L_ROCKER_KEY_GPIO_Port,L_ROCKER_KEY_Pin) == 0)//判断按下
	{
		keynum = L_ROCKER_KEY;
	}
	if(HAL_GPIO_ReadPin(R_ROCKER_KEY_GPIO_Port,R_ROCKER_KEY_Pin) == 0)//判断按下
	{
		keynum = R_ROCKER_KEY;
	}
	return keynum;
}
uint8_t	Read_L_MTS(void)
{
	return	HAL_GPIO_ReadPin(L_MTS_KEY_GPIO_Port,L_MTS_KEY_Pin);
}

uint8_t	Read_R_MTS(void)
{
	return	HAL_GPIO_ReadPin(R_MTS_KEY_GPIO_Port,R_MTS_KEY_Pin);
}

