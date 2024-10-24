#include "My_Task.h"

u8g2_t u8g2;

uint16_t ADC_Buff[4]={0};   //摇杆电位器值

float pitch,roll,yaw; 		  //欧拉角
short aacx,aacy,aacz;				//加速度传感器原始数据
short gyrox,gyroy,gyroz;		//陀螺仪原始数据

uint8_t Send_Data[32];		  //遥控器下发数据(32字节)
uint8_t Mpu6050_Data[26]; 	//陀螺仪数据

uint8_t KeyNum_Data[3];     //按键数据
uint8_t L_Rocker_Data[10];  //左摇杆数据
uint8_t R_Rocker_Data[10];  //右摇杆数据
uint8_t Speed_Data[5];      //电机速度档位(High Low)


/* 启动任务的配置 */
#define START_TASK_STACK 128
#define START_TASK_PRIORITY 3
TaskHandle_t start_task_handle;

/* 陀螺仪任务的配置 */
#define MPU6050_STACK 128
#define MPU6050_PRIORITY 2
TaskHandle_t mpu6050_handle;

/* 摇杆任务的配置 */
#define ROCKER_STACK 128
#define ROCKER_PRIORITY 1
TaskHandle_t rocker_handle;

/* 按键任务的配置 */
#define KEY_STACK 128
#define KEY_PRIORITY 1
TaskHandle_t key_handle;

/* 数据显示与发送任务的配置 */
#define DATA_STACK 128
#define DATA_PRIORITY 3
TaskHandle_t data_handle;

SemaphoreHandle_t mpu6050_sem_handle;
SemaphoreHandle_t rocker_sem_handle;
void freertos_start(void)
{
  mpu6050_sem_handle = xSemaphoreCreateBinary();
  rocker_sem_handle = xSemaphoreCreateBinary();
  /* 创建一个启动任务 */
  xTaskCreate((TaskFunction_t)start_task,               // 任务函数的地址
              (char *)"start_task",                     // 任务名字符串
              (configSTACK_DEPTH_TYPE)START_TASK_STACK, // 任务栈大小，默认最小128，单位4字节
              (void *)NULL,                             // 传递给任务的参数
              (UBaseType_t)START_TASK_PRIORITY,         // 任务的优先级
              (TaskHandle_t *)&start_task_handle);      // 任务句柄的地址
}

/**
** @brief:      启动任务：用来创建其他Task
** @param:      {void} *pvParameters
** @retval:     None
*/
void start_task(void *pvParameters)
{
  /* 进入临界区:保护临界区里的代码不会被打断 */
  taskENTER_CRITICAL();

  /* 创建多个任务 */
  xTaskCreate((TaskFunction_t)mpu6050_task,
              (char *)"mpu6050_task",
              (configSTACK_DEPTH_TYPE)MPU6050_STACK,
              (void *)NULL,
              (UBaseType_t)MPU6050_PRIORITY,
              (TaskHandle_t *)&mpu6050_handle);
  xTaskCreate((TaskFunction_t)rocker_task,
              (char *)"rocker_task",
              (configSTACK_DEPTH_TYPE)ROCKER_STACK,
              (void *)NULL,
              (UBaseType_t)ROCKER_PRIORITY,
              (TaskHandle_t *)&rocker_handle);
  xTaskCreate((TaskFunction_t)key_task,
              (char *)"key_task",
              (configSTACK_DEPTH_TYPE)KEY_STACK,
              (void *)NULL,
              (UBaseType_t)KEY_PRIORITY,
              (TaskHandle_t *)&key_handle);
  xTaskCreate((TaskFunction_t)data_task,
              (char *)"data_task",
              (configSTACK_DEPTH_TYPE)DATA_STACK,
              (void *)NULL,
              (UBaseType_t)DATA_PRIORITY,
              (TaskHandle_t *)&data_handle);
  /* 启动任务只需要执行一次即可，用完就删除自己 */
  vTaskDelete(NULL);

  /* 退出临界区 */
  taskEXIT_CRITICAL();
}

/**
** @brief:      陀螺仪
** @param:      {void} *pvParameters
** @retval:     None
*/
void mpu6050_task(void *pvParameters)
{
  while (1)
  {
		xSemaphoreTake(mpu6050_sem_handle,portMAX_DELAY);
    while(mpu_dmp_get_data(&pitch, &roll, &yaw));	//必须要用while等待，才能读取成功
    // MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		//得到加速度传感器数据
    // MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		//得到陀螺仪数据
    // printf("%.1f,%.1f,%.1f \r\n",roll,pitch,yaw);
    snprintf((char *)Mpu6050_Data,sizeof(Send_Data),"X:%.1f   Y:%.1f   Z:%.1f",roll,pitch,yaw);
		printf("MPU6050_handle:%d\r\n",(int32_t)uxTaskGetStackHighWaterMark(NULL));
  }
}

/**
** @brief:      按键
** @param:      {void} *pvParameters
** @retval:     None
*/
void key_task(void *pvParameters)
{
  while (1)
  {
		if(Read_L_MTS() == UP)//high
		{
      strcpy((char*)Speed_Data,"High");
    }
    else//low
    {
      strcpy((char*)Speed_Data,"Low");
    }
    sprintf((char *)KeyNum_Data,"%d",Key_Read());
    vTaskDelay(10);
  }
}

/**
** @brief:      摇杆
** @param:      {void} *pvParameters
** @retval:     None
*/
void rocker_task(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(rocker_sem_handle,portMAX_DELAY);
    snprintf((char *)L_Rocker_Data,sizeof(L_Rocker_Data),"X:%d Y:%d",L_ROCKER_X/100,L_ROCKER_Y/100);

    snprintf((char *)R_Rocker_Data,sizeof(R_Rocker_Data),"X:%d Y:%d",R_ROCKER_X/100,R_ROCKER_Y/100);
    printf("rocker_handle:%d\r\n",(int32_t)uxTaskGetStackHighWaterMark(NULL));
  }
}

/**
** @brief:      数据显示与发送
** @param:      {void} *pvParameters
** @retval:     None
*/
void data_task(void *pvParameters)
{
  while (1)
  {
    /* OLED清除缓冲区 */
    u8g2_ClearBuffer(&u8g2);
    /* 速度数据 */
    u8g2_DrawUTF8(&u8g2,32,12,"Speed: ");
    u8g2_DrawUTF8(&u8g2,32+u8g2_GetUTF8Width(&u8g2,"Speed: "),12,(char *)Speed_Data);
    /* 摇杆模式 */
    if(Read_R_MTS() == UP)
		{
      /* 摇杆数据 */
      u8g2_DrawUTF8(&u8g2,38,24,(char *)L_Rocker_Data);
      u8g2_DrawUTF8(&u8g2,38,34,(char *)R_Rocker_Data);
      /* 按键数据 */
      u8g2_DrawUTF8(&u8g2,34,44,"KeyNum: ");
      u8g2_DrawUTF8(&u8g2,34+u8g2_GetUTF8Width(&u8g2,"KeyNum: "),44,(char *)KeyNum_Data);

      /* 数据拼接 */
      snprintf((char *)Send_Data,sizeof(Send_Data),"%s,%s,%s,%s",Speed_Data,KeyNum_Data,L_Rocker_Data,R_Rocker_Data);
      xSemaphoreGive(rocker_sem_handle);
    }
    /* 陀螺仪模式 */
    else
    {
      /* 陀螺仪数据 */
      u8g2_DrawUTF8(&u8g2,38,54,"Mpu6050");
      u8g2_DrawUTF8(&u8g2,12,64,(char *)Mpu6050_Data);
      /* 数据拼接 */
      snprintf((char *)Send_Data,sizeof(Send_Data),"%s,%s",Speed_Data,Mpu6050_Data);
      xSemaphoreGive(mpu6050_sem_handle);
    }
    /* 发送数据给下位机 */
    NRF24L01_TxPacket(Send_Data);
    /* OLED发送缓冲区 */
    u8g2_SendBuffer(&u8g2);
		printf("data_handle:%d\r\n",(int32_t)uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(10);
  }
}


