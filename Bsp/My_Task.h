#ifndef __MY_TASK_H_
#define __MY_TASK_H_
#include "main.h"
#include "cmsis_os.h"
#include "Bsp_Key.h"
#include "u8g2.h"
#include "My_Task.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "nrf24L01.h"
#include "bsp_usart.h"

void freertos_start(void);

void start_task(void *pvParameters);

void mpu6050_task(void *pvParameters);

void key_task(void *pvParameters);

void rocker_task(void *pvParameters);

void data_task(void *pvParameters);



extern u8g2_t u8g2;

#endif
