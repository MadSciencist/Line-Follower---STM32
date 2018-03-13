  /****************************************
  *                                       *
  *       Title:  TB6612FNG LIBRARY       *
  *       Author: Mateusz Kryszczak       *
  *       Date: 01.2017                   *
  *       MCU: STM32F401                  *
  *                                       *
  *****************************************/

#ifndef _motors_H

#include "stm32f4xx.h"
#include "gpio.h"
#include "tim.h"

//user defines: which timer & channel
//warning: timer and register name should be also changed in motors.c in driveMotorx_y functions
#define MOTOR1_TIM              &htim2
#define MOTOR2_TIM              &htim2
#define MOTOR1_TIM_INST         TIM2
#define MOTOR2_TIM_INST         TIM2

#define MOTOR1_TIM_CHN          TIM_CHANNEL_1 
#define MOTOR2_TIM_CHN          TIM_CHANNEL_2 
#define MOTOR1_TIM_CHN_INST     CCR1
#define MOTOR2_TIM_CHN_INST     CCR2

//user PIN defines
#define MOTOR1_A_HIGH           HAL_GPIO_WritePin(GPIOC, MOT1A_Pin, GPIO_PIN_SET);
#define MOTOR1_A_LOW            HAL_GPIO_WritePin(GPIOC, MOT1A_Pin, GPIO_PIN_RESET);
#define MOTOR1_B_HIGH           HAL_GPIO_WritePin(GPIOC, MOT1B_Pin, GPIO_PIN_SET);
#define MOTOR1_B_LOW            HAL_GPIO_WritePin(GPIOC, MOT1B_Pin, GPIO_PIN_RESET);

#define MOTOR2_A_HIGH           HAL_GPIO_WritePin(GPIOC, MOT2A_Pin, GPIO_PIN_SET);
#define MOTOR2_A_LOW            HAL_GPIO_WritePin(GPIOC, MOT2A_Pin, GPIO_PIN_RESET);
#define MOTOR2_B_HIGH           HAL_GPIO_WritePin(MOT2B_GPIO_Port, MOT2B_Pin, GPIO_PIN_SET);
#define MOTOR2_B_LOW            HAL_GPIO_WritePin(MOT2B_GPIO_Port, MOT2B_Pin, GPIO_PIN_RESET);

//enable PWM generation
void Motor1StartPWM(void);
void Motor2StartPWM(void);

//disable PWM generation
void Motor1StoptPWM(void);
void Motor2StoptPWM(void);

//drive motor with value (0-100)
void Motor1DriveLeft(int16_t value);
void Motor1DriveRight(int16_t value);

void Motor2DriveLeft(int16_t value);
void Motor2DriveRight(int16_t value);

//value (0, 100) drive forward, (-100, 0) drive backward
void Motor1DriveByPid(int16_t value);
void Motor2DriveByPid(int16_t value);

//fast stop
void Motor1HardBrake(void);
void Motor2HardBrake(void);

//gentle stop
void Motor1CoastBrake(void);
void Motor2CoastBrake(void);

#define _motors_H
#endif