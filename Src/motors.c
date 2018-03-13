  /****************************************
  *                                       *
  *       Title:  TB6612FNG LIBRARY       *
  *       Author: Mateusz Kryszczak       *
  *       Date: 01.2017                   *
  *       MCU: STM32F401                  *
  *                                       *
  *****************************************/

#include "motors.h"

void Motor1StartPWM() {
  HAL_TIM_PWM_Start(MOTOR1_TIM, MOTOR1_TIM_CHN);
}

void Motor2StartPWM() {
  HAL_TIM_PWM_Start(MOTOR2_TIM, MOTOR2_TIM_CHN);
}

void Motor1StopPWM() {
  HAL_TIM_PWM_Stop(MOTOR1_TIM, MOTOR1_TIM_CHN);
}

void Motor2StopPWM() {
  HAL_TIM_PWM_Stop(MOTOR2_TIM, MOTOR2_TIM_CHN);
}

void Motor1DriveLeft(int16_t value){
  MOTOR1_B_LOW;
  MOTOR1_A_HIGH;
  MOTOR1_TIM_INST->MOTOR1_TIM_CHN_INST = value;
}

void Motor1DriveRight(int16_t value){
  MOTOR1_A_LOW;
  MOTOR1_B_HIGH;
  MOTOR1_TIM_INST->MOTOR1_TIM_CHN_INST = value;
}

void Motor2DriveLeft(int16_t value){
  MOTOR2_B_LOW;
  MOTOR2_A_HIGH;
  MOTOR2_TIM_INST->MOTOR2_TIM_CHN_INST = value;
}

void Motor2DriveRight(int16_t value){
  MOTOR2_A_LOW;
  MOTOR2_B_HIGH;
  MOTOR2_TIM_INST->MOTOR2_TIM_CHN_INST = value;
}

void Motor1HardBrake(){
  Motor1StopPWM();
  MOTOR1_B_LOW;
  MOTOR1_A_HIGH;
}
void Motor2HardBrake(){
  Motor2StopPWM();
  MOTOR2_B_LOW;
  MOTOR2_A_HIGH;
}

void Motor1CoastBrake(){
  MOTOR1_A_LOW;
  MOTOR1_B_LOW;
}

void Motor2CoastBrake(){
  MOTOR2_B_LOW;
  MOTOR2_A_LOW;
}

void Motor1DriveByPid(int16_t value){
  if(value >= 0){
    Motor1DriveLeft(value);
  }else{
    Motor1DriveRight(-value);
  }  
}

void Motor2DriveByPid(int16_t value){
  if(value >= 0){
    Motor2DriveLeft(value);
  }else{
    Motor2DriveRight(-value);
  }  
}

  