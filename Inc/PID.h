  /****************************************
  *                                       *
  *       Title:  PID LIBRARY             *
  *       Author: Mateusz Kryszczak       *
  *       Date: 19.01.2017                *
  *       MCU: STM32F401                  *
  *                                       *
  *****************************************/

#ifndef _PID_H_
    
#include "stm32f4xx.h"
#include "math.h"

void calculateSpeed(float* left, float* right);

typedef struct{
  uint16_t period;              //interval between PID computations
  float error;
  float integralSum;
  float kp;                     //do not change Kp Ki Kd manually, use PidSetParams(); 
  float ki;
  float kd;
  float posIntegralLimit;       //integral anti-windup
  float negIntegralLimit;       //integral anti-windup
  float lastError;      
  float lastFeedback;
  float posOutputLimit;         //bound the output
  float negOutputLimit;
} PID_Properties_t;

//derivative type calculation
typedef enum{
  noDerivative,
  derivativeOnError,
  derivativeOnFeedback,
} derivative_t;

//reverse output of the PID
typedef enum{
  noReverse,
  reverse,
} PID_Reverse_t;

//compute PID
//return 0 if null pointers
//return 1 if ok
uint8_t PID(PID_Properties_t* PID_Properties, float* pSetpoint, float* pFeedback, float* pOutput, derivative_t derivativeType, PID_Reverse_t pidReverse);
uint8_t PD(PID_Properties_t* PID_Properties, float* pSetpoint, float* pFeedback, float* pOutput, derivative_t derivativeType, PID_Reverse_t pidReverse); //simplified to PD

//set PID tunings
//return 0 if parameters are invalid (<0) or null pointer
//return 1 if set went ok
uint8_t PidSetParams(PID_Properties_t* PID_Properties, float _kp, float _ki, float _kd);

//helper function to compare two floats withing epsilon accuracy
//return true if floats are the same
static uint8_t Compare(float a, float b, float epsilon);

#define _PID_H_
#endif