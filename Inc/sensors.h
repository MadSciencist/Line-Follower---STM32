  /****************************************
  *                                       *
  *       Title:  PID LIBRARY             *
  *       Author: Mateusz Kryszczak       *
  *       Date: 19.01.2017                *
  *       MCU: STM32F401                  *
  *                                       *
  *****************************************/

#ifndef sensors_H_
#define sensors_H_


#include "stm32f4xx.h"
#include "usart_tasks.h"
#include <string.h>
#include "robot_misc.h"
    
#define NUM_OF_SENSORS 10
#define NUM_OF_SAMPLES 5

float sensorsRawToBin(const uint16_t* sensorsRaw, uint8_t* sensorsScaledAndConstrained, const uint8_t* calMax, const uint8_t* calMin, uint8_t* binarized);
int16_t raw_ADC_normalize(uint16_t* in, uint16_t calibrationValue, uint8_t shift_val, uint8_t* min, uint8_t* max); //to finish
void RAW_ADC_calibrateEachSensor(uint16_t* in, uint8_t* inMax, uint8_t* inMin);
float constrainSetSpeed(float set, float constrain);

static uint16_t map16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
static uint8_t map8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
static void fill1DArray(uint16_t* arr, uint8_t size, int val);
static void fill2DArray(uint16_t arr[NUM_OF_SAMPLES][NUM_OF_SENSORS], uint8_t size1, uint8_t size2, int val);

#endif