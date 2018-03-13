#ifndef ROBOT_MISC_H_

#include "stm32f4xx.h"

typedef struct requests
{
  volatile uint8_t request_calibration_begin;
  volatile uint8_t request_sensor_update;
  volatile uint8_t request_enable_feedback;
  volatile uint8_t request_all_parameters;
  volatile uint8_t request_write_flash;
  volatile uint8_t request_start_race;
} requests_t;

#define ROBOT_MISC_H_
#endif