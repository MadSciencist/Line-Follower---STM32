#ifndef _usart_tasks_H_

#define SIZEOF_BUFFER 14
#define SIZEOF_RECEIVING_BUFFER 11
#define FRAME_START_CHAR '<'
#define FRAME_STOP_CHAR '>'

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_uart.h"
#include "main.h"
#include "usart.h"
#include "robot_misc.h"
#include "PID.h"
#include "math.h"
#include <string.h>
#include "cQueue.h"

typedef struct queueRec {
  uint8_t buffer[SIZEOF_BUFFER];
} QueueRecord_t;

typedef enum //robot to PC
{
  Feedback, //current data
  SensorsTest, //all sensor real time data
  Voltage,
  
  SensorsP = 90,
  SensorsD,
  SensorsDerivativeType,
  SensorFeedbackRefreshRate,
  
  MotorLeftP = 100,
  MotorLeftI,
  MotorLeftILimit,
  MotorLeftD,
  MotorLeftDType,
  MotorLeftMinSpeed,
  MotorLeftMaxSpeed,
  MotorLeftSpeedOffset,
  
  MotorRightP = 200,
  MotorRightI,
  MotorRightILimit,
  MotorRightD,
  MotorRightDType,
  MotorRightMinSpeed,
  MotorRightMaxSpeed,
  MotorRightSpeedOffset,
  
  speedTest,
  CalibrationMaxValues,
  CalibrationMinValues,
  SensorBinaryThreshold,
  SensorBinaryValues,
  SensorsCalibratedAndConstrained,
  isWhiteOnBlackE,
    
} GuiParser_t;

typedef enum  //data from PC to robot, r means robot
{
  rRequestAllData,
  rWriteData,
  
  rStartRace,
  rStopRace,
  
  rEnableSensorUpdate,
  rSensorCalibrationBegin,
  rEnableFeedback,
  rSensorFeedbackRefreshRate,
  
  rSensorsP = 90,
  rSensorsD,
  rSensorDType,
  
  rMotorLeftP = 100,
  rMotorLeftI,
  rMotorLeftILimit,
  rMotorLeftD,
  rMotorLeftDType,
  rMotorLeftMinSpeed,
  rMotorLeftMaxSpeed,
  rMotorLeftSpeedOffset,
  
  rMotorRightP = 200,
  rMotorRightI,
  rMotorRightILimit,
  rMotorRightD,
  rMotorRightDType,
  rMotorRightMinSpeed,
  rMotorRightMaxSpeed,
  rMotorRightSpeedOffset,
  
  rManualControl,
  
  rSensorBinaryThreshold,
  rIsWhiteOnBlackE,
  
}RobotParser_t;

typedef enum{
  GUInoDerivative,
  errorDerivative,
  feedbackDerivative
} uartDerivativeType_t;

typedef struct{
  double kp;
  double kd;
  uartDerivativeType_t derivativeType;
  uint8_t calibrationMaxValues[10];
  uint8_t calibrationMinValues[10];
} uartSensorParameters_t;

typedef struct{
  volatile uint16_t uartUpdateRate;
  uint8_t sensorBinaryThreshold;
  uint8_t isWhiteOnBlack; /* 1 it is, 0 it is black board with white line */
  double backToTrackForce;
} uartMiscParameters_t;

typedef struct{
  double kp;
  double ki;
  double I_limit;
  double kd;
  uartDerivativeType_t derivativeType;
  double MinSpeed;
  double MaxSpeed;
  double SpeedOffset;
} uartDriveParameters_t;

//main data container
typedef struct{
  uartSensorParameters_t sensors;
  uartDriveParameters_t left;
  uartDriveParameters_t right;
  uartMiscParameters_t misc;
} RobotParameters_t;

void TransmitUart(QueueRecord_t* rec);

void uart_write_parameters(RobotParameters_t* p);
void uart_write_feedback(int16_t sensorFeedback, int16_t sensorP, int16_t sensorD, int8_t vel1, int8_t vel1Integral, int8_t vel2, int8_t vel2Integral, uint8_t current1, uint8_t current2);
void uart_write_voltage(uint16_t voltage, int16_t sensorsOutput);
void uart_write_sensors_status(uint8_t data[]);
void uart_write_calibration_result(GuiParser_t calType, uint8_t _buff[]);
void uart_write_binarized_sensors(uint8_t _buff[]);
void uart_write_calibrated_sensors(uint8_t _buff[]);
void uart_write_double(uint8_t message_ID, double val);
void uart_write_int16(uint8_t message_ID, int16_t val);
void uart_write_two_int16(uint8_t message_ID, int16_t va1, int16_t val2);
void uart_write_byte(uint8_t message_ID, uint8_t val);
void gen_message(uint8_t message_ID, uint8_t buff[]);
void parse_data(const uint8_t* rec_buff);
void checkIfReady();

#define _usart_tasks_H_
#endif