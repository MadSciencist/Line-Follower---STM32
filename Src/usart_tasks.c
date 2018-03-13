#include "usart_tasks.h"

typedef union {  
  double f;  
  uint8_t fBuff[sizeof(double)];  
} doubleConverter_t;

uint8_t Received[SIZEOF_RECEIVING_BUFFER];
extern  RobotParameters_t parameters;
extern PID_Properties_t PID_Left, PID_Right, PID_Sensors;
extern requests_t req;
Queue_t queue_struct;
QueueRecord_t _rec, _rec1;
extern int m1, m2;
extern volatile uint8_t uartReady;

/*
void TransmitUart(QueueRecord_t* rec)
{
if(queue_struct.cnt == 0)
{
HAL_StatusTypeDef s = HAL_UART_Transmit_IT(&huart1, rec->buffer, SIZEOF_BUFFER);
if(s == HAL_BUSY)
q_push(&queue_struct, rec);
  }else
q_push(&queue_struct, rec);
}*/

void TransmitUart(QueueRecord_t* rec)
{
  q_push(&queue_struct, rec);
  //HAL_UART_Transmit_IT(&huart1, rec->buffer, SIZEOF_BUFFER);
}

void uart_write_parameters(RobotParameters_t* p)
{  
  uart_write_double(SensorsP, p->sensors.kp);
  uartReady = 1;
  uart_write_double(SensorsD, p->sensors.kd);
  uart_write_byte(SensorsDerivativeType, p->sensors.derivativeType);
  uart_write_int16(SensorFeedbackRefreshRate, p->misc.uartUpdateRate);
  uart_write_byte(SensorBinaryThreshold, p->misc.sensorBinaryThreshold);
  
  uart_write_double(MotorLeftP, p->left.kp);
  uart_write_double(MotorLeftI, p->left.ki);
  uart_write_double(MotorLeftD, p->left.kd);
  uart_write_byte(MotorLeftDType, p->left.derivativeType); 
  uart_write_double(MotorLeftILimit, p->left.I_limit);
  uart_write_double(MotorLeftMaxSpeed, p->left.MaxSpeed);
  uart_write_double(MotorLeftMinSpeed, p->left.MinSpeed);
  uart_write_double(MotorLeftSpeedOffset, p->left.SpeedOffset);
  
  uart_write_double(MotorRightP, p->right.kp);
  uart_write_double(MotorRightI, p->right.ki);
  uart_write_double(MotorRightD, p->right.kd);
  uart_write_byte(MotorRightDType, p->right.derivativeType); 
  uart_write_double(MotorRightILimit, p->right.I_limit);
  uart_write_double(MotorRightMaxSpeed, p->right.MaxSpeed);
  uart_write_double(MotorRightMinSpeed, p->right.MinSpeed);
  uart_write_double(MotorRightSpeedOffset, p->right.SpeedOffset);
  uart_write_byte(isWhiteOnBlackE, p->misc.isWhiteOnBlack);
}

void uart_write_feedback(int16_t sensorFeedback, int16_t sensorP, int16_t sensorD, int8_t vel1, int8_t vel1Integral, int8_t vel2, int8_t vel2Integral, uint8_t current1, uint8_t current2)
{
  uint8_t _buff[] = {sensorFeedback, sensorP, sensorD, (vel1+127), (vel1Integral+127), (vel2+127), (vel2Integral+127), current1, current2, 0};
  gen_message(Feedback, _buff);
}

void uart_write_voltage(uint16_t voltage, int16_t sensorsOutput) //this might be separated into two functions
{
  uart_write_two_int16(Voltage, voltage, sensorsOutput);
}

void uart_write_sensors_status(uint8_t data[])
{
  gen_message(SensorsTest, data);
}

void uart_write_calibration_result(GuiParser_t calType, uint8_t _buff[])
{
  gen_message(calType, _buff);
}

void uart_write_binarized_sensors(uint8_t _buff[])
{
  gen_message(SensorBinaryValues, _buff);
}

void uart_write_calibrated_sensors(uint8_t _buff[])
{
  gen_message(SensorsCalibratedAndConstrained, _buff);
}

void uart_write_double(uint8_t message_ID, double val){   
  doubleConverter_t conv;
  conv.f = val;
  uint8_t _buff[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  memcpy(_buff, conv.fBuff, SIZEOF_BUFFER);
  gen_message(message_ID, _buff);
}

void uart_write_int16(uint8_t message_ID, int16_t val)
{   
  uint8_t _buff[] = {val, val >> 8, 0, 0, 0, 0, 0, 0, 0, 0};
  gen_message(message_ID, _buff);
}

void uart_write_two_int16(uint8_t message_ID, int16_t va1, int16_t val2)
{   
  uint8_t _buff[] = {va1, va1 >> 8, val2, val2 >> 8, 0, 0, 0, 0, 0, 0};
  gen_message(message_ID, _buff);
}

void uart_write_byte(uint8_t message_ID, uint8_t val)
{
  uint8_t _buff[] = {val, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  gen_message(message_ID, _buff);
}

void gen_message(uint8_t message_ID, uint8_t buff[])
{
  static uint8_t cntr = 0;
  if(cntr++ > 254) cntr = 0;
  
  uint8_t  _buff[] = {'<', cntr, message_ID, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7], buff[8], buff[9], '>'}; //14 bytes
  memcpy(_rec1.buffer, _buff, SIZEOF_BUFFER);
  TransmitUart(&_rec1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uartReady = 1;
  }
}

void checkIfReady()
{
  if(uartReady)
  {
    if(queue_struct.cnt > 0)
    {
      uartReady = 0;
      q_pop(&queue_struct, &_rec);
      HAL_UART_Transmit_IT(&huart1, _rec.buffer, SIZEOF_BUFFER);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  static uint8_t error_count = 0;
  error_count++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(Received[0] == (uint8_t)FRAME_START_CHAR && Received[SIZEOF_RECEIVING_BUFFER-1] == (uint8_t)FRAME_STOP_CHAR)
    parse_data(&Received[0]);
  
 // if(queue_struct.cnt > 0)
 // {
  //  q_pop(&queue_struct, &_rec);
  //  HAL_UART_Transmit_IT(&huart1, _rec.buffer, SIZEOF_BUFFER);
  //}
}

void parse_data(const uint8_t* rec_buff)
{
  switch (rec_buff[1])
  {
  case rRequestAllData:
    req.request_all_parameters = 1;
    break;
    
  case rStartRace:
    req.request_start_race = rec_buff[2];
    break;
    
  case rEnableSensorUpdate:
    if(rec_buff[2] == 1)
      req.request_sensor_update = 1;
    else req.request_sensor_update = 0;
    break;
    
  case rSensorCalibrationBegin:
    if(rec_buff[2] == 1)
      req.request_calibration_begin = 1;
    else if(rec_buff[2] == 0)
      req.request_calibration_begin = 0;
    break;
    
  case rEnableFeedback:
    if(rec_buff[2] == 1)
      req.request_enable_feedback = 1;
    else req.request_enable_feedback = 0;
    break;
    
  case rSensorFeedbackRefreshRate:{
    int16_t temp = 0;
    temp = (rec_buff[3] << 8) | rec_buff[2];
    if(temp > 0 && temp <= 100)
      parameters.misc.uartUpdateRate = temp;
    break;}
    
  case rWriteData:
    req.request_write_flash = 1;
    break;
    
    case rIsWhiteOnBlackE:
      parameters.misc.isWhiteOnBlack = rec_buff[2];
    break;
    
  case rManualControl:
    
    m1 = (rec_buff[3] << 8) | rec_buff[2];
    m2 = (rec_buff[5] <<8) | rec_buff[4];
    break;
    
    
    /*SENSORS*/
  case rSensorsP:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.sensors.kp = conv.f;
    PidSetParams(&PID_Sensors, parameters.sensors.kp, 0.0f, parameters.sensors.kd); 
    break;}
    
  case rSensorsD:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.sensors.kd = conv.f;
    PidSetParams(&PID_Sensors, parameters.sensors.kp, 0.0f, parameters.sensors.kd); 
    break;}
    
  case rSensorDType:
    switch(rec_buff[2]){
    case GUInoDerivative:
      parameters.sensors.derivativeType = GUInoDerivative;
      break;
    case errorDerivative:
      parameters.sensors.derivativeType = errorDerivative;
      break;
    case feedbackDerivative:
      parameters.sensors.derivativeType = feedbackDerivative;
      break;
    }break;
    
  case rSensorBinaryThreshold:
    if(rec_buff[2] > 0 && rec_buff[2] < 100)
      parameters.misc.sensorBinaryThreshold = rec_buff[2];
    break;
    
    /*MOTOR LEFT*/
  case rMotorLeftP:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.kp = conv.f;
    PidSetParams(&PID_Left, parameters.left.kp, parameters.left.ki, parameters.left.kd);
    break;}
    
  case rMotorLeftI:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.ki = conv.f;
    PidSetParams(&PID_Left, parameters.left.kp, parameters.left.ki, parameters.left.kd);
    break;}
    
  case rMotorLeftD:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.kd = conv.f;
    PidSetParams(&PID_Left, parameters.left.kp, parameters.left.ki, parameters.left.kd);
    break;}
    
  case rMotorLeftDType:
    switch(rec_buff[2]){
    case GUInoDerivative:
      parameters.left.derivativeType = GUInoDerivative;
      break;
    case errorDerivative:
      parameters.left.derivativeType = errorDerivative;
      break;
    case feedbackDerivative:
      parameters.left.derivativeType = feedbackDerivative;
      break;
    }break;
    
  case rMotorLeftILimit:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.I_limit = conv.f;
    PID_Left.negIntegralLimit = -1.0f * conv.f;
    PID_Left.posIntegralLimit = conv.f;
    PID_Left.negOutputLimit = -1.0f * conv.f;
    PID_Left.posOutputLimit = conv.f;
    parameters.left.MaxSpeed = conv.f;
    break;}
    
  case rMotorLeftMaxSpeed:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.MaxSpeed = conv.f;
    break;}
    
  case rMotorLeftMinSpeed:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.MinSpeed = conv.f;
    break;}
    
  case rMotorLeftSpeedOffset:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.left.SpeedOffset = conv.f;
    break;}
    
    
    /*MOTOR RIGHT*/
  case rMotorRightP:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.kp = conv.f;
    PidSetParams(&PID_Right, parameters.right.kp, parameters.right.ki, parameters.right.kd);
    break;}
    
  case rMotorRightI:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.ki = conv.f;
    PidSetParams(&PID_Right, parameters.right.kp, parameters.right.ki, parameters.right.kd);
    break;}
    
  case rMotorRightD:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.kd = conv.f;
    PidSetParams(&PID_Right, parameters.right.kp, parameters.right.ki, parameters.right.kd);
    break;}
    
  case rMotorRightDType:
    switch(rec_buff[2]){
    case GUInoDerivative:
      parameters.right.derivativeType = GUInoDerivative;
      break;
    case errorDerivative:
      parameters.right.derivativeType = errorDerivative;
      break;
    case feedbackDerivative:
      parameters.right.derivativeType = feedbackDerivative;
      break;
    }break;
    
  case rMotorRightILimit:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.I_limit = conv.f;
    PID_Right.negIntegralLimit = -1.0f * conv.f;
    PID_Right.posIntegralLimit = conv.f;
    PID_Right.negOutputLimit = -1.0f * conv.f;
    PID_Right.posOutputLimit = conv.f;
    parameters.right.MaxSpeed = conv.f;
    break;}
    
  case rMotorRightMaxSpeed:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.MaxSpeed = conv.f;
    break;}
    
  case rMotorRightMinSpeed:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.MinSpeed = conv.f;
    break;}
    
  case rMotorRightSpeedOffset:{
    doubleConverter_t conv;
    memcpy(&conv.fBuff[0], &rec_buff[2], SIZEOF_BUFFER);
    parameters.right.SpeedOffset = conv.f;
    break;}
    
  default:
    break;
  }
}