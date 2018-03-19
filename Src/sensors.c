#include "sensors.h"

extern  RobotParameters_t parameters;
extern requests_t req;


float sensorsRawToBin(const uint16_t* sensorsRaw, uint8_t* _sensorsScaledAndConstrained, const uint8_t* calMax, const uint8_t* calMin, uint8_t* binarized)
{
  uint8_t _sensors[NUM_OF_SENSORS];
  uint8_t numberOfSensorReading = 0;
  static float lastValidValue = 0.0f;
  float binarizedValue = 0.0f;
  
  //to 0-127 range
  for(int i = 0; i<NUM_OF_SENSORS; i++)
    _sensors[i] = sensorsRaw[i] >> 5;
  
  //scale to min-max range
  for(int i = 0; i< NUM_OF_SENSORS; i++)
    _sensorsScaledAndConstrained[i] = map8(_sensors[i], calMin[i], calMax[i], 0, 100);
  
  if(!parameters.misc.isWhiteOnBlack)
  {
    //actual thresholding
    for(int i = 0; i < NUM_OF_SENSORS; i++)
    {
      if(_sensorsScaledAndConstrained[i] > parameters.misc.sensorBinaryThreshold) 
      {
        binarized[i] = 1;
        numberOfSensorReading++;
      }
      else binarized[i] = 0;
    }
  }
  else
  {
    //actual thresholding
    for(int i = 0; i < NUM_OF_SENSORS; i++)
    {
      if(_sensorsScaledAndConstrained[i] < (100 - parameters.misc.sensorBinaryThreshold))
      {
        binarized[i] = 1;
        numberOfSensorReading++;
      }
      else binarized[i] = 0; 
    }
  }
  
  //weight values
  binarizedValue = (float)binarized[0]*-5.0f + (float)binarized[1] * -4.0f + (float)binarized[2] * -3.0f + (float)binarized[3] * -2.0f + (float)binarized[4] * -1.0f + (float)binarized[5] + (float)binarized[6] * 2.0f + (float)binarized[7] * 3.0f + (float)binarized[8] * 4.0f + (float)binarized[9] * 5.0f;
  if(numberOfSensorReading > 0)
  {
    binarizedValue = binarizedValue/(float)numberOfSensorReading;
    lastValidValue = binarizedValue;
  }
  else binarizedValue = lastValidValue * parameters.misc.backToTrackForce;//line lost -> get last valid and add some weight
  
  return binarizedValue;
}


float constrainSetSpeed(float set, float constrain)
{
  //return (set < constrain) ? set : constrain;
  if(set < constrain) return set;
  else return constrain;
}


//na czarnym sensor widzi max val

int16_t raw_ADC_normalize(uint16_t* in, uint16_t calibrationValue, uint8_t shift_val, uint8_t* min, uint8_t* max)
{
  uint8_t temp[NUM_OF_SENSORS];
  
  for(uint8_t i = 0; i < NUM_OF_SENSORS; i++)
  {
    temp[i] = in[i] >> shift_val;
    temp[i] = map16(temp[i], 0, 255, min[i], max[i]);
  }
  
  int16_t sum = temp[0]*-5 + temp[1] * -4 + temp[2] * -3 + temp[3] * -2 + temp[4] * -1 + temp[5] + temp[6] * 2 + temp[7] * 3 + temp[8] * 4 + temp[9] * 5;
  
  //return sum/=(calibrationValue/80); //-100 - 100 range
  return sum;
}

void RAW_ADC_calibrateEachSensor(uint16_t* in, uint8_t* inMax, uint8_t* inMin)
{
  uint16_t _max[NUM_OF_SAMPLES][NUM_OF_SENSORS], _min[NUM_OF_SAMPLES][NUM_OF_SENSORS];
  
  fill2DArray(_min, NUM_OF_SAMPLES, NUM_OF_SENSORS, 4095); //fill with 4095 to make possible find lower value
  fill2DArray(_max, NUM_OF_SAMPLES, NUM_OF_SENSORS, 0); //fill with 0 to make possible find higher value
  
  while(req.request_calibration_begin) //repeat till stop request
  {
    for(uint8_t i = 0; i < NUM_OF_SAMPLES; i++) //iterate through each sample
    {
      for(uint8_t j = 0; j < NUM_OF_SENSORS; j++) //iterate through each sensor
      { 
        HAL_Delay(1);
        if(in[j] > _max[i][j]) _max[i][j] = in[j];
        if(in[j] < _min[i][j]) _min[i][j] = in[j];
      }
    }
  }
  
  uint16_t sumMin[NUM_OF_SENSORS], sumMax[NUM_OF_SENSORS];
  
  fill1DArray(sumMin, NUM_OF_SENSORS, 0);
  fill1DArray(sumMax, NUM_OF_SENSORS, 0);
  
  
  //get sums to each sensor to finally calculate average
  for(uint8_t i = 0; i < NUM_OF_SAMPLES; i++)
  {
    for(uint8_t j = 0; j < NUM_OF_SENSORS; j++)
    {
      sumMin[j] += _min[i][j];
      sumMax[j] += _max[i][j];
    }
  }
  
  for(uint8_t i = 0; i < NUM_OF_SENSORS; i++)
  {
    inMax[i] = (sumMax[i] / NUM_OF_SAMPLES) >> 5; //calculate average and divide by 8 to scale into 0-127 range
    inMin[i] = (sumMin[i] / NUM_OF_SAMPLES) >> 5;
  }
}


/* helper function  with private scope */

static uint16_t map16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  uint16_t tmp;
  if((x - in_min) < 0)
    tmp = 0;
  else tmp = x - in_min;
  return (tmp * (out_max - out_min) / (in_max - in_min) + out_min);
}

static uint8_t map8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
  uint8_t tmp;
  if((x - in_min) < 0)
    tmp = 0;
  else tmp = x - in_min;
  return (tmp * (out_max - out_min) / (in_max - in_min) + out_min);
}

static void fill1DArray(uint16_t* arr, uint8_t size, int val)
{
  for(uint8_t i = 0; i < size; i++)
    arr[i] = val;
}

static void fill2DArray(uint16_t arr[NUM_OF_SAMPLES][NUM_OF_SENSORS], uint8_t size1, uint8_t size2, int val)
{
  for(uint8_t i = 0; i < size1; i++)
  {
    for(uint8_t j = 0; j < size2; j++)
    {
      arr[i][j] = val;
    }
  }    
}