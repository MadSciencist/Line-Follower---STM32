/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "motors.h"
#include "PID.h"
#include "sensors.h"
#include "usart_tasks.h"
#include "eeprom_emulator.h"
#include "robot_misc.h"

#define ADC_MEASUREMENTS 13
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//PID variables
volatile float sensorsSetpoint = 0.0f, sensorsFeedback  = 0.0f, sensorsOutput = 0.0f;
float leftOutput = 0.0f, leftSet = 0.0f, rightOutput = 0.0f, rightSet = 0.0f;
PID_Properties_t PID_Left, PID_Right, PID_Sensors;

//tests
int m1 = 0, m2 = 0;


//ADC  Variables
uint16_t ADC_RAW[ADC_MEASUREMENTS];
uint8_t binarizedSensors[NUM_OF_SENSORS], sensorsScaledAndConstrained[NUM_OF_SENSORS];

//Encoder variables
float speedLeft = 0.0f, speedRight = 0.0f;

//UART related variables
extern uint8_t Received[11];
extern Queue_t queue_struct;
RobotParameters_t parameters;
requests_t req; //need to zero all requests
volatile uint8_t uartReady = 0, motorsEnabled = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void FillParametersFakeData();
void ZeroAllRequests();
void presetPID();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM11){ //2 ms loop
     // __disable_irq();
    static float prevSensorsFeedback = 0.0f;
    
    sensorsFeedback = sensorsRawToBin(ADC_RAW, sensorsScaledAndConstrained, parameters.sensors.calibrationMaxValues, parameters.sensors.calibrationMinValues, binarizedSensors);
    //PD(&PID_Sensors, &sensorsSetpoint, &sensorsFeedback, &sensorsOutput, (derivative_t)parameters.sensors.derivativeType, noReverse);
    float PDout = (sensorsFeedback * PID_Sensors.kp) - ((sensorsFeedback - prevSensorsFeedback) * PID_Sensors.kd);
    if(PDout > 100.0f) PDout = 100.0f;
    if(PDout < -100.0f) PDout = -100.0f;
    
    leftSet = parameters.left.SpeedOffset + PDout;
    rightSet = parameters.right.SpeedOffset - PDout;
    
    if(leftSet < 0.0f) leftSet = 0.0f;
    if(rightSet < 0.0f) rightSet = 0.0f;
    leftSet = constrainSetSpeed(leftSet, parameters.left.MaxSpeed);
    rightSet = constrainSetSpeed(rightSet, parameters.right.MaxSpeed);
       
 
    calculateSpeed(&speedLeft, &speedRight);
    PID(&PID_Left, &leftSet, &speedLeft, &leftOutput, (derivative_t)parameters.left.derivativeType, noReverse);
    PID(&PID_Right, &rightSet, &speedRight, &rightOutput, (derivative_t)parameters.right.derivativeType, noReverse);
     
    if(motorsEnabled)
    {
     Motor1DriveByPid((int)leftOutput); //motor1 - lewy
     Motor2DriveByPid((int)rightOutput);
    }
    prevSensorsFeedback = sensorsFeedback;
  }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  q_init(&queue_struct, sizeof(QueueRecord_t), 25, FIFO, false);
  
  FillParametersFakeData(); //fake parameters struct
  
  ReadFromFlash(&parameters, sizeof(parameters), SECTOR5_FLASH_BEGINING);
  
  ZeroAllRequests(); //zero requests from parameters struct
  
  presetPID();
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint64_t TickStart = 0, TickPrevious = 0, TickStartShort, TickPreviousShort = 0;
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  Motor1StartPWM(); //17,5 kHz
  Motor2StartPWM(); //17,5 kHz
  Motor1DriveByPid(0);
  Motor2DriveByPid(0);
  HAL_TIM_Base_Start_IT(&htim11); //2ms IRQ 
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_RAW, ADC_MEASUREMENTS); //25 us whole cycle
  HAL_UART_Receive_DMA(&huart1, Received, SIZEOF_RECEIVING_BUFFER);  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    TickStart = HAL_GetTick();
    TickStartShort = TickStart;
    
    if(req.request_calibration_begin)
    {
      RAW_ADC_calibrateEachSensor(ADC_RAW, parameters.sensors.calibrationMaxValues, parameters.sensors.calibrationMinValues);
      uart_write_calibration_result(CalibrationMaxValues, parameters.sensors.calibrationMaxValues);
      uartReady = 1;
      uart_write_calibration_result(CalibrationMinValues, parameters.sensors.calibrationMinValues);
    }
  
   if((TickStartShort - TickPreviousShort) > 8)
   {
    checkIfReady(); //enqueue msg and send it
    TickPreviousShort = TickStartShort;
   }
    
    if((TickStart - TickPrevious) > parameters.misc.uartUpdateRate)
    {
      HAL_GPIO_TogglePin(GPIOC, LED_Pin);
      
      if(req.request_sensor_update)
      {
        uint8_t tmp_adc[] = {ADC_RAW[0] >> 5, ADC_RAW[1] >> 5, ADC_RAW[2] >> 5, ADC_RAW[3] >> 5, ADC_RAW[4] >> 5, ADC_RAW[5] >> 5, ADC_RAW[6] >> 5, ADC_RAW[7] >> 5, ADC_RAW[8] >> 5, ADC_RAW[9] >> 5};
        uart_write_sensors_status(tmp_adc);
        uartReady = 1;
        uart_write_binarized_sensors(binarizedSensors);
        uart_write_calibrated_sensors(sensorsScaledAndConstrained);
        uart_write_voltage(ADC_RAW[12], (int16_t)(sensorsFeedback));
        //uart_write_feedback(0, 0, 0, (int8_t)speedLeft, (int8_t)PID_Left.integralSum , (int8_t)speedRight, (int8_t)PID_Right.integralSum, ADC_RAW[10] >> 5, ADC_RAW[11] >> 5);
      }
      TickPrevious = TickStart;
    }
    
    //requests
    if(req.request_all_parameters)
    {
      uart_write_parameters(&parameters);
      req.request_all_parameters = 0;
    }
    if( req.request_write_flash)
    {
      WriteToFlash(&parameters, sizeof(parameters), SECTOR5_FLASH_BEGINING, FLASH_SECTOR_5, FLASH_VOLTAGE_RANGE_3);
      req.request_write_flash = 0;
    }
    if(req.request_start_race == 1)
    {
      motorsEnabled = 1;
      req.request_start_race = 0;
      PID_Left.integralSum = 0.0f;
      PID_Right.integralSum = 0.0f;
    }else if(req.request_start_race == 255) //STOP
    {
      motorsEnabled = 0;
      Motor1DriveByPid(0);
      Motor2DriveByPid(0);
      req.request_start_race = 0;
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void FillParametersFakeData()
{
  parameters.sensors.kp = 1.00;
  parameters.sensors.kd = 2.00;
  parameters.sensors.derivativeType = feedbackDerivative;
  parameters.misc.uartUpdateRate = 75;
  
  parameters.left.kp = 0;
  parameters.left.ki = 0;
  parameters.left.kd = 3.30;
  parameters.left.derivativeType = feedbackDerivative;
  parameters.left.I_limit = 4.00;
  parameters.left.MaxSpeed = 5.00;
  parameters.left.MinSpeed = 6.00;
  parameters.left.SpeedOffset = 7.00;
  
  parameters.right.kp = 3.14;
  parameters.right.ki = 3.24;
  parameters.right.kd = 3.34;
  parameters.right.derivativeType = feedbackDerivative;
  parameters.right.I_limit = 4.14;
  parameters.right.MaxSpeed = 5.14;
  parameters.right.MinSpeed = 6.14;
  parameters.right.SpeedOffset = 7.14;
}

void ZeroAllRequests()
{
  req.request_all_parameters = 0;
  req.request_enable_feedback = 0;
  req.request_calibration_begin = 0;
  req.request_sensor_update = 0;
  req.request_start_race = 0;
  req.request_write_flash = 0;
  parameters.misc.uartUpdateRate = 100;
  
  for(int i = 0; i < NUM_OF_SENSORS; i++)
  {
    parameters.sensors.calibrationMaxValues[i] = 0;
    parameters.sensors.calibrationMinValues[i] = 255;
  }
}

void presetPID()
{
  parameters.left.MaxSpeed = parameters.left.I_limit;
  PID_Left.negIntegralLimit = -1.0*parameters.left.I_limit;
  PID_Left.posIntegralLimit = parameters.left.I_limit;
  PID_Left.negOutputLimit = -1.0*parameters.left.I_limit;
  PID_Left.posOutputLimit = parameters.left.I_limit;
  PID_Left.period = 2;
  PidSetParams(&PID_Left, parameters.left.kp, parameters.left.ki, parameters.left.kd);
  
  parameters.right.MaxSpeed = parameters.right.I_limit;
  PID_Right.negIntegralLimit = -1.0*parameters.right.I_limit;
  PID_Right.posIntegralLimit = parameters.right.I_limit;
  PID_Right.negOutputLimit = -1.0*parameters.right.I_limit;
  PID_Right.posOutputLimit = parameters.right.I_limit;
  PID_Right.period = 2;
  PidSetParams(&PID_Right, parameters.right.kp, parameters.right.ki, parameters.right.kd);
  
  PID_Sensors.negIntegralLimit = 0.0f;
  PID_Sensors.posIntegralLimit = 0.0f;
  PID_Sensors.negOutputLimit = -100.0f;
  PID_Sensors.posOutputLimit = 100;
  PID_Sensors.period = 2;
  PidSetParams(&PID_Sensors, parameters.sensors.kp, 0.0f, parameters.sensors.kd);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
