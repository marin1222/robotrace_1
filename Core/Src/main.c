/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(double analog2)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int analog2, FILE *f)
#endif /*__GNUC__*/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void    init(void);        
void    led_out(void);      
void    MotorCtrl(void);    
void    LineTrace(void);    
void    SensUp(void);       
void    Side_SensUp(void);       
int     GetVol(void);       
void    Stop(void);

int timer1=0;
int ledval=1;
int MotorL_Rev=0,MotorR_Rev=0;
int MotorL=0,MotorR=0;
int ErrFlg=0;
int SensValBuf=0;
int SensorR,SensorL;
int Side_SensorR,Side_SensorL;
int Curve_Sensor;
int SensR[9],SensL[9];
int Side_SensR[9],Side_SensL[9],Curve_Sens[9];
double PGainCLB=0;
double IGainCLB=0;
double DGainCLB=0;
double SensVal_I,SensVal_IBuf=0;
char turnFlg=0;
char timeFlg=0;
char stopFlg=0;
char Stime=0;
int CommSpeed=9500;
char buf[1];
uint16_t analog1[2];
uint16_t analog2[4];
uint8_t button_state = 1;

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Transmit_IT(&huart2, (uint16_t *)analog2, sizeof(analog2)/sizeof(analog2[0]));
	HAL_UART_Receive_IT(&huart2, (uint16_t *)analog2, sizeof(analog2)/sizeof(analog2[0]));
 }*/

void SensUp(void){
int i,j,SensBuf;
  for (i = 0; i < 9; i++) {
    for (j = 9; j > i; j--) {  
      if (SensR[j-1] > SensR[j]) {
        SensBuf = SensR[j-1];
        SensR[j-1] = SensR[j];
        SensR[j] = SensBuf;
      }
      if (SensL[j-1] > SensL[j]) {
        SensBuf = SensL[j-1];
        SensL[j-1] = SensL[j];
        SensL[j] = SensBuf;
      }
    }
  }

    SensorR = SensR[5];
    SensorL = SensL[5];
    }

void Side_SensUp(void){
int i,j,Side_SensBuf;
  for (i = 0; i < 9; i++) {
    for (j = 9; j > i; j--) {
      if (Side_SensR[j-1] > Side_SensR[j]) {
        Side_SensBuf = Side_SensR[j-1];
        Side_SensR[j-1] = Side_SensR[j];
        Side_SensR[j] = Side_SensBuf;    
      }
      if (Side_SensL[j-1] > Side_SensL[j]) {
        Side_SensBuf = Side_SensL[j-1];
        Side_SensL[j-1] = Side_SensL[j];
        Side_SensL[j] = Side_SensBuf;
      }
      if (Curve_Sens[j-1] > Curve_Sens[j]) {
        Side_SensBuf = Curve_Sens[j-1];
        Curve_Sens[j-1] = Curve_Sens[j];
        Curve_Sens[j] = Side_SensBuf;
      }
    }
  }
    Side_SensorR = Side_SensR[5];
    Side_SensorL = Side_SensL[5];
    Curve_Sensor = Curve_Sens[5];
    }

void sensGet(){
    if(Stime >= 9) {
        Stime=0;
        } 
    SensR[Stime] = analog1[1];
    SensL[Stime] = analog1[0];
    Side_SensL[Stime] = analog2[1];
    Side_SensR[Stime] = analog2[1];
    Curve_Sens[Stime] = analog2[0];
    Stime++;
    }  
void LineTrace(void){
  int SensVal,SensVal_D;
  double PGain=70.1,IGain=0.000001,DGain=0.0;
  if(stopFlg >= 3) CommSpeed=0;
  SensVal = SensorR - SensorL;
  if(SensVal <= 0 && !turnFlg){       
    turnFlg=1;
    SensVal_I = 0;
  }else if (SensVal >= 0 && turnFlg){//turnR
    turnFlg=0;
    SensVal_I = 0;
  }
  SensVal_I = SensVal_I + SensVal;     
  SensVal_IBuf = SensVal_IBuf + SensVal;
  if(SensVal_I >= 100000000) SensVal_I = 100000000;
  if(SensVal_I <= (-100000000)) SensVal_I = (-100000000);
  SensVal_D = SensValBuf - SensVal;       //左右のセンサー値の差の変化
  SensValBuf = SensVal;
  MotorR = ((CommSpeed - ((SensVal * PGain)+(SensVal_I*IGain)-(SensVal_D*DGain)))/10); 
  if(MotorL < 0 ){
      MotorL = MotorL * (-1);
  }
  MotorL = ((CommSpeed + ((SensVal * PGain)+(SensVal_I*IGain)-(SensVal_D*DGain)))/10);
  if(MotorR < 0 ){
      MotorR = MotorR * (-1);
  }
  ledval = (SensVal_IBuf / (-100000));
  //MotorR = int(CommSpeed - (SensVal * PGainCLB));
  //MotorL = int(CommSpeed + (SensVal * PGainCLB));
  }
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
    if(htim->Instance == TIM2){
      sensGet();
    }
    if(htim->Instance == TIM3){
      SensUp();
      Side_SensUp();
      LineTrace();
    }
  }/*
  PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint16_t *)&analog2, 1, 0xFFFF);
	return analog2;
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,1);
  HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,1);
  HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,0);
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,0);
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *) analog1, 2) != HAL_OK){
    Error_Handler();
  }
  HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,0);
  if(HAL_ADC_Start_DMA(&hadc2, (uint32_t *) analog2, 4) != HAL_OK){
    Error_Handler();
  }
  while(button_state){
    button_state = HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin);
    HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,0);

  }
  //HAL_UART_Receive_IT(&huart2, (uint16_t *)analog2, sizeof(analog2)/sizeof(analog2[0]));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,MotorL);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,MotorR);

    if(Curve_Sensor<2550){
      CommSpeed=10000;
      HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin);
      HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,0);
    }else{
      CommSpeed=9500;
      HAL_GPIO_TogglePin(LED_4_GPIO_Port,LED_4_Pin);
      HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,0);
    }
    //HAL_UART_Transmit(&huart2, (uint16_t *)analog1, sizeof(analog2)/sizeof(analog2[0]), 0xFFFF);
   // HAL_UART_Receive(&huart2, (uint16_t *)analog1, sizeof(analog2)/sizeof(analog2[0]), 0xFFFF);
    printf("%d\r\n",Curve_Sens[0]);
    //HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,analog2[3]%2);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint16_t *)ptr,len,10);
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
