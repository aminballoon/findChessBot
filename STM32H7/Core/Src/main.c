/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//typedef RS485
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _FCY 240000000U
#define _PSC 240

#define STEPJ1 1
#define STEPJ2 2
#define STEPJ3 3
#define STEPJ4 4
#define STEPGripper 5

#define BUFFSIZE 4

#define ACK_ReceivedData_Address (uint8_t)0xAC
#define ACK_ProcessIsCompleted_Address (uint8_t)0xAD
#define ACK_CheckSumError_Address (uint8_t)0xEE

#define ENCPOS_JOINT1_Address (uint8_t)0xA4
#define ENCPOS_JOINT2_Address (uint8_t)0xB4
#define ENCPOS_JOINT3_Address (uint8_t)0xC4
#define ENCPOS_JOINT4_Address (uint8_t)0xD4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM12_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
double q1, q2, q3, q4;
double c0, c1, c2, c3, c4, c5;
volatile int16_t POSCNT[4];
bool State_Input_Joint_State;
bool State_Print_4_Joint_State;
bool State_Print_Gripper_State;
bool State_Checksum_Error;
bool State_Set_Home;
bool State_Activate_Gripper;
bool State_Deactivate_Gripper;
bool State_PID_Control_Timer;
bool State_Casade_Control_Timer;
uint8_t UART3_RXBUFFER[4], UART3_TXBUFFER_ACK[1];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/*
 * Polling RS485-Encoder Communication Non-void Function
 * Updated : 18 Mar 2021 16:44
 * */
uint16_t RS485Encoder(uint8_t _address)
{
	uint8_t _buff[2];
	volatile uint8_t checkbit_odd[7], checkbit_even[7];
	volatile char checkbit_odd_result, checkbit_even_result;
//	static uint16_t POSCNT[4];
	HAL_UART_Transmit(&huart4, &_address, 1, 2);
	if(HAL_UART_Receive(&huart4, _buff, 2, 2) == HAL_OK) // Check received data is completed.
	{
		/*
		 * Checksum
		 *
		 * The AMT21 encoder uses a checksum calculation for detecting transmission errors.
		 *
		 * The upper two bits of every response from the encoder are check bits.
		 *
		 * Those values are shown in the examples below as K1 and K0.
		 *
		 * The check bits are odd parity; K1 for the odd bits in the response,
		 * and K0 for the even bits in the response.
		 *
		 * These check bits are not part of the position,
		 * but are used to verify its validity. The remaining lower 14 bits are the useful data.
		 *
		 * Checkbit Formula
		 * Odd: K1 = !(H5^H3^H1^L7^L5^L3^L1)
		 * Even: K0 = !(H4^H2^H0^L6^L4^L2^L0)
		 */

		for (register int i = 0; i < 7; i++)
		{
			if(i < 3){
			  checkbit_odd[i] = (_buff[1] >> (7-(2*(i+1)))) & 0x01;
			  checkbit_even[i] = (_buff[1] >> (6-(2*(i+1)))) & 0x01;
			}
			else{
			  checkbit_odd[i] = (_buff[0] >> (7-(2*(i-3)))) & 0x01;
			  checkbit_even[i] = (_buff[0] >> (6-(2*(i-3)))) & 0x01;
			}
			checkbit_odd_result ^= checkbit_odd[i];
			checkbit_even_result ^= checkbit_even[i];
		}

		checkbit_odd_result = !checkbit_odd_result;
		checkbit_even_result = !checkbit_even_result;

		if(checkbit_odd_result == ((_buff[1] >> 7) & 0x01) && (checkbit_even_result) == ((_buff[1] >> 6) & 0x01)) //  If checksum is correct.
		{
			switch (_address){
				case ENCPOS_JOINT1_Address:
					POSCNT[0] = (uint16_t)(_buff[0] + ((_buff[1] & 0x3F) << 8));
					break;
				case ENCPOS_JOINT2_Address:
					POSCNT[1] = (uint16_t)(_buff[0] + ((_buff[1] & 0x3F) << 8));
					break;
				case ENCPOS_JOINT3_Address:
					POSCNT[2] = (uint16_t)(_buff[0] + ((_buff[1] & 0x3F) << 8));
					break;
				case ENCPOS_JOINT4_Address:
					POSCNT[3] = (uint16_t)(_buff[0] + ((_buff[1] & 0x3F) << 8));
					break;
			}
		}
	}
	switch (_address){
		case ENCPOS_JOINT1_Address:
			return POSCNT[0];
			break;
		case ENCPOS_JOINT2_Address:
			return POSCNT[1];
			break;
		case ENCPOS_JOINT3_Address:
			return POSCNT[2];
			break;
		case ENCPOS_JOINT4_Address:
			return POSCNT[3];
			break;
	}
	return -1;
}
void RS485ResetEncoder(uint8_t _address)
{
	if(_address == ENCPOS_JOINT1_Address || _address == ENCPOS_JOINT2_Address || _address == ENCPOS_JOINT3_Address || _address == ENCPOS_JOINT4_Address)
	{
		HAL_UART_Transmit(&huart4, (&_address + (uint8_t)0x02), 1, 1);
		HAL_UART_Transmit(&huart4, (&_address + (uint8_t)0x0A), 1, 1);
		switch (_address){
			case ENCPOS_JOINT1_Address:
				POSCNT[0] = 0;
				break;
			case ENCPOS_JOINT2_Address:
				POSCNT[1] = 0;
				break;
			case ENCPOS_JOINT3_Address:
				POSCNT[2] = 0;
				break;
			case ENCPOS_JOINT4_Address:
				POSCNT[3] = 0;
				break;
		}
	}
}
/*
 * Stepper motor driving function (Radian input)
 * Updated : 18 Mar 2021 16:44
 * */
void StepDriveRad(char _ch, double _ang_v)
{
	switch(_ch)
	{
		case STEPJ1:
		{
			/* Direction of Joint1's Stepper Motor */
			if(_ang_v < 0)
			{
				HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint1's Stepper Motor */
			if(_ang_v == 0) // To avoid TIM1->ARR is undefined value.
			{
				TIM1->CCR2 = 0;
				TIM1->ARR = 625-1;
			}
			else
			{
//				if(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) == HAL_OK)
//				{
//					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//				}
				TIM1->ARR = round((6.283*_FCY)/(1600*((TIM1->PSC)+1)*abs(_ang_v))) - 1;
				TIM1->CCR2 = round(((TIM1->ARR)+1)/2);
			}
			break;
		}
		case STEPJ2:
		{
			/* Direction of Joint2's Stepper Motor */
			if(_ang_v < 0)
			{
				HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint2's Stepper Motor */
			if(_ang_v == 0) // To avoid TIM2->ARR is undefined value.
			{
				TIM2->CCR3 = 0;
				TIM2->ARR = 625-1;
			}
			else
			{
				TIM2->ARR = round((6.283*_FCY)/(1600*((TIM2->PSC)+1)*abs(_ang_v))) - 1;
				TIM2->CCR3 = round(((TIM2->ARR)+1)/2);
			}
			break;
		}
		case STEPJ3:
		{
			/* Direction of Joint3's Stepper Motor */
			if(_ang_v < 0)
			{
				HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint3's Stepper Motor */
			if(_ang_v == 0) // To avoid TIM3->ARR is undefined value.
			{
				TIM3->CCR1 = 0;
			}
			else
			{
				TIM3->ARR = round((6.283*_FCY)/(1600*((TIM3->PSC)+1)*abs(_ang_v))) - 1;
				TIM3->CCR1 = round(((TIM3->ARR)+1)/2);
			}
			break;
		}
		case STEPJ4:
		{
			/* Direction of Joint4's Stepper Motor */
			if(_ang_v < 0)
			{
				HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint4's Stepper Motor */
			if(_ang_v == 0) // To avoid TIM4->ARR is undefined value.
			{
				TIM4->CCR3 = 0;
			}
			else
			{
				TIM4->ARR = round((6.283*_FCY)/(1600*((TIM4->PSC)+1)*abs(_ang_v))) - 1;
				TIM4->CCR3 = round(((TIM4->ARR)+1)/2);
			}
			break;
		}
		case STEPGripper:
		{
			/* Direction of Gripper's Stepper Motor */
			if(_ang_v < 0)
			{
				HAL_GPIO_WritePin(DIR_5_GPIO_Port, DIR_5_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_5_GPIO_Port, DIR_5_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Gripper's Stepper Motor */
			if(_ang_v == 0) // To avoid TIM2->ARR is undefined value.
			{
				TIM15->CCR2 = 0;
			}
			else
			{
				TIM15->ARR = round((6.283*_FCY)/(1600*((TIM15->PSC)+1)*abs(_ang_v))) - 1;
				TIM15->CCR2 = round(((TIM15->ARR)+1)/2);
			}
			break;
		}
		default:
		{
//			TIM1->CCR2 = 0;
//			TIM2->CCR3 = 0;
//			TIM3->CCR1 = 0;
//			TIM4->CCR3 = 0;
//			TIM15->CCR2 = 0;
		}
	}

}
/*
 * Stepper motor driving function (RPS input)
 * Updated : 18 Mar 2021 16:44
 * */
void StepDriveRPS(char _ch, double _rps)
{
	switch(_ch)
	{
		case STEPJ1:
		{
			/* Direction of Joint1's Stepper Motor */
			if(_rps < 0)
			{
				HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint1's Stepper Motor */
			if(_rps == 0) // To avoid TIM1->ARR is undefined value.
			{
				TIM1->CCR2 = 0;
			}
			else
			{
				TIM1->ARR = round(_FCY/(1600*((TIM1->PSC)+1)*abs(_rps))) - 1;
				TIM1->CCR2 = round(((TIM1->ARR)+1)/2);
			}
			break;
		}
		case STEPJ2:
		{
			/* Direction of Joint2's Stepper Motor */
			if(_rps < 0)
			{
				HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint2's Stepper Motor */
			if(_rps == 0) // To avoid TIM2->ARR is undefined value.
			{
				TIM2->CCR3 = 0;
			}
			else
			{
				TIM2->ARR = round(_FCY/(1600*((TIM2->PSC)+1)*abs(_rps))) - 1;
				TIM2->CCR3 = round(((TIM2->ARR)+1)/2);
			}
			break;
		}
		case STEPJ3:
		{
			/* Direction of Joint3's Stepper Motor */
			if(_rps < 0)
			{
				HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint3's Stepper Motor */
			if(_rps == 0) // To avoid TIM3->ARR is undefined value.
			{
				TIM3->ARR = 625-1;
				TIM3->CCR1 = 0;
			}
			else
			{
				TIM3->ARR = round(_FCY/(1600*((TIM3->PSC)+1)*abs(_rps))) - 1;
				TIM3->CCR1 = round(((TIM3->ARR)+1)/2);
			}
			break;
		}
		case STEPJ4:
		{
			/* Direction of Joint4's Stepper Motor */
			if(_rps < 0)
			{
				HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Joint4's Stepper Motor */
			if(_rps == 0) // To avoid TIM4->ARR is undefined value.
			{
				TIM4->ARR = 625-1;
				TIM4->CCR3 = 0;
			}
			else
			{
				TIM4->ARR = round(_FCY/(1600*((TIM4->PSC)+1)*abs(_rps))) - 1;
				TIM4->CCR3 = round(((TIM4->ARR)+1)/2);
			}
			break;
		}
		case STEPGripper:
		{
			/* Direction of Gripper's Stepper Motor */
			if(_rps < 0)
			{
				HAL_GPIO_WritePin(DIR_5_GPIO_Port, DIR_5_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(DIR_5_GPIO_Port, DIR_5_Pin, GPIO_PIN_RESET);
			}
			/* Angular Velocity of Gripper's Stepper Motor */
			if(_rps == 0) // To avoid TIM2->ARR is undefined value.
			{
				TIM15->CCR2 = 0;
			}
			else
			{
				TIM15->ARR = round(_FCY/(1600*((TIM15->PSC)+1)*abs(_rps))) - 1;
				TIM15->CCR2 = round(((TIM15->ARR)+1)/2);
			}
			break;
		}
		default:
		{
//			TIM1->CCR2 = 0;
//			TIM2->CCR3 = 0;
//			TIM3->CCR1 = 0;
//			TIM4->CCR3 = 0;
//			TIM15->CCR2 = 0;
		}
	}
}
void StepStop(char _ch)
{
	switch(_ch)
		{
			case STEPJ1:
			{
				TIM1->CCR2 = 0;
			}
			case STEPJ2:
			{
				TIM2->CCR3 = 0;
			}
			case STEPJ3:
			{
				TIM3->CCR1 = 0;
			}
			case STEPJ4:
			{
				TIM4->CCR3 = 0;
			}
			case STEPGripper:
			{
				TIM15->CCR2 = 0;
			}
			default:
			{
//				TIM1->CCR2 = 0;
//				TIM2->CCR3 = 0;
//				TIM3->CCR1 = 0;
//				TIM4->CCR3 = 0;
//				TIM15->CCR2 = 0;
			}
		}
}
//uuint16_t SPI_Encoder()
//{
//
//}
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
//  SysTick->LOAD = 480000 - 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_TIM4_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_TIM12_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

//  HAL_TIM_Base_Start_IT(&htim5);
//  HAL_TIM_Base_Start_IT(&htim12);
  TIM1->CCR2 = 0;
  TIM2->CCR2 = 0;
  TIM3->CCR2 = 0;
  TIM4->CCR2 = 0;
  TIM15->CCR2 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
  HAL_UART_Receive_IT(&huart3, UART3_RXBUFFER, 4);
  StepDriveRad(1, 6.23);
//  double i = 0.00;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(State_Checksum_Error)
	  {
		  State_Checksum_Error = 0;
		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_CheckSumError_Address;
		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
	  }
	  if(State_Input_Joint_State)
	  {
		  State_Input_Joint_State = 0;
		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
	  }
	  if(State_Print_4_Joint_State)
	  {
		  State_Print_4_Joint_State = 0;
//		  printf("\n%3d %3d %3d %3d\n\r", q1, q2, q3, q4);
		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
	  }
	  if(State_Activate_Gripper)
	  {
		  State_Activate_Gripper = 0;
		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
	  }
	  if(State_Deactivate_Gripper)
	  {
		  State_Deactivate_Gripper = 0;
		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
	  }
	  if(State_Set_Home)
	  {
		  State_Set_Home = 0;
		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
	  }
	  if(State_PID_Control_Timer)
	  {
//		  HAL_TIM_Base_Start_IT(&htim5);
		  State_PID_Control_Timer = 0;
	  }
	  if(State_Casade_Control_Timer)
	  {
//		  HAL_TIM_Base_Start_IT(&htim12);
		  State_Casade_Control_Timer = 0;
	  }

//	  i++;
	  HAL_Delay(500);
  }
  return 0;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 625-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 313;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 240-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 625-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 313;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 240-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 625-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 313;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 240-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 625-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 313;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 240-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 500-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 240-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 500-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 240-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 625-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 313;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|DIR_2_Pin|LD3_Pin|DIR_3_Pin
                          |DIR_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR_1_Pin|DIR_5_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin DIR_2_Pin LD3_Pin DIR_3_Pin
                           DIR_4_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|DIR_2_Pin|LD3_Pin|DIR_3_Pin
                          |DIR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LM2_Pin LM1_Pin LM6_Pin */
  GPIO_InitStruct.Pin = LM2_Pin|LM1_Pin|LM6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_1_Pin DIR_5_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DIR_1_Pin|DIR_5_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LM5_Pin */
  GPIO_InitStruct.Pin = LM5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LM5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LM4_Pin LM3_Pin */
  GPIO_InitStruct.Pin = LM4_Pin|LM3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/** Usable for printf function **/
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART2 and Loop until the end of transmission */
 HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ReceivedData_Address;
		HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		volatile uint8_t num_mode = UART3_RXBUFFER[0] & 0x0F;
		volatile int received_checksum = UART3_RXBUFFER[3];
		volatile int calculate_checksum = 0;
		for(register int i = 0; i < 3; i++)
		{
			calculate_checksum += UART3_RXBUFFER[i];
		}
		calculate_checksum = ~calculate_checksum;
		calculate_checksum = calculate_checksum & 0xFF;
		if (received_checksum == calculate_checksum)
		{
			switch(num_mode)
			{
				case 6:		// q1 Mode
				{
					q1 = (uint16_t)(((UART3_RXBUFFER[1] << 8) & 0xFF00) + (UART3_RXBUFFER[2] & 0x00FF));
					State_Input_Joint_State = 1;
					break;
				}
				case 7:		// q2 Mode
				{
					q2 = (uint16_t)(((UART3_RXBUFFER[1] << 8) & 0xFF00) + (UART3_RXBUFFER[2] & 0x00FF));
					State_Input_Joint_State = 1;
					break;
				}
				case 8:		// q3 Mode
				{
					q3 = (uint16_t)(((UART3_RXBUFFER[1] << 8) & 0xFF00) + (UART3_RXBUFFER[2] & 0x00FF));
					State_Input_Joint_State = 1;
					break;
				}
				case 9:		// q4 Mode
				{
					q4 = (uint16_t)(((UART3_RXBUFFER[1] << 8) & 0xFF00) + (UART3_RXBUFFER[2] & 0x00FF));
					State_Input_Joint_State = 1;
					break;
				}
				case 10:	// Set Home Mode
				{
					State_Set_Home = 1;
					break;
				}
				case 1:
				{
					break;
				}
				case 2:
				{
					break;
				}
				case 3:
				{
					break;
				}
				case 4:
				{
					break;
				}
				case 5:
				{
					break;
				}
				case 11:	// Request 4 Joint State Mode
				{
					State_Print_4_Joint_State = 1;
					break;
				}
				case 12:	// Request Gripper State Mode
				{
					State_Print_Gripper_State = 1;
					break;
				}
				case 13:	// Activate Gripper Mode
				{
					State_Activate_Gripper = 1;
					break;
				}
				case 14:	// Deactivate Gripper Mode
				{
					State_Deactivate_Gripper = 1;
					break;
				}
				case 15:
				{
					break;
				}
				default:
				{
					break;
				}
			}
			  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}
		else
		{
			  State_Checksum_Error = 1;
			  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}
		HAL_UART_Receive_IT(&huart3, UART3_RXBUFFER, BUFFSIZE);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*
	 * Default of Sampling Frequency : 2kHz.
	 *
	 * How to change frequency as follows:
	 * Change on TIMx's ARR Register (Input is frequency).
	 * TIMx->ARR = round(_FCY/((TIMx->PSC)+1)*freq) - 1;
	 *
	 * To turn ON/OFF
	 * Use function
	 */
  /* Timer5 Interrupt PID Position Control*/
  if (htim == &htim5)
  {

  }
  /* Timer12 Interrupt */
  if (htim == &htim12)
  {

  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
