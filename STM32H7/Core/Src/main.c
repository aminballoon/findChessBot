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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* PID Structure */
typedef struct findchessPID_t
{
	float Kp;
	float Ki;
	float Kd;
	float I_Err;
	float D_Err;
	float I_MinLimit;
	float I_MaxLimit;
	float outputMinLimit;
	float outputMaxLimit;
	float DeadBand;
	float Prev_Input;
} findchessPID_t;

typedef struct findchessJoint_t{
	float joint_config;
	findchessPID_t PIDPosition;
	findchessPID_t PIDVelocity;
} findchessJoint_t;

typedef struct findchessTaskspace_t{
	float X;
	float Y;
	float Z;
	float endEffYaw;
} findchessTaskspace_t;
findchessPID_t PID_Joint1, PID_Joint2, PID_Joint3, PID_Joint4, PID_JointGripper;
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

#define pi 3.1412

#define RES12           (uint8_t)0x0C
#define RES14           (uint8_t)0x0E
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
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
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
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max);
double mapf(double val, double in_min, double in_max, double out_min, double out_max);
void Update_Coff(int x1,int y1,int x2,int y2,float Time);
void PIDInit(findchessPID_t *_PID, float _Kp, float _Ki, float _Kd, float _Iminlimit, float _Imaxlimit, float _Ddbd);
float PIDCalculate(findchessPID_t *_PID, float _setPoint, float _inputValue);
void IPK_findChessBot(float X, float Y, float Z, float endEff_Yaw);
void StepDriveRad(char _ch, double _ang_v);
void servoGripper(long dutycycle);
void StepStop(char _ch);
void delayMicroseconds(uint32_t us);
uint16_t CRC16(uint8_t *buf, int len);
uint16_t AMT21_getPositionModbusRTU(uint8_t _device_addr, uint8_t resolution);
uint16_t AMT21_getPositionRS485(uint8_t _device_addr, uint8_t resolution);
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

float POSCNT[4] = {0.,0.,0.,0.};
float sample_time = 0.0005; // 2 khz
float q[4] = {0.,0.,0.,0.};
float dq[4] = {0.,0.,0.,0.};
float ddq[4] = {0.,0.,0.,0.};
float task_space[4] = {0.,0.,0.,0.};
float q_s[5], q_g[5], theta_q[5], ang_pos[5], ang_vel[5], ang_acc[5];
float C0x = 0.705,C2x,C3x,C0y,C2y,C3y;
float T;
float t = 0.;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void servoGripper(long dutycycle)
{
  if(dutycycle >= 0 && dutycycle <= 180)
  {
      long x = map(dutycycle, 0, 180, 0, 100);
      TIM15->CCR2 = x;
  }
  else{
      TIM15->CCR2 = 0;
  }
}
void delayMicroseconds(uint32_t us)
{
    __IO uint32_t currentTicks = SysTick->VAL;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  __IO uint32_t oldTicks = currentTicks;
  do {
    currentTicks = SysTick->VAL;
    elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks :
                    oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
}
/*
 * CRC16 Checksum for RS485 Modbus RTU Communication
 * Updated : 20 May 2021
 * */
uint16_t CRC16(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];        // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                        // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }

  return crc;
}

/*
 *
 * This function gets the absolute position from the AMT21 encoder using the RS485. The AMT21 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 *
 * AMT21 Encoder with RS485 Modbus RTU Communication Function
 * Updated : 20 May 2021
 * */
uint16_t AMT21_getPositionModbusRTU(uint8_t _device_addr, uint8_t resolution) {
  volatile uint8_t _dat[3] = {0,0,0};
  uint16_t currentPosition = 0;       //16-bit response from encoder;
  bool binaryArray[16];
  uint8_t buff[8] = {
    (uint8_t)_device_addr, // Devices Address
    (uint8_t)0x04, // Function code
    (uint8_t)0x00, // Start Address HIGH
    (uint8_t)0x01, // Start Address LOW
    (uint8_t)0x00, // Quantity HIGH
    (uint8_t)0x01, // Quantity LOW
    (uint8_t)0x00, // CRC LOW
    (uint8_t)0x00  // CRC HIGH
  };
  uint16_t crc = CRC16(&buff[0], 6);
  buff[6] = (uint8_t)(crc & 0xFF);
  buff[7] = (uint8_t)((crc >> 8) & 0xFF);
  HAL_UART_Transmit(&huart4, buff, 8, 1000);
  delayMicroseconds(30);
  HAL_UART_Receive(&huart4, (uint8_t *) &_dat, 3, 100);
  if(_dat[0] != 2){
      currentPosition = 0xFFFF;
      return currentPosition;
    }
  currentPosition = (currentPosition | _dat[1]) << 8;
  currentPosition = currentPosition | _dat[2];
  for(int i = 0; i < 16; i++){
      binaryArray[i] = (0x01) & (currentPosition >> (i));
  }
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition = currentPosition & 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }
  if ((resolution == RES12) && (currentPosition != 0xFFFF)){
      currentPosition = currentPosition >> 2;
  }
  return currentPosition;
}

/*
 *
 * This function gets the absolute position from the AMT21 encoder using the RS485. The AMT21 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 *
 * AMT21 Encoder with RS485 Communication Function
 * Updated : 20 May 2021
 * */
uint16_t AMT21_getPositionRS485(uint8_t _device_addr, uint8_t resolution){
//  uint16_t _dat;
  volatile uint8_t rawData[2];
  uint16_t currentPosition = 0;       //16-bit response from encoder;
  bool binaryArray[16];
  uint8_t buff[1] = {(uint8_t)_device_addr};
  HAL_UART_Transmit(&huart4, buff, 1, 100);
  delayMicroseconds(30);
  HAL_UART_Receive(&huart4, (uint8_t *) &rawData, 2, 100);
  currentPosition = (rawData[0] << 8) | rawData[1];
  for(int i = 0; i < 16; i++){
      binaryArray[i] = (0x01) & (currentPosition >> (i));
  }
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition = currentPosition & 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }
  if ((resolution == RES12) && (currentPosition != 0xFFFF)){
      currentPosition = currentPosition >> 2;
  }
  return currentPosition;
}

/*
 * PID Initialization Function
 * Updated : 22 Mar 2021
 * */
void PIDInit(findchessPID_t *_PID, float _Kp, float _Ki, float _Kd, float _Iminlimit, float _Imaxlimit, float _Ddbd){
	_PID->Kp = _Kp;
	_PID->Ki = _Ki;
	_PID->Kd = _Kd;
	_PID->I_Err = 0;
	_PID->D_Err = 0;
	_PID->I_MinLimit = _Iminlimit;
	_PID->I_MaxLimit = _Imaxlimit;
	_PID->DeadBand = _Ddbd;
}
/*
 * PID Calculate Function
 * Updated : 22 Mar 2021
 * */
float PIDCalculate(findchessPID_t *_PID, float _setPoint, float _inputValue){
	static float Previous_Err = 0;
	float Err = _setPoint - _inputValue;
	if (fabs(Err) < _PID->DeadBand){
		return 0;
	}
	_PID->I_Err += Err;
	_PID->D_Err -= Err - Previous_Err;
	if (_PID->I_Err > _PID->I_MaxLimit){
	    _PID->I_Err = _PID->I_MaxLimit;
	}
	else if (_PID->I_Err < _PID->I_MinLimit){
	    _PID->I_Err = _PID->I_MinLimit;
	}
	float output = (_PID->Kp * Err + _PID->Ki * _PID->I_Err + _PID->Kd * _PID->D_Err);
	if (output > _PID->outputMaxLimit){
		output = _PID->outputMaxLimit;
	}
	else if (output < _PID->outputMinLimit){
		output = _PID->outputMinLimit;
	}
	Previous_Err = Err;
	return output;
}
//float FPK(float _q[4])
//{
//	return
//}
//float *FVK(float _q[4])
//{
//	return
//}
//float *FAK(float _q[4])
//{
//	return
//}
//void Jacobian
/*
 * Inverse Pose Kinematic Function
 * Updated : 22 Mar 2021
 * */
void IPK_findChessBot(float X, float Y, float Z, float endEff_Yaw)
{
	const float l1 = 0.020, l2 = 0.370, l3 = 0.315,
			h1 = 0.125, h3 = 0.065, h4 = 0.190;
	float l12 = l1 + l2;

	float C3 = (X*X + Y*Y - l12*l12 - l3*l3) / (2 * l12 * l3);
	float S3 = sqrt(1 - (C3 * C3));
    float q3 = atan2(S3, C3);

    float S1 = -l3 * sin(q3) * X + (l12 + l3*cos(q3)) * Y;
    float C1 = l3 * sin(q3) * Y + (l12 + l3*cos(q3)) * X;
    float q1 = atan2(S1, C1);

    /*if(q1 < -pi/2 || q1 > pi/2)
    {
        printf("Out of range for q1\n");
		return;
    }
    if(Z + h4 - h3 - h1 < 0 || Z + h4 - h3 - h1 > 0.1)
    {
		printf("Out of range for q2\n");
		return;
	}
    if(q3 < -2.705 || q3 > -0.035 || q3 < 0.035 || q3 > 2.705)
    {
        printf("Out of range for q3\n");
		return;
    }
    if(endEff_Yaw - q1 - q3 < -2.880 || endEff_Yaw - q1 - q3 > 2.880)
    {
        printf("Out of range for q4\n");
		return;
    }*/
	q[0] = q1;
	q[1] = Z + h4 - h3 - h1;
	q[2] = q3;
	q[3] = endEff_Yaw - q1 - q3;
}
void IVK()
{

}
void IAK()
{

}

/*
 * Coefficient of Trajectory Generation Function
 * Updated : 20 Mar 2021
 * */
void Update_Coff(int x1,int y1,int x2,int y2,float Time)
{
    T = Time ;
    int delta_x = x2-x1;
    int delta_y = y2-y1;
//    float Trajectory_Theta = atan2(delta_y, delta_x);
//    float Trajectory_Magnitude = sqrt((delta_y * delta_y) + (delta_x * delta_x));
    float Time_2 = Time * Time ;
    float Time_3 = Time_2 * Time;
    C0x = delta_x;
    C2x = (3*delta_x)/Time_2;
    C3x = (2*delta_x)/Time_3;

    C0y = delta_y;
    C2y = (3*delta_y)/Time_2;
    C3y = (2*delta_y)/Time_3;
}
/*
 * Polling RS485-Encoder Communication Non-void Function
 * Updated : 18 Mar 2021 16:44
 * */

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
			ang_vel[0] = _ang_v;
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
				TIM1->ARR = round((6.283*_FCY)/(1600*((TIM1->PSC)+1)*abs(_ang_v))) - 1;
				TIM1->CCR2 = round(((TIM1->ARR)+1)/2);
			}
			break;
		}
		case STEPJ2:
		{
			ang_vel[1] = _ang_v;
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
			ang_vel[2] = _ang_v;
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
				TIM3->ARR = 625-1;
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
			ang_vel[3] = _ang_v;
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
				TIM4->ARR = 625-1;
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
			ang_vel[4] = _ang_v;
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
				TIM15->ARR = 625-1;
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

		}
	}

}
void StepStop(char _ch)
{
	switch(_ch)
		{
			case STEPJ1:
			{
				ang_vel[0] = 0;
				TIM1->CCR2 = 0;
				TIM1->ARR = 625-1;
			}
			case STEPJ2:
			{
				ang_vel[1] = 0;
				TIM2->CCR3 = 0;
				TIM2->ARR = 625-1;
			}
			case STEPJ3:
			{
				ang_vel[2] = 0;
				TIM3->CCR1 = 0;
				TIM3->ARR = 625-1;
			}
			case STEPJ4:
			{
				ang_vel[3] = 0;
				TIM4->CCR3 = 0;
				TIM4->ARR = 625-1;
			}
			case STEPGripper:
			{
				ang_vel[4] = 0;
				TIM15->CCR2 = 0;
				TIM15->ARR = 625-1;
			}
			default:
			{

			}
		}
}
//uint16_t SPI_Encoder()
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  uint8_t encoder_address[5] = {0xA4, 0xB4, 0xC4, 0xD4, 0x54};
//  uint16_t abs_position = 0;

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  PIDInit(&PID_Joint1, 1, 1, 1, -7, 7, 0);
  PIDInit(&PID_Joint2, 1, 1, 1, -7, 7, 0);
  PIDInit(&PID_Joint3, 1, 1, 1, -7, 7, 0);
  PIDInit(&PID_Joint4, 1, 1, 1, -7, 7, 0);

  TIM1->CCR2 = 0;
  TIM2->CCR3 = 0;
  TIM3->CCR1 = 0;
  TIM4->CCR3 = 0;
  TIM15->CCR2 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
  HAL_UART_Receive_IT(&huart3, UART3_RXBUFFER, 4);

  //create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;
//    uint8_t _addr[1] = {0x54};
//    uint16_t _temp = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	attempts = 0;

          //this function gets the encoder position and returns it as a uint16_t
          //send the function either res12 or res14 for your encoders resolution
          encoderPosition = AMT21_getPositionModbusRTU((uint8_t)0x54, RES14);

          //if the position returned was 0xFFFF we know that there was an error calculating the checksum
          //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
          while (encoderPosition == 0xFFFF && ++attempts < 3)
          {
              encoderPosition = AMT21_getPositionModbusRTU((uint8_t)0x54, RES14); //try again
          }

          if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
          {
            printf("Encoder 0 error. Attempts: %d\n", attempts); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
          }
          else //position was good, print to serial stream
          {

              printf("Encoder 0: %d\n", encoderPosition); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
          }
          HAL_Delay(500);
//	  if(State_Checksum_Error)
//	  {
//		  State_Checksum_Error = 0;
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_CheckSumError_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//	  }
//	  if(State_Input_Joint_State)
//	  {
//		  State_Input_Joint_State = 0;
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//	  }
//	  if(State_Print_4_Joint_State)
//	  {
//		  State_Print_4_Joint_State = 0;
//		  printf("q1 = %.3f, q2 = %.3f, q3 = %.3f, q4 = %.3f\n", q[0],q[1],q[2],q[3]);
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//	  }
//	  if(State_Activate_Gripper)
//	  {
//		  State_Activate_Gripper = 0;
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//	  }
//	  if(State_Deactivate_Gripper)
//	  {
//		  State_Deactivate_Gripper = 0;
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//	  }
//	  if(State_Set_Home)
//	  {
//		  State_Set_Home = 0;
//		  for(register int i = 0; i < 4; i++){
//		      q[i] = 0;
//		    }
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//	  }
//	  if(State_PID_Control_Timer)
//	  {
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//		  State_PID_Control_Timer = 0;
//	  }
//	  if(State_Casade_Control_Timer)
//	  {
//		  UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ProcessIsCompleted_Address;
//		  HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
//		  State_Casade_Control_Timer = 0;
//	  }
//      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//
//	StepStop(3);
//	HAL_Delay(5000);
//
//      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//
//	StepDriveRad(3, 3.00);
//	HAL_Delay(400);
//	      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//	      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//	      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//	StepStop(3);
//	HAL_Delay(5000);
//	      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//	      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//	      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
//	StepDriveRad(3, -3.00);
//	HAL_Delay(400);
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
//  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 1);
//    HAL_NVIC_EnableIRQ(TIM5_IRQn);
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
//  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 1);
//    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART4_DE_GPIO_Port, UART4_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_Button_Pin */
  GPIO_InitStruct.Pin = Blue_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin DIR_2_Pin LD3_Pin DIR_3_Pin
                           DIR_4_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|DIR_2_Pin|LD3_Pin|DIR_3_Pin
                          |DIR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LM2_Pin LM1_Pin */
  GPIO_InitStruct.Pin = LM2_Pin|LM1_Pin;
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

  /*Configure GPIO pin : UART4_DE_Pin */
  GPIO_InitStruct.Pin = UART4_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART4_DE_GPIO_Port, &GPIO_InitStruct);

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
					State_Input_Joint_State = 1;
					break;
				}
				case 7:		// q2 Mode
				{
					State_Input_Joint_State = 1;
					break;
				}
				case 8:		// q3 Mode
				{
					State_Input_Joint_State = 1;
					break;
				}
				case 9:		// q4 Mode
				{
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LM1_Pin || GPIO_Pin == LM2_Pin || GPIO_Pin == LM3_Pin || GPIO_Pin == LM4_Pin || GPIO_Pin == LM5_Pin)
	{
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

		  HAL_TIM_Base_Stop_IT(&htim5);
		  HAL_TIM_Base_Stop_IT(&htim12);

		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
		  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
		  Error_Handler();

	}
	if(GPIO_Pin == Blue_Button_Pin)
	  {
		UART3_TXBUFFER_ACK[0] = (uint8_t)ACK_ReceivedData_Address;
		HAL_UART_Transmit(&huart3, (uint8_t *)UART3_TXBUFFER_ACK, 1, 100);
		//		T += 3;
//		HAL_TIM_Base_Start_IT(&htim12);
	  }
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
	 * How to Start/Stop Timer Interrupt
	 * Use function
	 * HAL_TIM_Base_Start_IT(&htim5);
	 * HAL_TIM_Base_Stop_IT(&htim5);
	 *
	 */
  /* Timer5 Interrupt for PID Position Control.*/
  if (htim->Instance == TIM5)
  {
	    // PID Control Loop
	    StepDriveRad(1, PIDCalculate(&PID_Joint1, q[0], q_s[0]));
	    q_s[0] = q_s[0] + theta_q[0];
	    StepDriveRad(2, PIDCalculate(&PID_Joint2, q[1], q_s[1]));
	    q_s[1] = q_s[1] + theta_q[1];
	    StepDriveRad(3, PIDCalculate(&PID_Joint3, q[2], q_s[2]));
	    q_s[2] = q_s[2] + theta_q[2];
	    StepDriveRad(4, PIDCalculate(&PID_Joint4, q[3], q_s[3]));
	    q_s[3] = q_s[3] + theta_q[3];
  }
  /* Timer12 Interrupt for Trajectory Generation.*/
  if (htim->Instance == TIM12)
  {
	    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	    // Cubic Trajectory
	    float t_2 = t*t;
	    float t_3 = t_2 * t;
	    float Goal_position_x = C0x + (C2x*t_2) + (C3x*t_3);
	    float Goal_position_y = C0y + (C2y*t_2) + (C3y*t_3);
//	    float Goal_velocity_x = (2*C2x*t) + (3 * C3x*t_2);
//	    float Goal_velocity_y = (2*C2y*t) + (3 * C3y*t_2);

	    // Circle Trajectory


	    // Inverse Pose Kinematics
	    IPK_findChessBot(Goal_position_x, Goal_position_y, 0, 0);

	    // Inverse Velocity Kinematics
	    IVK();

	    // Inverse Acceleration Kinematics
	    IAK();

	    // Casade Control Loop
	    StepDriveRad(1, PIDCalculate(&PID_Joint1, q[0], q_s[0]));
	    q_s[0] = q_s[0] + theta_q[0];
	    StepDriveRad(2, PIDCalculate(&PID_Joint2, q[1], q_s[1]));
	    q_s[1] = q_s[1] + theta_q[1];
	    StepDriveRad(3, PIDCalculate(&PID_Joint3, q[2], q_s[2]));
	    q_s[2] = q_s[2] + theta_q[2];
	    StepDriveRad(4, PIDCalculate(&PID_Joint4, q[3], q_s[3]));
	    q_s[3] = q_s[3] + theta_q[3];

	    // Sample time 0.0005 seconds
	    if (t < T)
	    {
	        t = t + sample_time;
	    }
	    else
	    {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	    	t = T;
	    	C0x = Goal_position_x;
	    	C0y = Goal_position_y;
	    	HAL_TIM_Base_Stop_IT(&htim12);
	    	State_Casade_Control_Timer = 1;
	        // Stop Control Loop
	    }
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
	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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
