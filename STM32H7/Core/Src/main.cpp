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
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
//#if __cplusplus
//using namespace std;
//#include <iostream>
//#endif
#include "AMT21.h"
#include "Stepper.h"
#include "ServoMotor.h"
#include "RobotJoint.h"
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
uint32_t TIM_MS = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if defined(__GNUC__)
int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
#elif defined (__ICCARM__)
#include "LowLevelIOInterface.h"
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  HAL_UART_Transmit(&huart3, (uint8_t *) buffer, size, HAL_MAX_DELAY);
  return size;
}
#elif defined (__CC_ARM)
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __cplusplus

AMT21 encoderJ1(&huart4, 0xD4);
//AMT21 encoderJ2(&huart4, 0xB4);
AMT21 encoderJ3(&huart4, 0xC4);
AMT21 encoderJ4(&huart4, 0xA4);

Stepper stepperJ1(&htim3, TIM_CHANNEL_1, DIR_3_GPIO_Port, DIR_3_Pin);
Stepper stepperJ2(&htim1, TIM_CHANNEL_2, DIR_1_GPIO_Port, DIR_1_Pin);
Stepper stepperJ3(&htim15, TIM_CHANNEL_2, DIR_5_GPIO_Port, DIR_5_Pin);
Stepper stepperJ4(&htim2, TIM_CHANNEL_3, DIR_2_GPIO_Port, DIR_2_Pin);
RobotJoint fcb_joint1;
RobotJoint fcb_joint2;
RobotJoint fcb_joint3;
RobotJoint fcb_joint4;


ServoMotor gripper(&htim4, TIM_CHANNEL_3);
HAL_StatusTypeDef HALENCJ1OK, HALENCJ2OK, HALENCJ3OK, HALENCJ4OK;
#endif

volatile int8_t dq1 = 0, dq2 = 0, dq3 = 0, dq4 = 0;
volatile int8_t dx = 0, dy = 0, dz = 0, dyaw = 0;
volatile int16_t to_X_pose = 0, to_Y_pose = 0, to_Z_pose = 0, to_Yaw_pose = 0;
volatile float px,py,pz,pyaw;
volatile uint16_t CRCValue = 0;
volatile uint16_t ExpectedCRCValue = 0;
volatile int gripperstate;
#define Rx_BUFFER_SIZE   20
uint8_t Old_Rx_Buffer[Rx_BUFFER_SIZE] = { 0 };
uint8_t New_Rx_Buffer[Rx_BUFFER_SIZE] = { 0 };
volatile uint16_t cmdDataSize = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart3) {
		memcpy(Old_Rx_Buffer, &New_Rx_Buffer, Rx_BUFFER_SIZE);	// Keep buffer.
		memset(New_Rx_Buffer, 0, Rx_BUFFER_SIZE);	// Clear received data.
		if (Size - 2 > 0 && Size <= Rx_BUFFER_SIZE) {// Check if there's some data.
			cmdDataSize = Size - 2;	// Calculate data length.
			CRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t*) Old_Rx_Buffer,
					cmdDataSize); // Calculate data only by STM32 Hardware CRC.
			ExpectedCRCValue = Old_Rx_Buffer[cmdDataSize] << 8
					| Old_Rx_Buffer[cmdDataSize + 1]; // Read Expected CRC from Protocol.
			if (CRCValue == ExpectedCRCValue) { // Check if CRC value is equal to Expected CRC value.
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
				if (Old_Rx_Buffer[0] == 0x41 && cmdDataSize == 3) {	// Joint Jog q1
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dq1 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x42 && cmdDataSize == 3) {// Joint Jog q2
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dq2 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x43 && cmdDataSize == 3) {// Joint Jog q3
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dq3 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x44 && cmdDataSize == 3) {// Joint Jog q4
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dq4 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x51 && cmdDataSize == 3) {// Linear Jog X
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dx = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x52 && cmdDataSize == 3) {// Linear Jog Y
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dy = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x53 && cmdDataSize == 3) {// Linear Jog Z
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dz = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x54 && cmdDataSize == 3) {// Linear Jog Yaw
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dyaw = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
				} else if (Old_Rx_Buffer[0] == 0x61 && cmdDataSize == 5) { // Joint Jog 4q
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dq1 = Old_Rx_Buffer[1];
					dq2 = Old_Rx_Buffer[2];
					dq3 = Old_Rx_Buffer[3];
					dq4 = Old_Rx_Buffer[4];
				} else if (Old_Rx_Buffer[0] == 0x71 && cmdDataSize == 5) { // Linear Jog X,Y,Z,Yaw
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dx = Old_Rx_Buffer[1];
					dy = Old_Rx_Buffer[2];
					dz = Old_Rx_Buffer[3];
					dyaw = Old_Rx_Buffer[4];
				} else if (Old_Rx_Buffer[0] == 0x81 && cmdDataSize == 2) {// Servo
					gripperstate = Old_Rx_Buffer[1];
					if (gripperstate == 0) {
						gripper.GripperOpen();
					} else {
						gripper.GripperClose();
					}

				} else if (Old_Rx_Buffer[0] == 0x91 && cmdDataSize == 8) {
					px = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2] ;
					py = (Old_Rx_Buffer[3] << 8) | Old_Rx_Buffer[4] ;
					pz = (Old_Rx_Buffer[5] << 8) | Old_Rx_Buffer[6] ;
					pyaw = Old_Rx_Buffer[7];
				}
			}
		} else {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit_DMA(&huart3, (uint8_t*) "CRC16 error\n", 12);
		}
	} else {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*) "Protocol match error\n", 21);
	}
	/* start the DMA again */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*) New_Rx_Buffer,
			Rx_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}



uint16_t CRC16(uint8_t *buf, int len);
#ifdef __cplusplus
volatile float error_q1,error_q3;
volatile int State_Sethome = 1;
volatile int16_t posJ1, posJ3;
volatile int32_t posJ2;
volatile int32_t setpointJ1, setpointJ3;
volatile int direction_traj = 0;
volatile float errorJ1, errorJ3;
volatile float uJ1, uJ3;
volatile float chess_board_ang = 0.0;
volatile float debug_pos_x, debug_pos_y;
volatile u_int32_t county;
//volatile float C1,S1,C3,S3,q1,q3;
//static float L1 = 0.01325; // 0.053
//static float L2 = 0.370; // 0.36625
//static float L3 = 0.315;
//static float L12 = 0.38325;
//static float H1 = 0.125;
//static float H3 = 0.065;
//static float H4 = 0.190;

volatile float t = 0.0;
volatile const float Time = 3;

volatile const float C0_q1 = 0.6;
volatile const float C2_q1 = (3.0 * C0_q1) / (Time * Time);
volatile const float C3_q1 = (2.0 * C0_q1) / (Time * Time * Time);

volatile const float C0_q3 = 0.3;
volatile const float C2_q3 = (3.0 * C0_q3) / (Time * Time);
volatile const float C3_q3 = (2.0 * C0_q3) / (Time * Time * Time);

volatile float bug1, bug2, bug3, bug4, bug5;

volatile const float sample_time_100 = 0.01;
volatile const float sample_time_200 = 0.005;
volatile const float sample_time_500 = 0.002;
volatile const float sample_time_1000 = 0.001;
volatile const float sample_time_2000 = 0.0005;

const float RPM = 2.0 ;
volatile const float Time_circle = 60.0 / RPM ;
volatile const float chessboard_angular_velocity = RPM * 0.10472; // rpm to rad/s

volatile bool direction = true;
volatile float Goal_velocity_q1, Goal_velocity_q3;

const float pi = 3.14159265;

volatile float w_q1;
volatile float w_q2;
volatile float w_q3;
volatile float w_q4;
volatile float u_q1 = 0.0;
volatile float u_q2 = 0.0;
volatile float u_q3 = 0.0;
volatile float u_q4 = 0.0;

volatile float debug_j1_x11;
volatile float debug_j3_x11;
volatile float debug_j1_x21;
volatile float debug_j3_x21;
volatile float debug_wq1;
volatile float debug_wq3;



struct joint_state {
float q1, q2, q3, q4;
};
typedef struct joint_state joint_config;

struct robot_joint {
volatile int16_t Encoder;
volatile float Goal_Position, Goal_Velocity;
volatile float Kalman_Position, Kalman_Velocity, Kalman_Position_New, Kalman_Velocity_New;
volatile float Kp_p, Ki_p, Kd_p, Kp_v, Ki_v, Kd_v;
volatile float Error_p, Old_Error_p, Sum_Error_p, Error_v, Old_Error_v, Sum_Error_v;
volatile float Old_p, Old_v;
volatile float Output_Stepper_Frequency, Output_Joint_W;
volatile float X11 = 0;
volatile float X21 = 0;
volatile float p11 = 1;
volatile float p12 = 0;
volatile float p21 = 1;
volatile float p22 = 0;
volatile float kalman_pos = 0;
volatile float kalman_velo = 0;
volatile float Q = 0.095;
volatile float R = 0.00006;
volatile float C0, C1, C2, C3, C4, C5, T;

};

int num = 30;
float box_q1[30];
float box_q2[30];
float box_q3[30];
float box_q4[30];
float idx, idy;
typedef struct robot_joint fcb_joint;



#endif









void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// tim5 100 Hz
// tim7 1000 Hz
// tim12 2000 Hz
// tim6 200 Hz
// tim14 500Hz

if (htim == &htim6) {	//

//	encoderJ1.AMT21_Read();
//	HALENCJ1OK = encoderJ1.AMT21_Check_Value();
//	if (HALENCJ1OK == HAL_OK) {
//		fcb_joint1.Encoder = encoderJ1.getAngPos180() ;
//	}
//
//	encoderJ3.AMT21_Read();
//	HALENCJ3OK = encoderJ3.AMT21_Check_Value();
//	if (HALENCJ3OK == HAL_OK) {
//		fcb_joint3.Encoder = encoderJ3.getAngPos180() ;
//	}
//
//	float sethome_q1 = -0.4137;
//	float sethome_q2 = 0.1;
//	float sethome_q3 =   0.9638;
//	float sethome_yaw = -0.5501;
//
//	error_q1 = sethome_q1*2609.0 - fcb_joint1.Encoder;
//	error_q3 = sethome_q3*2609.0 - fcb_joint3.Encoder;
//
//	stepperJ1.StepperSetFrequency(error_q1*-2.0);
//	stepperJ3.StepperSetFrequency(error_q3/3);
//	if (abs(error_q3) < 20.0)
//	{
//		stepperJ3.StepperSetFrequency(0);
//	}
//	if ( abs(error_q1) < 15.0 )
//	{
//		State_Sethome = 0;
//		stepperJ1.StepperSetFrequency(0);
//		stepperJ3.StepperSetFrequency(0);
//		HAL_TIM_Base_Stop_IT(&htim6);
//		HAL_TIM_Base_Start_IT(&htim7);
//	}

}

if (htim == &htim14) {	//

}

if (htim == &htim12) {	//

}

if (htim == &htim5) {	//

//		encoderJ1.AMT21_Read();
//		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
//		if (HALENCJ1OK == HAL_OK) {
//			fcb_joint1.Encoder = encoderJ1.getAngPos180() ;
//		}

//		encoderJ2.AMT21_Read();
//		HALENCJ2OK = encoderJ2.AMT21_Check_Value();
//		if(HALENCJ2OK == HAL_OK){
//			encoderJ2.unwarp();
//			posJ2 = encoderJ2.getUnwarpValue();
//		}

//		encoderJ3.AMT21_Read();
//		HALENCJ3OK = encoderJ3.AMT21_Check_Value();
//		if (HALENCJ3OK == HAL_OK) {
//			fcb_joint3.Encoder = encoderJ3.getAngPos180() ;
//		}
//
//		encoderJ4.AMT21_Read();
//		HALENCJ4OK = encoderJ4.AMT21_Check_Value();
//		if (HALENCJ4OK == HAL_OK) {
//			fcb_joint4.Encoder = encoderJ4.getAngPos180() ;
//		}


//	int i;
//	for (i = 1; i < num; i++) {
//		box_q1[i - 1] = box_q1[i];
//		box_q2[i - 1] = box_q2[i];
//		box_q3[i - 1] = box_q3[i];
//		box_q4[i - 1] = box_q4[i];
//	}
//	box_q1[num - 1] = dq1 / 10.0;
//	box_q2[num - 1] = dq2 / 10.0;
//	box_q3[num - 1] = dq3 / 10.0;
//	box_q4[num - 1] = dq4 / 10.0;
//
//	u_q1 = 0.0;
//	u_q2 = 0.0;
//	u_q3 = 0.0;
//	u_q4 = 0.0;
//
//	for (i = 0; i < num; i++) {
//		u_q1 += box_q1[i];
//		u_q2 += box_q2[i];
//		u_q3 += box_q3[i];
//		u_q4 += box_q4[i];
//	}
//
//	stepperJ1.StepperSetFrequency(u_q1 * 3);
//	stepperJ2.StepperSetFrequency(u_q2 * 1.5);
//	stepperJ3.StepperSetFrequency(u_q3 * 2);
//	stepperJ4.StepperSetFrequency(u_q4 * 2);

//		stepperJ1.StepperSetFrequency(dq1);
//		stepperJ2.StepperSetFrequency(dq2);
//		stepperJ3.StepperSetFrequency(dq3);
//		stepperJ4.StepperSetFrequency(dq4);

//		Update_ivk(fcb_joint1.Encoder / 2609.0 , 0.0,fcb_joint3.Encoder / 2609.0, 0.0, dx/1000.0, dy/1000.0, dz/1000.0, 0.0);

		int i;
				for (i = 1 ; i<num ; i++)
				{
					box_q1[i-1] = box_q1[i];
					box_q2[i-1] = box_q2[i];
					box_q3[i-1] = box_q3[i];
					box_q4[i-1] = box_q4[i];
				}
				 box_q1[num-1] = w_q1;
				 box_q2[num-1] = dz;
				 box_q3[num-1] = w_q3;
				 box_q4[num-1] = w_q4;

				u_q1 = 0.0;
				u_q2 = 0.0;
				u_q3 = 0.0;
				u_q4 = 0.0;

				for(i = 0; i < num; i++)
				{
					u_q1 += box_q1[i];
					u_q2 += box_q2[i];
					u_q3 += box_q3[i];
					u_q4 += box_q4[i];
				}

				stepperJ1.StepperOpenLoopSpeed(u_q1/num*-1.0);
				stepperJ2.StepperSetFrequency(u_q2/3.0);
				stepperJ3.StepperOpenLoopSpeed(u_q3/num*1.0);
				stepperJ4.StepperOpenLoopSpeed(u_q4/num*-1.0);



}


	if (htim == &htim7) {

				encoderJ1.AMT21_Read();
				HALENCJ1OK = encoderJ1.AMT21_Check_Value();
				if (HALENCJ1OK == HAL_OK) {
					fcb_joint1.Encoder = encoderJ1.getAngPos180() / 2.609 ;
				}

//				encoderJ2.AMT21_Read();
//				HALENCJ2OK = encoderJ2.AMT21_Check_Value();
//				if(HALENCJ2OK == HAL_OK){
//					encoderJ2.unwarp();
//					fcb_joint2.Encoder = encoderJ2.getUnwarpValue() / 2.609 ;
//				}

				encoderJ3.AMT21_Read();
				HALENCJ3OK = encoderJ3.AMT21_Check_Value();
				if (HALENCJ3OK == HAL_OK) {
					fcb_joint3.Encoder = encoderJ3.getAngPos180() / 2.609 ;
				}

				encoderJ4.AMT21_Read();
				HALENCJ4OK = encoderJ4.AMT21_Check_Value();
				if (HALENCJ4OK == HAL_OK) {
					fcb_joint4.Encoder = encoderJ4.getAngPos180() / 2.609 ;
				}

				float t2 = t * t;
				float t3 = t * t * t;
				float t4 = t * t * t * t;
				float t5 = t * t * t * t * t;



		fcb_joint1.Goal_Velocity = fcb_joint1.C1 + (2.0*fcb_joint1.C2*t) + (3.0*fcb_joint1.C3*t2) + (4.0*fcb_joint1.C4*t3) + (5.0*fcb_joint1.C5*t4);
		fcb_joint1.Goal_Position = (fcb_joint1.C0 + (fcb_joint1.C1*t) + (fcb_joint1.C2*t2) + (fcb_joint1.C3*t3) + (fcb_joint1.C4*t4) + (fcb_joint1.C5*t5));

		fcb_joint3.Goal_Velocity = fcb_joint3.C1 + (2.0*fcb_joint3.C2*t) + (3.0*fcb_joint3.C3*t2) + (4.0*fcb_joint3.C4*t3) + (5.0*fcb_joint3.C5*t4);
		fcb_joint3.Goal_Position = (fcb_joint3.C0 + (fcb_joint3.C1*t) + (fcb_joint3.C2*t2) + (fcb_joint3.C3*t3) + (fcb_joint3.C4*t4) + (fcb_joint3.C5*t5));

//		a = ((2.0*fcb_joint1.C2) + (6.0*fcb_joint1.C3*t) + (12.0*fcb_joint1.C4*t2) + (20.0*fcb_joint1.C5*t3));

		// Fuck you
		//		chess_board_ang = chessboard_angular_velocity * t;
		//
		//		joint_config findchessbot_joint_state;
		//		debug_pos_x = (0.177 * cos(chess_board_ang)) + 0.4424 ;
		//		debug_pos_y = (0.177 * sin(chess_board_ang)) + 0.0106;
		//
		//		idx = -0.177 * sin(chess_board_ang)	* chessboard_angular_velocity;
		//		idy =  0.177 * cos(chess_board_ang)	* chessboard_angular_velocity ;
		//
		//		findchessbot_joint_state = find_IK(debug_pos_x, debug_pos_y, 0, 0);
		//		Update_ivk(fcb_joint1.Encoder / 2609.0 , 0, fcb_joint3.Encoder / 2609.0, 0, idx, idy, 0.0, 0.0);
		// Fuck you


		//		fcb_joint1.Error_p = fcb_joint1.Goal_Position - fcb_joint1.Encoder;
		//		fcb_joint3.Error_p = fcb_joint3.Goal_Position - fcb_joint3.Encoder;
		//		fcb_joint1.Sum_Error_p += fcb_joint1.Error_p;
		//		fcb_joint3.Sum_Error_p += fcb_joint3.Error_p;
		//
		fcb_joint1.KalmanFillter(fcb_joint1.Encoder);
		fcb_joint1.kalman_pos = fcb_joint1.X11;
		fcb_joint1.kalman_velo = fcb_joint1.X21;

		fcb_joint3.KalmanFillter(fcb_joint3.Encoder);
		fcb_joint3.kalman_pos = fcb_joint3.X11;
		fcb_joint3.kalman_velo = fcb_joint3.X21;

		fcb_joint1.Kp_p = 1.0;
		fcb_joint1.Ki_p = 0.0;
		fcb_joint1.Kd_p = 0.0;
		fcb_joint3.Kp_p = 1.0;
		fcb_joint3.Ki_p = 0.0;
		fcb_joint3.Kd_p = 0.0;

		fcb_joint1.Kp_v = 1.0;
		fcb_joint1.Ki_v = 0.0;
		fcb_joint1.Kd_v = 0.0;
		fcb_joint3.Kp_v = 1.0;
		fcb_joint3.Ki_v = 0.0;
		fcb_joint3.Kd_v = 0.0;

		fcb_joint1.Error_p = fcb_joint1.Goal_Position - fcb_joint1.Encoder;
		fcb_joint3.Error_p = fcb_joint3.Goal_Position - fcb_joint3.Encoder;
		fcb_joint1.Error_v = fcb_joint1.Goal_Velocity - fcb_joint1.kalman_velo;
		fcb_joint3.Error_v = fcb_joint3.Goal_Velocity - fcb_joint3.kalman_velo;

		fcb_joint1.Sum_Error_p += fcb_joint1.Error_p;
		fcb_joint3.Sum_Error_p += fcb_joint3.Error_p;
		fcb_joint1.Sum_Error_v += fcb_joint1.Error_v;
		fcb_joint3.Sum_Error_v += fcb_joint3.Error_v;

		fcb_joint1.Output_Joint_W = fcb_joint1.Goal_Velocity+
									(fcb_joint1.Kp_v * fcb_joint1.Error_v) +
									(fcb_joint1.Ki_v * fcb_joint1.Sum_Error_v) +
									(fcb_joint1.Kd_v * (fcb_joint1.Error_v - fcb_joint1.Old_v)) ;
		fcb_joint3.Output_Joint_W = fcb_joint3.Goal_Velocity +
									(fcb_joint3.Kp_v * fcb_joint3.Error_v) +
									(fcb_joint3.Ki_v * fcb_joint3.Sum_Error_v) +
									(fcb_joint3.Kd_v * (fcb_joint3.Error_v - fcb_joint3.Old_v)) ;

		stepperJ1.StepperOpenLoopSpeed(fcb_joint1.Output_Joint_W/1000.0);
		stepperJ3.StepperOpenLoopSpeed(fcb_joint3.Output_Joint_W/1000.0);

		fcb_joint1.Old_Error_p = fcb_joint1.Error_p;
		fcb_joint3.Old_Error_p = fcb_joint3.Error_p;
		fcb_joint1.Old_Error_v = fcb_joint1.Error_v;
		fcb_joint3.Old_Error_v = fcb_joint3.Error_v;

		fcb_joint1.Old_p = fcb_joint1.Encoder;
		fcb_joint3.Old_p = fcb_joint3.Encoder;
		fcb_joint1.Old_v = fcb_joint1.kalman_velo;
		fcb_joint3.Old_v = fcb_joint1.kalman_velo;

		t = t + (sample_time_1000);


		if (t >= fcb_joint1.T)
		{
			t = 0.0;
			direction_traj ^= 1;

			fcb_joint1.Sum_Error_p = 0;
			fcb_joint3.Sum_Error_p = 0;
			fcb_joint1.Sum_Error_v = 0;
			fcb_joint3.Sum_Error_v = 0;
			fcb_joint1.Old_Error_p = 0;
			fcb_joint3.Old_Error_p = 0;
			fcb_joint1.Old_Error_v = 0;
			fcb_joint3.Old_Error_v = 0;
			if (direction_traj == 0)
			{
				fcb_joint1.UpdateQuinticCoff(10.0, (fcb_joint1.Encoder), (fcb_joint1.Encoder) - 1570, 0.0, 0.0, 0.0, 0.0);
				fcb_joint3.UpdateQuinticCoff(10.0, (fcb_joint3.Encoder), (fcb_joint3.Encoder) - 1570, 0.0, 0.0, 0.0, 0.0);
			}
			else
			{
				fcb_joint1.UpdateQuinticCoff(10.0, (fcb_joint1.Encoder), (fcb_joint1.Encoder) + 1570, 0.0, 0.0, 0.0, 0.0);
				fcb_joint3.UpdateQuinticCoff(10.0, (fcb_joint3.Encoder), (fcb_joint3.Encoder) + 1570, 0.0, 0.0, 0.0, 0.0);
			}

		}

		//		if (t >= Time_circle)
		//				{
		//					t = 0.0;
		//					chess_board_ang = 0.0;
		//					direction_traj ^= 1;
		//				}

	}
}

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
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_UART4_Init();
	MX_TIM4_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_TIM15_Init();
	MX_CRC_Init();
	MX_UART7_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM12_Init();
	MX_TIM13_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	#ifdef __cplusplus
	stepperJ1.StepperSetFrequency(0.0f);
	stepperJ1.StepperSetMicrostep(8);
	stepperJ1.StepperSetRatio(42);
	stepperJ1.StepperEnable();

	stepperJ2.StepperSetFrequency(0.0f);
	stepperJ2.StepperSetMicrostep(8);
	stepperJ2.StepperSetRatio(3);
	stepperJ2.StepperEnable();

	stepperJ3.StepperSetFrequency(0.0f);
	stepperJ3.StepperSetMicrostep(8);
	stepperJ3.StepperSetRatio(9);
	stepperJ3.StepperEnable();

	stepperJ4.StepperSetFrequency(0.0f);
	stepperJ4.StepperSetMicrostep(8);
	stepperJ4.StepperSetRatio(3);
	stepperJ4.StepperEnable();

	//	stepperJ4.StepperSetMicrostep(1);
	//	stepperJ4.StepperSetRatio(1);

	gripper.setDegreeGripperClose(65);
	gripper.setDegreeGripperOpen(0);
	gripper.ServoEnable();
	fcb_joint3.Q = 0.12;
	fcb_joint3.R = 0.0001;
	HAL_Delay(3000);
	#endif

//		HAL_TIM_Base_Start_IT(&htim5); // Jog 		100 Hz
//		HAL_TIM_Base_Start_IT(&htim6); // Set home 	200 Hz
//		HAL_TIM_Base_Start_IT(&htim7); // Control 	1000 Hz
//		HAL_TIM_Base_Start_IT(&htim12); // 			2000 Hz
	//	HAL_TIM_Base_Start_IT(&htim14); // 			500Hz

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*) New_Rx_Buffer, Rx_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	//	stepperJ1.StepperOpenLoopSpeed(1.00f);


//	fcb_joint1.UpdateQuinticCoff(5.0, fcb_joint1.Encoder, fcb_joint1.Encoder + 785, 0.0, 0.0, 0.0, 0.0);
//	fcb_joint3.UpdateQuinticCoff(5.0, fcb_joint3.Encoder, fcb_joint3.Encoder + 785, 0.0, 0.0, 0.0, 0.0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		encoderJ1.AMT21_Read();
		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
		if (HALENCJ1OK == HAL_OK) {
			fcb_joint1.Encoder = encoderJ1.getAngPos180() / 2.609 ;
		}
		stepperJ1.StepperSetFrequency(20.0f); // + กลับด้าน +
//		stepperJ2.StepperSetFrequency(300.0f); // + กลับด้านจากแกน แต่ดีแล้ว +
//		stepperJ3.StepperSetFrequency(300.0f); // + ถูกด้าน +
//		stepperJ4.StepperSetFrequency(300.0f); // + กลับด้าน +

		}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

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
while (1) {
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

