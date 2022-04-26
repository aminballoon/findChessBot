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
#include "AS5047UABI.h"
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
AMT21 encoderJ2(&huart4, 0xB4);
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
RobotJoint fcb_Y;
RobotJoint fcb_X;

ServoMotor gripper(&htim4, TIM_CHANNEL_3);
HAL_StatusTypeDef HALENCJ1OK, HALENCJ2OK, HALENCJ3OK, HALENCJ4OK;

AS5047UABI chessABIEncoder(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2);
volatile float angle_chess, angle_chess_deg;

#endif

volatile float Setpoint_J2_Up = -200.0;
volatile float Setpoint_J2_Down = -9500.0;
volatile bool State_FIN = false;
volatile float Max_Time = 0;
volatile int8_t dq1 = 0, dq2 = 0, dq3 = 0, dq4 = 0;
volatile int8_t dx = 0, dy = 0, dz = 0, dyaw = 0;
volatile int16_t to_X_pose = 0, to_Y_pose = 0, to_Z_pose = 0, to_Yaw_pose = 0;
volatile float px, py, pz, pyaw;
volatile uint16_t CRCValue = 0;
volatile uint16_t ExpectedCRCValue = 0;
volatile int gripperstate, trajstate;
#define Rx_BUFFER_SIZE   20
uint8_t Old_Rx_Buffer[Rx_BUFFER_SIZE] = { 0 };
uint8_t New_Rx_Buffer[Rx_BUFFER_SIZE] = { 0 };
volatile uint16_t cmdDataSize = 0;

volatile bool Limit_sw_Z_Bot = false;
volatile bool Limit_sw_Z_Top = false;
volatile bool Limit_sw_Gripper = false;
volatile bool Limit_sw_Emergancy = false;
volatile bool joint13_on = false;

uint16_t CRC16(uint8_t *buf, int len);
#ifdef __cplusplus
// ############ Queue ############
#define LIMIT 20
volatile int queue[LIMIT];
volatile int Gripper_State[LIMIT];
volatile int first = -1;
volatile int bot = -1;
volatile int test_value, test_value_r, test_value_theta;
// ###############################

//

volatile float theta[] = { 2356, 2191, 1976, 1713, 1429, 1166, 951, 785, 2521,
		2356, 2111, 1768, 1373, 1030, 785, 620, 2737, 2601, 2356, 1893, 1249,
		785, 540, 405, 3000, 2944, 2820, 2356, 785, 322, 197, 142, 3283, 3339,
		3463, 3927, 5498, 5961, 6086, 6141, 3546, 3682, 3927, 4391, 5034, 5498,
		5743, 5878, 3762, 3927, 4172, 4515, 4910, 5253, 5498, 5663, 3927, 4092,
		4307, 4570, 4854, 5117, 5333, 5498 };

volatile float radias[] = { 247, 215, 190, 177, 177, 190, 215, 247, 215, 177,
		146, 127, 127, 146, 177, 215, 190, 146, 106, 79, 79, 106, 146, 190, 177,
		127, 79, 35, 35, 79, 127, 177, 177, 127, 79, 35, 35, 79, 127, 177, 190,
		146, 106, 79, 79, 106, 146, 190, 215, 177, 146, 127, 127, 146, 177, 215,
		247, 215, 190, 177, 177, 190, 215, 247 };

//
volatile char control_state = 0;
volatile int direction_traj = 0;
volatile float Setpoint_J1 = 0;
volatile float Setpoint_J3 = 0;

volatile int Balloon = 0;
volatile float t = 0;

volatile const float sample_time_100 = 0.01;
volatile const float sample_time_200 = 0.005;
volatile const float sample_time_500 = 0.002;
volatile const float sample_time_1000 = 0.0001;
volatile const float sample_time_2000 = 0.0005;

const float RPM = 2.0;
volatile const float Time_circle = 60.0 / RPM;
volatile const float chessboard_angular_velocity = RPM * 0.10472; // rpm to rad/s

const float pi = 3.14159265;

volatile float w_q1;
volatile float w_q2;
volatile float w_q3;
volatile float w_q4;
volatile float u_q1 = 0.0;
volatile float u_q2 = 0.0;
volatile float u_q3 = 0.0;
volatile float u_q4 = 0.0;

volatile float Robot_X, Robot_Y, Robot_Z, Robot_Yaw;
volatile float Planning_q1, Planning_q2, Planning_q3, Planning_q4;

bool Insert_queue(int value, int gripper_value) {
	if (bot == LIMIT - 1) {
		return false;
	} else {
		if (first == -1) {
			first = 0;
		}
		bot++;
		queue[bot] = value;
		Gripper_State[bot] = gripper_value;
		return true;
	}
}

int Call_queue() {
	int output;
	if (first == -1 || first > bot) {
		printf("Don't have stack in queue \n");
		for (int num = 0; num <= bot + 1; num++) {
			queue[num] = 0;
		}
		first = -1;
		bot = -1;
		printf("Clear queue \n");
		output = 255;
	} else {
		output = queue[first];
		first++;
	}
	return output;

}

void fcb_FK(float J_q1, float J_q2, float J_q3, float J_q4) {

	const float C1 = cos(J_q1 / 1000.0);
	const float S1 = sin(J_q1 / 1000.0);
	const float C13 = cos((J_q1 + J_q3) / 1000.0);
	const float S13 = sin((J_q1 + J_q3) / 1000.0);

	const float L1 = 0.01325; // 0.053
	const float L2 = 0.370; // 0.36625
	const float L3 = 0.315;
	const float L12 = 0.38325;
	const float H1 = 0.125;
	const float H3 = 0.065;
	const float H4 = 0.190;

	Robot_X = (L3 * C13) + (L12 * C1);
	Robot_Y = (L3 * S13) + (L12 * S1);
	Robot_Z = 0.38 + (J_q2 / 1000.0);
	Robot_Yaw = (J_q1 + J_q3 + J_q4) / 1000.0;

}

void fcb_IK(float gripper_linear_x, float gripper_linear_y,
		float gripper_linear_z, float gripper_angular_yaw) {
	float x = gripper_linear_x / 1000.0;
	float y = gripper_linear_y / 1000.0;
	float z = gripper_linear_z / 1000.0;
	float yaw = gripper_angular_yaw / 1000.0;

	static float L3 = 0.3150;
	static float L12 = 0.38325;
	static float H1 = 0.125;
	static float H3 = 0.065;
	static float H4 = 0.190;

	float C3 = ((x * x) + (y * y) - (L12 * L12) - (L3 * L3)) / (2 * L12 * L3);
	float S3 = -1.0 * sqrt(1 - (C3 * C3));
	float q3 = atan2(S3, C3);

	float L3S3 = L3 * S3;
	float L123C3 = L12 + (L3 * C3);

	float S1 = (-L3S3 * x) + (L123C3 * y);
	float C1 = (L3S3 * y) + (L123C3 * x);
	float q1 = atan2(S1, C1);
	float q4 = yaw - q1 - q3;
	float q2 = z + H4 - H3 - H1;

	Planning_q1 = q1 * 1000.0;
	Planning_q2 = q2 * 1000.0;
	Planning_q3 = q3 * 1000.0;
	Planning_q4 = q4 * 1000.0;

}

float offset_x = 430.0;
float offset_y = 9.5; //10.79371
float offset_x_new = 0;
float offset_y_new = 0;
int indexy;
float pos_x, pos_y;
void Update_State_Machine() {

	switch (control_state) {
	case 41: // Update Trajectory
		indexy = Call_queue();
		if (indexy != 255)
		{
			if (indexy == 99)
				{
					Planning_q1 = 1400;
					Planning_q3 = -1400;
					Planning_q4 = 0;
				}
			else if (indexy == 123)
			{
					Planning_q1 = 1100;
					Planning_q3 = -600;
					Planning_q4 = 0;
			}
			else
				{
					test_value_r = radias[indexy];
					test_value_theta = theta[indexy];
					offset_x_new = ((0.16075* (test_value_r * cos((test_value_theta + angle_chess) / 1000.0))) + 0.02289) / 10.0;
					offset_y_new = ((0.29560* (test_value_r * sin((test_value_theta + angle_chess) / 1000.0))) + 1.05911) / 10.0;
					pos_x = (test_value_r * cos((test_value_theta + angle_chess) / 1000.0)) + offset_x + offset_x_new;
					pos_y = (test_value_r * sin((test_value_theta + angle_chess) / 1000.0)) + offset_y + offset_y_new;
					fcb_IK(pos_x, pos_y, 0, 0);
				}
				Max_Time = 7;
				fcb_joint1.UpdateQuinticCoff(Max_Time, fcb_joint1.Encoder, Planning_q1, 0.0, 0.0, 0.0, 0.0);
				fcb_joint3.UpdateQuinticCoff(Max_Time, fcb_joint3.Encoder, Planning_q3, 0.0, 0.0, 0.0, 0.0);
				fcb_joint4.UpdateQuinticCoff(Max_Time, fcb_joint4.Encoder, Planning_q4, 0.0, 0.0, 0.0, 0.0);
				t = 0;
				joint13_on = true;
				HAL_TIM_Base_Start_IT(&htim14);
				control_state = 52;
		}

		else if (indexy == 255)
		{
			control_state = 0;
		}

//		Planning_q1
//		Planning_q3

		break;

	case 42:
		Max_Time = 12;
		joint13_on = false;
		fcb_joint2.UpdateQuinticCoff(Max_Time, fcb_joint2.Encoder,
				Setpoint_J2_Down, 0.0, 0.0, 0.0, 0.0);
		t = 0;
		HAL_TIM_Base_Start_IT(&htim14);
		control_state = 54;
		State_FIN = true;
		break;

	case 43:
		Max_Time = 12;
		joint13_on = false;
		fcb_joint2.UpdateQuinticCoff(Max_Time, fcb_joint2.Encoder,
				Setpoint_J2_Up, 0.0, 0.0, 0.0, 0.0);
		t = 0;
		HAL_TIM_Base_Start_IT(&htim14);
		control_state = 56;
		State_FIN = true;
		break;

	case 51:

//		State_FIN = true;
		break;

	case 52:
		HAL_TIM_Base_Stop_IT(&htim14);
		fcb_joint1.Goal_Velocity = 0;
		fcb_joint4.Goal_Velocity = 0;
		fcb_joint3.Goal_Velocity = 0;
		fcb_joint1.Output_Joint_W = 0;
		fcb_joint3.Output_Joint_W = 0;
		fcb_joint4.Output_Joint_W = 0;
		stepperJ1.StepperOpenLoopSpeedM(0.0);
		stepperJ4.StepperOpenLoopSpeedM(0.0);
		stepperJ3.StepperOpenLoopSpeedM(0.0);
		fcb_joint1.C0 = fcb_joint1.Encoder;
		fcb_joint4.C0 = fcb_joint4.Encoder;
		fcb_joint3.C0 = fcb_joint3.Encoder;
//		fcb_joint1.C0 = fcb_joint1.Goal_Position;
//		fcb_joint4.C0 = fcb_joint4.Goal_Position;
//		fcb_joint3.C0 = fcb_joint3.Goal_Position;

		fcb_joint1.C1 = 0;
		fcb_joint3.C1 = 0;
		fcb_joint1.C2 = 0;
		fcb_joint3.C2 = 0;
		fcb_joint1.C3 = 0;
		fcb_joint3.C3 = 0;
		fcb_joint1.C4 = 0;
		fcb_joint3.C4 = 0;
		fcb_joint1.C5 = 0;
		fcb_joint3.C5 = 0;
		fcb_joint4.C1 = 0;
		fcb_joint4.C2 = 0;
		fcb_joint4.C3 = 0;
		fcb_joint4.C4 = 0;
		fcb_joint4.C5 = 0;

		control_state = 42;
		State_FIN = true;
		break;

	case 53:

//		State_FIN = true;
		break;

	case 54:
		HAL_TIM_Base_Stop_IT(&htim14);
		fcb_joint2.Goal_Velocity = 0;
		stepperJ2.StepperOpenLoopSpeedM(0.0);
		fcb_joint2.C0 = fcb_joint2.Encoder;
		fcb_joint1.Output_Joint_W = 0;
		fcb_joint3.Output_Joint_W = 0;
		fcb_joint4.Output_Joint_W = 0;
		stepperJ1.StepperOpenLoopSpeedM(0.0);
		stepperJ4.StepperOpenLoopSpeedM(0.0);
		stepperJ3.StepperOpenLoopSpeedM(0.0);
		if (Gripper_State[first - 1] == 1) {
			gripper.GripperClose();
		} else if (Gripper_State[first - 1] == 2) {
			gripper.GripperOpen();
		}
//		for (int i =0 ;i <1000000;i++) {}
		fcb_joint2.C0 = 0;
		fcb_joint2.C1 = 0;
		fcb_joint2.C2 = 0;
		fcb_joint2.C3 = 0;
		fcb_joint2.C4 = 0;
		fcb_joint2.C5 = 0;
		fcb_joint2.Goal_Velocity = 0;
		control_state = 43;
		State_FIN = true;
		break;

	case 55:

		break;

	case 56:
		fcb_joint2.Goal_Velocity = 0;
		stepperJ2.StepperOpenLoopSpeedM(0.0);
		fcb_joint2.C0 = fcb_joint2.Encoder;
		fcb_joint1.Output_Joint_W = 0;
		fcb_joint3.Output_Joint_W = 0;
		fcb_joint4.Output_Joint_W = 0;
		stepperJ1.StepperOpenLoopSpeedM(0.0);
		stepperJ4.StepperOpenLoopSpeedM(0.0);
		stepperJ3.StepperOpenLoopSpeedM(0.0);
		HAL_TIM_Base_Stop_IT(&htim14);
		fcb_joint2.C1 = 0;
		fcb_joint2.C2 = 0;
		fcb_joint2.C3 = 0;
		fcb_joint2.C4 = 0;
		fcb_joint2.C5 = 0;
		fcb_joint2.Goal_Velocity = 0;
		control_state = 41;
		State_FIN = true;
		break;

	default:
		control_state = 0;
		State_FIN = false;
		break;
	}
}

int num = 30;
float box_q1[30];
float box_q2[30];
float box_q3[30];
float box_q4[30];
float idx, idy;

#endif

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
//				if (Old_Rx_Buffer[0] == 0x41 && cmdDataSize == 3) {	// Joint Jog q1
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dq1 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x42 && cmdDataSize == 3) {// Joint Jog q2
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dq2 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x43 && cmdDataSize == 3) {// Joint Jog q3
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dq3 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x44 && cmdDataSize == 3) {// Joint Jog q4
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dq4 = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x51 && cmdDataSize == 3) {// Linear Jog X
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dx = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x52 && cmdDataSize == 3) {// Linear Jog Y
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dy = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x53 && cmdDataSize == 3) {// Linear Jog Z
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dz = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
//				else if (Old_Rx_Buffer[0] == 0x54 && cmdDataSize == 3) {// Linear Jog Yaw
////					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
//					dyaw = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
//				}
				if (Old_Rx_Buffer[0] == 0x61 && cmdDataSize == 5) // Joint Jog 4q
						{
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dq1 = Old_Rx_Buffer[1];
					dq2 = Old_Rx_Buffer[2];
					dq3 = Old_Rx_Buffer[3];
					dq4 = Old_Rx_Buffer[4];
				} else if (Old_Rx_Buffer[0] == 0x71 && cmdDataSize == 5) // Linear Jog X,Y,Z,Yaw
						{
//					HAL_UART_Transmit_DMA(&huart3, &Old_Rx_Buffer[0], 1);
					dx = Old_Rx_Buffer[1];
					dy = Old_Rx_Buffer[2];
					dz = Old_Rx_Buffer[3];
					dyaw = Old_Rx_Buffer[4];
				} else if (Old_Rx_Buffer[0] == 0x85 && cmdDataSize == 2) {
					control_state = 51;
				} else if (Old_Rx_Buffer[0] == 0x86 && cmdDataSize == 3) //
						{
					int value_input = Old_Rx_Buffer[1];
					int value_gripper = Old_Rx_Buffer[2];
					Insert_queue(value_input, value_gripper);
				} else if (Old_Rx_Buffer[0] == 0x87 && cmdDataSize == 2) {
					char state_input = Old_Rx_Buffer[1];
					control_state = state_input;
					Update_State_Machine();
//					State_FIN = true;
				} else if (Old_Rx_Buffer[0] == 0x89 && cmdDataSize == 2) {
					Update_State_Machine();
				} else if (Old_Rx_Buffer[0] == 0x81 && cmdDataSize == 2) // Servo
						{
					gripperstate = Old_Rx_Buffer[1];
					if (gripperstate == 0) {
						gripper.GripperOpen();
//						HAL_UART_Transmit_DMA(&huart3, (uint8_t*) "A", 1);
					} else {
						gripper.GripperClose();
//						HAL_UART_Transmit_DMA(&huart3, (uint8_t*) "B", 1);
					}

				} else if (Old_Rx_Buffer[0] == 0x91 && cmdDataSize == 8) {
					px = (Old_Rx_Buffer[1] << 8) | Old_Rx_Buffer[2];
					py = (Old_Rx_Buffer[3] << 8) | Old_Rx_Buffer[4];
					pz = (Old_Rx_Buffer[5] << 8) | Old_Rx_Buffer[6];
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// tim5 100 Hz
// tim7 1000 Hz
// tim12 2000 Hz
// tim16 1000 Hz
// tim6 200 Hz
// tim14 500Hz

	if (htim == &htim6) {	//

	}

	if (htim == &htim12) {	//

	}

	if (htim == &htim7) {	//

	}

	if (htim == &htim5) {	//
		encoderJ1.AMT21_Read();
		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
		if (HALENCJ1OK == HAL_OK) {
			fcb_joint1.Encoder = encoderJ1.getAngPos180() / 2.609;
		}

		encoderJ2.AMT21_Read();
		HALENCJ2OK = encoderJ2.AMT21_Check_Value();
		if (HALENCJ2OK == HAL_OK) {
			encoderJ2.unwarp();
			fcb_joint2.Encoder = encoderJ2.getUnwarpValue() / 2.609;
		}

		encoderJ3.AMT21_Read();
		HALENCJ3OK = encoderJ3.AMT21_Check_Value();
		if (HALENCJ3OK == HAL_OK) {
			fcb_joint3.Encoder = encoderJ3.getAngPos180() / 2.609;
		}

		encoderJ4.AMT21_Read();
		HALENCJ4OK = encoderJ4.AMT21_Check_Value();
		if (HALENCJ4OK == HAL_OK) {
			fcb_joint4.Encoder = encoderJ4.getAngPos180() / 2.609;
		}
		angle_chess = chessABIEncoder.getMRadAngle();
		angle_chess_deg = chessABIEncoder.getDegAngle();
//		current_angle = chessSPIEncoder.getRawRotation();
//		current_angle_map = chessSPIEncoder.read2angle(current_angle);
//		angle = current_angle_map - zero_position_map;
//		angle = chessSPIEncoder.normalize(angle);
//			int i;
//			for (i = 1; i < num; i++) {
//				box_q1[i - 1] = box_q1[i];
//				box_q2[i - 1] = box_q2[i];
//				box_q3[i - 1] = box_q3[i];
//				box_q4[i - 1] = box_q4[i];
//			}
//			box_q1[num - 1] = dq1 / 10.0;
//			box_q2[num - 1] = dq2 / 10.0;
//			box_q3[num - 1] = dq3 / 10.0;
//			box_q4[num - 1] = dq4 / 10.0;
//
//			u_q1 = 0.0;
//			u_q2 = 0.0;
//			u_q3 = 0.0;
//			u_q4 = 0.0;
//
//			for (i = 0; i < num; i++) {
//				u_q1 += box_q1[i];
//				u_q2 += box_q2[i];
//				u_q3 += box_q3[i];
//				u_q4 += box_q4[i];
//			}

		//		stepperJ1.StepperSetFrequency(u_q1 * 3);
		//		stepperJ2.StepperSetFrequency(u_q2 * 1.5);
		//		stepperJ3.StepperSetFrequency(u_q3 * 2);
		//		stepperJ4.StepperSetFrequency(u_q4 * 2);
		//
//			stepperJ1.StepperSetFrequency(dq1);
//			stepperJ2.StepperSetFrequency(dq2*10.0);
//			stepperJ3.StepperSetFrequency(dq3);
//			stepperJ4.StepperSetFrequency(dq4);

		fcb_X.UpdateIVK(fcb_joint1.Encoder, 0.0, fcb_joint3.Encoder, 0.0, dx,
				dy, dz, 0.0);

		stepperJ1.StepperOpenLoopSpeedM(fcb_X.w_q1);
//			stepperJ2.StepperOpenLoopSpeedM(fcb_joint2.Goal_Velocity);
		stepperJ3.StepperOpenLoopSpeedM(fcb_X.w_q3);

//			int i;
//					for (i = 1 ; i<num ; i++)
//					{
//						box_q1[i-1] = box_q1[i];
//						box_q2[i-1] = box_q2[i];
//						box_q3[i-1] = box_q3[i];
//						box_q4[i-1] = box_q4[i];
//					}
//					 box_q1[num-1] = fcb_X.w_q1;
//					 box_q2[num-1] = dz;
//					 box_q3[num-1] = fcb_X.w_q3;
//					 box_q4[num-1] = fcb_X.w_q4;
//
//					u_q1 = 0.0;
//					u_q2 = 0.0;
//					u_q3 = 0.0;
//					u_q4 = 0.0;
//
//					for(i = 0; i < num; i++)
//					{
//						u_q1 += box_q1[i];
//						u_q2 += box_q2[i];
//						u_q3 += box_q3[i];
//						u_q4 += box_q4[i];
//					}

//					stepperJ1.StepperOpenLoopSpeed(u_q1/num);
//					stepperJ2.StepperSetFrequency(u_q2/3.0);
//					stepperJ3.StepperOpenLoopSpeed(u_q3/num);
//					stepperJ4.StepperOpenLoopSpeed(u_q4/num);

	}

	if (htim == &htim16) {
		encoderJ1.AMT21_Read();
		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
		if (HALENCJ1OK == HAL_OK) {
			fcb_joint1.Encoder = encoderJ1.getAngPos180() / 2.609;
		}

		encoderJ2.AMT21_Read();
		HALENCJ2OK = encoderJ2.AMT21_Check_Value();
		if (HALENCJ2OK == HAL_OK) {
			encoderJ2.unwarp();
			fcb_joint2.Encoder = encoderJ2.getUnwarpValue() / 2.609;
		}

		encoderJ3.AMT21_Read();
		HALENCJ3OK = encoderJ3.AMT21_Check_Value();
		if (HALENCJ3OK == HAL_OK) {
			fcb_joint3.Encoder = encoderJ3.getAngPos180() / 2.609;
		}

		encoderJ4.AMT21_Read();
		HALENCJ4OK = encoderJ4.AMT21_Check_Value();
		if (HALENCJ4OK == HAL_OK) {
			fcb_joint4.Encoder = encoderJ4.getAngPos180() / 2.609;
		}
		angle_chess = chessABIEncoder.getMRadAngle();
		angle_chess_deg = chessABIEncoder.getDegAngle();
//		current_angle = chessSPIEncoder.getRawRotation();
//		current_angle_map = chessSPIEncoder.read2angle(current_angle);
//		angle = current_angle_map - zero_position_map;
//		angle = chessSPIEncoder.normalize(angle);

		fcb_joint1.KalmanFillter(fcb_joint1.Encoder);
		fcb_joint1.kalman_pos = fcb_joint1.X11;
		fcb_joint1.kalman_velo = fcb_joint1.X21;

		fcb_joint2.KalmanFillter(fcb_joint2.Encoder);
		fcb_joint2.kalman_pos = fcb_joint2.X11;
		fcb_joint2.kalman_velo = fcb_joint2.X21;

		fcb_joint3.KalmanFillter(fcb_joint3.Encoder);
		fcb_joint3.kalman_pos = fcb_joint3.X11;
		fcb_joint3.kalman_velo = fcb_joint3.X21;

		if (State_FIN) {
			Balloon++;
			Update_State_Machine();
			Update_State_Machine();
			State_FIN = false;
		}

	}

	if (htim == &htim14) {
		// ######################## Encoder ##########################################
//		encoderJ1.AMT21_Read();
//		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
//		if (HALENCJ1OK == HAL_OK) {
//			fcb_joint1.Encoder = encoderJ1.getAngPos180() / 2.609 ;
//		}
//
//		encoderJ2.AMT21_Read();
//		HALENCJ2OK = encoderJ2.AMT21_Check_Value();
//		if(HALENCJ2OK == HAL_OK){
//			encoderJ2.unwarp();
//			fcb_joint2.Encoder = encoderJ2.getUnwarpValue() / 2.609 ;
//		}
//
//
//		encoderJ3.AMT21_Read();
//		HALENCJ3OK = encoderJ3.AMT21_Check_Value();
//		if (HALENCJ3OK == HAL_OK) {
//			fcb_joint3.Encoder = encoderJ3.getAngPos180() / 2.609 ;
//		}
//
//		encoderJ4.AMT21_Read();
//		HALENCJ4OK = encoderJ4.AMT21_Check_Value();
//		if (HALENCJ4OK == HAL_OK) {
//			fcb_joint4.Encoder = encoderJ4.getAngPos180() / 2.609 ;
//		}
		// #############################################################################

		// ######################## Setpoint ##########################################
		float t2 = t * t;
		float t3 = t * t * t;
		float t4 = t * t * t * t;
		float t5 = t * t * t * t * t;

//		fcb_FK(fcb_joint1.Encoder, 0, fcb_joint3.Encoder, fcb_joint4.Encoder);
//
//		fcb_X.Goal_Velocity = (fcb_X.C1 + (2.0*fcb_X.C2*t) + (3.0*fcb_X.C3*t2) + (4.0*fcb_X.C4*t3) + (5.0*fcb_X.C5*t4));
//		fcb_X.Goal_Position = ((fcb_X.C0 + (fcb_X.C1*t) + (fcb_X.C2*t2) + (fcb_X.C3*t3) + (fcb_X.C4*t4) + (fcb_X.C5*t5)));
//
//		fcb_X.UpdateIVK(fcb_joint1.Encoder, 0.0, fcb_joint3.Encoder, fcb_joint4.Encoder, fcb_X.Goal_Velocity, fcb_X.Goal_Velocity, 0.0, 0.0);
////	fcb_X.FindIK(fcb_X.Goal_Position/1000.0, Robot_Y, Robot_Z, Robot_Yaw);
//
//		fcb_joint1.Goal_Velocity = fcb_X.w_q1;
////	fcb_joint1.Goal_Position = fcb_X.q1;
//		fcb_joint3.Goal_Velocity = fcb_X.w_q3;
//		fcb_joint4.Goal_Velocity = fcb_X.w_q4;
////	fcb_joint3.Goal_Position = fcb_X.q3;

		fcb_joint1.Goal_Velocity = fcb_joint1.C1 + (2.0 * fcb_joint1.C2 * t)
				+ (3.0 * fcb_joint1.C3 * t2) + (4.0 * fcb_joint1.C4 * t3)
				+ (5.0 * fcb_joint1.C5 * t4);
		fcb_joint1.Goal_Position = (fcb_joint1.C0 + (fcb_joint1.C1 * t)
				+ (fcb_joint1.C2 * t2) + (fcb_joint1.C3 * t3)
				+ (fcb_joint1.C4 * t4) + (fcb_joint1.C5 * t5));

		fcb_joint2.Goal_Velocity = fcb_joint2.C1 + (2.0 * fcb_joint2.C2 * t)
				+ (3.0 * fcb_joint2.C3 * t2) + (4.0 * fcb_joint2.C4 * t3)
				+ (5.0 * fcb_joint2.C5 * t4);
		fcb_joint2.Goal_Position = (fcb_joint2.C0 + (fcb_joint2.C1 * t)
				+ (fcb_joint2.C2 * t2) + (fcb_joint2.C3 * t3)
				+ (fcb_joint2.C4 * t4) + (fcb_joint2.C5 * t5));

		fcb_joint3.Goal_Velocity = fcb_joint3.C1 + (2.0 * fcb_joint3.C2 * t)
				+ (3.0 * fcb_joint3.C3 * t2) + (4.0 * fcb_joint3.C4 * t3)
				+ (5.0 * fcb_joint3.C5 * t4);
		fcb_joint3.Goal_Position = (fcb_joint3.C0 + (fcb_joint3.C1 * t)
				+ (fcb_joint3.C2 * t2) + (fcb_joint3.C3 * t3)
				+ (fcb_joint3.C4 * t4) + (fcb_joint3.C5 * t5));

		fcb_joint4.Goal_Velocity = fcb_joint4.C1 + (2.0 * fcb_joint4.C2 * t)
				+ (3.0 * fcb_joint4.C3 * t2) + (4.0 * fcb_joint4.C4 * t3)
				+ (5.0 * fcb_joint4.C5 * t4);
		fcb_joint4.Goal_Position = (fcb_joint4.C0 + (fcb_joint4.C1 * t)
				+ (fcb_joint4.C2 * t2) + (fcb_joint4.C3 * t3)
				+ (fcb_joint4.C4 * t4) + (fcb_joint4.C5 * t5));

		// #############################################################################

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
		//		Update_ivk(fcb_joint1.Encoder / 2.609 , 0, fcb_joint3.Encoder / 2.609, 0, idx, idy, 0.0, 0.0);
		// Fuck you

		//		fcb_joint1.Error_p = fcb_joint1.Goal_Position - fcb_joint1.Encoder;
		//		fcb_joint3.Error_p = fcb_joint3.Goal_Position - fcb_joint3.Encoder;
		//		fcb_joint1.Sum_Error_p += fcb_joint1.Error_p;
		//		fcb_joint3.Sum_Error_p += fcb_joint3.Error_p;

		fcb_joint1.Kp_p = 0.4;
		fcb_joint1.Ki_p = 0.0;
		fcb_joint1.Kd_p = 0.0;

		fcb_joint2.Kp_p = 0.03;
		fcb_joint2.Ki_p = 0.0;
		fcb_joint2.Kd_p = 0.0;

		fcb_joint3.Kp_p = 0.15;
		fcb_joint3.Ki_p = 0.0;
		fcb_joint3.Kd_p = 0.0;

//		fcb_joint1.Kp_p = 0.0;
//		fcb_joint1.Ki_p = 0.0;
//		fcb_joint1.Kd_p = 0.0;
//		fcb_joint3.Kp_p = 0.0;
//		fcb_joint3.Ki_p = 0.0;
//		fcb_joint3.Kd_p = 0.0;

		fcb_joint1.Kp_v = 0.0; // 3.2
		fcb_joint1.Ki_v = 0.0;
		fcb_joint1.Kd_v = 0.0;

		fcb_joint2.Kp_v = 0.0; // 3.2
		fcb_joint2.Ki_v = 0.0;
		fcb_joint2.Kd_v = 0.0;

		fcb_joint3.Kp_v = 0.0; // 3.2
		fcb_joint3.Ki_v = 0.0;
		fcb_joint3.Kd_v = 0.0;

		fcb_joint1.Error_p = fcb_joint1.Goal_Position - fcb_joint1.kalman_pos;
		fcb_joint3.Error_p = fcb_joint3.Goal_Position - fcb_joint3.kalman_pos;
		fcb_joint1.Error_v = fcb_joint1.Goal_Velocity - fcb_joint1.kalman_velo;
		fcb_joint3.Error_v = fcb_joint3.Goal_Velocity - fcb_joint3.kalman_velo;

		fcb_joint2.Error_p = fcb_joint2.Goal_Position - fcb_joint2.kalman_pos;
		fcb_joint2.Error_v = fcb_joint2.Goal_Velocity - fcb_joint2.kalman_velo;

		fcb_joint1.Sum_Error_p += fcb_joint1.Error_p;
		fcb_joint3.Sum_Error_p += fcb_joint3.Error_p;
		fcb_joint1.Sum_Error_v += fcb_joint1.Error_v;
		fcb_joint3.Sum_Error_v += fcb_joint3.Error_v;

		fcb_joint2.Sum_Error_p += fcb_joint2.Error_p;
		fcb_joint2.Sum_Error_v += fcb_joint2.Error_v;

		fcb_joint1.Output_Joint_W = fcb_joint1.Goal_Velocity
				+ (fcb_joint1.Kp_p * fcb_joint1.Error_p)
				+ (fcb_joint1.Ki_p * fcb_joint1.Sum_Error_p)
				+ (fcb_joint1.Kd_p * (fcb_joint1.Error_p - fcb_joint1.Old_p))
				+ (fcb_joint1.Kp_v * fcb_joint1.Error_v)
				+ (fcb_joint1.Ki_v * fcb_joint1.Sum_Error_v)
				+ (fcb_joint1.Kd_v * (fcb_joint1.Error_v - fcb_joint1.Old_v));

		fcb_joint2.Output_Joint_W = (fcb_joint2.Goal_Velocity)
				+ (fcb_joint2.Kp_p * fcb_joint2.Error_p)
				+ (fcb_joint2.Ki_p * fcb_joint2.Sum_Error_p)
				+ (fcb_joint2.Kd_p * (fcb_joint2.Error_p - fcb_joint2.Old_p))
				+ (fcb_joint2.Kp_v * fcb_joint2.Error_v)
				+ (fcb_joint2.Ki_v * fcb_joint2.Sum_Error_v)
				+ (fcb_joint2.Kd_v * (fcb_joint2.Error_v - fcb_joint2.Old_v));

		fcb_joint3.Output_Joint_W = (fcb_joint3.Goal_Velocity)
				+ (fcb_joint3.Kp_p * fcb_joint3.Error_p)
				+ (fcb_joint3.Ki_p * fcb_joint3.Sum_Error_p)
				+ (fcb_joint3.Kd_p * (fcb_joint3.Error_p - fcb_joint3.Old_p))
				+ (fcb_joint3.Kp_v * fcb_joint3.Error_v)
				+ (fcb_joint3.Ki_v * fcb_joint3.Sum_Error_v)
				+ (fcb_joint3.Kd_v * (fcb_joint3.Error_v - fcb_joint3.Old_v));
		if (joint13_on) {
			stepperJ1.StepperOpenLoopSpeedM(fcb_joint1.Output_Joint_W);
			stepperJ2.StepperOpenLoopSpeedM(0.0);
			stepperJ3.StepperOpenLoopSpeedM(fcb_joint3.Output_Joint_W);
			stepperJ4.StepperOpenLoopSpeedM(fcb_joint4.Goal_Velocity);
		} else {
			stepperJ1.StepperOpenLoopSpeedM(0.0);
			stepperJ3.StepperOpenLoopSpeedM(0.0);
			stepperJ2.StepperOpenLoopSpeedM(fcb_joint2.Goal_Velocity);
			stepperJ4.StepperOpenLoopSpeedM(0.0);
		}

//		stepperJ1.StepperOpenLoopSpeedM(0.0);
//		stepperJ3.StepperOpenLoopSpeedM(0.0);

//		stepperJ1.StepperOpenLoopSpeedM(fcb_joint1.Goal_Velocity);
//		stepperJ2.StepperOpenLoopSpeedM(fcb_joint2.Goal_Velocity);
//		stepperJ3.StepperOpenLoopSpeedM(fcb_joint3.Goal_Velocity);
//		stepperJ4.StepperOpenLoopSpeedM(fcb_joint4.Goal_Velocity);
//
//		stepperJ1.StepperOpenLoopSpeedM(fcb_joint1.Goal_Velocity);
//		stepperJ2.StepperOpenLoopSpeedM(fcb_joint2.Goal_Velocity);
//		stepperJ3.StepperOpenLoopSpeedM(0);


		fcb_joint1.Old_Error_p = fcb_joint1.Error_p;
		fcb_joint2.Old_Error_p = fcb_joint2.Error_p;
		fcb_joint3.Old_Error_p = fcb_joint3.Error_p;
		fcb_joint1.Old_Error_v = fcb_joint1.Error_v;
		fcb_joint2.Old_Error_v = fcb_joint2.Error_v;
		fcb_joint3.Old_Error_v = fcb_joint3.Error_v;

		fcb_joint1.Old_p = fcb_joint1.kalman_pos;
		fcb_joint3.Old_p = fcb_joint3.kalman_pos;
		fcb_joint2.Old_p = fcb_joint2.kalman_pos;
		fcb_joint2.Old_v = fcb_joint2.kalman_velo;
		fcb_joint1.Old_v = fcb_joint1.kalman_velo;
		fcb_joint3.Old_v = fcb_joint1.kalman_velo;

		t = t + sample_time_500;
		if (t >= Max_Time) {
			t = (int) 0;
			fcb_joint1.Sum_Error_p = 0;
			fcb_joint2.Sum_Error_p = 0;
			fcb_joint3.Sum_Error_p = 0;

			fcb_joint1.Sum_Error_v = 0;
			fcb_joint2.Sum_Error_v = 0;
			fcb_joint3.Sum_Error_v = 0;

			fcb_joint1.Old_Error_p = 0;
			fcb_joint2.Old_Error_p = 0;
			fcb_joint3.Old_Error_p = 0;

			fcb_joint1.Old_Error_v = 0;
			fcb_joint2.Old_Error_v = 0;
			fcb_joint3.Old_Error_v = 0;

			fcb_joint1.Old_p = 0;
			fcb_joint2.Old_p = 0;
			fcb_joint3.Old_p = 0;

			fcb_joint1.Old_v = 0;
			fcb_joint2.Old_v = 0;
			fcb_joint3.Old_v = 0;
			State_FIN = true;
		}

	}
	if (htim == &htim17) {
		uint8_t encoder_state[12] = { (uint8_t) (((int16_t) fcb_joint1.Encoder
				>> 16) & 0xFF), (uint8_t) (((int16_t) fcb_joint1.Encoder >> 8)
				& 0xFF), (uint8_t) (((int16_t) fcb_joint1.Encoder) & 0xFF),
				(uint8_t) (((int32_t) fcb_joint2.Encoder >> 16) & 0xFF),
				(uint8_t) (((int32_t) fcb_joint2.Encoder >> 8) & 0xFF),
				(uint8_t) (((int32_t) fcb_joint2.Encoder) & 0xFF),
				(uint8_t) (((int32_t) fcb_joint3.Encoder >> 16) & 0xFF),
				(uint8_t) (((int16_t) fcb_joint3.Encoder >> 8) & 0xFF),
				(uint8_t) (((int16_t) fcb_joint3.Encoder) & 0xFF),
				(uint8_t) (((int32_t) fcb_joint4.Encoder >> 16) & 0xFF),
				(uint8_t) (((int16_t) fcb_joint4.Encoder >> 8) & 0xFF),
				(uint8_t) (((int16_t) fcb_joint4.Encoder) & 0xFF), };
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) &encoder_state, 12);
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
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*) New_Rx_Buffer,
			Rx_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

#ifdef __cplusplus
	stepperJ1.StepperSetFrequency(0.0f);
	stepperJ1.StepperSetMicrostep(8);
	stepperJ1.StepperSetRatio(42);
	stepperJ1.StepperEnable();

	stepperJ2.StepperSetFrequency(0.0f);
	stepperJ2.StepperSetMicrostep(16);
	stepperJ2.StepperSetRatio(3);
	stepperJ2.StepperEnable();

	stepperJ3.StepperSetFrequency(0.0f);
	stepperJ3.StepperSetMicrostep(16);
	stepperJ3.StepperSetRatio(9);
	stepperJ3.StepperEnable();

	stepperJ4.StepperSetFrequency(0.0f);
	stepperJ4.StepperSetMicrostep(1);
	stepperJ4.StepperSetRatio(3);
	stepperJ4.StepperEnable();

//	gripper.setDegreeGripperClose(80);
	gripper.setDegreeGripperClose(145);
//	gripper.setDegreeGripperClose(40);
	gripper.setDegreeGripperOpen(57);
	gripper.ServoEnable();
	gripper.GripperClose();
	while (!Limit_sw_Z_Top) {
		stepperJ2.StepperSetFrequency(1200.0f);
	}
	stepperJ2.StepperSetFrequency(0.0f);
	HAL_Delay(200);
	stepperJ2.StepperSetFrequency(-800.0f);
	HAL_Delay(2000);
	stepperJ2.StepperSetFrequency(0.0f);
	HAL_Delay(1000);
	gripper.GripperOpen();
	Limit_sw_Z_Top = false;

	encoderJ1.AMT21_Read();
	HALENCJ1OK = encoderJ1.AMT21_Check_Value();
	if (HALENCJ1OK == HAL_OK) {
		fcb_joint1.Encoder = encoderJ1.getAngPos180() / 2.609;
	}

	encoderJ2.AMT21_Read();
	HALENCJ2OK = encoderJ2.AMT21_Check_Value();
	if (HALENCJ2OK == HAL_OK) {
		encoderJ2.unwarp();
		encoderJ2.setUnwarpZero();
		fcb_joint2.Encoder = encoderJ2.getUnwarpValue() / 2.609;
	}

	encoderJ3.AMT21_Read();
	HALENCJ3OK = encoderJ3.AMT21_Check_Value();
	if (HALENCJ3OK == HAL_OK) {
		fcb_joint3.Encoder = encoderJ3.getAngPos180() / 2.609;
	}

	encoderJ4.AMT21_Read();
	HALENCJ4OK = encoderJ4.AMT21_Check_Value();
	if (HALENCJ4OK == HAL_OK) {
		fcb_joint4.Encoder = encoderJ4.getAngPos180() / 2.609;
	}

//	angle_chess = chessSPIEncoder.readAngle();
//	  zero_position = chessSPIEncoder.getRawRotation();
//	  zero_position_map = chessSPIEncoder.read2angle(zero_position);

	fcb_joint1.p11 = 3.60381982 / 100000000.0;
	fcb_joint1.p12 = 1.08884194 / 10000000.0;
	fcb_joint1.p21 = 1.0888423 / 10000000.0;
	fcb_joint1.p22 = 6.59951866 / 10000000.0;

	fcb_joint3.p11 = 3.60381982 / 100000000.0;
	fcb_joint3.p12 = 1.08884194 / 10000000.0;
	fcb_joint3.p21 = 1.0888423 / 10000000.0;
	fcb_joint3.p22 = 6.59951866 / 10000000.0;

	fcb_joint1.Q = 0.001;
	fcb_joint1.R = 0.000001;

	fcb_joint2.Q = 0.001;
	fcb_joint2.R = 0.00003;

	fcb_joint3.Q = 0.001;
	fcb_joint3.R = 0.000001;

//	fcb_joint1.KalmanFillter(fcb_joint1.Encoder);
	fcb_joint1.X11 = fcb_joint1.Encoder;
	fcb_joint1.X21 = 0.0;
	fcb_joint1.kalman_pos = fcb_joint1.X11;
	fcb_joint1.kalman_velo = 0.0;
	fcb_joint1.Goal_Position = fcb_joint1.Encoder;

//	fcb_joint3.KalmanFillter(fcb_joint3.Encoder);
	fcb_joint2.X11 = fcb_joint2.Encoder;
	fcb_joint2.X21 = 0.0;
	fcb_joint2.kalman_pos = fcb_joint2.X11;
	fcb_joint2.kalman_velo = 0.0;
	fcb_joint2.Goal_Position = fcb_joint2.Encoder;

//	fcb_joint3.KalmanFillter(fcb_joint3.Encoder);
	fcb_joint3.X11 = fcb_joint3.Encoder;
	fcb_joint3.X21 = 0.0;
	fcb_joint3.kalman_pos = fcb_joint3.X11;
	fcb_joint3.kalman_velo = 0.0;
	fcb_joint3.Goal_Position = fcb_joint3.Encoder;

#endif

//		HAL_TIM_Base_Start_IT(&htim5); // Jog 		100 Hz
//		HAL_TIM_Base_Start_IT(&htim6); // Set home 	200 Hz
//		HAL_TIM_Base_Start_IT(&htim7); // Control 	1000 Hz
//		HAL_TIM_Base_Start_IT(&htim12); // 			2000 Hz
//		HAL_TIM_Base_Start_IT(&htim14); // 			500Hz

	chessABIEncoder.setZero();
	chessABIEncoder.EncoderReadEnable();
	chessABIEncoder.setZero();
	HAL_TIM_Base_Start_IT(&htim16); // 			1000Hz
	HAL_TIM_Base_Start_IT(&htim17);	// Joint State 50Hz
//	encoderJ2.AMT21_Set_Zero();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLQ = 96;
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

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 //
 //}
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LM_Z_BOTTOM_Pin) {	// Limit Switch Bottom Z-axis
		Limit_sw_Z_Bot = true;
	}
	if (GPIO_Pin == LM_Z_TOP_Pin) {	// Limit Switch Top Z-axis
		Limit_sw_Z_Top = true;
	}
	if (GPIO_Pin == LM_SERVO_Pin) {	// Limit Switch Servo
		Limit_sw_Gripper = true;
	}
	if (GPIO_Pin == LM_LED_Pin) {	// Limit Switch LED on Cabinet
		Limit_sw_Emergancy = true;
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

