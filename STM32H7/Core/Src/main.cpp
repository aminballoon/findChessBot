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
//#if __cplusplus
//using namespace std;
//#include <iostream>
//#endif
#include "AMT21.h"
#include "Stepper.h"
#include "Actuator.h"
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

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart7_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint32_t TIM_MS = 0;
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
static void MX_CRC_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
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
void Update_Coff(int kalman_pos, int y1, int kalman_velo, int y2, float Time);
void IPK_findChessBot(float X, float Y, float Z, float endEff_Yaw);
uint16_t CRC16(uint8_t *buf, int len);
#ifdef __cplusplus

//using namespace std;

AMT21 encoderJ1(&huart4, 0xD4);
//AMT21 encoderJ3(&huart4, 0xC4);

Stepper stepperJ1(&htim3, TIM_CHANNEL_1, DIR_3_GPIO_Port, DIR_3_Pin);
//Stepper stepperJ2(&htim2, TIM_CHANNEL_3, DIR_2_GPIO_Port, DIR_2_Pin);

//Stepper stepperJ1(&htim1, TIM_CHANNEL_2, DIR_1_GPIO_Port, DIR_1_Pin);
//Stepper stepperJ2(&htim2, TIM_CHANNEL_3, DIR_2_GPIO_Port, DIR_2_Pin);
Stepper stepperJ3(&htim15, TIM_CHANNEL_2, DIR_5_GPIO_Port, DIR_5_Pin);
//Stepper stepperJ4(&htim4, TIM_CHANNEL_3, DIR_4_GPIO_Port, DIR_4_Pin);
HAL_StatusTypeDef HALENCJ1OK, HALENCJ3OK;

volatile int16_t posJ1, posJ3 ;
volatile int32_t setpointJ1, setpointJ3;
volatile int direction_traj = 0;
volatile float errorJ1, errorJ3;
volatile float uJ1, uJ3;
volatile float chess_board_ang = 0.0;
volatile float debug_pos_x, debug_pos_y;
volatile u_int32_t county;
volatile float C1,S1,C3,S3,q1,q3;
static float L1 = 0.013245;
static float L2 = 0.370;
static float L3 = 0.315;
static float L12 = 0.383245;
static float H1 = 0.125;
static float H3 = 0.065;
static float H4 = 0.190;

volatile float t = 0.0;
volatile const float Time = 3;

volatile const float C0_q1 = 0.9;
volatile const float C2_q1 = (3.0*C0_q1) / (Time*Time);
volatile const float C3_q1 = (2.0*C0_q1) / (Time*Time*Time);

volatile const float C0_q3 = 0.8;
volatile const float C2_q3 = (3.0*C0_q3) / (Time*Time);
volatile const float C3_q3 = (2.0*C0_q3) / (Time*Time*Time);

volatile float bug1,bug2,bug3,bug4,bug5 ;

volatile const float sample_time_100 = 0.01;
volatile const float sample_time_200 = 0.005;
volatile const float sample_time_500 = 0.002;
volatile const float sample_time_1000 = 0.001;
volatile const float sample_time_2000 = 0.0005;

volatile const float chessboard_angular_velocity = 	2.0 * 0.10472; // rpm to rad/s

volatile bool direction = true;
volatile float Goal_velocity_q1,Goal_velocity_q3 ;

const float pi = 3.14159265;
//volatile float X11 = 0;
//volatile float X21 = 0;
//volatile float p11 = 1;
//volatile float p12 = 0;
//volatile float p21 = 0;
//volatile float p22 = 1;
volatile float kalman_pos = 0;
volatile float kalman_velo = 0;
//volatile float Q = 0.01 ;
//volatile float R = 0.00001;
volatile const float dt = sample_time_1000;
volatile const float dt2 = dt*dt;
volatile const float dt3 = dt*dt*dt;
volatile const float dt4 = dt*dt*dt*dt;

volatile float velocity_kalman_q1, velocity_kalman_q3, velocity_kalman_q1_new, velocity_kalman_q3_new, kalman_velo_input;
volatile float position_kalman_q1, position_kalman_q3, position_kalman_q1_new, position_kalman_q3_new;
volatile float unwrap_pose = 0;
volatile float Error_Old_q1 = 0.0;
volatile float Error_Old_q3 = 0.0;
volatile float Sum_Error_q1 = 0.0;
volatile float Sum_Error_q3 = 0.0;

struct joint_state {
    float q1,q2,q3,q4;
};
typedef struct joint_state joint_config;

struct robot_joint{
	volatile int16_t Encoder ;
	volatile float Goal_Position, Goal_Velocity ;
	volatile float Kalman_Position, Kalman_Velocity, Kalman_Position_New, Kalman_Velocity_New ;

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
	volatile float Q = 0.01 ;
	volatile float R = 0.0001;
};
typedef struct robot_joint fcb_joint;

struct robot_kinematic{
	volatile float Pos_x, Pos_y, Pos_z, Ori_yaw;
};
typedef struct robot_kinematic fcb_kinematic;

fcb_joint fcb_joint1, fcb_joint2, fcb_joint3, fcb_joint4;


void Update_ivk(float Vx, float Vy, float Vz, float Wz)
{
	float S13 = sin(q1+q3);
	float C13 = cos(q1+q3);
	float S3 = sin(q3);
	float S1 = sin(q1);
	float C1 = cos(q1);
	float L12 = L1 + L2;
	float L3S3 = L3 * S3;

	volatile float w_q1 = (Vx*C13 + Vy*S13)/(S3*L12);
	volatile float w_q2 = Vz;
	volatile float w_q3 = -(Vx*(L3*C13 + L1*C1 + L2*C1))/(L3*S3*L12) - (Vy*(L3*S13 + L1*S1 + L2*S1))/(L3*S3*L12);
	volatile float w_q4 = (Vx*C1 + Vy*S1 + L3*Wz*S3)/(L3*S3);

};

#endif

//void KalmanFilter(float theta_k,float X1,float X2,float P11,float P12,float P21,float P22, fcb_joint aaaa)
//{
//	bug1 = aaaa.Encoder;
//	float X1 =
//	X11 = X1 + (X2*dt) - ((X1 - theta_k + X2*dt)*(P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
//	X21 = X2 - (((Q*pow(dt,3))/2 + P22*dt + P21)*(X1 - theta_k + X2*dt))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
//	p11 = -((P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)) - 1)*(P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
//	p12 = -((P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)) - 1)*((Q*pow(dt,3))/2 + P22*dt + P12);
//	p21 = P21 + P22*dt + (Q*pow(dt,3))/2 - (((Q*pow(dt,3))/2 + P22*dt + P21)*(P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
//	p22 = P22 + Q*pow(dt,2) - (((Q*pow(dt,3))/2 + P22*dt + P12)*((Q*pow(dt,3))/2 + P22*dt + P21))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
////	 X11 = X1 + (X2*dt) - ((X1 - theta_k + X2*dt)*(P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
////	 X21 = X2 - (((Q*dt3)/2 + P22*dt + P21)*(X1 - theta_k + X2*dt))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
////	 p11 = -((P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)) - 1)*(P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
////	 p12 = -((P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)) - 1)*((Q*dt3)/2 + P22*dt + P12);
////     p21 = P21 + P22*dt + (Q*dt3)/2 - (((Q*dt3)/2 + P22*dt + P21)*(P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
////	 p22 = P22 + Q*dt2 - (((Q*dt3)/2 + P22*dt + P12)*((Q*dt3)/2 + P22*dt + P21))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
//}

fcb_joint KalmanFilter(float theta_k, fcb_joint joint)
{
//	bug1 = joint.Encoder;
	float X1 = joint.X11;
	float X2 = joint.X21;
	float P11 = joint.p11;
	float P12 = joint.p12;
	float P21 = joint.p21;
	float P22 = joint.p22;
	float Q = joint.Q;
	float R = joint.R;
	joint.X11 = X1 + (X2*dt) - ((X1 - theta_k + X2*dt)*(P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
	joint.X21 = X2 - (((Q*pow(dt,3))/2 + P22*dt + P21)*(X1 - theta_k + X2*dt))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
	joint.p11 = -((P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)) - 1)*(P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
	joint.p12 = -((P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)) - 1)*((Q*pow(dt,3))/2 + P22*dt + P12);
	joint.p21 = P21 + P22*dt + (Q*pow(dt,3))/2 - (((Q*pow(dt,3))/2 + P22*dt + P21)*(P11 + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
	joint.p22 = P22 + Q*pow(dt,2) - (((Q*pow(dt,3))/2 + P22*dt + P12)*((Q*pow(dt,3))/2 + P22*dt + P21))/(P11 + R + P21*dt + (Q*pow(dt,4))/4 + dt*(P12 + P22*dt));
	return  joint;

	//	 X11 = X1 + (X2*dt) - ((X1 - theta_k + X2*dt)*(P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
//	 X21 = X2 - (((Q*dt3)/2 + P22*dt + P21)*(X1 - theta_k + X2*dt))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
//	 p11 = -((P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)) - 1)*(P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
//	 p12 = -((P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)) - 1)*((Q*dt3)/2 + P22*dt + P12);
//   p21 = P21 + P22*dt + (Q*dt3)/2 - (((Q*dt3)/2 + P22*dt + P21)*(P11 + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt)))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
//	 p22 = P22 + Q*dt2 - (((Q*dt3)/2 + P22*dt + P12)*((Q*dt3)/2 + P22*dt + P21))/(P11 + R + P21*dt + (Q*dt2)/4 + dt*(P12 + P22*dt));
}

joint_config find_IK(float gripper_linear_x, float gripper_linear_y, float gripper_linear_z, float gripper_angular_yaw)
{
	bug1 = gripper_linear_x*gripper_linear_x;
	bug2 = gripper_linear_y*gripper_linear_y;
	bug3 = L12*L12;
	bug4 = L3*L3 ;
	C3 = ((gripper_linear_x*gripper_linear_x)+(gripper_linear_y*gripper_linear_y)-(L12*L12)-(L3*L3)) / (2*L12*L3);
	S3 = sqrt(1-(C3*C3));
	q3 = atan2(S3,C3);

	float L3S3 = L3*S3;
	float L123C3 = L12 + (L3*C3);

	S1 = (-L3S3*gripper_linear_x) + (L123C3*gripper_linear_y);
	C1 = (L3S3*gripper_linear_y) + (L123C3*gripper_linear_x);
	q1 = atan2(S1,C1);
	float q4 = gripper_angular_yaw - q1 - q3;
	float q2 = gripper_linear_z + H4 - H3 - H1;

	joint_config buff;
	buff.q1 = q1;
	buff.q2 = C3;
	buff.q3 = q3;
	buff.q4 = S3;

    return buff;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// tim5 100 Hz
	// tim7 1000 Hz
	// tim12 2000 Hz
	// tim6 200 Hz
	// tim14 500Hz
	if (htim == &htim6){	//

	}
	if (htim == &htim14){	//

	}
	if (htim == &htim5){	//
	}
	if (htim == &htim12){	//

	}
	if (htim == &htim7) { 	//

		encoderJ1.AMT21_Read();
		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
		if (HALENCJ1OK == HAL_OK) {
			fcb_joint1.Encoder = encoderJ1.getAngPos180() ;
		}

//		encoderJ3.AMT21_Read();
//		HALENCJ3OK = encoderJ3.AMT21_Check_Value();
//		if (HALENCJ3OK == HAL_OK) {
//			fcb_joint3.Encoder = encoderJ3.getAngPos180() ;
//		}



	    float t_2 = t*t;
	    float t_3 = t*t*t;

	    fcb_joint3.Goal_Position =  C0_q1 + (C2_q1*t_2) - (C3_q1*t_3);

//	    kalman_pos = fcb_joint1.Goal_Position;
	    kalman_pos = (fcb_joint1.Old_p - fcb_joint1.Encoder);

	    kalman_velo_input =  kalman_pos ;

	    if (direction_traj == 1){
	    	fcb_joint1.Goal_Position = unwrap_pose + (C0_q1 + (C2_q1*t_2) - (C3_q1*t_3)) - 0.8 ;
	    	fcb_joint1.Goal_Velocity = ((2.0*C2_q1*t) - (3.0 * C3_q1*t_2)) * -2 ;
	    	fcb_joint3.Goal_Velocity = ((2.0*C2_q1*t) - (3.0 * C3_q1*t_2)) * -2 ;
	    }
	    else
	    {
	    	fcb_joint1.Goal_Position = unwrap_pose - (C0_q1 + (C2_q1*t_2) - (C3_q1*t_3)) + 0.8;
	    	fcb_joint1.Goal_Velocity = ((2.0*C2_q1*t) - (3.0 * C3_q1*t_2)) * 2 ;
	    	fcb_joint3.Goal_Velocity = ((2.0*C2_q1*t) - (3.0 * C3_q1*t_2)) * 2 ;
	    }



//		fcb_joint1.Goal_Velocity = sin(0.314 * 2 * t) * 2000;
//		fcb_joint3.Goal_Velocity = sin(0.314 * 2 * t) * 4000;

	    chess_board_ang = chessboard_angular_velocity * t;

		joint_config findchessbot_joint_state;
//		debug_pos_x = 0.247*cos(chess_board_ang)+0.42744;
//		debug_pos_y = 0.247*sin(chess_board_ang)+0.00059371;
//		findchessbot_joint_state = find_IK(
//				debug_pos_x,
//				debug_pos_y,
//				0,
//				0);

//		fcb_joint1.Goal_Position = findchessbot_joint_state.q1 * 2607;
//		fcb_joint3.Goal_Position = findchessbot_joint_state.q3 * 2607;

//		fcb_joint1.Goal_Position = sin(chess_board_ang) * 1500.0;
//		fcb_joint3.Goal_Position = sin(chess_board_ang) * 2500.0;

//		setpointJ1 = Goal_velocity_q1;
//		setpointJ3 = Goal_velocity_q3;


		fcb_joint1.Error_p = fcb_joint1.Goal_Position - fcb_joint1.Encoder;
		fcb_joint3.Error_p = fcb_joint3.Goal_Position - fcb_joint3.Encoder;
		fcb_joint1.Sum_Error_p += fcb_joint1.Error_p;
		fcb_joint3.Sum_Error_p += fcb_joint3.Error_p;

		fcb_joint1.Kp_p = 0.0 ;
		fcb_joint1.Ki_p = 0.0 ;
		fcb_joint1.Kd_p = 0.0 ;

		fcb_joint1.Kp_v = 0.0 ;
		fcb_joint1.Ki_v = 0.0 ;
		fcb_joint1.Kd_v = 0.0 ;

		fcb_joint3.Kp_p = 0.0 ;
		fcb_joint3.Ki_p = 0.0 ;
		fcb_joint3.Kd_p = 0.0 ;

		fcb_joint3.Kp_v = 0.0 ;
		fcb_joint3.Ki_v = 0.0 ;
		fcb_joint3.Kd_v = 0.0 ;

//		KalmanFilter(float theta_k,float kalman_pos,float kalman_velo,float P11,float P12,float P21,float P22);
//		KalmanFilter(fcb_joint1.Encoder/ 2609.0 , X11, X21, p11, p12, p21, p22, fcb_joint1);
		fcb_joint1 = KalmanFilter(fcb_joint1.Encoder/ 2609.0 ,fcb_joint1);
//		fcb_joint1.Output_Stepper_Frequency = (fcb_joint1.Kp_p * fcb_joint1.Error_p) +
//											  (fcb_joint1.Ki_p * fcb_joint1.Sum_Error_p) +
//											  (fcb_joint1.Kd_p * (fcb_joint1.Error_p - fcb_joint1.Old_Error_p));
//
//		fcb_joint3.Output_Stepper_Frequency = (fcb_joint3.Kp_p * fcb_joint3.Error_p) +
//											  (fcb_joint3.Ki_p * fcb_joint3.Sum_Error_p) +
//											  (fcb_joint3.Kd_p * (fcb_joint3.Error_p - fcb_joint3.Old_Error_p));
//
//		fcb_joint1.Output_Stepper_Frequency = (fcb_joint1.Kp_p * fcb_joint1.Error_p);
//		fcb_joint3.Output_Stepper_Frequency = (fcb_joint3.Kp_p * fcb_joint3.Error_p);

		fcb_joint1.Output_Stepper_Frequency = fcb_joint1.Goal_Position;
		fcb_joint3.Output_Stepper_Frequency = fcb_joint3.Goal_Position;



		fcb_joint1.Old_Error_p = fcb_joint1.Error_p;
		fcb_joint3.Old_Error_p = fcb_joint3.Error_p;
		fcb_joint1.Old_p = fcb_joint1.Encoder;
		fcb_joint3.Old_p = fcb_joint3.Encoder;


//
		#ifdef __cplusplus
//		stepperJ1.StepperSetFrequency(300.0f);

//		stepperJ1.StepperSetFrequency(uJ1);
//		stepperJ3.StepperSetFrequency(0.0f);

		stepperJ1.StepperOpenLoopSpeed(fcb_joint1.Goal_Velocity);
		stepperJ3.StepperOpenLoopSpeed(fcb_joint3.Goal_Velocity);

		#endif

		t = t + (sample_time_1000) ;
		if (t >= Time)
		{
			t = 0.0;
			direction_traj ^= 1;
			unwrap_pose =  fcb_joint1.Goal_Position;
		}

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
	stepperJ1.StepperSetMicrostep(16);
	stepperJ1.StepperSetRatio(3);
	stepperJ1.StepperEnable();
//	stepperJ2.StepperSetMicrostep(1);
//	stepperJ2.StepperSetRatio(1);
//	stepperJ1.StepperSetFrequency(15842.0f);
	stepperJ3.StepperSetFrequency(0.0f);
	stepperJ3.StepperSetMicrostep(8);
	stepperJ3.StepperSetRatio(9);
	stepperJ3.StepperEnable();

//	stepperJ4.StepperSetMicrostep(1);
//	stepperJ4.StepperSetRatio(1);
#endif

	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim12);
	HAL_TIM_Base_Start_IT(&htim14);

	// Encoder
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
//	stepperJ1.StepperOpenLoopSpeed(1.00f);

	// Backup
	fcb_joint1.Kp_p = 0.0 ;
	fcb_joint1.Kp_v = 0.0 ;
	fcb_joint1.Ki_p = 0.0 ;
	fcb_joint1.Ki_v = 0.0 ;
	fcb_joint1.Kd_p = 0.0 ;
	fcb_joint1.Kd_p = 0.0 ;

	fcb_joint2.Kp_p = 0.0 ;
	fcb_joint2.Kp_v = 0.0 ;
	fcb_joint2.Ki_p = 0.0 ;
	fcb_joint2.Ki_v = 0.0 ;
	fcb_joint2.Kd_p = 0.0 ;
	fcb_joint2.Kd_p = 0.0 ;

	fcb_joint3.Kp_p = 0.0 ;
	fcb_joint3.Kp_v = 0.0 ;
	fcb_joint3.Ki_p = 0.0 ;
	fcb_joint3.Ki_v = 0.0 ;
	fcb_joint3.Kd_p = 0.0 ;
	fcb_joint3.Kd_p = 0.0 ;

	fcb_joint4.Kp_p = 0.0 ;
	fcb_joint4.Kp_v = 0.0 ;
	fcb_joint4.Ki_p = 0.0 ;
	fcb_joint4.Ki_v = 0.0 ;
	fcb_joint4.Kd_p = 0.0 ;
	fcb_joint4.Kd_p = 0.0 ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		encoderJ1.AMT21_Read();
//		HALENCJ1OK = encoderJ1.AMT21_Check_Value();
//		if (HALENCJ1OK == HAL_OK) {
//			fcb_joint1.Encoder = encoderJ1.getAngPos180() ;
//		}

//		encoderJ3.AMT21_Read();
//		HALENCJ3OK = encoderJ3.AMT21_Check_Value();
//		if (HALENCJ3OK == HAL_OK) {
//			fcb_joint3.Encoder = encoderJ3.getAngPos180() ;
//		}

//		for (int i = 0;i < 500; i++){
//			HAL_GPIO_TogglePin(STEP33_GPIO_Port, STEP33_Pin);
//			HAL_Delay(10);
//		}
//		stepperJ1.StepperSetFrequency(-300.0f);
//		stepperJ3.StepperSetFrequency(500.0f);
//		HAL_Delay(2500);

//		stepperJ1.StepperSetFrequency(0.0f);
//		stepperJ3.StepperSetFrequency(0.0f);
//		HAL_Delay(3000);

//		stepperJ1.StepperSetFrequency(300.0f);
//		stepperJ3.StepperSetFrequency(-500.0f);
//		HAL_Delay(2500);

//		stepperJ1.StepperSetFrequency(0.0f);
////		stepperJ3.StepperSetFrequency(0.0f);
//		HAL_Delay(1000);

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
  htim1.Init.Prescaler = 200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim3.Init.Prescaler = 200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2400-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 0;
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
  htim4.Init.Prescaler = 200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
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
  htim5.Init.Prescaler = 200-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 12000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 200-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1200-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  htim12.Init.Prescaler = 200-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 600-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 200-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 600-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 200-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2400-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  htim15.Init.Period = 20000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart4.Init.BaudRate = 2000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /*Configure GPIO pin : Blue_Button_Pin_Pin */
  GPIO_InitStruct.Pin = Blue_Button_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_Button_Pin_GPIO_Port, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

