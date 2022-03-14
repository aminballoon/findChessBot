/*
 * RobotJoint.h
 *
 *  Created on: Mar 9, 2022
 *      Author: SakuranohanaTH
 */

#ifndef INC_ROBOTJOINT_H_
#define INC_ROBOTJOINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32h7xx_hal.h" // include for STM32H7xx HAL

class RobotJoint {
public:
	RobotJoint();
	~RobotJoint();

	volatile const float dt = 0.001;
	volatile const float dt2 = pow(this->dt, 2);
	volatile const float dt3 = pow(this->dt, 3);
	volatile const float dt4 = pow(this->dt, 4);

	static constexpr float L1 = 0.01325; // 0.053
	static constexpr float L2 = 0.370; // 0.36625
	static constexpr float L3 = 0.315;
	static constexpr float L12 = 0.38325;
	static constexpr float H1 = 0.125;
	static constexpr float H3 = 0.065;
	static constexpr float H4 = 0.190;

	volatile float bug1, bug2, bug3, bug4, bug5;

	float q1, q2, q3, q4;

	volatile float w_q1;
	volatile float w_q2;
	volatile float w_q3;
	volatile float w_q4;

	volatile float Encoder;

	volatile float Goal_Position, Goal_Velocity;
	volatile float Kalman_Position, Kalman_Velocity, Kalman_Position_New,
			Kalman_Velocity_New;

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
	volatile float Q = 0.1;
	volatile float R = 0.000001;
	volatile float C0, C1, C2, C3, C4, C5, T;

	void FindIK(float gripper_linear_x, float gripper_linear_y,
			float gripper_linear_z, float gripper_angular_yaw);

	void UpdateIVK(float _q1, float _q2, float _q3, float _q4, float Vx,
			float Vy, float Vz, float Wz);

	void UpdateQuinticCoff(float T, float Start_pos, float Final_pos,
			float Start_velocity, float Final_velocity,
			float Start_acceleration, float Final_acceleration);

	void KalmanFillter(float theta_k);

private:
};

#ifdef __cplusplus
}
#endif
#endif /* INC_ROBOTJOINT_H_ */
