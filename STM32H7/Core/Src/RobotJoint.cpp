/*
 * RobotJoint.cpp
 *
 *  Created on: Mar 9, 2022
 *      Author: SakuranohanaTH
 */

#include "RobotJoint.h"

RobotJoint::RobotJoint() {

}
RobotJoint::~RobotJoint() {

}
void RobotJoint::UpdateIVK(float _q1, float _q2, float _q3, float _q4, float Vx,
		float Vy, float Vz, float Wz) {
	float S13 = sin(_q1 + _q3);
	float C13 = cos(_q1 + _q3);
	float S3 = sin(_q3);
	float S1 = sin(_q1);
	float C1 = cos(_q1);
	float L3S3 = this->L3 * S3;

	this->w_q1 = (Vx * C13 + Vy * S13) / (S3 * this->L12);
	this->w_q2 = Vz;
	this->w_q3 = -(Vx * (this->L3 * C13 + this->L1 * C1 + this->L2 * C1))
			/ (L3S3 * this->L12)
			- (Vy * (this->L3 * S13 + this->L1 * S1 + this->L2 * S1))
					/ (L3S3 * this->L12);
	this->w_q4 = (Vx * C1 + Vy * S1 + this->L3 * Wz * S3) / (L3S3);
}
void RobotJoint::UpdateQuinticCoff(float T, float Start_pos, float Final_pos,
		float Start_velocity, float Final_velocity, float Start_acceleration,
		float Final_acceleration) {
	this->C0 = Start_pos;
	this->C1 = Start_velocity;
	this->C2 = Start_acceleration / 2.0;

	const float A = Final_pos
			- (Start_pos + (Start_velocity * T)
					+ (Start_acceleration * T * T / 2));
	const float B = Final_velocity
			- (Start_velocity + (Start_acceleration * T));
	const float C = Final_acceleration - Start_acceleration;

	const float T2 = T * T;
	const float T3 = T * T * T;
	const float T4 = T * T * T * T;
	const float T5 = T * T * T * T * T;

	this->C3 = (10.0 * A / T3) - (4.0 * B / T2) + (C / (2.0 * T));
	this->C4 = (-15.0 * A / T4) + (7.0 * B / T3) - (C / T2);
	this->C5 = (6.0 * A / T5) - (3.0 * B / T4) + (C / (2.0 * T3));
	this->T = T;
}
void RobotJoint::KalmanFillter(float theta_k) {
	float X1 = this->X11;
	float X2 = this->X21;
	float P11 = this->p11;
	float P12 = this->p12;
	float P21 = this->p21;
	float P22 = this->p22;
	float Q = this->Q;
	float R = this->R;

	this->X11 = X1 + (X2 * this->dt)
			- ((X1 - theta_k + X2 * this->dt)
					* (P11 + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
							+ this->dt * (P12 + P22 * this->dt)))
					/ (P11 + R + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
							+ this->dt * (P12 + P22 * this->dt));
	this->X21 = X2
			- (((Q * pow(this->dt, 3)) / 2 + P22 * this->dt + P21)
					* (X1 - theta_k + X2 * this->dt))
					/ (P11 + R + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
							+ this->dt * (P12 + P22 * this->dt));
	this->p11 = -((P11 + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
			+ this->dt * (P12 + P22 * this->dt))
			/ (P11 + R + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
					+ this->dt * (P12 + P22 * this->dt)) - 1)
			* (P11 + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
					+ this->dt * (P12 + P22 * this->dt));
	this->p12 = -((P11 + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
			+ this->dt * (P12 + P22 * this->dt))
			/ (P11 + R + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
					+ this->dt * (P12 + P22 * this->dt)) - 1)
			* ((Q * pow(this->dt, 3)) / 2 + P22 * this->dt + P12);
	this->p21 = P21 + P22 * this->dt + (Q * pow(this->dt, 3)) / 2
			- (((Q * pow(this->dt, 3)) / 2 + P22 * this->dt + P21)
					* (P11 + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
							+ this->dt * (P12 + P22 * this->dt)))
					/ (P11 + R + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
							+ this->dt * (P12 + P22 * this->dt));
	this->p22 = P22 + Q * pow(this->dt, 2)
			- (((Q * pow(this->dt, 3)) / 2 + P22 * this->dt + P12)
					* ((Q * pow(this->dt, 3)) / 2 + P22 * this->dt + P21))
					/ (P11 + R + P21 * this->dt + (Q * pow(this->dt, 4)) / 4
							+ this->dt * (P12 + P22 * this->dt));

}
void RobotJoint::FindIK(float gripper_linear_x, float gripper_linear_y,
		float gripper_linear_z, float gripper_angular_yaw) {
	this->bug1 = gripper_linear_x * gripper_linear_x;
	this->bug2 = gripper_linear_y * gripper_linear_y;
	this->bug3 = this->L12 * this->L12;
	this->bug4 = this->L3 * this->L3;
	float C3 = ((gripper_linear_x * gripper_linear_x)
			+ (gripper_linear_y * gripper_linear_y) - (this->L12 * this->L12)
			- (this->L3 * this->L3)) / (2 * this->L12 * this->L3);
	float S3 = sqrt(1 - (C3 * C3));
	float _q3 = atan2(S3, C3);

	float L3S3 = this->L3 * S3;
	float L123C3 = this->L12 + (this->L3 * C3);

	float S1 = (-L3S3 * gripper_linear_x) + (L123C3 * gripper_linear_y);
	float C1 = (L3S3 * gripper_linear_y) + (L123C3 * gripper_linear_x);
	float _q1 = atan2(S1, C1);
	float _q4 = gripper_angular_yaw - q1 - q3;
	float _q2 = gripper_linear_z + this->H4 - this->H3 - this->H1;

	this->q1 = _q1;
	this->q2 = C3;
	this->q3 = _q3;
	this->q4 = S3;
}
