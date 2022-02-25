/*
 * ServoMotor.cpp
 *
 *  Created on: Jan 30, 2022
 *      Author: SakuranohanaTH
 */
#include "ServoMotor.h"
ServoMotor::ServoMotor(TIM_HandleTypeDef *_servo_htim,
		uint32_t _SERVO_TIM_CHANNEL) {
	this->servo_htim = _servo_htim;
	this->SERVO_TIM_CHANNEL = _SERVO_TIM_CHANNEL;

	/*
	 * FPWM for Servo Motor is 50Hz or 20ms
	 *
	 * FPWM = FTIMER / ((PSC+1) * (ARR+1)) ; FTIMER = 24000000, PSC = 239, ARR = 19999
	 * FPWM = 50 Hz = 20 ms
	 *
	 * dutyPWM = CRR / (ARR+1)
	 * CRR = dutyPWM * (ARR+1)
	 *
	 * General for 180 degrees servo motor
	 *
	 * 1ms when 0deg
	 * 1ms/20ms = 0.05 = 5%
	 *
	 * CRR = 0.05 * (19999 + 1) = 1000
	 *
	 * 1.5ms when 90deg
	 * 1.5ms/20ms = 0.075 = 7.5%
	 *
	 * CRR = 0.075 * (19999 + 1) = 1500
	 *
	 * 2ms when 180deg
	 * 2ms/20ms = 0.1 = 10%
	 *
	 * CRR = 0.1 * (19999 + 1) = 2000
	 *
	 */
	this->servo_htim->Instance->PSC = _PSC_SERVO_MOTOR - 1U;
	this->servo_htim->Instance->ARR = _ARR_SERVO_MOTOR - 1U;
}
ServoMotor::~ServoMotor() {
}
void ServoMotor::ServoEnable() {
	HAL_TIM_PWM_Start(this->servo_htim, this->SERVO_TIM_CHANNEL);	// Enable PWM for Servo Motor
}
void ServoMotor::ServoDisable() {
	HAL_TIM_PWM_Stop(this->servo_htim, this->SERVO_TIM_CHANNEL);	// Disable PWM for Servo Motor
}
void ServoMotor::ServoRotateDegree(int32_t _sdegrotate) {
	if (this->SERVO_TIM_CHANNEL == TIM_CHANNEL_1) {
		this->servo_htim->Instance->CCR1 = (uint32_t) (this->GripperMap(
				_sdegrotate, this->minsdeg, this->maxsdeg, // Mapping degree value to CRR value
				round((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.05f),	// 1ms (default to 0 degree)
				floor((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.15f))); // 2ms (default to 180 degree)
	}

	else if (this->SERVO_TIM_CHANNEL == TIM_CHANNEL_2) {
		this->servo_htim->Instance->CCR2 = (uint32_t) (this->GripperMap(
				_sdegrotate, this->minsdeg, this->maxsdeg, // Mapping degree value to CRR value
				round((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.05f),	// 1ms (default to 0 degree)
				floor((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.15f))); // 2ms (default to 180 degree)
	}

	else if (this->SERVO_TIM_CHANNEL == TIM_CHANNEL_3) {
		this->servo_htim->Instance->CCR3 = (uint32_t) (this->GripperMap(
				_sdegrotate, this->minsdeg, this->maxsdeg, // Mapping degree value to CRR value
				round((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.05f),	// 1ms (default to 0 degree)
				floor((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.15f))); // 2ms (default to 180 degree)
	}

	else if (this->SERVO_TIM_CHANNEL == TIM_CHANNEL_4) {
		this->servo_htim->Instance->CCR4 = (uint32_t) (this->GripperMap(
				_sdegrotate, this->minsdeg, this->maxsdeg, // Mapping degree value to CRR value
				round((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.05f),	// 1ms (default to 0 degree)
				floor((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.15f))); // 2ms (default to 180 degree)
	}

	else if (this->SERVO_TIM_CHANNEL == TIM_CHANNEL_5) {
		this->servo_htim->Instance->CCR5 = (uint32_t) (this->GripperMap(
				_sdegrotate, this->minsdeg, this->maxsdeg, // Mapping degree value to CRR value
				round((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.05f),	// 1ms (default to 0 degree)
				floor((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.15f))); // 2ms (default to 180 degree)
	}

	else if (this->SERVO_TIM_CHANNEL == TIM_CHANNEL_6) {
		this->servo_htim->Instance->CCR6 = (uint32_t) (this->GripperMap(
				_sdegrotate, this->minsdeg, this->maxsdeg, // Mapping degree value to CRR value
				round((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.05f),	// 1ms (default to 0 degree)
				floor((float) ((this->servo_htim->Instance->ARR) + 1U) * 0.15f))); // 2ms (default to 180 degree)
	}

	else {
		this->servo_htim->Instance->CCR1 = 0;
		this->servo_htim->Instance->CCR2 = 0;
		this->servo_htim->Instance->CCR3 = 0;
		this->servo_htim->Instance->CCR4 = 0;
		this->servo_htim->Instance->CCR5 = 0;
		this->servo_htim->Instance->CCR6 = 0;
	}
}
void ServoMotor::setDegreeGripperOpen(int32_t _sdegopen) {
	this->sdegopen = _sdegopen;
	this->isSetDegreeGripperOpen = true;
}
void ServoMotor::setDegreeGripperClose(int32_t _sdegclose) {
	this->sdegclose = _sdegclose;
	this->isSetDegreeGripperClose = true;
}
void ServoMotor::GripperClose() {
	if (!this->isSetDegreeGripperClose)
		return;
	this->ServoRotateDegree(this->sdegclose);
}
void ServoMotor::GripperOpen() {
	if (!this->isSetDegreeGripperOpen)
		return;
	this->ServoRotateDegree(this->sdegopen);
}
int32_t ServoMotor::GripperMap(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
