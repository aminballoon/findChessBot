/*
 * ServoMotor.cpp
 *
 *  Created on: Jan 30, 2022
 *      Author: SakuranohanaTH
 */
#include "ServoMotor.h"
ServoMotor::ServoMotor(TIM_HandleTypeDef *_servo_htim, uint32_t _SERVO_TIM_CHANNEL) {
	this->servo_htim = _servo_htim;
	this->SERVO_TIM_CHANNEL = _SERVO_TIM_CHANNEL;
	this->servo_htim->Instance->PSC = _PSC_SERVO_MOTOR - 1U;
	this->servo_htim->Instance->ARR = _ARR_SERVO_MOTOR - 1U;
}
ServoMotor::~ServoMotor() {
}
void ServoMotor::ServoEnable(){
//	HAL_TIM_PWM_Start(this->htim, Channel)
}

