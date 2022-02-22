/*
 * Stepper.cpp
 *
 *  Created on: Jan 29, 2022
 *      Author: SakuranohanaTH
 */
#include "Stepper.h"

Stepper::Stepper(TIM_HandleTypeDef *_stepper_htim, uint32_t _STEPPER_TIM_CHANNEL,
		GPIO_TypeDef *_DIRPort, uint32_t _DIRPin) {
	this->stepper_htim = _stepper_htim;
	this->STEPPER_TIM_CHANNEL = _STEPPER_TIM_CHANNEL;
	this->stepper_htim->Instance->PSC = _PSC_STEPPER_MOTOR - 1U;
	this->minFrequency = 60.0f;
	this->maxFrequency = 20000.0f;
	this->StepperSetFrequency(0.0f);
	this->DIRPort = _DIRPort;
	this->DIRPin = _DIRPin;
}
Stepper::~Stepper() {
}
void Stepper::StepperEnable(void) { // ENABLE PIN IS OFF AS DEFAULT!!!
	HAL_TIM_PWM_Start(this->stepper_htim, this->STEPPER_TIM_CHANNEL);
}
void Stepper::StepperDisable(void) { // ENABLE PIN IS OFF AS DEFAULT!!!
	HAL_TIM_PWM_Stop(this->stepper_htim, this->STEPPER_TIM_CHANNEL);
}
void Stepper::StepperSetFrequency(float _frequency) {
	this->frequency = _frequency ;

	float f;
	if (fabs(this->frequency) <= this->minFrequency)
		f = this->minFrequency;
	else if (fabs(this->frequency) >= this->maxFrequency)
		f = this->maxFrequency;
	else
		f = _frequency;

	if (this->frequency >= 0.001f) {
		HAL_GPIO_WritePin(this->DIRPort, this->DIRPin, GPIO_PIN_SET);
		this->stepper_htim->Instance->ARR = round(
				(_FCY / ((this->stepper_htim->Instance->PSC + 1U) * (f))) - 1U);
		if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_1) {
			this->stepper_htim->Instance->CCR1 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_2) {
			this->stepper_htim->Instance->CCR2 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_3) {
			this->stepper_htim->Instance->CCR3 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_4) {
			this->stepper_htim->Instance->CCR4 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_5) {
			this->stepper_htim->Instance->CCR5 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_6) {
			this->stepper_htim->Instance->CCR6 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else {
			this->stepper_htim->Instance->CCR1 = 0;
			this->stepper_htim->Instance->CCR2 = 0;
			this->stepper_htim->Instance->CCR3 = 0;
			this->stepper_htim->Instance->CCR4 = 0;
			this->stepper_htim->Instance->CCR5 = 0;
			this->stepper_htim->Instance->CCR6 = 0;
		}

	} else if (this->frequency <= -0.001f) {
		HAL_GPIO_WritePin(this->DIRPort, this->DIRPin, GPIO_PIN_RESET);
		this->stepper_htim->Instance->ARR = round(
				(_FCY / ((this->stepper_htim->Instance->PSC + 1U) * fabs(f))) - 1U);
		if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_1) {
			this->stepper_htim->Instance->CCR1 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_2) {
			this->stepper_htim->Instance->CCR2 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_3) {
			this->stepper_htim->Instance->CCR3 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_4) {
			this->stepper_htim->Instance->CCR4 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_5) {
			this->stepper_htim->Instance->CCR5 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_6) {
			this->stepper_htim->Instance->CCR6 = round(
					(this->stepper_htim->Instance->ARR + 1U) / 2U);
		}

		else {
			this->stepper_htim->Instance->CCR1 = 0;
			this->stepper_htim->Instance->CCR2 = 0;
			this->stepper_htim->Instance->CCR3 = 0;
			this->stepper_htim->Instance->CCR4 = 0;
			this->stepper_htim->Instance->CCR5 = 0;
			this->stepper_htim->Instance->CCR6 = 0;
		}
	} else {
		if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_1) {
			this->stepper_htim->Instance->CCR1 = 0;
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_2) {
			this->stepper_htim->Instance->CCR2 = 0;
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_3) {
			this->stepper_htim->Instance->CCR3 = 0;
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_4) {
			this->stepper_htim->Instance->CCR4 = 0;
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_5) {
			this->stepper_htim->Instance->CCR5 = 0;
		}

		else if (this->STEPPER_TIM_CHANNEL == TIM_CHANNEL_6) {
			this->stepper_htim->Instance->CCR6 = 0;
		}

		else {
			this->stepper_htim->Instance->CCR1 = 0;
			this->stepper_htim->Instance->CCR2 = 0;
			this->stepper_htim->Instance->CCR3 = 0;
			this->stepper_htim->Instance->CCR4 = 0;
			this->stepper_htim->Instance->CCR5 = 0;
			this->stepper_htim->Instance->CCR6 = 0;
		}
	}
}
void Stepper::StepperSetMaxFrequency(float _maxFrequency) {
	this->maxFrequency = fabs((float) _maxFrequency);
}
void Stepper::StepperSetRatio(float _ratio) {
	this->ratio = fabs(_ratio);
}
void Stepper::StepperSetMicrostep(uint8_t _microstep) {
	this->microStep = fabs(_microstep);
}
void Stepper::StepperOpenLoopSpeed(float _speed) {
//	if (_speed > -0.07853981634f && _speed < 0.07853981634f) { // upper than abs(-20Hz) and lower than 20Hz
//		this->StepperSetFrequency(0.0f);
//	} else {
		this->StepperSetFrequency(
				(float) (_speed * this->microStep * this->ratio * this->SPR * 1
						/ (2.0f * PI)));
//	}
}
float Stepper::getFrequency()
{
	return this->frequency;
}
