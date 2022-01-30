/*
 * Stepper.c
 *
 *  Created on: 29 ม.ค. 2565
 *      Author: SakuranohanaTH
 */
#include "Stepper.h"

Stepper::Stepper(TIM_HandleTypeDef *_htim, uint32_t _TIMER_CHANNEL,
		GPIO_TypeDef *_DIRPort, uint32_t _DIRPin) {
	this->htim = _htim;
	this->TIM_CHANNEL = _TIMER_CHANNEL;
	this->htim->Instance->PSC = _PSC - 1U;
	this->minFrequency = 20.0f;
	this->maxFrequency = 20000.0f;
	this->StepperSetFrequency(0.0f);
	this->DIRPort = _DIRPort;
	this->DIRPin = _DIRPin;

}
Stepper::~Stepper() {
}
void Stepper::StepperEnable(void) { // ENABLE PIN IS OFF AS DEFAULT!!!
	HAL_TIM_PWM_Start(this->htim, this->TIM_CHANNEL);
}
void Stepper::StepperDisable(void) { // ENABLE PIN IS OFF AS DEFAULT!!!
	HAL_TIM_PWM_Stop(this->htim, this->TIM_CHANNEL);
}
void Stepper::StepperSetFrequency(float _frequency) {
	this->frequency = _frequency;

	float f;
	if (fabs(this->frequency) <= this->minFrequency)
		f = this->minFrequency;
	else if (fabs(this->frequency) >= this->maxFrequency)
		f = this->maxFrequency;
	else
		f = _frequency;

	if (this->frequency > 0.001f && this->frequency != 0.0f) {
		HAL_GPIO_WritePin(this->DIRPort, this->DIRPin, GPIO_PIN_SET);
		this->htim->Instance->ARR = round(
				(_FCY / ((this->htim->Instance->PSC + 1U) * f)) - 1U);
		if (this->TIM_CHANNEL == TIM_CHANNEL_1) {
			this->htim->Instance->CCR1 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_2) {
			this->htim->Instance->CCR2 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_3) {
			this->htim->Instance->CCR3 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_4) {
			this->htim->Instance->CCR4 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_5) {
			this->htim->Instance->CCR5 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_6) {
			this->htim->Instance->CCR6 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else {
			this->htim->Instance->CCR1 = 0;
			this->htim->Instance->CCR2 = 0;
			this->htim->Instance->CCR3 = 0;
			this->htim->Instance->CCR4 = 0;
			this->htim->Instance->CCR5 = 0;
			this->htim->Instance->CCR6 = 0;
		}

	} else if (this->frequency < 0.001f  &&  this->frequency != 0.0f) {
		HAL_GPIO_WritePin(this->DIRPort, this->DIRPin, GPIO_PIN_RESET);
		this->htim->Instance->ARR = round(
				(_FCY / ((this->htim->Instance->PSC + 1U) * fabs(f))) - 1U);
		if (this->TIM_CHANNEL == TIM_CHANNEL_1) {
			this->htim->Instance->CCR1 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_2) {
			this->htim->Instance->CCR2 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_3) {
			this->htim->Instance->CCR3 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_4) {
			this->htim->Instance->CCR4 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_5) {
			this->htim->Instance->CCR5 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_6) {
			this->htim->Instance->CCR6 = round(
					(this->htim->Instance->ARR + 1U ) / 2U );
		}

		else {
			this->htim->Instance->CCR1 = 0;
			this->htim->Instance->CCR2 = 0;
			this->htim->Instance->CCR3 = 0;
			this->htim->Instance->CCR4 = 0;
			this->htim->Instance->CCR5 = 0;
			this->htim->Instance->CCR6 = 0;
		}
	} else {
//		this->STEP.write(0.0f);
		if (this->TIM_CHANNEL == TIM_CHANNEL_1) {
			this->htim->Instance->CCR1 = 0;
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_2) {
			this->htim->Instance->CCR2 = 0;
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_3) {
			this->htim->Instance->CCR3 = 0;
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_4) {
			this->htim->Instance->CCR4 = 0;
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_5) {
			this->htim->Instance->CCR5 = 0;
		}

		else if (this->TIM_CHANNEL == TIM_CHANNEL_6) {
			this->htim->Instance->CCR6 = 0;
		}

		else {
			this->htim->Instance->CCR1 = 0;
			this->htim->Instance->CCR2 = 0;
			this->htim->Instance->CCR3 = 0;
			this->htim->Instance->CCR4 = 0;
			this->htim->Instance->CCR5 = 0;
			this->htim->Instance->CCR6 = 0;
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
	if(_speed > -0.07853981634f && _speed < 0.07853981634f){	// upper than abs(-20Hz) and lower than 20Hz
		this->StepperSetFrequency(0.0f);
	}
	else{
		this->StepperSetFrequency((float)(_speed*this->microStep*this->ratio*this->SPR*1/(2.0f*PI)));
	}
}
