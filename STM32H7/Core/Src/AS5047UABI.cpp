/*
 * AS5047UABI.cpp
 *
 *  Created on: Apr 26, 2022
 *      Author: SakuranohanaTH
 */
#include "AS5047UABI.h"

AS5047UABI::AS5047UABI(TIM_HandleTypeDef *_enc_htim, uint32_t _ENC_TIM_CHANNEL1, uint32_t _ENC_TIM_CHANNEL2){
	this->enc_htim = _enc_htim;
	this->ENC_TIM_CHANNEL1 = _ENC_TIM_CHANNEL1;
	this->ENC_TIM_CHANNEL2 = _ENC_TIM_CHANNEL2;
}
AS5047UABI::~AS5047UABI(){

}
void AS5047UABI::EncoderReadEnable(){
	HAL_TIM_Encoder_Start(this->enc_htim, this->ENC_TIM_CHANNEL1);
	HAL_TIM_Encoder_Start(this->enc_htim, this->ENC_TIM_CHANNEL2);
}
void AS5047UABI::EncoderReadDisable(){
	HAL_TIM_Encoder_Stop(this->enc_htim, this->ENC_TIM_CHANNEL1);
	HAL_TIM_Encoder_Stop(this->enc_htim, this->ENC_TIM_CHANNEL2);
}
void AS5047UABI::setZero(){
	this->enc_htim->Instance->CNT = 0;
}
uint16_t AS5047UABI::getRawCounter(){
	return this->enc_htim->Instance->CNT;
}
float AS5047UABI::getDegAngle(){
	return ((float)this->enc_htim->Instance->CNT / 2047.0f) * 360.0f;
}
float AS5047UABI::getRadAngle(){
	return ((float)this->enc_htim->Instance->CNT / 2047.0f) * 6.28f;
}
float AS5047UABI::getMRadAngle(){
	return ((float)this->enc_htim->Instance->CNT / 2047.0f) * 6280.0f;
}

