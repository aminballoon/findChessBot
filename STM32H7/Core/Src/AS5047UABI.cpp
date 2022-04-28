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
	int i = this->enc_htim->Instance->CNT ;
	int x = this->enc_htim->Instance->CNT / 1000;
	return (float)i;
}
float AS5047UABI::getRadAngle(){
	int i = this->enc_htim->Instance->CNT ;
	int x = this->enc_htim->Instance->CNT / 1000;

	return (float)((1-x)*i*0.00314) + (x*(i-1998)*0.003146293);
}
float AS5047UABI::getMRadAngle(){
	int i = this->enc_htim->Instance->CNT ;
	int x = this->enc_htim->Instance->CNT / 1000;
	return (float)((1-x)*i*3.14) + (x*(i-1998)*3.146293);
}

