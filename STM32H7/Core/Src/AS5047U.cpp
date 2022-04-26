/*
 * AS5047U.cpp
 *
 *  Created on: Apr 25, 2022
 *      Author: SakuranohanaTH
 */
#include "AS5047U.h"
#include "crc.h"

AS5047U::AS5047U(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_port, uint16_t _cs_pin, bool _onCRC, float _offset){
	this->hspiHandle = _hspi;
	this->CSGPIOTypedef = _cs_port;
	this->CSGPIOPin = _cs_pin;
	if(_onCRC){
		this->hcrcHandle = &hcrc;
		this->onCRC = true;
	}
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);
	this->CORDIC_Overflow = 0;
	this->Offset_Compensation_Not_Finished = 0;
	this->Watchdog_Error = 0;
	this->CRC_Error = 0;
	this->Command_Error = 0;
	this->Framing_Error = 0;
	this->P2ram_Error = 0;
	this->P2ram_Warning = 0;
	this->MagHalf = 0;
	this->Agc_warning = 0;
	this->Offset = _offset * 0.000383495f;
}

AS5047U::~AS5047U(){

}

float AS5047U::EncPulse2Rad_Read(uint8_t inv_dir){
	this->AS5047U_Position_Highspeed_Read(inv_dir);
	return (this->Position * 0.000383495f) - this->Offset;
//	return (Enc->Position * 0.000383495f);
}


//
///*
// * Data Frame Format
// *
// * for 24 bits Data frame
// * 23	x			(Do not Care -> 0)
// * 22	R/W			(0 for read , 1 for write)
// * 21:8 ADDR[13:0]
// * 7:0	CRC-8
// *
// *
// * for 16 bits Data frame high throughput
// *
// * 15 	x			(Do not Care -> 0)
// * 14 	R/W			(0 for read , 1 for write)
// * 13:0 ADDR[13:0]
// */
//
//
//
//
///*
// * This function for Non-Volatile Registers (OTP) Only
// */
//
inline void AS5047U::AS5047U_Write(uint16_t Register_Address, uint16_t Data){
	if(!(this->onCRC)){
		return;
	}
	uint8_t Buffer[3] = {};

	Buffer[0] = ((uint8_t) (Register_Address >> 8)) & 0xBF;
	Buffer[1] = (uint8_t) (Register_Address & 0xFF);
	Buffer[2] = (uint8_t) HAL_CRC_Calculate(this->hcrcHandle, (uint32_t *)Buffer, 2) ^ 0xFF;
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	for (uint16_t i=0; i <= 550 ; i++);  			//delay before sent data (#Base clock 550MHz)
	HAL_SPI_Transmit(this->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);

	Buffer[0] = (uint8_t) (Data >> 8);
	Buffer[1] = (uint8_t) (Data & 0xFF);
	Buffer[2] = (uint8_t) HAL_CRC_Calculate(this->hcrcHandle, (uint32_t *)Buffer, 2) ^ 0xFF;
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	for (uint16_t i=0; i <= 550 ; i++);  			//delay before sent data (#Base clock 550MHz)
	HAL_SPI_Transmit(this->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);
}

inline HAL_StatusTypeDef AS5047U::AS5047U_Read(uint16_t Register_Address, uint16_t *Data){
	if(!(this->onCRC)){
		return HAL_ERROR;
	}
	uint8_t Buffer[3] = {};

	for (uint16_t i=0; i <= 480 ; i++);  			//delay before sent data (#Base clock 550MHz)
	Buffer[0] = ((uint8_t) (Register_Address >> 8)) | 0x40;
	Buffer[1] = (uint8_t) (Register_Address & 0xFF);
	Buffer[2] = (uint8_t) HAL_CRC_Calculate(this->hcrcHandle, (uint32_t *)Buffer, 2) ^ 0xFF;
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);

	for (uint16_t i=0; i <= 480 ; i++);  			//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	HAL_SPI_Receive(this->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);
	uint8_t AS5047U_crc = (uint8_t) HAL_CRC_Calculate(this->hcrcHandle, (uint32_t*)Buffer, 2) ^ 0xFF;
	if (AS5047U_crc == Buffer[2]){
		return HAL_OK;
	}
	else{
		return HAL_ERROR;
	}
}
//
//
///*
// * This function for read Encoder without CRC
// * (high throughput)
// */
inline uint16_t AS5047U::AS5047U_Position_Highspeed_Read(uint8_t dir){
	uint8_t cmd[2] = { 0x3F,0xFF };
	uint8_t Buffer[2] = {};
	for (uint16_t i=0; i <= 400; i++);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
//	for (uint16_t i=0; i <= 400; i++);
	HAL_SPI_Transmit(this->hspiHandle, cmd, 2, 100);
//	for (uint16_t i=0; i <= 400; i++);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);

	for (uint16_t i=0; i <= 400; i++);			//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
//	for (uint16_t i=0; i <= 400; i++);
	HAL_SPI_Receive(this->hspiHandle, Buffer, 2, 100);
//	for (uint16_t i=0; i <= 400; i++);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);
	if (dir == 1){
		this->Position = (uint16_t)((((uint16_t)Buffer[0]&0x3F) << 8) | (uint16_t)Buffer[1]) ^ 0x3FFF;
	}
	else{
		this->Position = ((((uint16_t)Buffer[0]&0x3F) << 8) | (uint16_t)Buffer[1]);
	}
	return this->Position;
}

inline int16_t AS5047U::AS5047U_Speed_Highspeed_Read() {

	uint8_t cmd[2] = { 0x3F, 0xFF };
	int16_t Output_Data;
	uint8_t Buffer[2] = { };
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->hspiHandle, cmd, 2, 1);
		HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);

	for (uint16_t i = 0; i <= 550; i++);		//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	HAL_SPI_Receive(this->hspiHandle, Buffer, 2, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);

	if ((Buffer[0] >> 5) == 0) {
		Output_Data = (((int16_t) Buffer[0] & 0x3F) << 8) | (int16_t) Buffer[1];
	} else if ((Buffer[0] >> 5) == 1) {
		Output_Data = (((int16_t) Buffer[0] | 0xC0) << 8) | (int16_t) Buffer[1];
	}
	return Output_Data;
}

void AS5047U::AS5047U_Read_Error() {
	uint16_t Status;
	uint8_t cmd[2] = { 0x00,0x01 };
	uint8_t Buffer[2] = {};
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->hspiHandle, cmd, 2, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);
	for (uint16_t i=0; i <= 550 ; i++);			//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_RESET);
	HAL_SPI_Receive(this->hspiHandle, Buffer, 2, 1);
	HAL_GPIO_WritePin(this->CSGPIOTypedef, this->CSGPIOPin, GPIO_PIN_SET);
	Status = (((uint16_t)Buffer[0]&0x3F) << 8) | (uint16_t)Buffer[1];

	AS5047U_Read(0x0001, &Status);
	this->CORDIC_Overflow 					= (uint8_t) ((Status >> 10) & 0x01);
	this->Offset_Compensation_Not_Finished 	= (uint8_t) ((Status >> 9) & 0x01);
	this->Watchdog_Error 					= (uint8_t) ((Status >> 7) & 0x01);
	this->CRC_Error 						= (uint8_t) ((Status >> 6) & 0x01);
	this->Command_Error 					= (uint8_t) ((Status >> 5) & 0x01);
	this->Framing_Error 					= (uint8_t) ((Status >> 4) & 0x01);
	this->P2ram_Error 						= (uint8_t) ((Status >> 3) & 0x01);
	this->P2ram_Warning 					= (uint8_t) ((Status >> 2) & 0x01);
	this->MagHalf 							= (uint8_t) ((Status >> 1) & 0x01);
	this->Agc_warning 						= (uint8_t) (Status & 0x01);
}
