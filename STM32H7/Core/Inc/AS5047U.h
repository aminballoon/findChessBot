/*
 * AS5047U.h
 *
 *  Created on: Apr 25, 2022
 *  Created Author : Thansak Pongpaket
 *  Modified Author: SakuranohanaTH
 *
 */

#ifndef INC_AS5047U_H_
#define INC_AS5047U_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx.h"
#include <math.h>


/* Volatile Registers Addresses */

#define	AS5047U_NOP_ADDRESS	0x0000
#define AS5047U_ERRFL_ADDRESS	0x0001
#define AS5047U_PROG_ADDRESS	0x0003
#define AS5047U_DIA_ADDRESS	0x3FF5
#define AS5047U_AGC_ADDRESS	0x3FF9
#define AS5047U_SIN_DATA_ADDRESS	0x3FFA
#define AS5047U_COS_DATA_ADDRESS	0x3FFB
#define AS5047U_VEL_ADDRESS	0x3FFC
#define AS5047U_MAG_ADDRESS	0x3FFD
#define AS5047U_ANGLEUNC_ADDRESS	0x3FFE
#define AS5047U_ANGLECOM_ADDRESS	0x3FFF
#define AS5047U_ECC_CHECKSUM_ADDRESS	0x00D1

/* Non-Volatile Registers Addresses */

#define AS5047U_ZPOSM_ADDRESS	0x0016
#define AS5047U_ZPOSL_ADDRESS	0x0017
#define AS5047U_SETTINGS1_ADDRESS	0x0018
#define AS5047U_SETTINGS2_ADDRESS	0x0019
#define AS5047U_SETTINGS3_ADDRESS	0x001A
#define AS5047U_ECC_ADDRESS	0x001B

#define AS5047U_TO_WRITE	0
#define AS5047U_TO_READ		1

#define AS5047U_RD 0x4000    // bit 14 = "1" is Read + parity even
#define AS5047U_WR 0x3FFF    // bit 14 = "0" is Write



class AS5047U{
public:
	AS5047U(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_port, uint16_t _cs_pin, bool _onCRC, float _offset);
	~AS5047U();
	void AS5047U_Write(uint16_t Register_Address, uint16_t Data);
	HAL_StatusTypeDef AS5047U_Read(uint16_t Register_Address, uint16_t *Data);
	uint16_t AS5047U_Position_Highspeed_Read(uint8_t dir);
	int16_t AS5047U_Speed_Highspeed_Read();
	void AS5047U_Read_Error();
	float EncPulse2Rad_Read(uint8_t inv_dir);
private:
	SPI_HandleTypeDef *hspiHandle;
	CRC_HandleTypeDef *hcrcHandle;
	GPIO_TypeDef *CSGPIOTypedef;
	uint16_t CSGPIOPin;
	uint16_t Position = 0;

	bool onCRC = false;

	float Offset = 0.0f;

	uint8_t CORDIC_Overflow = 0;
	uint8_t Offset_Compensation_Not_Finished = 0;
	uint8_t Watchdog_Error = 0;
	uint8_t CRC_Error = 0;
	uint8_t Command_Error = 0;
	uint8_t Framing_Error = 0;
	uint8_t P2ram_Error = 0;
	uint8_t P2ram_Warning = 0;
	uint8_t MagHalf = 0;
	uint8_t Agc_warning = 0;

};

#ifdef __cplusplus
}
#endif




#endif /* INC_AS5047U_H_ */
