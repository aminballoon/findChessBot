/*
 * AS5047U.h
 *
 *  Created on: Apr 25, 2022
 *      Author: SakuranohanaTH
 */

#ifndef INC_AS5047U_H_
#define INC_AS5047U_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#include <math.h>

class AS5047U{

	uint8_t errorFlag = 0;
	uint16_t _cs;
	uint16_t cs;
	GPIO_TypeDef* _ps;
	SPI_HandleTypeDef* _spi;
	uint8_t dout;
	uint8_t din;
	uint8_t clk;
	uint16_t position;
	uint16_t transaction(uint16_t data);

	public:

	/**
	 *	Constructor
	 */
	AS5047U(SPI_HandleTypeDef *hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs);

	/**
	 * Initialiser
	 * Sets up the SPI interface
	 */
	void init();

	/**
	 * Closes the SPI connection
	 */
	void close();

	/**
	 * Open the SPI connection
	 */
	void open();

	/*
	 * Read a register from the sensor
	 * Takes the address of the register as a 16 bit word
	 * Returns the value of the register
	 */
	uint16_t read(uint16_t registerAddress);

	/*
	 * Write to a register
	 * Takes the 16-bit  address of the target register and the 16 bit word of data
	 * to be written to that register
	 * Returns the value of the register after the write has been performed. This
	 * is read back from the sensor to ensure a sucessful write.
	 */
	uint16_t write(uint16_t registerAddress, uint16_t data);

	/**
	 * Returns the raw angle directly from the sensor
	 */
	uint16_t getRawRotation();

	/**
	 * Get the rotation of the sensor relative to the zero position.
	 *
	 * @return {int} between -2^13 and 2^13
	 */
	int getRotation();

	/**
	 * returns the value of the state register
	 * @return 16 bit word containing flags
	 */
	uint16_t getState();

	/*
	 * Check if an error has been encountered.
	 */
	uint8_t error();


	/**
	 * Returns the value used for Automatic Gain Control (Part of diagnostic
	 * register)
	 */
	uint8_t getGain();


	/*
	 * Get and clear the error register by reading it
	 */
	uint16_t getErrors();

	/*
	 * Set the zero position
	 */
	void setZeroPosition(uint16_t arg_position);

	/*
	 * Returns the current zero position
	 */
	uint16_t getZeroPosition();

	/*
	 * Returns normalized angle value
	 */
	float normalize(float angle);

	/*
	 * Returns calculated angle value
	 */
	float read2angle(uint16_t angle);

	private:

	uint8_t spiCalcEvenParity(uint16_t value);

};

/* Private defines -----------------------------------------------------------*/

const int AS5047U_CLEAR_ERROR_FLAG              = 0x0001;
const int AS5047U_PROGRAMMING_CONTROL           = 0x0003;
const int AS5047U_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const int AS5047U_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const int AS5047U_DIAG_AGC                      = 0x3FFD;
const int AS5047U_MAGNITUDE                     = 0x3FFE;
const int AS5047U_ANGLE                         = 0x3FFF;


#ifdef __cplusplus
}
#endif




#endif /* INC_AS5047U_H_ */
