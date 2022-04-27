/*
 * AS5047UABI.h
 *
 *  Created on: Apr 26, 2022
 *      Author: SakuranohanaTH
 */

#ifndef INC_AS5047UABI_H_
#define INC_AS5047UABI_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx.h"
#include <math.h>


class AS5047UABI{
public:
	AS5047UABI(TIM_HandleTypeDef *_enc_htim, uint32_t _ENC_TIM_CHANNEL1, uint32_t _ENC_TIM_CHANNEL2);
	~AS5047UABI();
	void EncoderReadEnable();
	void EncoderReadDisable();
	void setZero();
	uint16_t getRawCounter();
	float getDegAngle();
	float getRadAngle();
	float getMRadAngle();
private:
	TIM_HandleTypeDef *enc_htim;
    uint32_t	ENC_TIM_CHANNEL1;
    uint32_t	ENC_TIM_CHANNEL2;
    uint8_t enc_dir = 0;
    uint8_t enc_resolution = 0x04;
};


#ifdef __cplusplus
}
#endif




#endif /* INC_AS5047UABI_H_ */
