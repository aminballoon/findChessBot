/*
 * ServoMotor.h
 *
 *  Created on: Jan 30, 2022
 *      Author: SakuranohanaTH
 */

#ifndef INC_SERVOMOTOR_H_
#define INC_SERVOMOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32h7xx_hal.h"
#include "stdint.h"

class ServoMotor{
public:
	ServoMotor(TIM_HandleTypeDef *_servo_htim, uint32_t _SERVO_TIM_CHANNEL);
	~ServoMotor();
	void ServoEnable();
	void ServoDisable();
	void ServoRotateDegree();
	void setDegreeGripperOpen();
	void setDegreeGripperClose();
	void GripperClose();
	void GripperOpen();
	long GripperMap(long x, long in_min, long in_max, long out_min, long out_max);
private:
    TIM_HandleTypeDef *servo_htim;
    uint32_t	SERVO_TIM_CHANNEL;
	uint8_t sdegopen = 0;
	uint8_t sdegclose = 0;
};

#ifdef __cplusplus
}
#endif


#endif /* INC_SERVOMOTOR_H_ */
