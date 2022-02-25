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
#include "stdbool.h"

class ServoMotor{
public:
	ServoMotor(TIM_HandleTypeDef *_servo_htim, uint32_t _SERVO_TIM_CHANNEL);
	~ServoMotor();
	void ServoEnable();
	void ServoDisable();
	void ServoRotateDegree(int32_t _sdegrotate);
	void setDegreeGripperOpen(int32_t _sdegopen);
	void setDegreeGripperClose(int32_t _sdegclose);
	void GripperClose();
	void GripperOpen();
	int32_t GripperMap(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
private:
    TIM_HandleTypeDef *servo_htim;
    uint32_t	SERVO_TIM_CHANNEL;
    int32_t sdegopen = 0;
	int32_t sdegclose = 0;
	int32_t minsdeg = 0;
	int32_t maxsdeg = 180;
	bool isSetDegreeGripperOpen = false;
	bool isSetDegreeGripperClose = false;
};

#ifdef __cplusplus
}
#endif


#endif /* INC_SERVOMOTOR_H_ */
