/*
 * PID.h
 *
 *  Created on: 29 ม.ค. 2565
 *      Author: SakuranohanaTH
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"
#include "stm32h7xx_hal.h" // include for STM32H7xx HAL

typedef struct {
	float kp;		// Kp value
	float ki;		// Ki value
	float kd;		// Kd value
	uint8_t isInit;	//
	float s, p;
	uint8_t isSatOn;
	float u_lim;	// Upper limitation value
	float l_lim;	// Lower limitation value
} Controller;

void PIDInit(Controller *pid);
void PIDDeinit(Controller *pid);
void setKp(Controller *pid, float _kp);
void setKi(Controller *pid, float _ki);
void setKd(Controller *pid, float _kd);
void setPID(Controller *pid, float _kp, float _ki, float _kd);
float getKp(Controller *pid);
float getKi(Controller *pid);
float getKd(Controller *pid);
void turnSatOn(Controller *pid, float _l_lim, float _u_lim);
void turnSatOff(Controller *pid);
float saturate(Controller *pid, float _input);
float computePID(Controller *pid, float _setpoint, float _feedback);
float computePIDFF(Controller *pid, float _setpoint, float _kff, float _feedback);

#endif /* INC_CONTROLLER_H_ */
