/*
 * PID.c
 *
 *  Created on: 29 ม.ค. 2565
 *      Author: SakuranohanaTH
 */
#include "Controller.h"

void PIDInit(Controller *pid) {
	pid->kp = 0.0f;		// Kp value
	pid->ki = 0.0f;		// Ki value
	pid->kd = 0.0f;		// Kd value
	pid->s = 0.0f, pid->p = 0.0f;
	pid->isSatOn = 0;
	pid->u_lim = 0.0;	// Upper limitation value
	pid->l_lim = 0.0;	// Lower limitation value
	pid->isInit = 1;
}
void PIDDeinit(Controller *pid) {
	pid->isInit = 0;
}
void setPID(Controller *pid, float _kp, float _ki, float _kd) {
	pid->kp = _kp;
	pid->ki = _ki;
	pid->kd = _kd;
}
void setKp(Controller *pid, float _kp) {
	pid->kp = _kp;
}
void setKi(Controller *pid, float _ki) {
	pid->ki = _ki;
}
void setKd(Controller *pid, float _kd) {
	pid->kd = _kd;
}
float getKp(Controller *pid) {
	return pid->kp;
}
float getKi(Controller *pid) {
	return pid->ki;
}
float getKd(Controller *pid) {
	return pid->kd;
}
void turnSatOn(Controller *pid, float _l_lim, float _u_lim) {
	pid->l_lim = _l_lim;
	pid->u_lim = _u_lim;
	pid->isSatOn = 1;
}
void turnSatOff(Controller *pid) {
	pid->isSatOn = 0;
	pid->l_lim = 0;
	pid->u_lim = 0;
}
float saturate(Controller *pid, float _input) {
	float _output;
	if (_input <= pid->l_lim) {
		_output = pid->l_lim;
	} else if (_input >= pid->u_lim) {
		_output = pid->u_lim;
	} else {
		_output = _input;
	}
	return _output;
}
float computePID(Controller *pid, float _setpoint, float _feedback){
	float _value;
    if (pid->isInit){
        float _err = _setpoint-_feedback;
        pid->s += _err;
        float i_term = pid->ki*pid->s;
        if (pid->isSatOn)
            _value = saturate(pid, (pid->kp*_err + i_term + pid->kd*(_err-pid->p)));
        else
            _value = pid->kp*_err + i_term + pid->kd*(_err-pid->p);
        pid->p = _err;
    } else {
        _value = 0;
    }
    return _value;
}
float computePIDFF(Controller *pid, float _setpoint, float _kff, float _feedback){
	float _value;
    if (pid->isInit){
        float _err = _setpoint-_feedback;
        pid->s += _err;
        float i_term = pid->ki*pid->s;
        if (pid->isSatOn)
            _value = saturate(pid, (pid->kp*_err + i_term + pid->kd*(_err-pid->p) + _kff));
        else
            _value = pid->kp*_err + i_term + pid->kd*(_err-pid->p) + _kff;
        pid->p = _err;
    } else {
        _value = 0;
    }
    return _value;
}
