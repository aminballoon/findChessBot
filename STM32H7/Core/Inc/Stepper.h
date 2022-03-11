/*
 * Stepper.h
 *
 *  Created on: Jan 29, 2022
 *      Author: SakuranohanaTH
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32h7xx_hal.h" // include for STM32H7xx HAL

/*
 * Note
 *
 * TIM1 - Channel 2 Stepper Joint 1
 * TIM2 - Channel 3 Stepper Joint 2
 * TIM3 - Channel 1 Stepper Joint 3
 * TIM4 - Channel 3 Stepper Joint 4
 * TIM15 - Channel 2 Stepper Joint 5 / Servo Motor Joint 5
 *
 */

class Stepper {	// THIS CLASS, ENABLE PIN IS OFF AS DEFAULT!!!
    public:
        Stepper(TIM_HandleTypeDef *_stepper_htim, uint32_t _STEPPER_TIM_CHANNEL, GPIO_TypeDef *_DIRPort, uint32_t _DIRPin);
        ~Stepper();
        void        StepperEnable(void);
        void        StepperDisable(void);
        void        StepperSetFrequency(float _frequency);
        void        StepperSetMaxFrequency(float _max_frequency);
        void        StepperSetRatio(float _ratio);
        void        StepperSetMicrostep(uint8_t _microstep);
        void        StepperOpenLoopSpeed(float speed); //open-loop speed rad/s
        float		getFrequency();

    private:
        TIM_HandleTypeDef *stepper_htim;
        uint32_t	STEPPER_TIM_CHANNEL;
        GPIO_TypeDef *DIRPort;
        uint32_t	DIRPin;
        float       frequency; 		//	Hz
        float		minFrequency = 20.0f;	// Hz
        float       maxFrequency = 20000.0f;	// 	Hz
        float       microStep = 1;	// 	micro-step configuration on hardware
        float       SPR = 200.0f; 	//	Step(s) per revolution from datasheet NEMA17, NEMA23
        float       ratio = 1.0f;	//	Reducing Ratio on pulley
};

#ifdef __cplusplus
}
#endif
#endif /* INC_STEPPER_H_ */
