/*
 * AMT21.h
 *
 *  Created on: Jan 26, 2022
 *      Author: SakuranohanaTH
 */

#ifndef INC_AMT21_H_
#define INC_AMT21_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Include
 */
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stdint.h"

class AMT21{
    public:
        AMT21(UART_HandleTypeDef *_amt21_huart, uint8_t _address);
        AMT21(UART_HandleTypeDef *_amt21_huart, uint8_t _address, GPIO_TypeDef *_DE_port, uint16_t _DE_Pin);
        ~AMT21();

        void AMT21_Read();
        HAL_StatusTypeDef AMT21_Check_Value();

        uint16_t getRawValue();
        int16_t getAngPos180();
        int16_t getAngPos360();
    private:
    	UART_HandleTypeDef *amt21_huart;
    	GPIO_TypeDef *DE_port;
    	uint16_t DE_pin;
    	uint8_t address;

    	uint16_t uart_buf;
//    	float ang_pos;
    	int16_t raw_value;
    	uint16_t value;
    	uint8_t k0;
    	uint8_t k1;

};
#ifdef __cplusplus
}
#endif
#endif /* INC_AMT21_H_ */
