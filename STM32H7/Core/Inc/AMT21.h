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
#include "stm32h7xx_hal.h"
#include "stdint.h"

/*
 * STRUCT
 */
//typedef struct {
//	UART_HandleTypeDef *uartHandle;
////	GPIO_TypeDef *DE_port;
////	uint16_t DE_pin;
//	uint8_t address;
//
//	uint16_t uart_buf;
//	uint16_t position;
//	uint8_t k0;
//	uint8_t k1;
//} AMT21_typedef;

class AMT21{
    public:
        AMT21();
        ~AMT21();
//        JointState  state;
        void        setID(uint8_t);
        void        setRatio(float ratio);
        uint8_t     getID();
        void        setChecksum(bool checksumRequirement);
        bool        getChecksum();
        HAL_StatusTypeDef checkValue();
        int16_t    read();
        int16_t    read(uint8_t);
        double      readPosition();
        double      position();     //Filtered position
        float       velocity();     //Filtered position
        //Kalman Filter
        void        setKdt(float kalmanDt);
        float       getKdt();
        void        setSigmaW(float covariance);
        float       getSigmaW();
        void        setSigmaA(float covariance);
        float       getSigmaA();
        void        kmfInit();
        void        kmfEstimate();
        bool        continuous = 0;
    private:
//        RawSerial&  SER;
//        DigitalOut  FLOW;
        uint8_t     ID;
        float       ratio = 1.0f;   //Velocity ratio between encoder and destination
        bool        check = 1;      //Allow to calculate checksum
        int32_t     k_wrap = 0;
        int8_t      initial_pose = 0;
        //Kalman Filter
        bool        k_init = 0;
        float       p11, p12, p21, p22;
        float       x_hat_1, x_hat_2;
        float       k_prev_pos;
        float       kdt = 0.005f;   //Kalman Filter sampling time max: 0.0002883 sec(3468.208 Hz)
        float       sigma_a = 0.1f;
        float       sigma_w = 0.008f;
        unsigned char checksum(uint16_t);
};

/*
 * FUNCTIONS
 */
//void AMT21_initialise(AMT21 *dev, UART_HandleTypeDef *uartHandle,
//		uint8_t address);
//void AMT21_read_value(AMT21 *dev);
//HAL_StatusTypeDef AMT21_check_value(AMT21 *dev);
#ifdef __cplusplus
}
#endif
#endif /* INC_AMT21_H_ */
