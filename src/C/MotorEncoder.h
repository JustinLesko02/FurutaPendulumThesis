/*
 * MotorEncoder.h
 *
 *  Created on: Jan 9, 2026
 *      Author: justinlesko
 */

#ifndef INC_MOTORENCODER_H_
#define INC_MOTORENCODER_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

/*
 */
typedef struct motor_encoder {
	TIM_HandleTypeDef* htim;
	TIM_TypeDef* TIM;
	int ppr;
} motor_encoder;

void initialize_motor_encoder(motor_encoder* motor_encoder);
uint32_t read_motor_encoder(motor_encoder* motor_encoder);


#endif /* INC_MOTORENCODER_H_ */

