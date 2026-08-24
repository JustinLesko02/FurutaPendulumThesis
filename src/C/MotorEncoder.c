/*
 * MotorEncoder.c
 *
 *  Created on: Jan 26, 2026
 *      Author: justinlesko
 */

#include "stm32l4xx_hal.h"
#include "MotorEncoder.h"
#include <stdint.h>



void initialize_motor_encoder(motor_encoder* motor_encoder){
	HAL_TIM_Encoder_Start(motor_encoder->htim, TIM_CHANNEL_ALL);
}

uint32_t read_motor_encoder(motor_encoder* motor_encoder){
	return motor_encoder->TIM->CNT;
}




