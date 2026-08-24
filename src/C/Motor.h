/*
 * Motor.h
 *
 *  Created on: Jan 9, 2026
 *      Author: justinlesko
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/*
 */
typedef struct motor {

TIM_HandleTypeDef* htim;

TIM_TypeDef* TIM;

uint32_t Channel1;

uint32_t Channel2;

uint32_t Channel3;

int32_t effort;

int32_t commutation_offset;

GPIO_TypeDef* GPIOIn;

uint16_t In1;

uint16_t In2;

uint16_t In3;

uint32_t max_PWM;

int32_t encoder_offset;

int32_t encoder_zero;

uint16_t Va;
uint16_t Vb;
uint16_t Vc;


} motor;

void motor_initialize(motor* motor);
void motor_seteffort(motor* motor, int16_t effort);
void set_motor_encoder_offset(motor* motor, int encoder_offset_set);
void motor_enable(motor* motor);
void motor_disable(motor* motor);
void motor_calibrate(motor* motor);
void motor_commutate(motor* motor, int encoder_ticks);


#endif /* INC_MOTOR_H_ */
