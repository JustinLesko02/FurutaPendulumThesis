/*
 * PendulumEncoder.h
 *
 *  Created on: Jan 9, 2026
 *      Author: justinlesko
 */

#ifndef INC_PENDULUMENCODER_H_
#define INC_PENDULUMENCODER_H_
#include <stdint.h>
#include "stm32l4xx_hal.h"

/*
 */
typedef struct i2c_encoder {

I2C_HandleTypeDef* hi2c;

int initial_pendulum;

int encoder_angle;

} i2c_encoder;

void initialize_pendulum_encoder(i2c_encoder*);
void read_pendulum_encoder (i2c_encoder*);
void set_encoder_angle (i2c_encoder*);

#endif /* INC_MOTOR_H_ */

