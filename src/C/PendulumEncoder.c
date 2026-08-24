/*
 * PendulumEncoder.c
 *
 *  Created on: Jan 9, 2026
 *      Author: justinlesko
 */
#include "PendulumEncoder.h"

//uint8_t I2C_buf[8];

static const uint8_t MAG_ADDRESS = 0b0110110 <<1; /**< I2C Address of the cal ic*/
static const uint8_t ZMCO = 0X00; /**< register Address of start position*/
static const uint8_t ZPOS1 = 0X01; /**< register Address of stop position 8-11*/
static const uint8_t ZPOS2 = 0X02; /**< register Address of stop position 0-7*/
static const uint8_t MPOS1 = 0X03; /**< register Address of XXXXXXXX*/
static const uint8_t MPOS2 = 0X04; /**< register Address of XXXXXXXX*/
static const uint8_t MANG1 = 0X05; /**< register Address of XXXXXXXX*/
static const uint8_t MANG2 = 0X06; /**< register Address of XXXXXXXX position*/
static const uint8_t CONF1 = 0X07; /**< register Address of XXXXXXXX position*/
static const uint8_t CONF2 = 0X08; /**< register Address of XXXXXXXX position*/
static const uint8_t RAWANGLE1 = 0X0C; /**< register Address of XXXXXXXX position*/
static const uint8_t RAWANGLE2 = 0X0D; /**< register Address of XXXXXXXX position*/
static const uint8_t ANGLE1 = 0X0E; /**< register Address of XXXXXXXX position*/
static const uint8_t ANGLE2 = 0X0F; /**< register Address of XXXXXXXX position*/
static const uint8_t STATUS = 0X0B; /**< register Address of XXXXXXXX position*/
static const uint8_t AGC = 0X1A; /**< register Address of XXXXXXXX position*/
static const uint8_t MAGNITUDE1 = 0X1B; /**< register Address of XXXXXXXX position*/
static const uint8_t MAGNITUDE2 = 0X1C; /**< register Address of XXXXXXXX position*/
static const uint8_t BURN = 0XFF; /**< register Address of XXXXXXXX position*/
static const uint8_t nop = 0X00; /**< register Address of XXXXXXXX position*/
static const uint8_t rd_pos = 0X10; /**< register Address of XXXXXXXX position*/
uint8_t I2C_buf[8];
int initial_pendulum = 0;
int encoder_ppr = 4096;
int rawangle = 0;
int encoder_initialized = 0;

void initialize_pendulum_encoder(i2c_encoder* encoder){
	read_pendulum_encoder(encoder);
}

void read_pendulum_encoder (i2c_encoder* encoder){
	int ret = HAL_I2C_Mem_Read_DMA(encoder->hi2c, MAG_ADDRESS, RAWANGLE1, 1, I2C_buf, 2);
		if ( ret != HAL_OK ) {
				//if not, throw and error
				//uart_print("ErrorRX Pendulum Encoder \r\n");
		}
		else {

		  rawangle = (I2C_buf[0]*256+I2C_buf[1]); // *360/4096
//		  int len = sprintf(&buf, "Pendulum Encoder value is: %d\n\r", rawangle);
//		  HAL_UART_Transmit(&huart2, &buf, len, 100);
		  (rawangle+encoder_ppr-encoder->initial_pendulum)%encoder_ppr;
		}
}

void set_encoder_angle(i2c_encoder* encoder){
	rawangle = (I2C_buf[0]*256+I2C_buf[1]); // *360/4096
	//		  int len = sprintf(&buf, "Pendulum Encoder value is: %d\n\r", rawangle);
	//		  HAL_UART_Transmit(&huart2, &buf, len, 100);
	encoder->encoder_angle = (rawangle+encoder_ppr-encoder->initial_pendulum)%encoder_ppr;
	if (encoder_initialized == 0){
		encoder->initial_pendulum = encoder->encoder_angle;
		initial_pendulum = encoder->initial_pendulum;
		encoder_initialized = 1;
	}
}
