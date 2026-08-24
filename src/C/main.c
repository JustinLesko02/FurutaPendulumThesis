/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "PendulumEncoder.h"
#include "MotorEncoder.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

motor bldcmotor = {.htim = &htim1,
				  .TIM = TIM1,
				  .Channel1 = TIM_CHANNEL_1,
				  .Channel2 = TIM_CHANNEL_2,
				  .Channel3 = TIM_CHANNEL_3,
				  .effort = 50,
				  .commutation_offset = 0,
				  .GPIOIn = GPIOC,
				  .In1 = GPIO_PIN_10,
				  .In2 = GPIO_PIN_11,
				  .In3 = GPIO_PIN_12,
				  .max_PWM = 3199,
	  	  	  	  .encoder_offset = 0,
				  .encoder_zero = 0,
				  .Va = 0,
				  .Vb = 0,
				  .Vc = 0
	  	  	  	  };
motor_encoder Motor_encoder = {.htim = &htim2,
		  .TIM = TIM2,
		  .ppr = 8192
};

i2c_encoder pendulum_encoder = {.hi2c = &hi2c1,
.initial_pendulum = 0,
.encoder_angle = 0};

char echo_buf[1];
const int dwelltime = 200;

char buf[100];
int transmit_flag = 0;
uint32_t current_ticks = 0;
uint32_t start_ticks = 0;

uint8_t SPI_TX[2];
uint8_t SPI_RX[2];
int len = 0;
int STATE = 0;
uint32_t encoderTicks = 0;
uint32_t encoderTicksLast = 0;
uint32_t motor_speed = 0;
uint32_t pendulumangle = 0;
uint32_t pendulumspeed = 0;
uint32_t Etheta = 0;
int Ia = 0;
int Ib = 0;
int Ialpha = 0;
int Ibeta = 0;
uint16_t ADC_VAL[4];
int motor_flag = 0;


int prevMainState = 3;
int ADC_done_flag = 0;
int motor_calibrated_flag = 0;
int swing_up_flag = 0;
int balance_flag = 0;
int Vbus = 12;
int Vd = 5;


int Duty = 0;
uint32_t encoder_val = 0;
int calibration_toggle = 0;
int calibration_ticks = 0;
int calibration_length = 286;


int32_t encoder_calibration_queue[motor_calibration_length];

int initial_encoder = 0;
int32_t motor_velocity = 0;
uint32_t previous_ticks = 0;
uint32_t previous_encoder = 0;
int encoder_pos = 0;
int encoder_neg = 0;
int32_t encoder_val_pos = 0;
int32_t encoder_val_neg = 0;
int direction_flag = 0;
int32_t pendulum_angle = 0;
int32_t pendulum_angle_previous = 0;
int32_t pendulum_speed = 0;
int dticks = 0;
uint32_t data_ticks = 0;
int commutation_step = 0;
int commutation_reverse_flag = 0;
int initial_motor_encoder = 0;
int initial_pendulum_encoder = 0;
int16_t calibration_error_array[motor_calibration_length];
int32_t motor_calibration_error = 0;

int UART_TX_Flag = 0;

uint16_t V[3];

int Vaprint = 0;
int Vbprint = 0;
int Vcprint = 0;

int controls_flag = 0;
static const int manual_control = 0;
static const int tweaked_control = 1;
int read_flag = 0;


int interface_state = 0;
int control_state = 0;
int prev_control_state = 4;
int prev_interface_state = 4;
int on_flag = 0;
int off_flag = 0;
int model_calibration_flag = 0;

int32_t pendulum_integral_error = 0;

int32_t pendulum_speed_queue[state_record_length];
int32_t motor_speed_queue[state_record_length];
uint32_t motor_angle_queue[state_record_length];
int pendulum_angle_queue[state_record_length];
uint32_t state_ticks[state_record_length];
int state_queue_index = 0;

int interface_ticks = 0;

static const int S0_INIT = 0; /**< register Address of XXXXXXXX position*/
static const int S1_IDLE = 1; /**< register Address of XXXXXXXX position*/
static const int S2_CALIBRATE_MOTOR = 2; /**< register Address of XXXXXXXX position*/
static const int S3_CONTROL = 3; /**< register Address of XXXXXXXX position*/
static const int S4_CALIBRATE_MODEL = 4; /**< register Address of XXXXXXXX position*/


int commutation_speed = 2000; //Hz
int read_state_speed = 200; //Hz
const int16_t chirp_lookup_table[] = {
0, 6, 13, 19, 25, 31, 37, 43,
48, 54, 59, 64, 68, 73, 77, 81,
84, 88, 90, 93, 95, 97, 98, 99,
100, 100, 100, 99, 98, 97, 95, 93,
90, 88, 84, 81, 77, 73, 68, 64,
59, 54, 48, 43, 37, 31, 25, 19,
13, 6, 0, -6, -13, -19, -25, -31,
-37, -43, -48, -54, -59, -64, -68, -73,
-77, -81, -84, -88, -90, -93, -95, -97,
-98, -99, -100, -100, -100, -99, -98, -97,
-95, -93, -90, -88, -84, -81, -77, -73,
-68, -64, -59, -54, -48, -43, -37, -31,
-25, -19, -13, -6};

int chirp_counter = 0;
int chirp_tick = 0;
int chirp_period = 0;
static const int max_chirp_period = 2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  Ialpha = 2*Ia/3-(Ib-Ic);
//  Ibeta = (2/sqrt(3))*(Ib-Ic);
//  Id = Ialpha*cos(theta)+Ibeta*sin(theta);
//  Iq = Ibeta*cos(theta)-Ialpha*sin(theta);
  HAL_UART_Receive_IT(&huart2, echo_buf, 1); // Setup first receive interrupts



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//---------TASK1_CONTROL---------------//

/*
 * STATE 0: INIT INITIALIZES BOTH THE MOTOR AND MOTOR ENCODER TIMERS TO THE PROPER SETTINGS
 */
	  	  if (control_state == S0_INIT) {

	  		if (prev_control_state!= control_state) {
	  			uart_print("control_state:init/r/n");
	  			prev_control_state = control_state;
	  		}
	  		motor_initialize(&bldcmotor);
	  		initialize_motor_encoder(&Motor_encoder);
	  		initialize_pendulum_encoder(&pendulum_encoder);
	  		control_state = S1_IDLE;
	  		TIM3->ARR = 3999;
	  		TIM4->ARR = (TIM3->ARR+1)*5-1;
	  		HAL_TIM_Base_Start_IT(&htim4);
	  		HAL_TIM_Base_Start_IT(&htim3);
	  	  }
/*
 * STATE 1: IDLE WAITS FOR THE INTERFACE TO SEND THE GO SIGNAL BEFORE STARTING MEASUREMENTS AND MOVING TO CALIBRATION
 */
	  	  else if (control_state == S1_IDLE) {

	  		if (prev_control_state!= control_state) {
				uart_print("control_state:idle/r/n");
				prev_control_state = control_state;
			}

	  		//IF THE READ FLAG IS ON, READ THE STATE FOR UI SENDING PRUPOSES
	  		if (read_flag == 1){
	  		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	  		//read_state();
	  		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	  		read_flag = 0;
	  		}

	  		//IF THE CONTROLS FLAG IS ON, GO TO CONTROL, SET THE OFFSET TO 90, ENABLE THE MOTOR
	  		if (controls_flag){
	  			control_state = S3_CONTROL;
				bldcmotor.encoder_offset = 73;
				motor_enable(&bldcmotor);
	  		}

	  		//IF THE MOTOR CALIBRATION FLAG IS ON, GO TO CALIBRATE MOTOR STATE, ENABLE MOTOR, RESET ALL CALIBRATION VARIABLES, START TICKS FOR CALIBRATION,
	  		else if (motor_calibrated_flag){
	  			control_state = S2_CALIBRATE_MOTOR;
				motor_enable(&bldcmotor);
				bldcmotor.effort = -100;
				//HAL_ADC_Start_DMA(&hadc1, ADC_VAL, 4);
				start_ticks = HAL_GetTick();
				current_ticks = HAL_GetTick();
				commutation_step = 0;
				off_flag = 0;
				bldcmotor.encoder_offset = 0;
				bldcmotor.encoder_zero = 0;
				motor_calibration_error = 0;
				HAL_TIM_Base_Stop_IT(&htim3);
	  		}

	  		//IF THE MODEL CALIBRATION FLAG IS ON, GO TO MODEL CALIBRATION STATE AND RESET THE FLAG
	  		else if (model_calibration_flag){
	  			control_state = S4_CALIBRATE_MODEL;
	  			bldcmotor.encoder_offset = 73;
	  			model_calibration_flag = 0;
	  			motor_enable(&bldcmotor);

	  		}
	  	  }

/*
 * STATE 2: CALIBRATION CALIBRATES THE MOTOR OFFSET
 */
	  	  else if (control_state == S2_CALIBRATE_MOTOR) {

			if (prev_control_state!= control_state) {
				uart_print("control_state:calibrate/r/n");
				prev_control_state = control_state;
			}

			/*
			 * Every 10 milliseconds, the motor commutates 1 tick forward - offset is set to 0 so there is no
			 *
			 */
			if ((motor_calibrated_flag == 1) && ((HAL_GetTick()-current_ticks)>10)){
				current_ticks = HAL_GetTick();
				motor_commutate(&bldcmotor, commutation_step);
				if (commutation_step == motor_calibration_length/2){
					commutation_reverse_flag = 1;
				}

				if (commutation_reverse_flag == 1){
					calibration_error_array[motor_calibration_length-1-commutation_step] = read_motor_encoder(&Motor_encoder)-commutation_step;
					commutation_step--;
					if (commutation_step == -1){
						motor_calibrated_flag = 2;
						for (int i = 0; i < motor_calibration_length; ++i){
							motor_calibration_error += calibration_error_array[i];
						}
						motor_calibration_error = motor_calibration_error/motor_calibration_length;
						bldcmotor.encoder_zero = motor_calibration_error;
						commutation_reverse_flag = 0;
						off_flag = 1;

					}

				}
				else {
				calibration_error_array[commutation_step] = read_motor_encoder(&Motor_encoder)-commutation_step;
				commutation_step++;

				}

			}
			if (off_flag == 1){
				control_state = S1_IDLE;
				motor_disable(&bldcmotor);
				bldcmotor.effort = 0;
				off_flag = 0;
				motor_calibrated_flag = 0;
				HAL_TIM_Base_Start_IT(&htim3);
			}

		  }


	  	  else if (control_state == S3_CONTROL) {

			if (prev_control_state!= control_state) {
				uart_print("control_state:on/r/n");
				prev_control_state = control_state;
			}
			if (read_flag == 1){
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				//read_state();
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				read_flag = 0;
			}
			encoder_val= motor_angle_queue[state_record_length-1];
			pendulum_angle= pendulum_angle_queue[state_record_length-1];
//			pendulum_speed= pendulum_speed_queue[4];
//			motor_velocity= pendulum_speed_queue[4];
			pendulum_speed= 0;
			motor_velocity= 0;
			for (int i=0; i<state_record_length-1; ++i){
				pendulum_speed+= pendulum_speed_queue[i]/state_record_length;
				motor_velocity+= motor_speed_queue[i]/state_record_length;
			}

			if (controls_flag == manual_control){
				if (direction_flag){
					bldcmotor.effort = 60;
				}

				else {
					bldcmotor.effort = -60;
				}
			}

			else if (controls_flag == tweaked_control){
				if (swing_up_flag == 1){
				//swing_up

					if ((1848<=pendulum_angle) && (pendulum_angle<= 2248)){
						len = sprintf(&buf, "Pendulum Swung Up\r\n");
						HAL_UART_Transmit_IT(&huart2, &buf, len);
						swing_up_flag = 0;
						balance_flag = 1;
						pendulum_integral_error = 0;
					}

				else {

					if ((pendulum_speed<0) && (pendulum_angle>3072 || pendulum_angle<1024)){

						bldcmotor.effort = -100;//(abs(pendulum_angle-1024)%2048)*100/1024;
					}

					else if ((pendulum_speed>0) && (pendulum_angle>3072 || pendulum_angle<1024)){
						bldcmotor.effort = 100;//(abs(pendulum_angle-1024)%2048)*100/1024;
					}

					else if ((pendulum_speed>0) && (pendulum_angle<3072 && pendulum_angle>1024)){
						bldcmotor.effort = -((pendulum_angle-2048)%1024)*40/1024;
					}

					else if ((pendulum_speed<0) && (pendulum_angle<3072 && pendulum_angle>1024)){
						bldcmotor.effort = ((pendulum_angle-2048)%1024)*40/1024;
					}

				}
			}
			if (balance_flag == 1){
				//balance
				//pendulum_integral_error+=(pendulum_angle-2048+30)%1024;
				bldcmotor.effort = ((pendulum_angle-2017)%1024)*2500/4096-(pendulum_speed*250/25000)+motor_velocity*50/5000;

				if ((pendulum_angle<= 1848) || (pendulum_angle>= 2248)){
					balance_flag = 0;
					swing_up_flag = 1;
				}

			}

			//clamp effort


			}



			if (off_flag == 1){
				control_state = S1_IDLE;
				motor_disable(&bldcmotor);
				bldcmotor.effort =  0;
				off_flag = 0;
				controls_flag = 0;
				//HAL_TIM_Base_Stop_IT(&htim3);
			}
			//motor.encoder_offset++;
			encoder_val = read_motor_encoder(&Motor_encoder)%Motor_encoder.ppr;
			//motor_commutate(&bldcmotor, encoder_val);
			V[0] = bldcmotor.Va;
			V[1] = bldcmotor.Vb;
			V[2] = bldcmotor.Vc;

		  }

	  	else if (control_state == S4_CALIBRATE_MODEL) {

			if (prev_control_state!= control_state) {
				uart_print("control_state:on/r/n");
				prev_control_state = control_state;
			}

			if ((HAL_GetTick() - chirp_tick)>chirp_period){
				chirp_tick = HAL_GetTick();
				bldcmotor.effort = chirp_lookup_table[chirp_counter%(sizeof(chirp_lookup_table)/sizeof(chirp_lookup_table[0]))];
				chirp_counter++;
				if (chirp_counter == 50*sizeof(chirp_lookup_table)/sizeof(chirp_lookup_table[0])){
					chirp_counter = 0;
					chirp_period++;
					if (chirp_period == max_chirp_period+1){
						off_flag = 1;
						chirp_period = 0;
						chirp_tick = 0;
						chirp_counter= 0;
					}
				}
			}

			if (off_flag == 1){
				control_state = S1_IDLE;
				motor_disable(&bldcmotor);
				bldcmotor.effort =  0;
				off_flag = 0;
			}



	  	}






//---------TASK2_INTERFACE---------------//

		if (interface_state == S0_INIT) {

			if (prev_interface_state!= interface_state) {
				uart_print("interface_state:init/r/n");
				prev_interface_state= interface_state;

			}


			interface_state = S1_IDLE;


		  }
		else if (interface_state == S1_IDLE){

			if (prev_interface_state!= interface_state) {
				uart_print("interface_state:idle/r/n");
				prev_interface_state= interface_state;
			}
			encoder_val= motor_angle_queue[4];
			pendulum_angle= pendulum_angle_queue[4];
			pendulum_speed = pendulum_speed_queue[4];
			motor_velocity = motor_speed_queue[4];

//			for (int i=0; i<4; ++i){
//				pendulum_speed+= pendulum_speed_queue[i]/5;
//				motor_velocity+= motor_speed_queue[i]/5;
//			}

			if (((HAL_GetTick()-interface_ticks)>5) && UART_TX_Flag){

				interface_ticks = HAL_GetTick();
				UART_TX_Flag = 0;
				send_data(encoder_val, pendulum_angle,motor_velocity,pendulum_speed, bldcmotor.effort);
			}

//			send_data();


		}



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (echo_buf[0] == 'a'){
		on_flag = 1;
	}

	if (echo_buf[0] == 's'){
		off_flag = 1;
	}

	if (echo_buf[0] == 'j'){
		direction_flag = 1;
		controls_flag = manual_control;
	}

	if (echo_buf[0] == 'k'){
		direction_flag = 0;
		controls_flag = manual_control;
	}

	if (echo_buf[0] == 'c'){
		model_calibration_flag = 1;
	}

	if (echo_buf[0] == 'm'){
		motor_calibrated_flag = 1;

	}

	if (echo_buf[0] == 't'){
		controls_flag = tweaked_control;
		swing_up_flag = 1;
	}

	HAL_UART_Receive_IT(&huart2, echo_buf, 1); // setup new callback

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	UART_TX_Flag = 1;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if (htim == &htim3){
		motor_commutate(&bldcmotor, read_motor_encoder(&Motor_encoder)%Motor_encoder.ppr);
	}
	if (htim == &htim4){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		read_pendulum_encoder(&pendulum_encoder);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);


	}
}

int32_t calculate_velocity(uint32_t previous_encoder, uint32_t current_encoder, uint32_t dticks, int ppr){
	int32_t diff = (int32_t) (current_encoder) - (int32_t) (previous_encoder);
	if (dticks == 0) {
		dticks = 1;
	}

	if (diff>(ppr)/2){
		return (diff-ppr)/((int32_t) (dticks));
	}

	else if (diff<(-(ppr)/2)){
		return (diff+(ppr))/((int32_t) (dticks));
	}

	else {
		return diff/((int32_t) (dticks));
	}
}
void read_state(){

	if (state_queue_index){
		state_ticks[state_queue_index] = state_ticks[state_queue_index-1]+1;
	}
	else{
		state_ticks[state_queue_index] = 0;
	}

	if (state_queue_index<state_record_length-1){
		state_queue_index++;
	}
	else {
		int i;
		for (i = 1; i < state_record_length; ++i){
			pendulum_angle_queue[i-1] = pendulum_angle_queue[i];
			motor_angle_queue[i-1] = motor_angle_queue[i];
			state_ticks[i-1] = state_ticks[i];
			pendulum_speed_queue[i-1] = pendulum_speed_queue[i];
			motor_speed_queue[i-1] = motor_speed_queue[i];
		}
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		pendulum_angle_queue[state_queue_index] = pendulum_encoder.encoder_angle;
		motor_angle_queue[state_queue_index] = read_motor_encoder(&Motor_encoder)%Motor_encoder.ppr;
		motor_speed_queue[state_queue_index] = calculate_velocity(motor_angle_queue[state_record_length-2], motor_angle_queue[state_record_length-1], state_ticks[state_record_length-2]-state_ticks[state_record_length-1], Motor_encoder.ppr)*400;
		pendulum_speed_queue[state_queue_index] = calculate_velocity(pendulum_angle_queue[state_record_length-2], pendulum_angle_queue[state_record_length-1], state_ticks[state_record_length-2]-state_ticks[state_record_length-1], 4096)*400;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	ADC_done_flag = 1;

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c){
	set_encoder_angle(&pendulum_encoder);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	read_state();
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);



}

void uart_print( char *msg){
	len = sprintf(buf, msg);
	HAL_UART_Transmit_IT(&huart2, buf, len);
}
//void printmsg(UART_HandleTypeDef *huart, char message){
//	len = sprintf(&buf, "Message");
//	HAL_UART_Transmit(&huart2, &buf, len, 100);
//}
void send_data(int motor_angle, int pendulum_angle,  int motor_speed, int pendulum_speed, int motor_effort)
{
	len = sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n", pendulum_angle, motor_angle, pendulum_speed, motor_speed, motor_effort, HAL_GetTick());
	HAL_UART_Transmit_IT(&huart2, buf, len);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
