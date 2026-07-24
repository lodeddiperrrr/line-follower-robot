/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
	WAIT,
	RUN,
	ERR,
} State;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define ADC_COUNT 7U
#define ERROR_COUNT 10
#define Kp .04
#define Ki .01
#define Kd .01
#define MIDDLE_ADC_POS 400U
#define MAX_SPEED 255U
#define BASE_SPEED_L 150U
#define BASE_SPEED_R 150U

State state;
uint16_t adc_vals[ADC_COUNT];
int last_error;
int errors[ERROR_COUNT];
char tx_buffer[20];
volatile uint8_t rx_byte;        // Stores single incoming byte

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void led_state(State state);
int get_adc_pos(void); // read from adc_vals
void pid_controller(void); //
void move_motors(int left_motor, int right_motor);
void stop_motors(void);
void past_errors(int error);
int error_sum(int index);

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

  state = WAIT;
  led_state(state);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// --- ADC
	for(uint8_t i = 0; i < ADC_COUNT; i++)
	{
		HAL_ADC_Start(&hadc1); // Trigger the next Rank in the sequence
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		{
			adc_vals[i] = HAL_ADC_GetValue(&hadc1);
		}
	}
	HAL_ADC_Stop(&hadc1); // Turn off the ADC until the next cycle
	HAL_Delay(1);

	// --- SUPERLOOP

	led_state(state);
	if(state == WAIT){
		if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET ){
			state = RUN;
		}
	} else if (state == RUN) {
		pid_controller();
		// TODO: if receive stop signal -> stop motors go to wait state
	} else {
		// TODO: stop motors
	}

  }


  // --------------------------------------------------
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void led_state(State state){
	switch(state) {
		case WAIT:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case RUN:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case ERR:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}

int get_adc_pos(void) {
    uint16_t pos = 0;                 // Expanded to uint16_t to avoid overflow
    uint8_t active = 0;
    for (int i = 0; i < ADC_COUNT; i++) {
        if (adc_vals[i] < 400) {
            active++;
            pos += (i + 1) * 100;     // Scale position by 100
        }
    }
    if (active == 0) {
        return 0;
    }
    return pos / active;              // E.g., position 3.5 becomes integer 350
}

void pid_controller(void){
	int position = get_adc_pos();
	int error = MIDDLE_ADC_POS - position;
	past_errors(error);

    int P = error*Kp;
    int I = error_sum(5)*Ki;
    int D = (error - last_error)*Kd;

    last_error = error;
    int motor_speed = P + I + D;
    int left_motor_speed = BASE_SPEED_L + motor_speed;
    int right_motor_speed = BASE_SPEED_R - motor_speed;
    if(left_motor_speed > MAX_SPEED){
    	left_motor_speed = MAX_SPEED;
    }
    if(right_motor_speed> MAX_SPEED){
    	right_motor_speed = MAX_SPEED;
    }
    move_motors(left_motor_speed, right_motor_speed);
}
void move_motors(int left_motor, int right_motor){
	// MOTOR A (RIGHT)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, right_motor);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

	// MOTOR B (LEFT)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_motor);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void stop_motors(void){
	// MOTOR A (RIGHT)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_SPEED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	// MOTOR B (LEFT)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MAX_SPEED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void past_errors(int error){
	for(int i = ERROR_COUNT-1; i > 0; i--){
		errors[i] = errors[i - 1];
	}
	errors[0] = error;
}

int error_sum(int index){
	int total_sum = 0;
	for(int i = 0; i < index; i++){
		total_sum += errors[i];
	}
	return total_sum;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {

    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_byte == '1')
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // LED ON
        }
        else if (rx_byte == '0')
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
            state = WAIT;
            stop_motors();
        }
        // Re-arm interrupt for the next byte
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
    }
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
#ifdef USE_FULL_ASSERT
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
