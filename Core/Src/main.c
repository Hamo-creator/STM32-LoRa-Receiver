/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "LoRa.h"
#include "string.h"
#include "SERVO.h"
#include "MATH.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SEND_DATA   0xEE
#define RECV_DATA   0xFF

#define SERVO_MIN_PULSE_WIDTH 500   // Minimum pulse width for servo in microseconds
#define SERVO_MAX_PULSE_WIDTH 2500  // Maximum pulse width for servo in microseconds
#define SERVO_MIN_ANGLE 0           // Minimum angle for servo in degrees
#define SERVO_MAX_ANGLE 180         // Maximum angle for servo in degrees

#define SERVO1  0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

LoRa myLoRa;
uint8_t NewData = 0;
uint8_t RX_BUF[25];
uint8_t TX_BUF[5];
uint8_t RX_CNT = 0;
uint8_t TX_CNT = 0;
uint8_t REC_DATA_SIZE;
uint16_t LoRa_State = 0;
uint16_t Data[10];

uint16_t pulseValue = 0;
uint16_t PWM_res = 0;
float temp = 0.0;
uint16_t min_pulse_width = 0;      //650ms
uint16_t max_pulse_width = 0;     //2000ms

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint16_t writeServo(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t adc_value, uint16_t min_pulse_width, uint16_t max_pulse_width);
	

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to calculate the pulse width for a given angle
uint32_t calc_pulse_width(float angle) {
    float pulse_width = ((angle - SERVO_MIN_ANGLE) * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) + SERVO_MIN_PULSE_WIDTH;
    return (uint32_t)(pulse_width);
}

// Function to set the servo angle
void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle) {
    uint32_t pulse_width = calc_pulse_width(angle);
    //__HAL_TIM_SET_COMPARE(htim, channel, pulse_width);
		htim->Instance->CCR1 = pulse_width;
		HAL_TIM_PWM_Start(htim, channel);
}

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	myLoRa = newLoRa();
	
	myLoRa.CS_port               = NSS_GPIO_Port;
	myLoRa.CS_pin                = NSS_Pin;
	myLoRa.reset_port            = RST_GPIO_Port;
	myLoRa.reset_pin             = RST_Pin;
	myLoRa.DIO0_port             = DIO0_GPIO_Port;
	myLoRa.DIO0_pin              = DIO0_Pin;
	myLoRa.hSPIx                 = &hspi1;
	
	myLoRa.frequency             = 433;
	myLoRa.spredingFactor        = SF_7;
	myLoRa.bandWidth             = BW_125KHz;
	myLoRa.crcRate               = CR_4_5;
	myLoRa.power                 = POWER_20db;
	myLoRa.overCurrentProtection = 100;
	myLoRa.preamble              = 8;
	
	if(LoRa_init(&myLoRa) == LORA_OK)
		LoRa_State = 1;
	else
		LoRa_State = 0;
	
	LoRa_startReceiving(&myLoRa);
	
	SERVO_Init(SERVO1);
	min_pulse_width = SERVO_Get_MinPulse(SERVO1);
	max_pulse_width = SERVO_Get_MaxPulse(SERVO1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(NewData)
		{
			REC_DATA_SIZE = LoRa_receive(&myLoRa, RX_BUF, sizeof(RX_BUF));
			NewData = 0;
			
			if(RX_BUF[22] == SEND_DATA)
			{
				if(LoRa_transmit(&myLoRa, TX_BUF, sizeof(TX_BUF), 500))
				{
					TX_CNT = (TX_CNT + 1) % 100;
					TX_BUF[1] = TX_CNT;
				}
			}
			else if(RX_BUF[22] == RECV_DATA)
			{
				RX_CNT = (RX_CNT + 1) % 100;
				HAL_GPIO_TogglePin(GPIOB, LED_REC_Pin);
			}
		}
		for(int i = 0; i < 10;i += 2){
			Data[i/2] = RX_BUF[i] | RX_BUF[i+1] << 8;
		}
		
		//PWM_res = Data[3];
		//temp = ((max_pulse_width - min_pulse_width) / 4096.0);
		//pulseValue = (PWM_res * temp) + min_pulse_width;
		pulseValue = MAP(Data[0], 0, 4095, min_pulse_width, max_pulse_width);
		SERVO_RawMove(SERVO1, pulseValue);
		HAL_Delay(1);
		//SERVO_MoveTo(SERVO1, pulseValue);
		//SERVO_Sweep(SERVO1);
		//set_servo_angle(&htim1, TIM_CHANNEL_1, Data[3]);
		//pulseValue  = writeServo(&htim1, TIM_CHANNEL_1, Data[3], min_pulse_width, max_pulse_width);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}

void servo_write(int angle)
{
	htim1.Instance->CCR1 = map(0,180,50,250,angle);
}

uint16_t writeServo(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t adc_value, uint16_t min_pulse_width, uint16_t max_pulse_width)
{
	uint16_t pulse_width = 0;
	
	//Calculate the pulse width based on the ADC value
	pulse_width = (((max_pulse_width - min_pulse_width) * adc_value) / 4095) + min_pulse_width;
	
	//Set the pulse width for the specified timer channel
	//__HAL_TIM_SET_COMPARE(htim, ch, pulse_width);
	htim->Instance->CCR1 = pulse_width;
	HAL_TIM_PWM_Start(htim, ch);
	return pulse_width;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
