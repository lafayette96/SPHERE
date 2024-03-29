/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * Source code of spherical robot 'SPHERE' constructed by Adrian Luberda on 
	* AGH University of Science in Technology as part of Engineering and 
	* Master Thesis in years: 2018 - 2020.
	*
	* This is the final version of the code.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <string.h>
#include "gpio.h"
#include "mpu6050.h"
#include <math.h>
//#include "stdlib.h"
//#include "stdint.h" // makes uint8_t available


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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t received_data1;
uint8_t received_data[4];

int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
float ax, ay, az, gx, gy, gz, temperature, roll, pitch;
float q0_mah, q1_mah, q2_mah, q3_mah;
float q0_mad, q1_mad, q2_mad, q3_mad;
uint8_t buffer[128];
uint8_t send_data[50];
uint16_t size = 0;

float pitch_comp, roll_comp; //values from complementary filter 
float pitchGyro, pitchAccel;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

//PRZERWANIE - odbi�r danych z uarta (wiele znak�w, sterowanie predkoscia i kierunkiem)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//uint8_t send_data[50];
	//uint16_t size = 0;
	
	uint8_t servo = received_data[0]; // S - bez blokady, T - z blokada
	uint8_t direction = received_data[1]; //Left, Right, albo Forward, Reverse
	int speed = atoi(&received_data[2]);// + atoi(&received_data[3]); //  0-99% ???
	
	//TIM4->CCR3 = 74; 		// Servo bez blokady: 40-0.8ms, 50-1.0ms, 80-1.6ms, 100-2.0ms, 110-2.2ms  74- STOP
	//TIM2->CCR2 = 56; // Servo z blokada: 20-0.4ms, 130-2.6ms  32- lewe skrajne, 56-srodek, 80-prawe skrajne
	
	if(servo == 'S' || servo == 's') { //jezeli chcemy sterowac predkoscia [S]peed
		if( direction == 'F' || direction == 'f'){    //jazda do przodu [F]orward
			TIM4->CCR3 = (int)(74 - (34 * speed/100)); //zadajemy predkosc proporcjonalna do zadanej wahadla bez blokady
			//size = sprintf(send_data, "SF%d, TIM4: %d, Roll: %.2f\n", speed, TIM4->CCR3, pitch_comp);
		}
		else if(direction == 'R' || direction == 'r') {      //jazda do tylu [R]everse
			TIM4->CCR3 = (int)(74 + (34 * speed/100)); //zadajemy predkosc proporcjonalna do zadanej wahadla bez blokady
			//size = sprintf(send_data, "SR%d, TIM4: %d, Roll: %.2f\n", speed, TIM4->CCR3, pitch_comp);
		}
		else{		// znaki inne niz przewidziane
			//size = sprintf(send_data, "Odebrana wiadomosc zawiera nieznane znaki!\n");
		}
	}
	else if(servo == 'T' || servo == 't'){        // jezeli chcemy skrecac [T]urn
		if(direction == 'R' || direction == 'r'){   // skrecamy w prawo [R]ight
			TIM2->CCR2 = (int)(56 - (30 * speed/100));
			//size = sprintf(send_data, "TR%d, TIM2: %d, Roll: %.2f\n", speed, TIM2->CCR2, pitch_comp);
		}
		else if(direction == 'L' || direction == 'l') {    //skrecamy w lewo [L]eft
			TIM2->CCR2 = (int)(56 + (30 * speed/100));
			//size = sprintf(send_data, "TL%d, TIM2: %d, Roll: %.2f\n", speed, TIM2->CCR2, pitch_comp);
		}
		else{   // znaki inne niz przewidziane
			//size = sprintf(send_data, "Odebrana wiadomosc zawiera nieznane znaki!\n");
		}
	}
	else {  // znaki inne niz przewidziane
		//size = sprintf(send_data, "Odebrana wiadomosc zawiera nieznane znaki!\n");
	}
	
	
	//size = sprintf(send_data, "Roll: %f.2f  Pitch: %f.2f \n", roll, pitch);
	
	//size = sprintf(send_data, "Odebrana wiadomosc: '%s'\n\r", received_data);
	//wyslanie przygotowanej wiadomosci
	//HAL_UART_Transmit_IT(&huart1, send_data, size);
  //Ponowne wlaczenie nasluchiwania
  HAL_UART_Receive_IT(&huart1, received_data, 4);
   //Dioda informujaca o tym ze UART odebral dane
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}


//funkcja wysylajaca dane do komputera z czestotliwoscia 1Hz (przerwanie od timera)
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { 
//	uint8_t data[50];// Tablica przechowujaca wysylana wiadomosc.
//	uint16_t size = 0; // Rozmiar wysylanej wiadomosci 
////static uint16_t cnt = 0; // Licznik wyslanych wiadomosci
//	
//	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Zmiana stanu pinu na diodzie LED (pomaranczowej)
//	size = sprintf(send_data, "%.2f,%.2f,%.2f \n", roll_comp, pitchGyro, pitchAccel);
//	//size = sprintf(data, "Roll: %f  Pitch: %f \n", roll, pitch);  // Wiadomosc do wyslania i przypisanie ilosci wysylanych znakow do zmiennej size.
////	
//	HAL_UART_Transmit_IT(&huart1, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan 
////	HAL_UART_Receive_IT(&huart1, received_data, 4);
////	//++cnt; // Zwiekszenie licznika wyslanych wiadomosci.
//} 


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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_UART_Receive_IT(&huart1, received_data, 4); //odbieramy 4 znaki z uart1 i zapisujemy do received_data
	TIM4->CCR3 = 74;
	//TIM4->CCR3 = 74; 		// Servo bez blokady: 40-0.8ms, 50-1.0ms, 80-1.6ms, 100-2.0ms, 110-2.2ms  74- STOP
	TIM2->CCR2 = 56; // Servo z blokada: 20-0.4ms, 130-2.6ms  32[0.6ms]- lewe skrajne, 56[1.12ms]-srodek, 80[1.6ms]-prawe skrajne
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

// IMU Functions:
	MPU6050_Init(&hi2c1);

  MPU6050_SetInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
  MPU6050_SetInterruptDrive(MPU6050_INTDRV_PUSHPULL);
  MPU6050_SetInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
  MPU6050_SetInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);

  MPU6050_SetIntEnableRegister(0); // Disable all interrupts

  // Enable Motion interrputs, possibly unnecessary
  MPU6050_SetDHPFMode(MPU6050_DHPF_5);

  MPU6050_SetIntMotionEnabled(1);
  MPU6050_SetIntZeroMotionEnabled(1);
  MPU6050_SetIntFreeFallEnabled(1);

  MPU6050_SetFreeFallDetectionDuration(2);
  MPU6050_SetFreeFallDetectionThreshold(5);

  MPU6050_SetMotionDetectionDuration(5);
  MPU6050_SetMotionDetectionThreshold(2);

  MPU6050_SetZeroMotionDetectionDuration(2);
  MPU6050_SetZeroMotionDetectionThreshold(4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
  /*
		HAL_Delay(1000);
		TIM2->CCR2 = 32;
		HAL_Delay(1000);
		TIM2->CCR2 = 56;
		HAL_Delay(1000);
		TIM2->CCR2 = 80;
		HAL_Delay(1000);
		TIM2->CCR2 = 80;
		HAL_Delay(1000);
		TIM2->CCR2 = 100;
		HAL_Delay(1000);
		TIM2->CCR2 = 110;
		HAL_Delay(1000);
		TIM2->CCR2 = 130;*/
		
		MPU6050_GetAccelerometerScaled(&ax, &ay, &az);
	  MPU6050_GetGyroscopeScaled(&gx, &gy, &gz);
		MPU6050_GetRollPitch(&roll, &pitch);
		
		ComplementaryFilter(&roll_comp, &pitch_comp);//, &pitchGyro, &pitchAccel);
		
		//size = sprintf(send_data, "Roll: %.2f  Pitch: %.2f \n", roll, pitch);
		
		
		size = sprintf(send_data, "%.2f,%.2f \n", roll, pitch_comp);
		
		//size = sprintf(send_data, "%.2f,%.2f,%.2f\n", roll_comp, pitchGyro, pitchAccel);
		//size = sprintf(send_data, "%.2f,%.2f,%.2f,%.2f\n", roll, pitch, roll_comp, pitch_comp); //roll_comp is actually pitch_comp and vice versa
		//size = sprintf(send_data, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", ax, ay, az, gx, gy, gz, roll_comp, pitch_comp);
		//size = sprintf(send_data, "%.2f,%.2f,%.2f\n", gx, ay, az);
		//size = sprintf(send_data, "%.2f,%.2f,%.2f\n", ay, az, gx);
		HAL_UART_Transmit_IT(&huart1, send_data, size);

		HAL_Delay(10); //can go down to 18-20
	
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1679;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1679;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 8399;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : MEMS_CS_Pin */
  GPIO_InitStruct.Pin = MEMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MEMS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
