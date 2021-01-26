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
#define EPS  (float) 0.08f		
#define P1   (float) -0.000067f
#define P2   (float) -0.000287f
#define P3   (float) 0.871698f
#define KP   (float) 0.9f//0.7f //1.0f //0.751
#define KI   (float) -0.035f//-0.2f //-0.8f
#define KD   (float) -0.07f//-0.11f//-0.125f
#define	TD   (float) 0.1f//0.1f
#define TP   (float) 0.014f
#define TRES (float) 0.0f
#define TF   (float) 0.14f 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t received_data1;
uint8_t received_data[4];

int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
float ax, ay, az, gx, gy, gz, temperature, roll, pitch;
float q0_mah, q1_mah, q2_mah, q3_mah;
float q0_mad, q1_mad, q2_mad, q3_mad;
uint8_t buffer[128];
static uint8_t send_data[40];
uint16_t size = 0;

float pitch_comp, roll_comp; //values from complementary filter 
float pitchGyro, pitchAccel;

uint16_t counter = 0;

/*  *************PID**************  */ 
//PID variables:
float sphereRoll_SetPoint = 0.0;
float roll_error = 0.0;
float prev_roll_error = 0;
float prev_prev_roll_error = 0;
float control = 0.0;
float prev_control = 0.0;
float prev_prev_control = 0.0;

 /* Choose a controller type
		0 - no controller
		1 - PID                      */ 
uint8_t controller_type = 0;
float wspE  = (KP*TD+KD);
float wspE1 = (KI*TP*TD + KP*(TP-2*TD) - 2*KD);
float wspE2 = (TP-TD)*(KI*TP - KP) + KD;
float wspU1 = -(TP-2*TD);
float wspU2 = -(TD-TP);

/* *************END PID**************  */ 


/* Mode 2 */
//float proportional = 0;
//float integrator = 0;
//float differentiator = 0;
//float Kp = -0.7; //2.0;
//float Ki = 0.1;
//float Kd = 0.35; //0.0; 
//float T = 0.1; //0.01428;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6){
		do tim6 stuff
	}
	if (htim->Instance == TIM7){
		do tim7 stuff
	}
}
*/

//Timer interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if(htim->Instance == TIM10){ //70Hz Interrupt for sending data to PC
		
		//MPU6050_GetAccelerometerScaled(&ax, &ay, &az);
	  //MPU6050_GetGyroscopeScaled(&gx, &gy, &gz);
		
		//ComplementaryFilter(&roll_comp, &pitch_comp);//, &pitchGyro, &pitchAccel);
		//MPU6050_GetRollPitch(&roll, &pitch);
		
		//Outputformat for debugging:
		//size = sprintf(send_data, "%.2f,%.2f,%d \n", control, pitch_comp, TIM2->CCR2);
		
		//output format for GUI:
		size = sprintf(send_data, "%.2f,%.2f,%d,%d \n", roll_comp, pitch_comp, TIM2->CCR2, TIM3->CCR1);
		HAL_UART_Transmit_DMA(&huart1, send_data, size);

	}
	
	if(htim->Instance == TIM13) { // 200Hz interrupt for PID
		
		ComplementaryFilter(&roll_comp, &pitch_comp);
		//MPU6050_GetRollPitch(&roll, &pitch);
		
		//Calculate error between set and actual roll angle value
		// sphereRoll_SetPoint = 0.0
		// pitch_comp e[-45,45] deg
		roll_error = sphereRoll_SetPoint - pitch_comp; //pitch_comp is actually roll_comp
		
		//Calculate control
		control = (roll_error*wspE + prev_roll_error*wspE1 + prev_prev_roll_error*wspE2 + prev_control*wspU1 + prev_prev_control*wspU2)/TD;
		
		//proportional = Kp * roll_error;
		//integrator = integrator + 0.5f * Ki * T * (roll_error + prev_roll_error);
		
		//control = proportional + integrator; // + differentiator;
		
		if(control>98)
			 control = 98;
		else if(control<-98)
			 control = -98;
		
		if(controller_type == 0) { //if controller_type is a PID
			TIM2->CCR2 = (int)(57 + (30 * control/100));
		}
		 
//		if(control > 0){   //turn [R]ight
//			TIM2->CCR2 = (int)(57 + (30 * control/100));
//		}
//		else if(control < 0) {    //turn [L]eft
//			TIM2->CCR2 = (int)(57 + (30 * control/100));
//		} else {
//			TIM2->CCR2 = 57;
//		}
		
		prev_prev_control = prev_control;
		prev_control = control;
		prev_prev_roll_error = prev_roll_error;
		prev_roll_error = roll_error;
		
		//size = sprintf(send_data, "%.2f,%d \n", pitch_comp, TIM2->CCR2);
		//size = sprintf(send_data, "%.2f,%.2f,%d,%d \n", roll, pitch_comp, TIM2->CCR2, counter);
		//HAL_UART_Transmit_DMA(&huart1, send_data, size);
	}
	
}

//UART Interrupt - get data from UART (Bluetooth module). 4 digits, speed and turn control. 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	/*
	//(OLD SERVO: TIM4->CCR3 = 74; 		// Servo bez blokady: 40-0.8ms, 50-1.0ms, 80-1.6ms, 100-2.0ms, 110-2.2ms  74- STOP)
	//TIM4->CCR3 = (int)(74 - (34 * speed/100)); //zadajemy predkosc proporcjonalna do zadanej wahadla bez blokady
	//size = sprintf(send_data, "SF%d, TIM4: %d, Roll: %.2f\n", speed, TIM4->CCR3, pitch_comp);
	//TIM4->CCR3 = (int)(74 + (34 * speed/100)); //zadajemy predkosc proporcjonalna do zadanej wahadla bez blokady
	//size = sprintf(send_data, "SR%d, TIM4: %d, Roll: %.2f\n", speed, TIM4->CCR3, pitch_comp);
	
	//NEW SERVO:  TIM4->CCR3 = 300;       //360 servo: 90-MAX SPEED CCW, 300-STOP, 530-MAX SPEED CW
	//TURN SERVO: TIM2->CCR2 = 56; // Servo z blokada: 20-0.4ms, 130-2.6ms  32- lewe skrajne, 56-srodek, 80-prawe skrajne
	
	//uint8_t send_data[50];
	//uint16_t size = 0;
	*/
	
	//uint8_t controller = atoi(&received_data[0]);
	uint8_t servo = received_data[0]; // S - bez blokady, T - z blokada
	uint8_t direction = received_data[1]; //Left, Right, albo Forward, Reverse
	int speed = atoi(&received_data[2]); // will read till the end of message? // + atoi(&received_data[3]); //  0-99% ???

	//controller_type = controller;
	
	
	if(servo == 'S' || servo == 's') { //jezeli chcemy sterowac predkoscia [S]peed
		if( direction == 'F' || direction == 'f'){    //jazda do przodu [F]orward
			TIM3->CCR1 = (int)(300 + (210 * speed/100)); 
		}
		else if(direction == 'R' || direction == 'r') {      //jazda do tylu [R]everse
			TIM3->CCR1 = (int)(300 - (210 * speed/100)); 	
		}
		else{		// Other unforseen messages
			//size = sprintf(send_data, "Odebrana wiadomosc zawiera nieznane znaki!\n");
		}
	}
	else if(servo == 'T' || servo == 't'){        // jezeli chcemy skrecac [T]urn
		if(direction == 'R' || direction == 'r'){   // skrecamy w prawo [R]ight
			if(controller_type == 0){
			TIM2->CCR2 = (int)(57 - (30 * speed/100));
			//size = sprintf(send_data, "TR%d, TIM2: %d, Roll: %.2f\n", speed, TIM2->CCR2, pitch_comp);
			}
		}
		else if(direction == 'L' || direction == 'l') {			//skrecamy w lewo [L]eft
			if(controller_type == 0){
			TIM2->CCR2 = (int)(57 + (30 * speed/100));
			//size = sprintf(send_data, "TL%d, TIM2: %d, Roll: %.2f\n", speed, TIM2->CCR2, pitch_comp);
			}
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
	
  
  HAL_UART_Receive_DMA(&huart1, received_data, 4); //Enable listening
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15); //LED informing that UART collected data
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_UART_Receive_DMA(&huart1, received_data, 4); //odbieramy 4 znaki z uart1 i zapisujemy do received_data

	//TIM4->CCR3 = 74; 		// Servo bez blokady: 40-0.8ms, 50-1.0ms, 80-1.6ms, 100-2.0ms, 110-2.2ms  74- STOP
	//TIM4->CCR3 = 300;
	TIM3->CCR1 = 300;
	TIM2->CCR2 = 57; // Servo z blokada: 20-0.4ms, 130-2.6ms  32[0.6ms]- lewe skrajne, 56[1.12ms]-srodek, 80[1.6ms]-prawe skrajne
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

// IMU Functions:

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	
	MPU6050_Init(&hi2c1);
	
	//PID
	//wspE  = (KP*TD+KD);
	//wspE1 = (KI*TP*TD + KP*(TP-2*TD) - 2*KD);
	//wspE2 = (TP-TD)*(KI*TP - KP) + KD;
	//wspU1 = -(TP-2*TD);
	//wspU2 = -(TD-TP);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 419;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 419;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 239;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 9999;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 83;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
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
