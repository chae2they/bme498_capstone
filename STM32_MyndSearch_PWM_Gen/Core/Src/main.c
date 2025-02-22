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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// RELEVANT VARIABLES/ARRAYS
//value from usb
uint8_t channel_value[8] = {0};

//duty cycle value
//uint8_t dutyCycle[8] = {0};
//sample values
uint8_t test[8] = {128, 0, 0, 0, 0 , 0, 0, 0};

// Value for Saving Dlow and Dhigh
// 0 and 1 for channel 1 low, high
// 2 and 3 for channel 2, low, high, etc.
uint8_t duty_cycle_limits[16] = {0};

//Update PWM Function - MEANT TO BE USED DURING OPERATION, NOT FOR TUNING
static void updatePWMValues(uint8_t* newDutyCycle){
	 int dutyCycle[8] = {0};
	 dutyCycle[0] = (newDutyCycle[0] * 10000) / 255; //tim2 ch1
	//dutyCycle[1] = (newDutyCycle[1] * (999+1)) / 255; //tim2 ch2

	//dutyCycle[2] = (newDutyCycle[2] * (799+1)) / 255; //tim3 ch1
	//dutyCycle[3] = (newDutyCycle[3] * (799+1)) / 255; //tim3 ch4

	//dutyCycle[4] = (newDutyCycle[4] * (999+1)) / 255; //tim4 ch1
	//dutyCycle[5] = (newDutyCycle[5] * (999+1)) / 255; //tim4 ch2

	//dutyCycle[6] = (newDutyCycle[6] * (799+1)) / 255; //tim8 ch1
	//dutyCycle[7] = (newDutyCycle[7] * (799+1)) / 255; //tim8 ch2

	    TIM2->CCR1 = dutyCycle[0];
	    //TIM2->CCR2 = dutyCycle[1];

	    //TIM3->CCR1 = dutyCycle[2];
	    //TIM3->CCR2 = dutyCycle[3];

	    //TIM4->CCR1 = dutyCycle[4];
	    //TIM4->CCR2 = dutyCycle[5];

	    //TIM8->CCR1 = dutyCycle[6];
	    //TIM8->CCR3 = dutyCycle[7];
}

// Test Rotary Encoder
void RotaryEncoderTest(){
	uint8_t TxBuffer[50] = {'\0'};
	uint8_t TxBufferLen = 50;

	// Start all counter at 0 anyway
	TIM5->CNT = 0;
	while(1)
	{
		// See rotary value can change
		sprintf(TxBuffer, "Encoder Counter Value = %d\n\r", (TIM5->CNT>>2));
		CDC_Transmit_FS(TxBuffer, TxBufferLen);
		HAL_Delay(100);

		if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
		{
			sprintf(TxBuffer, "Button Pressed\n");
			CDC_Transmit_FS(TxBuffer, TxBufferLen);
			break;
		}
	}
}

// Actual Rotary Encoder Usage
void Set_DLow_and_DHigh_For_Channel(uint8_t channel){
	if(channel == 0 || channel >=9)
	{
		return;
	}

	uint8_t index_low = (channel-1)*2;
	uint8_t index_high = index_low + 1;

	// Initialize both duty cycle limits and counter to 0
	duty_cycle_limits[index_low] = 0;
	duty_cycle_limits[index_high] = 0;
	TIM5->CNT = 0;

	TurnOffAllChannelLED();

	// First lock into tuning lower index.
	// PWM for JUST THIS CHANNEL should be running, with the duty cycle being set here.
	while(1){
		duty_cycle_limits[index_low] = (TIM5->CNT)>>2;
		int low_duty_cycle = (duty_cycle_limits[index_low] * 10000) / 255; //tim2 ch1
		SetLevelLED(duty_cycle_limits[index_low]);

		switch(channel){
			case 1:
				HAL_GPIO_WritePin(GPIOE, Ch1LowLED_Pin, GPIO_PIN_SET);
				TIM2->CCR1 = low_duty_cycle;	// Set duty cycle so that we can tune as we see it
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOC, Ch2LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 2
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOF, Ch3LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 3
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOG, Ch4LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 4
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOG, Ch5LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 5
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOD, Ch6LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 6
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOF, Ch7LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 7
				break;
			case 8:
				HAL_GPIO_WritePin(GPIOF, Ch8LowLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 8
				break;
			default:
		}

		// Push certain button to move onto tuning high
		if(!HAL_GPIO_ReadPin(GPIOA, SetupButton_Pin)) {
			break;
		}
	}

	TurnOffAllChannelLED();

	// Now lock into tuning high index.
	// PWM should still be running just for THIS CHANNEL
	while(1){
		duty_cycle_limits[index_high] = (TIM5->CNT)>>2;
		int high_duty_cycle = (duty_cycle_limits[index_high] * 10000) / 255; //tim2 ch1
		SetLevelLED(duty_cycle_limits[index_high]);

		switch(channel){
			case 1:
				HAL_GPIO_WritePin(GPIOE, Ch1HighLED_Pin, GPIO_PIN_SET);
				TIM2->CCR1 = high_duty_cycle;	// Set duty cycle so that we can tune as we see it
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOC, Ch2HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 2
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOF, Ch3HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 3
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOG, Ch4HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 4
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOG, Ch5HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 5
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOD, Ch6HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 6
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOF, Ch7HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 7
				break;
			case 8:
				HAL_GPIO_WritePin(GPIOG, Ch8HighLED_Pin, GPIO_PIN_SET);
				// FOR CHANNEL 8
				break;
			default:
		}

		// Push certain button to move onto tuning high
		if(!HAL_GPIO_ReadPin(GPIOA, SetupButton_Pin)) {
			break;
		}
	}
	TurnOffAllChannelLED();
}

void TurnOffAllChannelLED(){
	HAL_GPIO_WritePin(GPIOE, Ch1LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, Ch1HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOC, Ch2LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Ch2HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOF, Ch3LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, Ch3HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOG, Ch4LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, Ch4HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOG, Ch5LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, Ch5HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOD, Ch6LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, Ch6HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOF, Ch7LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, Ch7HighLED_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOF, Ch8LowLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, Ch8HighLED_Pin, GPIO_PIN_RESET);
}

void TurnAllLevelLEDOff(){
	HAL_GPIO_WritePin(GPIOE, LevelLED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LevelLED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LevelLED3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LevelLED4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LevelLED5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LevelLED6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LevelLED7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LevelLED8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LevelLED9_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LevelLED10_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LevelLED11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LevelLED12_Pin, GPIO_PIN_RESET);
}

void SetLevelLED(uint8_t level) {
	// level is 0 to 255
	// There are 12 pins, not the finest division
	TurnAllLevelLEDOff();

	// Fall through and set all LEDs below level on to indicate that much power via LED
	switch(uint8_t(level/21)){
	case 12:
		HAL_GPIO_WritePin(GPIOD, LevelLED12_Pin, GPIO_PIN_SET);
	case 11:
		HAL_GPIO_WritePin(GPIOD, LevelLED11_Pin, GPIO_PIN_SET);
	case 10:
		HAL_GPIO_WritePin(GPIOD, LevelLED10_Pin, GPIO_PIN_SET);
	case 9:
		HAL_GPIO_WritePin(GPIOB, LevelLED9_Pin, GPIO_PIN_SET);
	case 8:
		HAL_GPIO_WritePin(GPIOB, LevelLED8_Pin, GPIO_PIN_SET);
	case 7:
		HAL_GPIO_WritePin(GPIOB, LevelLED7_Pin, GPIO_PIN_SET);
	case 6:
		HAL_GPIO_WritePin(GPIOB, LevelLED6_Pin, GPIO_PIN_SET);
	case 5:
		HAL_GPIO_WritePin(GPIOE, LevelLED5_Pin, GPIO_PIN_SET);
	case 4:
		HAL_GPIO_WritePin(GPIOE, LevelLED4_Pin, GPIO_PIN_SET);
	case 3:
		HAL_GPIO_WritePin(GPIOE, LevelLED3_Pin, GPIO_PIN_SET);
	case 2:
		HAL_GPIO_WritePin(GPIOE, LevelLED2_Pin, GPIO_PIN_SET);
	case 1:
		HAL_GPIO_WritePin(GPIOE, LevelLED1_Pin, GPIO_PIN_SET);
	default:
	}

}

// Test Function to just verify you can program STM32
void Flash_LED(){
	HAL_GPIO_TogglePin (GPIOB, LED1_Pin);
	HAL_GPIO_TogglePin (GPIOB, LED2_Pin);
	HAL_GPIO_TogglePin (GPIOB, LED3_Pin);
	HAL_Delay(500);
}

// Test function to verify that Python Script successfully wrote data into STM32
// Basically, just reads channel_value[0] and shows its lowest 3 bits
// Since Python goes from 0 to 10, we expect to see:
// 000, 001, 010, 011, 100, 101, 110, 111, (1)000, (1)001, (1)010, then repeat
void Display_First_Channel_Smallest3Bits() {
	uint8_t testbyte = channel_value[0];
	if(testbyte & 1U){
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
	}
	else{
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
	}
	if(testbyte & 2U){
	  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
	}
	else{
	  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
	}
	if(testbyte & 4U){
	  HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
	}
	else{
	  HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(100);
}

// Below is the Handler, Called from CDC_Receive_FS() in USB_DEVICE -> App -> usbd_cdc_if.c
// Make Edits there

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
//	HAL_GPIO_TogglePin (GPIOB, LED2_Pin);
	for(int i = 0; i<8; i++){
		channel_value[i] = Buf[i];
	}
//	HAL_GPIO_TogglePin (GPIOB, LED2_Pin);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
//static void updatePWMValues(uint8_t newDutyCycle[8]);
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

   HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

   HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
   //TIM2->CCR1 = 5000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Basic LED Blink Test to make sure that your MCU can be programmed.
	  // Flash_LED();

	  // Testing dynamic memory writing with just the user LED for testing
	  // Display_First_Channel_Smallest3Bits();

	  // PWM update call
	  // updatePWMValues(test);

	  // Test Rotary Encoder
	  // RotaryEncoderTest();

	  {
		  // CALIBRATION SECTION WOULD LOOK LIKE THIS:
		  // 1. SET ALL PWM DUTY CYCLE TO 0
		  TIM2->CCR1 = 0;
		  TIM2->CCR2 = 0;
		  // TODO: Rest of timer here...

		  // 2. If there's button press, call Set_DLow_and_DHigh_For_Channel for that button
		  // Note that Set_DLow_and_DHigh_For_Channel() is a BLOCKING FUNCTION. Only external things like PWM runs
		  if(!HAL_GPIO_ReadPin(Ch1Button_GPIO_Port, Ch1Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(1);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch2Button_GPIO_Port, Ch2Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(2);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch3Button_GPIO_Port, Ch3Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(3);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch4Button_GPIO_Port, Ch4Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(4);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch5Button_GPIO_Port, Ch5Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(5);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch6Button_GPIO_Port, Ch6Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(6);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch7Button_GPIO_Port, Ch7Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(7);
		  }
		  else if(!HAL_GPIO_ReadPin(Ch8Button_GPIO_Port, Ch8Button_Pin)) {
			  Set_DLow_and_DHigh_For_Channel(8);
		  }

		  // 3. If there's a button press for going to operation mode, get out of Setup state.
		  else if(!HAL_GPIO_ReadPin(SetupButton_GPIO_Port, SetupButton_Pin)){
			  // TODO: Write Code that switches state machine
		  }

		  // Otherwise just loop and wait for button presses
	  }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1020;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 799;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Ch1LowLED_Pin|Ch1HighLED_Pin|LevelLED1_Pin|LevelLED2_Pin
                          |LevelLED3_Pin|LevelLED4_Pin|LevelLED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Ch2LowLED_Pin|Ch2HighLED_Pin|SetUpStatusLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Ch3LowLED_Pin|Ch3HighLED_Pin|Ch7LowLED_Pin|Ch7HighLED_Pin
                          |Ch8LowLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LevelLED6_Pin|LevelLED7_Pin|LevelLED8_Pin
                          |LevelLED9_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Ch8HighLED_Pin|Ch4HighLED_Pin|Ch4LowLED_Pin|Ch5HighLED_Pin
                          |Ch5LowLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LevelLED10_Pin|LevelLED11_Pin|LevelLED12_Pin|Ch6LowLED_Pin
                          |Ch6HighLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Ch1Button_Pin */
  GPIO_InitStruct.Pin = Ch1Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch1Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch1LowLED_Pin Ch1HighLED_Pin LevelLED1_Pin LevelLED2_Pin
                           LevelLED3_Pin LevelLED4_Pin LevelLED5_Pin */
  GPIO_InitStruct.Pin = Ch1LowLED_Pin|Ch1HighLED_Pin|LevelLED1_Pin|LevelLED2_Pin
                          |LevelLED3_Pin|LevelLED4_Pin|LevelLED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch2Button_Pin */
  GPIO_InitStruct.Pin = Ch2Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch2Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch2LowLED_Pin Ch2HighLED_Pin SetUpStatusLED_Pin */
  GPIO_InitStruct.Pin = Ch2LowLED_Pin|Ch2HighLED_Pin|SetUpStatusLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch3Button_Pin Ch8Button_Pin */
  GPIO_InitStruct.Pin = Ch3Button_Pin|Ch8Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch3LowLED_Pin Ch3HighLED_Pin Ch7LowLED_Pin Ch7HighLED_Pin
                           Ch8LowLED_Pin */
  GPIO_InitStruct.Pin = Ch3LowLED_Pin|Ch3HighLED_Pin|Ch7LowLED_Pin|Ch7HighLED_Pin
                          |Ch8LowLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : SetupButton_Pin */
  GPIO_InitStruct.Pin = SetupButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SetupButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LevelLED6_Pin LevelLED7_Pin LevelLED8_Pin
                           LevelLED9_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LevelLED6_Pin|LevelLED7_Pin|LevelLED8_Pin
                          |LevelLED9_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch7Button_Pin */
  GPIO_InitStruct.Pin = Ch7Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch7Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch8HighLED_Pin Ch4HighLED_Pin Ch4LowLED_Pin Ch5HighLED_Pin
                           Ch5LowLED_Pin */
  GPIO_InitStruct.Pin = Ch8HighLED_Pin|Ch4HighLED_Pin|Ch4LowLED_Pin|Ch5HighLED_Pin
                          |Ch5LowLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LevelLED10_Pin LevelLED11_Pin LevelLED12_Pin Ch6LowLED_Pin
                           Ch6HighLED_Pin */
  GPIO_InitStruct.Pin = LevelLED10_Pin|LevelLED11_Pin|LevelLED12_Pin|Ch6LowLED_Pin
                          |Ch6HighLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Ch4Button_Pin Ch5Button_Pin */
  GPIO_InitStruct.Pin = Ch4Button_Pin|Ch5Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ch6Button_Pin */
  GPIO_InitStruct.Pin = Ch6Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ch6Button_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
