/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "I2CNetworkCommon.h"
#include "HostDispatchCommands.h"
#include "TimeHelpers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define PERIPHERAL_COUNT 4 //How many boards we've got working as peripherals.
#define CYCLE_COUNT 4 //How many times we fill the buffer before stopping.

#define PRIME_BLINK_COUNT 3
#define PRIME_BLINK_TOTAL_TIME_MS 500

#define SAMPLE_TO_READ_TIME_MS 1000 //How long after we send the signal to sample do we start checking if the peripherals are ready to send.

#define NOT_READY_ADDITIONAL_DELAY_MS 50 //When we ask if a peripheral is done sampling, and it say no, how long we wait before asking again.

#define FIRST_DEVICE_DELAY_MS 35 //What the delay, in milliseconds, will be on the first device after sending the "Start Sampling" command.
#define DELAY_ADD_PER_DEVICE_MS 0 //How much time, in milliseconds, is added to the delay of each device after the first (cumulative).

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
int isPrimed = 0;
int isStarted = 0;
int isFinished = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//void SendSampleParameters(I2C_HandleTypeDef *hi2c, uint8_t peripheralAddress, uint8_t deviceCount, uint16_t bufferSize, uint8_t cycleCount, uint8_t delayMS);

void PrintUARTMessage(UART_HandleTypeDef *huart, const char message[]);

void BlinkLight(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int blinkCount, int totalTimeMS, GPIO_PinState endState);

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
	//HAL_StatusTypeDef ret;
	//uint8_t buf[1];

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //Green to indicate this is the host.
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);\
  //Make sure other lights are reset, so we know we're not a peripheral and we know we're not primed.
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  GPIO_PinState userButtonState = GPIO_PIN_RESET;

  //Confirm we're on.
  PrintUARTMessage(&huart3, "Host Initialized.");

  //Enable register for the timer.
  TIM2->CR1 = TIM_CR1_CEN;


  //Declare array that holds the time in microseconds when we send the signal for each peripheral to start.
  //This is used later to calculate the offset for their final times and synchronize them.
  int32_t startTimesUs[4];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(isPrimed == 0)
	  {
		  GPIO_PinState state = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
		  if(state != userButtonState)
		  {
			  if(state == GPIO_PIN_SET)
			  {
				  //Submit a message saying we're starting.
				  PrintUARTMessage(&huart3, "\r\nPriming");

				  //Blink light to indicate priming.
				  BlinkLight(LD2_GPIO_Port, LD2_Pin, PRIME_BLINK_COUNT, PRIME_BLINK_TOTAL_TIME_MS, GPIO_PIN_SET);

				  for(int i = 1; i <= PERIPHERAL_COUNT; i++)
				  {
					  uint8_t address = i << 1; //Shifted over by one for I2C.

					  //Calculate the delay.
					  uint8_t delayTimeMS = FIRST_DEVICE_DELAY_MS + (i - 1) * DELAY_ADD_PER_DEVICE_MS; //Subtract one from i since we start at 1, not 0.

					  SendSampleParamsCommand(&hi2c1, address, CYCLE_COUNT, delayTimeMS);
				  }

				  isPrimed = 1;
			  }

			  userButtonState = state;

		  }
	  }

	  if(isPrimed == 1 && isStarted == 0)
	  {
		  GPIO_PinState state = HAL_GPIO_ReadPin(SoundDetectorGate_GPIO_Port, SoundDetectorGate_Pin);
		  if(state == GPIO_PIN_SET)
		  {
			  //Turn on red LED to indicate we're dispatching.
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

			  ResetClock(htim2);

			  for(int i = 1; i <= PERIPHERAL_COUNT; i++)
			  {
				  uint8_t address = i << 1; //Shifted over by one for I2C.
				  BeginSamplingCommand(&hi2c1, address);

				  //Record the time.
				  uint32_t startTicks = ReadCurrentTicks(htim2);
				  startTimesUs[i - 1] = TicksToSubSecond(htim2, startTicks, MICROSECOND_DIVIDER);
			  }

			  //Subtract the first time from each start time element so that they are the offsets.
			  uint32_t firstTime = startTimesUs[0];
			  for(int i = 0; i < PERIPHERAL_COUNT; i++)
			  {
				  startTimesUs[i] -= firstTime;
			  }

			  isStarted = 1;
			  //Allow another sent packet and turn off blue prime indicator LED.
			  //Wait a short time to make it so the samples are likely ready (but we'll still be asking before receiving).
			  HAL_Delay(SAMPLE_TO_READ_TIME_MS);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

			  //Each peripheral needs its device numbers adjusted to be above the preceeding peripherals. Same with samples.
			  //Use the below numbers to add to the values in each peripheral after the first, and then add back to it.
			  int highestDeviceNumber = 0;
			  int highestSampleNumber = 0;

			  //Calculate how many samples to get per device.
			  //Go through each peripheral and wait for it to be ready, then receive sample.
			  for(int i = 1; i <= PERIPHERAL_COUNT; i++)
			  {
				  uint8_t address = i << 1; //Shifted over by one for I2C.

				  //Wait for the device to be ready.
				  enum BooleanReturnValue isReady = False;

				  while(isReady == False)
				  {
					  isReady = CheckFinishedCommand(&hi2c1, address);
				  }

				  //Now it's just something other than "not ready" so handle other situations.
				  if(isReady == BadData || isReady == Timeout)
				  {
					  //Flash red light in a distinct pattern to show there was an error here.
					  for(int b = 0; b < 3; b++)
					  {
						  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
						  HAL_Delay(120);
						  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
						  HAL_Delay(80);
					  }
					  continue; //Skip to the next peripheral.
				  }

				  //If we're here, the device said it's ready to send back the data. So get it.

				  //Wait a short amount of time for the listener to activate again. I guess.
				  HAL_Delay(5);

				  //Get total number of packets that we need to receive.
				  uint16_t totalPackets = RequestTotalPacketCountCommand(&hi2c1, address);

				  if(totalPackets == 0)
				  {
					  //Flash red once to show error here.
					  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
					  HAL_Delay(120);
					  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
					  HAL_Delay(80);

					  continue; //In the future, we could log an error here or something.
				  }

				  int highestDeviceNumberBeforeThisPeripheral = highestDeviceNumber;
				  int highestSampleNumberBeforeThisPeripheral = highestSampleNumber;

				  //Get each sample header and sample individually.
				  for(int j = 0; j < totalPackets; j++) //TODO: This is only first four.
				  {
					  //Get the header.
					  samplePacketHeader* header = calloc(sizeof(samplePacketHeader), 1);
					  RequestSampleHeaderCommand(&hi2c1, address, j, header);

					  //Change the sample and device numbers according to the peripheral index.
					  int additiveDeviceNumber = highestDeviceNumberBeforeThisPeripheral + header->DeviceID;
					  header->DeviceID = additiveDeviceNumber;
					  if(additiveDeviceNumber > highestDeviceNumber)
					  {
						  highestDeviceNumber = additiveDeviceNumber;
					  }

					  int additiveSampleNumber = highestSampleNumberBeforeThisPeripheral + header->SampleID;
					  header->SampleID = additiveSampleNumber;
					  if(additiveSampleNumber > highestSampleNumber)
					  {
						  highestSampleNumber = additiveSampleNumber;
					  }

					  //Adjust the start and ennd times based on the offset at which the devive was sent the sample message.
					  uint32_t startOffset = startTimesUs[i];
					  header->startTimeUs += startOffset;
					  header->endTimeUs += startOffset;

					  uint16_t samplesPerPacket = header->SampleCount;
					  uint16_t* data = calloc(sizeof(uint16_t), samplesPerPacket);

					  HAL_Delay(5); //Let it get back to the main loop.

					  //Get the actual data.
					  RequestSampleDataCommand(&hi2c1, address, samplesPerPacket, j, data); //i for now, just to get the samples from the first cycle.

					  //Flip the lights to indicate it's sending stuff.
					  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

					  //Transmit the data to the PC.
					  TransmitSamplePacketToPC(&huart3, *header, data);

					  //Wait a sec to make sure light are seen, then switch them off.
					  HAL_Delay(25);
					  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

					  /*
					  //Just for debug.
					  uint16_t debugData[samplesPerPacket];
					  for(int d = 0; d < samplesPerPacket; d++)
					  {
						  debugData[d] = data[d];
					  }
					  */

					  free(header);
					  free(data);

					  HAL_Delay(5); //Let it get back to the main loop.
				  }

				  //Increment once more so the next set starts at one higher.
				  highestDeviceNumber++;
				  highestSampleNumber++;
			  }

			  //TODO: These need to go somewhere at the end.
			  isPrimed = 0;
			  isStarted = 0;
		  }
		  //Make sure green light is on, as we flashed them while uploading.
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.Timing = 0x20303E5D;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SoundDetectorGate_Pin */
  GPIO_InitStruct.Pin = SoundDetectorGate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SoundDetectorGate_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void PrintUARTMessage(UART_HandleTypeDef *huart, const char message[])
{
	HAL_UART_Transmit(huart, (uint8_t*)message, strlen(message), 10);
	char newLine[3] = "\r\n";
	HAL_UART_Transmit(huart, (uint8_t*)newLine, 2, 10);
}

void BlinkLight(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int blinkCount, int totalTimeMS, GPIO_PinState endState)
{
	int timePerHalfBlink = totalTimeMS / blinkCount / 2;
	GPIO_PinState oppositeOfEndState = endState == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;

	for(int i = 0; i < blinkCount; i++)
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, oppositeOfEndState);
		HAL_Delay(timePerHalfBlink);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, endState);
		HAL_Delay(timePerHalfBlink);
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
