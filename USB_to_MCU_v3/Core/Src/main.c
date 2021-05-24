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
#include "stdio.h"
#include "string.h"
#include "CAN_ID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Buffer length should be a power of 2! Expressed in bytes.

#define USART_RX_BUFF_LENGTH 64

#define HEADER_LENGTH 2

#define CAN_ID_L_MASK 0b11100000
#define CAN_DATA_LENGTH_MASK 0b00011110

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Debugging:
#define ITM_Port32(n)	(*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pTxHeader;
uint32_t pTxMailbox;
uint8_t rx_buffer[USART_RX_BUFF_LENGTH];

//For Packet rate debug
uint8_t speed_count = 0;
uint8_t totalLaps_count = 0;
uint8_t lastlaptime_count = 0;
uint8_t currentlaptime_count = 0;
uint8_t bestlaptime_count = 0;
uint8_t carposition_count = 0;
uint8_t currentlapnum_count= 0;
uint8_t throttle_count = 0;
uint8_t brake_count = 0;
uint8_t gear_count = 0;
uint8_t revlightspercent_count = 0;
uint8_t tiressurfacetempRL_count = 0;
uint8_t tiressurfacetempRR_count = 0;
uint8_t tiressurfacetempFR_count = 0;
uint8_t tiressurfacetempFL_count = 0;
uint8_t enginetemperature_count = 0;
uint8_t safetycarstatus_count = 0;
uint8_t steer_count = 0;

uint8_t rx_full_cplt  = 0;

//	for CAN debug.
uint32_t pTxMailbox_debug;
CAN_TxHeaderTypeDef pTxHeader_debug;
CAN_RxHeaderTypeDef pRxHeader_debug;
CAN_FilterTypeDef sFilterConfig_debug;
uint8_t data_out_debug = 0, data_in_debug = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
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
  //	Debug purposes for printf() debug:
  ITM_Port32(31) = 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  //	Debug purposes for printf() debug:
  ITM_Port32(31) = 2;

   pTxHeader.IDE = CAN_ID_STD;
   pTxHeader.RTR = CAN_RTR_DATA;

  //	pTxHeader configuration for CAN debug
  pTxHeader_debug.DLC = 1;
  pTxHeader_debug.IDE = CAN_ID_STD;
  pTxHeader_debug.RTR = CAN_RTR_DATA;
  pTxHeader_debug.StdId = 0x244;

  //	sFilter configuration for CAN debug.
  sFilterConfig_debug.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig_debug.FilterIdHigh = 0x245 <<5;
  sFilterConfig_debug.FilterMaskIdLow = 0;
  sFilterConfig_debug.FilterMaskIdHigh = 0x000;
  sFilterConfig_debug.FilterMaskIdLow = 0;
  sFilterConfig_debug.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig_debug.FilterActivation = ENABLE;

  //	Setup the filter in the registers and enable interrupt to receive CAN packets for CAN debug.
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig_debug);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  //	Setup remaining CAN configurations.
  HAL_CAN_Start(&hcan);


  //Start receiving UART in the USART rx_buffer
  HAL_UART_Receive_DMA(&huart2, (rx_buffer), USART_RX_BUFF_LENGTH);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 15;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 57600;
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
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */



/**************************************************************************************************************************
 * ______________________________________________USART BUFFER HALF FULL CALLBACK___________________________________________
 ***************************************************************************************************************************/

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
//			printf("USART_Rx_buff FULL.\n");
//			printf("Initiate packet sending...\n");


			uint8_t *curr_byte_p = rx_buffer; //points to the start of the usart header.
			unsigned short can_id = 0;
			unsigned char can_id_l = 0;

//			Loop over the usart second half buffer
			while (curr_byte_p < (rx_buffer + USART_RX_BUFF_LENGTH/2) ){
				can_id  = *curr_byte_p;
				can_id <<= 3;
				can_id_l = *(curr_byte_p + 1) & CAN_ID_L_MASK;
				can_id_l >>= 5;

				pTxHeader.StdId = can_id + can_id_l;
				pTxHeader.DLC = (*(curr_byte_p + 1) & CAN_DATA_LENGTH_MASK) >> 1;
				HAL_CAN_AddTxMessage(&hcan, &pTxHeader, (curr_byte_p + 2), &pTxMailbox);
				printf("send sensor ID: %x with data length %d to node %x.\n", *(curr_byte_p + 2), pTxHeader.DLC, pTxHeader.StdId);
//				switch (*(curr_byte_p + 2)) {
//
//				//lastLapTime has been send out
//				case 0x70:
//					lastlaptime_count += 1;
//					if (lastlaptime_count > 100) {
//						lastlaptime_count = 0;
//						}
//					break;
//
//				//speed has been send out
//				case 0x77:
//					speed_count += 1;
//					if (speed_count > 100) {
//						speed_count = 0;
//					}
//					break;
//
//				case 0x73:
//					carposition_count += 1;
//					if (carposition_count > 100) {
//						carposition_count = 0;
//										}
//					break;
//				}
				curr_byte_p += 8;
									}
//					printf("HALF FULL send complete\n");


		}


	/**************************************************************************************************************************
	 * ______________________________________________USART BUFFER COMPLETELY FULL CALLBACK___________________________________________
	 ***************************************************************************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//

			rx_full_cplt +=1 ;
			HAL_UART_Receive_DMA(&huart2, (rx_buffer), USART_RX_BUFF_LENGTH);

			uint8_t *curr_byte_p = rx_buffer + USART_RX_BUFF_LENGTH/2;
			unsigned short can_id = 0;
			unsigned char can_id_l = 0;

//			Loop over the usart second half buffer
			while (curr_byte_p < (rx_buffer + USART_RX_BUFF_LENGTH) ){
				can_id  = *curr_byte_p;
				can_id <<= 3;
				can_id_l = *(curr_byte_p + 1) & CAN_ID_L_MASK;
				can_id_l >>= 5;

				pTxHeader.StdId = can_id + can_id_l;
				pTxHeader.DLC = (*(curr_byte_p + 1) & CAN_DATA_LENGTH_MASK) >> 1;
				HAL_CAN_AddTxMessage(&hcan, &pTxHeader, (curr_byte_p + 2), &pTxMailbox);
				printf("send sensor ID: %x with data length %d to node %x.\n", *(curr_byte_p + 2), pTxHeader.DLC, pTxHeader.StdId);

//				switch (*(curr_byte_p + 2)) {
//
//								//lasLapTime has been send out
//								case 0x70:
//									lastlaptime_count += 1;
//									if (lastlaptime_count > 100) {
//										lastlaptime_count = 0;
//										}
//									break;
//
//								//speed has been send out
//								case 0x77:
//									speed_count += 1;
//									if (speed_count > 100) {
//										speed_count = 0;
//										}
//									break;
//
//								case 0x73:
//									carposition_count += 1;
//									if (carposition_count > 100) {
//										carposition_count = 0;
//									}
//									break;
//								}
					curr_byte_p += 8;
					}

//					printf("HALF FULL send complete\n");


		}


//	Function for debugging with printf()
int _write(int file, char *ptr, int len)
	{
	  /* Implement your write code here, this is used by puts and printf for example */
	  int DataIdx;
	  for(DataIdx=0 ; DataIdx<len ; DataIdx++)
	    ITM_SendChar(*ptr++);
	  return len;
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
