
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "user_5110.h"
#include "ina219.h"
#include "esp8266at.h"
#include "MQTTPacket.h"
#include "transport.h"
#include "eeprom.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId taskAquisicaoHandle;
uint32_t taskAquisicaoBuffer[ 128 ];
osStaticThreadDef_t taskAquisicaoControlBlock;
osThreadId taskAcionamentoHandle;
uint32_t taskAcionamentoBuffer[ 128 ];
osStaticThreadDef_t taskAcionamentoControlBlock;
osThreadId taskDisplayHandle;
uint32_t taskDisplayBuffer[ 128 ];
osStaticThreadDef_t taskDisplayControlBlock;
osThreadId taskComunicacoaHandle;
uint32_t taskComunicacoaBuffer[ 128 ];
osStaticThreadDef_t taskComunicacoaControlBlock;
osMutexId mutexConfigHandle;
osStaticMutexDef_t mutexConfigControlBlock;
osMutexId mutexStatusHandle;
osStaticMutexDef_t mutexStatusControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

Status_t Status;
Config_t Config;
char buffer[50];
char payload[200];
uint32_t buffAdc[3];
int seq = 0;



//Reimplementação das funções para direcionar o printf para a UART2
int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart1, &*c, 1, 10);
	return ch;
}

int _write(int file,char *ptr, int len){
	int DataIdx;
	for(DataIdx= 0; DataIdx< len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
void leitura(void const * argument);
void motores(void const * argument);
void impressao(void const * argument);
void rede(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void sampleserial_init(void);
void sampleserial_close(void);
int samplesend(unsigned char *address, unsigned int bytes);
int samplerecv(unsigned char *address, unsigned int maxbytes);
int mqtt_start(void);
int mqtt_pub(void);
/* */

/* You will use your hardware specifics here, see transport.h. */
static transport_iofunctions_t iof = {samplesend, samplerecv};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Config.tempoLigadoDia = 8;
	Config.tempoDesligadoDia = 15;
	Config.tempoLigadoNoite = 8;
	Config.tempoDesligadoNoite = 15;
	Config.Imax = 2000;
	Config.VbatMin = 0000;
	Config.VcargaMin = 0000;
	Config.tempAguaMax = 35;
	Config.tempProcMax  = 65;
	Config.thresholdDia  = 3000;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of mutexConfig */
  osMutexStaticDef(mutexConfig, &mutexConfigControlBlock);
  mutexConfigHandle = osMutexCreate(osMutex(mutexConfig));

  /* definition and creation of mutexStatus */
  osMutexStaticDef(mutexStatus, &mutexStatusControlBlock);
  mutexStatusHandle = osMutexCreate(osMutex(mutexStatus));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of taskAquisicao */
  osThreadStaticDef(taskAquisicao, leitura, osPriorityNormal, 0, 128, taskAquisicaoBuffer, &taskAquisicaoControlBlock);
  taskAquisicaoHandle = osThreadCreate(osThread(taskAquisicao), NULL);

  /* definition and creation of taskAcionamento */
  osThreadStaticDef(taskAcionamento, motores, osPriorityNormal, 0, 128, taskAcionamentoBuffer, &taskAcionamentoControlBlock);
  taskAcionamentoHandle = osThreadCreate(osThread(taskAcionamento), NULL);

  /* definition and creation of taskDisplay */
  osThreadStaticDef(taskDisplay, impressao, osPriorityIdle, 0, 128, taskDisplayBuffer, &taskDisplayControlBlock);
  taskDisplayHandle = osThreadCreate(osThread(taskDisplay), NULL);

  /* definition and creation of taskComunicacoa */
  osThreadStaticDef(taskComunicacoa, rede, osPriorityLow, 0, 128, taskComunicacoaBuffer, &taskComunicacoaControlBlock);
  taskComunicacoaHandle = osThreadCreate(osThread(taskComunicacoa), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIM800_RST_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ESP_RST_Pin|LCD_CS_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SIM800_RST_Pin RELE2_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = SIM800_RST_Pin|RELE2_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_DATA_Pin */
  GPIO_InitStruct.Pin = DS18B20_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DS18B20_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELE1_Pin ESP_RST_Pin LCD_CS_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = RELE1_Pin|ESP_RST_Pin|LCD_CS_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	 if(huart->Instance==USART3) {
		 Wifi_RxCallBack();
	 }

	 if(huart->Instance==USART2) {
		 //
	 }

	 if(huart->Instance==USART1) {
		 //
	 }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  seq = 1;
}



int mqtt_start(void){

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	int rc = 0;
	int mysock = 0;
	unsigned char buf[200];
	int buflen = sizeof(buf);
	int len = 0;
	MQTTString topicString = MQTTString_initializer;
	int req_qos = 0;
	char* payload = "mypayload123";
	int payloadlen = strlen(payload);
	MQTTTransport mytransport;

    sampleserial_init();

	mysock = transport_open(&iof);
	if(mysock < 0) {
		printf("Erro conectar socket \n\r");
		//return mysock;
	}


	mytransport.sck = &mysock;
	mytransport.getfn = transport_getdatanb;
	mytransport.state = 0;
	data.clientID.cstring = "meu1234";
	data.keepAliveInterval = 20;
	data.cleansession = 1;


	len = MQTTSerialize_connect(buf, buflen, &data);
	/* This one blocks until it finishes sending, you will probably not want this in real life,
	in such a case replace this call by a scheme similar to the one you'll see in the main loop */

	//printf("length %d\n\r", len);

	rc = transport_sendPacketBuffer(mysock, buf, len);

	//printf("Sent %d - %d - %d \n\r", mysock, buf, len);
	/* wait for connack */
//	do {
//		int frc;
//		if ((frc=MQTTPacket_readnb(buf, buflen, &mytransport)) == CONNACK){
//			unsigned char sessionPresent, connack_rc;
//			printf("Entrou CoNNack %d , %d, %d\n\r", frc, buflen, buf);
//			if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0){
//				printf("Unable to connect, return code %d\n\r", connack_rc);
//				return -1;
//			}
//			break;
//		}
//		else if (frc == -1){
//			printf("MQTT not connected\n");
//			return -1;
//		}
//	} while (1);  //handle timeouts here

	HAL_Delay(2000);

	printf("MQTT connected\n\r");

	return 0;

}

int mqtt_pub(void){

	int mysock = 0;
	unsigned char buf[200];
	int buflen = sizeof(buf);
	int len = 0;
	MQTTString topicString = MQTTString_initializer;
	int payloadlen = strlen(payload);

	topicString.cstring = "pubtopic28102018";
	len = MQTTSerialize_publish(buf, buflen, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
	transport_sendPacketBuffer(mysock, buf, len);

	return 0;

}


int mqtt_sub(void){

	int mysock = 0;
	int rc = 0;
	unsigned char buf[200];
	int buflen = sizeof(buf);
	int len = 0;
	int msgid = 1;
	int req_qos = 0;
	MQTTString topicString = MQTTString_initializer;
	MQTTTransport mytransport;

	mytransport.sck = &mysock;
	mytransport.getfn = transport_getdatanb;
	mytransport.state = 0;

	/* subscribe */
	topicString.cstring = "substopic";
	len = MQTTSerialize_subscribe(buf, buflen, 0, msgid, 1, &topicString, &req_qos);

		/* This is equivalent to the one above, but using the non-blocking functions. You will probably not want this in real life,
		in such a case replace this call by a scheme similar to the one you'll see in the main loop */
		transport_sendPacketBuffernb_start(mysock, buf, len);
		while((rc=transport_sendPacketBuffernb(mysock)) != TRANSPORT_DONE);
		do {
			int frc;
			if ((frc=MQTTPacket_readnb(buf, buflen, &mytransport)) == SUBACK){ /* wait for suback */
				unsigned short submsgid;
				int subcount;
				int granted_qos;

				rc = MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, buf, buflen);
				if (granted_qos != 0){
					printf("granted qos != 0, %d\n", granted_qos);
					return -1;
				}
				break;
			}
			else if (frc == -1)
				return -1;
		} while (1); /* handle timeouts here */
		printf("Subscribed\n");

	return 0;

}


int sub_read(void){

	int mysock = 0;
	int rc = 0;
	MQTTTransport mytransport;

	mytransport.sck = &mysock;
	mytransport.getfn = transport_getdatanb;
	mytransport.state = 0;
	unsigned char buf[200];
	int buflen = sizeof(buf);

	if((rc=MQTTPacket_readnb(buf, buflen, &mytransport)) == PUBLISH){
				unsigned char dup;
				int qos;
				unsigned char retained;
				unsigned short msgid;
				int payloadlen_in;
				unsigned char* payload_in;
				int rc;
				MQTTString receivedTopic;

				rc = MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,	&payload_in, &payloadlen_in, buf, buflen);

				printf("message arrived %d %s\n\r", payloadlen_in, payload_in);

	} else if(rc == -1){
		return -1;
	}

	return 0;
}

void sampleserial_init(void)
{

	if(Wifi_TcpIp_StartTcpConnection(Connection.LinkId, "37.187.106.16", 1883, 1000)==0){
		printf("ERROR connecting Mosquitto\n\r");
	}

//	if(Wifi_TcpIp_StartTcpConnection(Connection.LinkId, "192.168.0.15", 1883, 500)==0){
//		printf("ERROR connecting Mosquitto\n\r");
//	}

}

void sampleserial_close(void)
{
	Wifi_TcpIp_Close(Connection.LinkId);
}

int samplesend(unsigned char *address, unsigned int bytes)
{
	if(Wifi_TcpIp_SendDataTcp(Connection.LinkId, bytes, address)==0){
		return -1;
	}
	return bytes;
}

int samplerecv(unsigned char *address, unsigned int maxbytes)
{


	if (Wifi.GotNewData){
		 //Wifi.GotNewData=false;
		 printf("recv %d - %d \n\r",maxbytes, Wifi.RxBufferForData);
		 memcpy(address,Wifi.RxBufferForData,maxbytes);
	     //Wifi.RxDataLen = 0;
	 }
	 return  maxbytes;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_leitura */
/**
  * @brief  Function implementing the taskAquisicao thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_leitura */
void leitura(void const * argument)
{

	/* USER CODE BEGIN 5 */
	int corrente;
	int potencia;
	int tensaoCarga;
	int tensaoPainel = 0;
	int tensaoBateria = 0;
	int tempProc = 0;
	int tempAgua;
	uint8_t  isDia;
	uint8_t  isFail;
	uint8_t isError;
	uint8_t  normalTemp;
	int tempProcMax;
	int	tempAguaMax;
	int	VbatMin;
	int	VcargaMin;
	int Imax;
	int thresholdDia;

	osMutexWait(mutexConfigHandle, osWaitForever);
	tempProcMax = Config.tempProcMax;
	tempAguaMax = Config.tempAguaMax;
	thresholdDia = Config.thresholdDia;
	VbatMin = Config.VbatMin;
	VcargaMin = Config.VcargaMin;
	Imax = Config.Imax;
	osMutexRelease(mutexConfigHandle);

	ina219Init();
	Ds18b20_Init();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buffAdc, 3);

	/* Infinite loop */
	for(;;)
	{
		HAL_ADC_Start(&hadc1);

		corrente = ina219GetCurrent_mA();
		potencia = ina219GetPower_mW();
		tensaoCarga = ina219GetBusVoltage();

		Ds18b20_ManualConvert();

		if (seq == 1){
			tempProc = COMPUTATION_TEMPERATURE_STD_PARAMS(buffAdc[2]);
			tensaoPainel = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(buffAdc[0]);
			tensaoBateria = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(buffAdc[1]);
			seq = 0;
		}

		if (ds18b20[0].DataIsValid) {
			tempAgua = (ds18b20[0].Temperature * 100);
			isError = 0;
		}else{
			isError = 1;
			tempAgua = 25;
		}

		if (tempAgua > tempAguaMax)
			normalTemp = 0;
		else
			normalTemp = 1;

		if (tensaoPainel > thresholdDia)
			isDia = 1;
		else
			isDia = 0;

		if (tempProc > tempProcMax)
			isFail = 1;
		else if (tensaoBateria < VbatMin)
			isFail = 1;
		else if (tensaoCarga < VcargaMin)
			isFail = 1;
		else if (corrente > Imax)
			isFail = 1;
		else
			isFail = 0;

		osMutexWait(mutexStatusHandle, 1000);
		Status.isDia  = isDia;
		Status.isFail = isFail;
		Status.isError = isError;
		Status.normalTemp = normalTemp;
		Status.tempAgua = tempAgua;
		Status.tempProc = tempProc;
		Status.tensaoCarga = tensaoCarga;
		Status.tensaoBateria = tensaoBateria;
		Status.tensaoPainel = tensaoPainel;
		Status.corrente = corrente;
		Status.potencia = potencia;
		osMutexRelease(mutexStatusHandle);

		HAL_IWDG_Refresh(&hiwdg);

		printf("sensores\n\r");
		osDelay(5000);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motores */
/**
* @brief Function implementing the taskAcionamento thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motores */
void motores(void const * argument)
{
  /* USER CODE BEGIN motores */

	TickType_t timeBefore;
	TickType_t totalTime;
	uint8_t  numMotor = 1;
	uint8_t  statusMotor = 0;
	uint8_t  isDia = 0;
	uint8_t  isFail = 0;
	uint8_t  normalTemp = 1;

	osMutexWait(mutexConfigHandle, osWaitForever);
	TickType_t tempoLigadoDia = Config.tempoLigadoDia * 60000;
	TickType_t tempoDesligadoDia = Config.tempoDesligadoDia * 60000;
	TickType_t tempoLigadoNoite = Config.tempoLigadoNoite * 60000;
	TickType_t tempoDesligadoNoite = Config.tempoDesligadoNoite * 60000;
	osMutexRelease(mutexConfigHandle);

	HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
	osDelay(5000);
	HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_RESET);
	osDelay(5000);
	HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_SET);

	timeBefore = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		printf("motores T %d \n\r", totalTime);

		osMutexWait(mutexStatusHandle, osWaitForever);
		isDia = Status.isDia;
		isFail = Status.isFail;
		normalTemp = Status.normalTemp;
		osMutexRelease(mutexStatusHandle);

		totalTime = xTaskGetTickCount() - timeBefore;

		if (!isFail){
			if (normalTemp){
				if (isDia){
					if ((totalTime > tempoDesligadoDia) & (statusMotor == 0)) {
						totalTime = 0;
						statusMotor = 1;
						timeBefore = xTaskGetTickCount();
						if (numMotor == 1){
							HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
							numMotor = 2;
							osMutexWait(mutexStatusHandle, 1000);
							Status.motor1 = 1;
							osMutexRelease(mutexStatusHandle);
							printf("Ligado Motor1\n\r");
						} else {
							HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_RESET);
							numMotor = 1;
							osMutexWait(mutexStatusHandle, 1000);
							Status.motor2 = 1;
							osMutexRelease(mutexStatusHandle);
							printf("Ligado Motor2\n\r");
						}
					}
					if ((totalTime > tempoLigadoDia) & (statusMotor == 1)) {
						totalTime = 0;
						timeBefore = xTaskGetTickCount();
						statusMotor = 0;
						HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_SET);
						osMutexWait(mutexStatusHandle, 1000);
						Status.motor1 = 0;
						Status.motor2 = 0;
						osMutexRelease(mutexStatusHandle);
						printf("Desligado Motores\n\r");
					}
				}else{
					if ((totalTime > tempoDesligadoNoite) & (statusMotor == 0)) {
						totalTime = 0;
						statusMotor = 1;
						timeBefore = xTaskGetTickCount();
						if (numMotor == 1){
							HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
							numMotor = 2;
							osMutexWait(mutexStatusHandle, 1000);
							Status.motor1 = 1;
							osMutexRelease(mutexStatusHandle);
							printf("Ligado Motor1\n\r");
						} else {
							HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_RESET);
							numMotor = 1;
							osMutexWait(mutexStatusHandle, 1000);
							Status.motor2 = 1;
							osMutexRelease(mutexStatusHandle);
							printf("Ligado Motor2\n\r");
						}
					}
					if ((totalTime > tempoLigadoNoite) & (statusMotor == 1)) {
						totalTime = 0;
						timeBefore = xTaskGetTickCount();
						statusMotor = 0;
						HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_SET);
						osMutexWait(mutexStatusHandle, 1000);
						Status.motor1 = 0;
						Status.motor2 = 0;
						osMutexRelease(mutexStatusHandle);
						printf("Desligado Motores\n\r");
					}
				}
			}else{
				statusMotor = 1;
				timeBefore = 0; //##todo
				HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_RESET);
				osMutexWait(mutexStatusHandle, 1000);
				Status.motor1 = 1;
				Status.motor2 = 1;
				osMutexRelease(mutexStatusHandle);
				printf("Temperatura da Água Elevada - Ambos Motores Ligados\n\r");
			}
		}else{
			statusMotor = 0;
			HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RELE2_GPIO_Port, RELE2_Pin, GPIO_PIN_SET);
			osMutexWait(mutexStatusHandle, 1000);
			Status.motor1 = 0;
			Status.motor2 = 0;
			osMutexRelease(mutexStatusHandle);
			printf("Desligado Motores por Falha\n\r");
		}

		HAL_IWDG_Refresh(&hiwdg);

		osDelay(10000);
	}
  /* USER CODE END motores */
}

/* USER CODE BEGIN Header_impressao */
/**
* @brief Function implementing the taskDisplay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_impressao */
void impressao(void const * argument)
{
	/* USER CODE BEGIN impressao */
	int tensaoCarga;
	int corrente;
	int potencia;
	int	tempAgua;
	int	tempProc;
	int	tensaoBateria;
	int tensaoPainel;
	char motor1;
	char motor2;
	char isDia;
	char isFail;
	char isError;
	char normalTemp;
	char conexao;

	LCD_Init();
	LCD_Clear();

	/* Infinite loop */
	for(;;)
	{

		osMutexWait(mutexStatusHandle, 1000);
		conexao = Status.conexao;
		tensaoCarga = Status.tensaoCarga;
		corrente = Status.corrente;
		potencia = Status.potencia;
		tempAgua = Status.tempAgua;
		tempProc = Status.tempProc;
		tensaoBateria = Status.tensaoBateria;
		tensaoPainel = Status.tensaoPainel;
		motor1 = Status.motor1;
		motor2 = Status.motor2;
		isDia = Status.isDia;
		isFail = Status.isFail;
		isError = Status.isError;
		normalTemp = Status.normalTemp;
		osMutexRelease(mutexStatusHandle);

		sprintf(buffer,"P:%5d I:%4d ", potencia, corrente);
		LCD_Write_String(0,0,buffer);
		sprintf(buffer,"Vc:%5d Vb:%5d", tensaoCarga, tensaoBateria);
		LCD_Write_String(0,1,buffer);
		sprintf(buffer,"Vp:%5d Tp:%3d", tensaoPainel, tempProc);
		LCD_Write_String(0,2,buffer);
		sprintf(buffer,"Ta:3%d Er:%d Fa:%d", tempAgua, isError, isFail );
		LCD_Write_String(0,3,buffer);
		sprintf(buffer,"M1:%d M2:%d",motor1 ,motor2 );
		LCD_Write_String(0,4,buffer);
		sprintf(buffer,"Tn:%d Co:%d D:%d", normalTemp, conexao, isDia);
		LCD_Write_String(0,5,buffer);


		printf("conexao %d \n\r", conexao);
		printf("isDia %d \n\r", isDia);
		printf("isFail %d \n\r", isFail);
		printf("isError %d \n\r", isError);
		printf("normalTemp %d \n\r", normalTemp);
		printf("tempAgua %d \n\r", tempAgua);
		printf("tempProc %d \n\r", tempProc);
		printf("tensaoCarga %d \n\r", tensaoCarga);
		printf("tensaoBateria %d \n\r", tensaoBateria);
		printf("tensaoPaine %d \n\r", tensaoPainel);
		printf("corrente %d \n\r", corrente);
		printf("potencia %d \n\r", potencia);
		printf("Mot1 %d Mot2 %d \n\r", motor1, motor2);


		osDelay(1000);
	}
	/* USER CODE END impressao */
}

/* USER CODE BEGIN Header_rede */
/**
* @brief Function implementing the taskComunicacoa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rede */
void rede(void const * argument)
{
	/* USER CODE BEGIN rede */
	char conexao;

	Wifi_Init();

	Wifi_SetMode(WifiMode_Station);

	conexao = Wifi_Station_ConnectToAp("","",NULL);

	osMutexWait(mutexStatusHandle, 1000);
	Status.conexao = conexao;
	osMutexRelease(mutexStatusHandle);

	//sampleserial_init();

	//mqtt_start();

	//mqtt_sub();

	/* Infinite loop */
	for(;;)
	{
//		if (Wifi_TcpIp_GetConnectionStatus())
//			conexao = Wifi.TcpIpConnections[0].status;
//		else
//			conexao = -99;

		osMutexWait(mutexStatusHandle, 1000);
		Status.conexao = conexao;
		osMutexRelease(mutexStatusHandle);

		sprintf(payload,"P:%d", ina219GetPower_mW());
		//mqtt_pub();
		printf("rede\n\r");
		osDelay(5000);
	}
	/* USER CODE END rede */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
