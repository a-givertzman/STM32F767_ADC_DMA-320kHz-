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
#include "adc.h"
#include "dma.h"
#include "lwip.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct netif gnetif;
enum {
	ADC_BUF_LEN = 512,
	UDP_BUF_LEN = ADC_BUF_LEN * 2
};
uint8_t udp_buf[UDP_BUF_LEN];
uint16_t adc_buf[ADC_BUF_LEN];
//float adcVoltage[ADC_BUF_LEN];
struct udp_pcb *upcb;
struct pbuf *txBuf;
//int counter = 8;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///
///
static void udpClient_send(void) {

//  char data[100];
//  uint16_t adc_buf[ADC_BUF_LEN];
//  for (uint16_t i = 0; i < ADC_BUF_LEN; i++) {
//	  adc_buf[i] = i;
//  }
//  int len = sprintf(adc_buf, "sending UDP client message %d\n", counter);
	int len = UDP_BUF_LEN;
	txBuf = pbuf_alloc(PBUF_TRANSPORT, UDP_BUF_LEN, PBUF_RAM);
	if (txBuf != NULL) {
		/* copy data to pbuf */
		pbuf_take(txBuf, adc_buf, len);
		/* send udp data */
		err_t err = udp_send(upcb, txBuf);
		if (err != ERR_OK) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
//			HAL_Delay(50);
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
		}
	}
	pbuf_free(txBuf);
}
///
///
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //LED_BLUE
// __NOP();
//	if (hadc->Instance == ADC1) {
//		for (uint16_t i = 0; i < ADC_BUF_LEN; i++) {
////			adcVoltage[i] = adc_buf[i]; // * 0.000805664;
//			adcVoltage[i] = adc_buf[i] * 3.3 / 4095;
//		}
//	}
	udpClient_send();
//	counter++;
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //LED_BLUE
}
///
///
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
//	__NOP();
//	return;
//	if (hadc->Instance == ADC1) {
//		for (uint8_t i = 0; i < ADC_BUF_LEN / 2; i++) {
//			adcVoltage[i] = adc_buf[i] * 3.3 / 4095;
//		}
//	}
//}
///
///
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		// HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //LED_GREEN
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //LED_BLUE
//		__NOP();
	}
}
///
///
void HAL_ADC_ErrorCallback (ADC_HandleTypeDef * hadc) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //LED_GREEN
//	HAL_Delay(50);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //LED_GREEN
}
///
///
void ADC_DMAError (DMA_HandleTypeDef * hdma) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //LED_BLUE
//	HAL_Delay(50);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //LED_BLUE
//	HAL_Delay(50);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
//	HAL_Delay(50);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
}
///
///
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	/* Copy the data from the pbuf */
//	strncpy (buffer, (char *)p->payload, p->len);
	/*increment message count */
//	counter++;
	/* Free receive pbuf */
	pbuf_free(p);
}
///
///
void udpClient_connect(void) {
	err_t err;
	/* 1. Create a new UDP control block  */
	upcb = udp_new();
	/* Bind the block to module's IP and port */
	ip_addr_t myIPaddr;
	IP_ADDR4(&myIPaddr, 192, 168, 120, 173);
	u16_t udpPort = 15180;
	udp_bind(upcb, &myIPaddr, udpPort);
	/* configure destination IP address and port */
	ip_addr_t DestIPaddr;
	IP_ADDR4(&DestIPaddr, 192, 168, 120, 172);
	err= udp_connect(upcb, &DestIPaddr, udpPort);
	if (err == ERR_OK) {
		/* 2. Send message to server */
		udpClient_send();
		/* 3. Set a receive callback for the upcb */
		udp_recv(upcb, udp_receive_callback, NULL);
	}
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  udpClient_connect();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
	  ethernetif_input(&gnetif);

	  sys_check_timeouts();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
