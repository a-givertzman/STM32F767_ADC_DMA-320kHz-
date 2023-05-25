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
#define UDP_SYN 22
#define UDP_EOT 4
#define CHAN_ADDR 3
enum {
  UDP_TYPE_INT8 = 8,
  UDP_TYPE_DOUBLE = 16,
  UDP_TYPE_ARRAY = 32
};

extern struct netif gnetif;
#define ADC_BUF_LEN 1024
#define ADC_BUF_HALF_LEN ADC_BUF_LEN / 2
#define UDP_BUF_HALF_LEN ADC_BUF_LEN
#define UDP_BUF_LEN (ADC_BUF_LEN * 2)
// uint8_t udp_buf[UDP_BUF_LEN];
uint16_t adc_buf[ADC_BUF_LEN];
// uint16_t adc_buf_test[ADC_BUF_LEN];
//float adcVoltage[ADC_BUF_LEN];
struct udp_pcb *upcb;
static struct pbuf *txBuf;
int APP_ERROR_CODE = 0;
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
#define UDP_HEAD_BUF_LEN 3
const uint8_t buf_head[UDP_HEAD_BUF_LEN] = {UDP_SYN, CHAN_ADDR, UDP_TYPE_ARRAY};
///
/// 
static void udpClient_send(void) {
  err_t err = udp_send(upcb, txBuf);
  if (err != ERR_OK) {
    APP_ERROR_CODE = 1;
  }
}
/// 
///  
static void buildBuferHalf(uint8_t half) {
  uint16_t *buf;
  if (half == 1) {
    buf = &(adc_buf[0]);
  } else {
    buf = &(adc_buf[ADC_BUF_HALF_LEN]);
  }
	if (txBuf != NULL) {
		pbuf_take_at(
      txBuf, 
      buf,
      UDP_BUF_HALF_LEN,   // length
      UDP_HEAD_BUF_LEN    // offset
    );
	  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); //PC10_out
		udpClient_send();
	  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); //PC10_out
	}
}
///
///
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11); //PC11_out
    // delayTick(2);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11); //PC11_out
	  buildBuferHalf(1);
	}
}
///
///
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11); //PC11_out
    // delayTick(2);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11); //PC11_out
	  buildBuferHalf(2);
	}
}
///
///
void delayTick(u16_t ticks) {
  while (ticks >= 0) {
    ticks--;
  }
}
///
///
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// 	if (htim->Instance == TIM1) {
// 	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); //PC10_out
// 		// HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //LED_GREEN
// 	  // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //LED_BLUE
// //		__NOP();
// 	}
// }
///
///
void HAL_ADC_ErrorCallback (ADC_HandleTypeDef * hadc) {
  APP_ERROR_CODE = 16;
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
}
///
///
void ADC_DMAError (DMA_HandleTypeDef * hdma) {
  APP_ERROR_CODE = 32;
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
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
		// udpClient_send();
		/* 3. Set a receive callback for the upcb */
		udp_recv(upcb, udp_receive_callback, NULL);
	}
}
///
///
void testLeds(int count) {
  if (count <= 0) {count = 1;};
  int duration = 24;  // milles
  for (uint16_t i = 0; i < count; i++) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
    HAL_Delay(duration);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
    HAL_Delay(duration);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //LED_GREEN
    HAL_Delay(duration);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //LED_GREEN
  }
}
///
///
void errorLeds(int err) {
  int count = 2;
  int duration = 24;  // milles
  // ERROR
  if (err > 0 && err <= 15) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
      HAL_Delay(duration);
    }
  }
  // WARNING
  if (err > 15 && err <= 31) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
      HAL_Delay(duration);
    }
  }
  // INFO1
  if (err > 31 && err <= 63) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
      HAL_Delay(duration);
    }
  }
  // INFO2
  if (err > 63 && err <= 127) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //LED_GREEN
      HAL_Delay(duration);
    }
  }
}
///
///
void handleError() {
  int e = APP_ERROR_CODE;
  APP_ERROR_CODE = 0;
  errorLeds(e);
}
///
/// | u8  | u8   | u8   | u8[1024]  | 
/// | SYN | ADDR | TYPE | DATA      |
/// SYN = 22 - message starts with
/// ADDR = 0...255 - an address of the signal
/// TYPE
///       8 - 1 byte integer value
///       16 - 2 byte float value
///       32 - u16[1024] an array of 2 byte values of length 512
void prepareTxBuf(void) {
	txBuf = pbuf_alloc(PBUF_TRANSPORT, UDP_BUF_HALF_LEN + UDP_HEAD_BUF_LEN, PBUF_RAM);
  pbuf_take_at(txBuf, &buf_head, UDP_HEAD_BUF_LEN, 0);
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
  prepareTxBuf();
  testLeds(2);
  udpClient_connect();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
  testLeds(1);
  // for (uint16_t i = 0; i < ADC_BUF_LEN; i++) {
  //     adc_buf_test[i] = i;
  // }  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
    if (APP_ERROR_CODE > 0) {
      handleError();
    }
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
