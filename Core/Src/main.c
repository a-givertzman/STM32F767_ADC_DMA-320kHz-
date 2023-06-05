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
#define false 0;
#define true 1;
enum {
  ERROR_RED = 255, 
  WARNING_RED_BLUE = 64, 
  WARNING_RED_GREEN = 32,
  INFO1_BLUE = 16, 
  INFO2_GREEN = 8,
  INFO3_BLUE_GREEN = 4,
};
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
#define UDP_HEAD_BUF_LEN 4
uint8_t buf_head[UDP_HEAD_BUF_LEN] = {
  UDP_SYN, 
  CHAN_ADDR, 
  UDP_TYPE_ARRAY,
  0,
};
// uint8_t udp_buf[UDP_BUF_LEN];
uint16_t adc_buf[ADC_BUF_LEN];
// uint16_t adc_buf_test[ADC_BUF_LEN];
//float adcVoltage[ADC_BUF_LEN];
struct udp_pcb *upcb = NULL;
static struct pbuf *txBuf = NULL;
uint8_t udp_sent_count = 0;
uint8_t udp_received_count = 0;
int APP_ERROR_CODE = 0;
int APP_ERROR_COUNT = 0;
u8_t isConnected = false;
u8_t connect = false;
u8_t connectCount = 0;
ip_addr_t remoteAddr;
u16_t remotePort = 0;
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
void errorLeds(int err) {
  int count = 2;
  int duration = 32;  // milles
  // ERROR
  if (err == ERROR_RED) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
      HAL_Delay(duration);
    }
  }
  // WARNING RED_BLUE
  if (err == WARNING_RED_BLUE) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
      HAL_Delay(duration);
    }
  }
  // WARNING RED_GREEN
  if (err == WARNING_RED_GREEN) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //LED_GREEN
      HAL_Delay(duration);
    }
  }
  // INFO1
  if (err == INFO1_BLUE) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
      HAL_Delay(duration);
    }
  }
  // INFO2
  if (err == INFO2_GREEN) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //LED_GREEN
      HAL_Delay(duration);
    }
  }
  // INFO3
  if (err == INFO3_BLUE_GREEN) {
    for (uint16_t i = 0; i < count; i++) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  //LED_BLUE
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  //LED_GREEN
      HAL_Delay(duration);
    }
  }
}
///
/// 
static void udpClientSend(void) {
  buf_head[3] = udp_sent_count;
  err_t err = pbuf_take_at(txBuf, &buf_head, UDP_HEAD_BUF_LEN, 0);
  // err_t err = pbuf_take_at(txBuf, &udp_sent_count, 1, 3);
  if (err == ERR_OK) {
    err = udp_send(upcb, txBuf);
    udp_sent_count++;
    if (err != ERR_OK) {
      APP_ERROR_CODE = WARNING_RED_BLUE;//WARNING_RED_GREEN;
    }
  } else {
      APP_ERROR_CODE = WARNING_RED_GREEN;
  }
}
///
///
void txBufClear(void) {
  if (txBuf != NULL) {
  	pbuf_free(txBuf);
  }
}
///
/// | u8  | u8   | u8   | u8    | u8[1024]  | 
/// | SYN | ADDR | TYPE | count | DATA      |
/// SYN = 22 - message starts with
/// ADDR = 0...255 - an address of the signal
/// TYPE
///       8 - 1 byte integer value
///       16 - 2 byte float value
///       32 - u16[1024] an array of 2 byte values of length 512
err_t txBufPrepare(void) {
	txBuf = pbuf_alloc(PBUF_TRANSPORT, UDP_BUF_HALF_LEN + UDP_HEAD_BUF_LEN, PBUF_RAM);
  err_t err = pbuf_take_at(txBuf, &buf_head, UDP_HEAD_BUF_LEN, 0);
  if (err != ERR_OK) {
    errorLeds(WARNING_RED_GREEN);
    // APP_ERROR_CODE = WARNING_RED_BLUE;
  }
  return err;
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
	if (isConnected && txBuf != NULL) {
		pbuf_take_at(
      txBuf, 
      buf,
      UDP_BUF_HALF_LEN,   // length
      UDP_HEAD_BUF_LEN    // offset
    );
	  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); //PC10_out
		udpClientSend();
	  // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); //PC10_out
    if (connect) {
      isConnected = false;
    }
  }
}
///
///
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
	  buildBuferHalf(1);
	}
}
///
///
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
	  buildBuferHalf(2);
	}
}
///
///
void delayTick(int16_t ticks) {
  while (ticks > 0) {
    ticks--;
  }
}
///
///
void softReset(void) {
  errorLeds(ERROR_RED);
  NVIC_SystemReset();
  while (1) {}
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
  APP_ERROR_CODE = WARNING_RED_BLUE;
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
}
///
///
void ADC_DMAError (DMA_HandleTypeDef * hdma) {
  APP_ERROR_CODE = WARNING_RED_GREEN;
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
}
///
///
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
  udp_received_count++;
  errorLeds(INFO2_GREEN);
	// err_t err;
  uint8_t buf[2];
  uint8_t udpSynBuf[2] = {UDP_SYN, UDP_EOT};
  if (p != NULL) {
    memcpy(buf, p->payload, 2);
    int synOk = memcmp(buf, udpSynBuf, 2);
    if (synOk == 0) {
      // isConnected = false;
      udp_sent_count = 0;
      udp_received_count = 0;
      remoteAddr = *addr;
      remotePort = port;
      connect = true;
      // txBufClear();
      // txBufPrepare();

      // err= udp_connect(upcb, addr, port);
      // APP_ERROR_CODE = INFO2_GREEN;
      // isConnected = true;
      // if (err != ERR_OK) {
      //   APP_ERROR_CODE = WARNING_RED_BLUE;
      // }
    } else {
      APP_ERROR_CODE = WARNING_RED_BLUE;
    }
  }
	pbuf_free(p);
}
///
///
void udpClientConnect(void) {
	// err_t err;
	/* 1. Create a new UDP control block  */
	upcb = udp_new();
	/* Bind the block to module's IP and port */
	ip_addr_t myIPaddr;
	IP_ADDR4(&myIPaddr, 192, 168, 100, 173);
	u16_t udpPort = 15180;
	udp_bind(upcb, &myIPaddr, udpPort);
	/* configure destination IP address and port */
	ip_addr_t DestIPaddr;
	IP_ADDR4(&DestIPaddr, 192, 168, 100, 255);
	// err= udp_connect(upcb, &DestIPaddr, udpPort);
	// if (err == ERR_OK) {
	// 	/* 2. Send message to server */
	// 	// udpClientSend();
	// 	/* 3. Set a receive callback for the upcb */
		udp_recv(upcb, udp_receive_callback, NULL);
	// } else {
  //   errorLeds(WARNING_RED_BLUE);
  //   errorLeds(WARNING_RED_BLUE);
  // }
}
///
///
void testLeds(int count) {
  if (count <= 0) {count = 1;};
  int duration = 64;  // milles
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
void handleError() {
  int e = APP_ERROR_CODE;
  APP_ERROR_CODE = 0;
  if (e >= 255) {
    APP_ERROR_COUNT++;
    if (APP_ERROR_COUNT > 3) {
      softReset();
    }
  }
  errorLeds(e);
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
  // testLeds(2);
  udpClientConnect();
  txBufPrepare();
  testLeds(1);
  HAL_Delay(64);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_buf, ADC_BUF_LEN);
  APP_ERROR_CODE = INFO3_BLUE_GREEN;
  // for (uint16_t i = 0; i < ADC_BUF_LEN; i++) {
  //     adc_buf_test[i] = i;
  // }  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED_RED
    if (APP_ERROR_CODE > 0) {
      handleError();
    }
    if (connect) {
      if (!isConnected) {
        HAL_Delay(64);
        // testLeds(2);
        errorLeds(INFO3_BLUE_GREEN);
        errorLeds(INFO3_BLUE_GREEN);
        errorLeds(INFO3_BLUE_GREEN);
        txBufClear();
        err_t err = txBufPrepare();
        if (err == ERR_OK) {
          HAL_Delay(64);
          // udp_disconnect(upcb);
          // upcb->remote_ip != NULL
          err_t err = udp_connect(upcb, &remoteAddr, remotePort);
          if (err == ERR_OK) {
            // APP_ERROR_CODE = INFO2_GREEN;
            errorLeds(INFO2_GREEN);
            connect = false;
            isConnected = true;
          } else {
            errorLeds(WARNING_RED_BLUE);
            // APP_ERROR_CODE = WARNING_RED_GREEN;
          }
        } else {
          errorLeds(WARNING_RED_GREEN);
          // APP_ERROR_CODE = WARNING_RED_BLUE;
        }
        connectCount--;
        if (connectCount < 1) {
          connectCount = 3;
          connect = false;
        }
      }
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
