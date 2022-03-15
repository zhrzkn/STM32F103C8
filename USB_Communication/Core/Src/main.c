/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void Flash_Copy(uint32_t OKUNACAK_adres,uint32_t YAZILACAK_adres,uint8_t KILO_byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char *data = "Hello World from USB CDC\n";
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
  /* USER CODE BEGIN 2 */

 /* Flash_Copy(0X08000000,0X08008000,5);  //32 ilk Kbyte lık kısım son 32Kbtye lık kısma kopyalanıyor*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  CDC_Transmit_FS((uint8_t *)data, strlen(data) );
  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//**************FLAS OKUMA YAZMA KISMI*******************
uint32_t Read_Flash(uint32_t adr) {
	uint32_t *Pntr = (uint32_t*) adr;
	return (*Pntr);
}
void Unlock_Flash(void)  //FLAS KILIDINI ACMA
{
	FLASH->KEYR = 0x45670123; //flash kilidini acmak i�in keyr ye KEY1 VE KEY2 GIRILIYOR
	FLASH->KEYR = 0xCDEF89AB;
}

void Lock_Flash(void)  //flas yeniden kilitleniyor
{
	FLASH->CR = 0x00000080;
}

void Erase_Flash(uint32_t adr) {
	FLASH->CR |= 0x00000002;           //PER enable
	FLASH->AR = adr;                   //silinecek adres
	FLASH->CR |= 0x00000040;           //STRT enable
	while ((FLASH->SR & 0x00000001));  //busy kontrol ediliyor
	FLASH->CR &= ~0x00000042;          //FLAS->CR ilk konumuna aliniyor(kilit hala acik)

}

void Write_Flash(uint32_t adr, uint16_t data) {
	FLASH->CR |= 0x00000001;           //PG enable
	*(__IO uint16_t*) adr = data;      //istenilen adrese istenilen data yaziliyor
	while ((FLASH->SR & 0x00000001));  //islem bitene kadar busy bekleniyor
}
void Flash_Copy(uint32_t OKUNACAK_adres,uint32_t YAZILACAK_adres,uint8_t KILO_byte) { //BURADA RAMDAKI BOLGE BILGILERI FLASH A KAYDEDILIYOR(64kb tın son 4kb tına)

	/*uint32_t data;*/
	uint32_t okunacak_adres = OKUNACAK_adres;;
	uint32_t yazilacak_adres = YAZILACAK_adres;
    uint8_t  kilo_byte=KILO_byte;

    if((OKUNACAK_adres+(1024*KILO_byte)) > YAZILACAK_adres) Error_Handler(); //HATALI YAZMAYI ONLEMEK ICIN

	int x;
	int y;

	for (y = 0; y < kilo_byte; y++) {
		Unlock_Flash();
		Erase_Flash(yazilacak_adres);           //yazlılacak adrese ait ilgili sector siliniyor

		for (x = 0; x < 256; x++) {
			//okuma kısmı
			/*data = Read_Flash(okunacak_adres);   //okunacak adres bolgesindeki 4 Byte lik bilgi okunuyor*/
			okunacak_adres += 4;

			/*yazma kısmı
			Write_Flash(yazilacak_adres, data);  //flasha ilk 2 Byte si yazılıyor
			yazilacak_adres += 2;
			data = data >> 16;
			Write_Flash(yazilacak_adres, data);  //flasha son 2 Byte si yazılıyor
			yazilacak_adres += 2;*/
		}
		Lock_Flash();
	}

}



void deinitEverything()
{
	/*
	 *   MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
	 */
	//-- reset peripherals to guarantee flawless start of user application
//	HAL_GPIO_DeInit(LED_GPIO_Port, LED_Pin);
//	HAL_GPIO_DeInit(USB_ENABLE_GPIO_Port, USB_ENABLE_Pin);
//	USBD_DeInit(&hUsbDeviceFS);

//	  __HAL_RCC_GPIOD_CLK_DISABLE();
//	  __HAL_RCC_GPIOA_CLK_DISABLE();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
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
