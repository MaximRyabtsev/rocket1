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
#include <string.h>

#define    W25_ENABLE_RESET  0x66
#define    W25_RESET  0x99
#define    W25_GET_JEDEC_ID  0x9f
#define    W25_READ  0x03
#define    W25_SECTOR_ERASE 0x20
#define    W25_BLOCK_ERASE  0xD8
#define    W25_CHIP_ERASE 0xC7
#define    W25_WRITE_DISABLE  0x04
#define    W25_WRITE_ENABLE 0x06
#define    W25_READ_STATUS_1  0x05
#define    W25_READ_STATUS_2  0x35
#define    W25_READ_STATUS_3  0x15
#define    W25_WRITE_STATUS_1 0x01
#define    W25_WRITE_STATUS_2 0x31
#define    W25_WRITE_STATUS_3 0x11
#define    W25_PAGE_PROGRAMM  0x02

#define cs_set() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct
{
  uint16_t  PageSize;
  uint32_t  PageCount;
  uint32_t  SectorSize;
  uint32_t  SectorCount;
  uint32_t  BlockSize;
  uint32_t  BlockCount;
  uint32_t  NumKB;
  uint8_t   SR1;
  uint8_t   SR2;
  uint8_t   SR3;
  uint8_t   high_cap;
  uint8_t   StatusRegister1;
  uint8_t   StatusRegister2;
  uint8_t   StatusRegister3;
}w25_info_t;

w25_info_t  w25_info;
uint8_t buf[10]; //буфер для передачи
char str1[30];
uint8_t rx_buf[1025]; // для приема


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void SPI1_Send (uint8_t *dt, uint16_t cnt) //мусорная обертка
{
  HAL_SPI_Transmit (&hspi1, dt, cnt, 5000); 
}

void SPI1_Recv (uint8_t *dt, uint16_t cnt) //мусорная обертка 2
{
  HAL_SPI_Receive (&hspi1, dt, cnt, 5000);
}

void W25_Reset (void) // сброс микры, по идее не стирает данные
{
  cs_set();
  buf[0] = W25_ENABLE_RESET;
  buf[1] = W25_RESET;
  SPI1_Send(buf, 2);
  cs_reset();
}

void W25_Read_Data(uint32_t addr, uint8_t* data, uint32_t sz) //считывание данных размером sz по адресу addr
{
  cs_set();
  buf[0] = W25_READ;
  buf[1] = (addr >> 16) & 0xFF;
  buf[2] = (addr >> 8) & 0xFF;
  buf[3] = addr & 0xFF;
  SPI1_Send(buf, 4);
  SPI1_Recv(data, sz);
  cs_reset();
}

uint32_t W25_Read_ID(void) считывание jid
{
  uint8_t dt[4];
  buf[0] = W25_GET_JEDEC_ID;
  cs_set();
  SPI1_Send(buf, 1);
  SPI1_Recv(dt,3);
  cs_reset();
  return (dt[0] << 16) | (dt[1] << 8) | dt[2];
}

void W25_Ini(void) // инициализация, передачу в uart можно убрать
{
  HAL_Delay(100); // хз зачем
  W25_Reset();
  unsigned int id = W25_Read_ID();
  w25_info.PageSize=256;
  w25_info.BlockCount=16;
    w25_info.SectorSize=0x1000;
    w25_info.SectorCount=w25_info.BlockCount*16;
    w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
    w25_info.BlockSize=w25_info.SectorSize*16;
    w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
    sprintf(str1,"Page Size: %d Bytes\r\n",(unsigned int)w25_info.PageSize);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    sprintf(str1,"Page Count: %u\r\n",(unsigned int)w25_info.PageCount);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    sprintf(str1,"Sector Size: %u Bytes\r\n",(unsigned int)w25_info.SectorSize);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    sprintf(str1,"Sector Count: %u\r\n",(unsigned int)w25_info.SectorCount);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    sprintf(str1,"Block Size: %u Bytes\r\n",(unsigned int)w25_info.BlockSize);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    sprintf(str1,"Block Count: %u\r\n",(unsigned int)w25_info.BlockCount);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    sprintf(str1,"Capacity: %u KB\r\n",(unsigned int)w25_info.NumKB);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
    HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
    sprintf(str1,"ID:0x%X\r\n",id);
    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
  HAL_Delay(100);

}

void W25_Write_Enable(void) // разрешить запись, нам это мб и не понадобится
{
	cs_set();
  buf[0] = W25_WRITE_ENABLE;
	SPI1_Send(buf, 1);
	cs_reset();
	HAL_Delay(1);
}

void W25_Write_Disable(void)// запретить запись, нам это мб и не понадобится
{
	cs_set();
  buf[0] = W25_WRITE_DISABLE;
	SPI1_Send(buf, 1);
	cs_reset();
	HAL_Delay(1);
}
	void W25_Wait_Write_End(void) // ожидание конца записи
	{
	  HAL_Delay(1); // ну что за мусор
	  cs_set();
	  buf[0] = W25_READ_STATUS_1;
	  SPI1_Send(buf, 1);
	  do{
	    SPI1_Recv(buf,1);
	    w25_info.StatusRegister1 = buf[0];
	    HAL_Delay(1);
	  }
	  while((w25_info.StatusRegister1 & 0x01) == 0x01);
	  cs_reset();
	}

	void W25_Erase_Sector(uint32_t addr) // стереть сектор начиная с addr
	{
	  W25_Wait_Write_End();
	  W25_Set_Block_Protect(0x00);
	  addr = addr * w25_info.SectorSize;
	  W25_Write_Enable();
	  cs_set();
	  buf[0] = W25_SECTOR_ERASE;
	    if(w25_info.high_cap)
	    {
	      buf[1] = (addr >> 24) & 0xFF;
	      buf[2] = (addr >> 16) & 0xFF;
	      buf[3] = (addr >> 8) & 0xFF;
	      buf[4] = addr & 0xFF;
	      SPI1_Send(buf, 5);
	    }
	    else
	    {
	      buf[1] = (addr >> 16) & 0xFF;
	      buf[2] = (addr >> 8) & 0xFF;
	      buf[3] = addr & 0xFF;
	      SPI1_Send(buf, 4);
	    }
	    cs_reset();
	    W25_Wait_Write_End();
	    W25_Write_Disable();
	    W25_Set_Block_Protect(0x0F);

	}
		      
	void W25_Erase_Block(uint32_t addr) // аналогично для блока
	{
	  W25_Wait_Write_End();
	  addr = addr * w25_info.BlockSize;
	  W25_Write_Enable();
	  cs_set();
	  buf[0] = W25_BLOCK_ERASE;
	  if(w25_info.high_cap)
	  {
	    buf[1] = (addr >> 24) & 0xFF;
	    buf[2] = (addr >> 16) & 0xFF;
	    buf[3] = (addr >> 8) & 0xFF;
	    buf[4] = addr & 0xFF;
	    SPI1_Send(buf, 5);
	  }
	  else
	  {
	    buf[1] = (addr >> 16) & 0xFF;
	    buf[2] = (addr >> 8) & 0xFF;
	    buf[3] = addr & 0xFF;
	    SPI1_Send(buf, 4);
	  }
	  cs_reset();
	  W25_Wait_Write_End();
	  HAL_Delay(1);
	}

	void W25_Erase_Chip(void) // полностью чип
	{
	  W25_Wait_Write_End();
	  W25_Write_Enable();
	  cs_set();
	  buf[0] = W25_CHIP_ERASE;
	  SPI1_Send(buf, 1);
	  cs_reset();
	  W25_Wait_Write_End();
	  HAL_Delay(10);
	}

	void W25_Write_Data(uint32_t addr, uint8_t* data, uint32_t sz) // записать data размером sz начиная с адреса addr
	{
	  W25_Wait_Write_End();
	  W25_Set_Block_Protect(0x00);
	  W25_Write_Enable();
	  cs_set();
	  buf[0] = W25_PAGE_PROGRAMM;
	  buf[1] = (addr >> 16) & 0xFF;
	      buf[2] = (addr >> 8) & 0xFF;
	      buf[3] = addr & 0xFF;
	      SPI1_Send(buf, 4);
	    SPI1_Send(data, sz);
	    cs_reset();
	      W25_Wait_Write_End();
	      W25_Write_Disable();
	      W25_Set_Block_Protect(0x0F);
	}

	void W25_Set_Block_Protect(uint8_t val) // зачем то защита одного блока
	{
	    buf[0] = 0x50;
	    cs_set();
	    SPI1_Send(buf, 1);
	    cs_reset();
	    buf[0] = W25_WRITE_STATUS_1;
	    buf[1] = ((val & 0x0F) << 2);
	    cs_set();
	    SPI1_Send(buf, 2);
	    cs_reset();
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str1[30];
	  unsigned int addr=0;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  W25_Ini();
  W25_Erase_Chip();

  for(uint16_t k=0; k<4; k++) // считываем всю флешку с выводом в uart, можно заюзать чтобы после полета считывать данные
          {
            W25_Read_Data(k*256, rx_buf, 256);
            for(uint8_t i=0; i<16; i++)
            {
              addr = k*256 + i*16;
              sprintf(str1,"%08X: ", addr);
              HAL_UART_Transmit(&huart1,(uint8_t*)str1,10,0x1000);
              for(uint8_t j=0; j<16; j++)
              {
                sprintf(str1,"%02X", rx_buf[(uint16_t)i*16 + (uint16_t)j]);
                HAL_UART_Transmit(&huart1,(uint8_t*)str1,2,0x1000);
                if(j==7) HAL_UART_Transmit(&huart1,(uint8_t*)"|",1,0x1000);
                else HAL_UART_Transmit(&huart1,(uint8_t*)" ",1,0x1000);
              }
              HAL_UART_Transmit(&huart1,(uint8_t*)"| ",1,0x1000);
              for(uint8_t j=0; j<16; j++)
              {
                if ((rx_buf[(uint16_t)i*16 + (uint16_t)j] == 0x0A) ||
                    (rx_buf[(uint16_t)i*16 + (uint16_t)j] == 0x0D)) sprintf(str1," ");
                else sprintf(str1,"%c", (char) rx_buf[(uint16_t)i*16 + (uint16_t)j]);
                HAL_UART_Transmit(&huart1,(uint8_t*)str1,1,0x1000);
              }
              HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
            }
            HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
          }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
