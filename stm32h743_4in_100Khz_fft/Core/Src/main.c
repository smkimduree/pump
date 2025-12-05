/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fft.h"
#include "modbus.h"
#include "w25qxx_qspi.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t ucAdcSuccess = 0;
uint8_t ucAdcCurrIndex = 0;
uint8_t ucAdcConvIndex = 0;
uint16_t usAdcReadCnt = 0;
#define DMA_BUFFER __attribute__((section(".dma_buffer")))
//#define DMA_DATA_LEN 16
#define DMA_DATA_LEN 8192
DMA_BUFFER uint32_t unAdcBuff[(DMA_DATA_LEN+100)];
//__IO uint32_t unAdcBuff[8];
int16_t usAdcReadBuff1[2][FFT_LENGTH];
int16_t usAdcReadBuff2[2][FFT_LENGTH];
int16_t usAdcReadBuff3[2][FFT_LENGTH];
int16_t usAdcReadBuff4[2][FFT_LENGTH];
uint16_t usAdcFuncFlag = 0;
uint32_t unAdcAddBuff[4]={0,0,0,0};
int32_t unAdcRmsBuff[4]={8192,8192,8192,8192};

extern void FFT_Proc(uint8_t ch,uint16_t *src);
extern volatile float fMagPowerBuff[4][(FFT_LENGTH/2)];
extern uint16_t usDetectBuff[4][5];

extern uint8_t ucUart3RxCnt;
extern uint8_t ucUart3RxBuff[];
extern uint8_t ucUart3TxSucc;
#define UART_SEND_MAX 10240
uint8_t ucUart1TxBuff[UART_SEND_MAX];
uint8_t ucUart1TxQueBuff[UART_SEND_MAX];
uint8_t ucUsbTxBuff[UART_SEND_MAX];
uint16_t usUsbSendCnt = 0;
uint16_t usUart1SendCnt = 0;
uint16_t usUart1SendTail = 0;
uint16_t usUart1SendHead = 0;
uint8_t ucUart1Sending = 0;

uint8_t ucDipSwVal = 0;
uint8_t ucSD_Mount = 0;
uint8_t ucSD_Open = 0;
uint8_t ucSD_FileNum = 0;
uint32_t unFileSaveCnt = 0;
char SD_Path[10];
int16_t ausAdcReadVal[DMA_DATA_LEN];

int16_t ssParaBuff[2048];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t GPIO_DIP_SW_Input(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void MODBUS_RX_PROCESS(void);

void Uart1SendQueue(uint8_t * src, uint16_t len)
{
	int i;
	for(i=0;i<len;i++)
	{
		if(usUart1SendCnt < UART_SEND_MAX)
		{
			ucUart1TxQueBuff[usUart1SendHead] = src[i];
			usUart1SendHead++;
			usUart1SendHead %= UART_SEND_MAX;
			usUart1SendCnt++;
		}
		else
		{
			ucUart1TxQueBuff[usUart1SendHead] = src[i];
			usUart1SendHead++;
			usUart1SendHead %= UART_SEND_MAX;
			usUart1SendTail = usUart1SendHead;
		}
	}
}

void Uart1PutQueue(uint8_t data)
{
	if(usUart1SendCnt < UART_SEND_MAX)
	{
		ucUart1TxQueBuff[usUart1SendHead] = data;
		usUart1SendHead++;
		usUart1SendHead %= UART_SEND_MAX;
		usUart1SendCnt++;
	}
	else
	{
		ucUart1TxQueBuff[usUart1SendHead] = data;
		usUart1SendHead++;
		usUart1SendHead %= UART_SEND_MAX;
		usUart1SendTail = usUart1SendHead;
	}
}

int Uart1GetQueue()
{
	uint8_t ch;
	if(usUart1SendCnt > 0)
	{
		ch = ucUart1TxQueBuff[usUart1SendTail];
		usUart1SendTail++;
		usUart1SendTail %= UART_SEND_MAX;
		usUart1SendCnt--;
		return (int)ch;
	}
	else
	{
		usUart1SendTail = 0;
		usUart1SendHead = 0;
		return -1;
	}
}

int Uart1ReadQueue(uint8_t *dst)
{
	uint8_t ch;
	int i, res;
	res = 0;
	while(usUart1SendCnt > 0)
	{
		dst[res++] = ucUart1TxQueBuff[usUart1SendTail];
		usUart1SendTail++;
		usUart1SendTail %= UART_SEND_MAX;
		usUart1SendCnt--;
	}
	usUart1SendTail = 0;
	usUart1SendHead = 0;
	return res;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int nPowerVal = 0;
	int nSendCnt = 0;
	int nsend_len=0;
	int i,j;
	uint16_t us_res;
	int ret;
	FIL MyFile;
	uint32_t byteswritten;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  BSP_SD_Init();
//  HAL_TIM_Base_Start(&htim6);
  // calibrate ADC for better accuracy and start it w/ interrupt
    if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
                  Error_Handler();

//    if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
//                  Error_Handler();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)unAdcBuff, DMA_DATA_LEN);
//    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)unAdcBuff, 16);
    // start pwm generation
    if(HAL_TIM_Base_Start(&htim6) != HAL_OK)   Error_Handler();

    HAL_TIM_Base_Start(&htim7);
/*
    w25qxx_Init();
    w25qxx_GetID();
    W25qxx_Read((uint8_t*)ssParaBuff,0,100);
   	if((ssParaBuff[0]!=0x55AA) || (ssParaBuff[49]!=0x55AA))
   	{
   		ucUart1TxBuff[0] = 'C';
   		ucUart1TxBuff[1] = 'W';
   		us_res = (ssParaBuff[0]&0xF000)>>12;
   		ucUart1TxBuff[2] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[0]&0x0F00)>>8;
   		ucUart1TxBuff[3] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[0]&0x00F0)>>4;
   		ucUart1TxBuff[4] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[0]&0x000F)>>0;
   		ucUart1TxBuff[5] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[49]&0xF000)>>12;
   		ucUart1TxBuff[6] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[49]&0x0F00)>>8;
   		ucUart1TxBuff[7] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[49]&0x00F0)>>4;
   		ucUart1TxBuff[8] = (us_res>9)?us_res-10+'A':us_res+'0';
   		us_res = (ssParaBuff[49]&0x000F)>>0;
   		ucUart1TxBuff[9] = (us_res>9)?us_res-10+'A':us_res+'0';

   		HAL_UART_Transmit_IT(&huart1, &ucUart1TxBuff[0], 10);
   		W25qxx_EraseSector(0);
//		W25qxx_EraseChip(); // approx 13 second to execute
		ssParaBuff[0] = 0x55AA;
		ssParaBuff[1] = 10000;
		ssParaBuff[2] = 5000;
		ssParaBuff[3] = 3000;
		ssParaBuff[4] = 2000;
		ssParaBuff[5] = 10000;
		ssParaBuff[6] = 5000;
		ssParaBuff[7] = 3000;
		ssParaBuff[8] = 2000;
		ssParaBuff[9] = 10000;
		ssParaBuff[10] = 5000;
		ssParaBuff[11] = 3000;
		ssParaBuff[12] = 2000;
		ssParaBuff[13] = 10000;
		ssParaBuff[14] = 5000;
		ssParaBuff[15] = 3000;
		ssParaBuff[16] = 2000;
		ssParaBuff[49] = 0x55AA;
		W25qxx_Write((uint8_t*)ssParaBuff, 0, 100);
   	}
*/

    /* Enable D-Cache---------------------------------------------------------*/
 //   SCB_EnableICache();
    SCB_EnableDCache();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ucUart3RxCnt = 0;
  RS485_RD_MODE;
  HAL_UART_Receive_IT(&huart3, &ucUart3RxBuff[ucUart3RxCnt], 1);
  FFT_Buff_Init();
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
  while (1)
  {
	  ucDipSwVal = GPIO_DIP_SW_Input();

	  FFT_Timer_Proc();
//	  if(ucAdcSuccess)
	  if(usAdcFuncFlag)
	  {
		  if(((ucDipSwVal & 0x08) != 0) || (ucSD_Mount == 0))  ret = f_mount(&SDFatFS, "0:", 0);

		  if(ret == 0)ucSD_Mount = 1;

		  if(((ucDipSwVal & 0x08) == 0) || (ucSD_Mount != 0))
		  {
			  if(ucSD_Open) f_close(&MyFile);
			  ret = f_mount(0, "0:", 0);
			  ucSD_Open = 0;
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		  }

		  if(((ucDipSwVal & 0x08) == 0) || (ucSD_Mount == 0))
		  {
	//		  ucAdcSuccess = 0;
	 //		  SCB_EnableDCache();
			  if((usAdcFuncFlag & 0x0003) != 0)
			  {
				  usAdcFuncFlag &= 0x00FC;
				  FFT_Proc(0,&usAdcReadBuff1[ucAdcConvIndex][0]);
			  }
			  else if((usAdcFuncFlag & 0x000C) != 0)
			  {
				  usAdcFuncFlag &= 0x00F3;
	//			  FFT_Proc(1,&usAdcReadBuff2[ucAdcConvIndex][0]);
			  }
			  else if((usAdcFuncFlag & 0x0030) != 0)
			  {
				  usAdcFuncFlag &= 0x00CF;
	//			  FFT_Proc(2,&usAdcReadBuff3[ucAdcConvIndex][0]);
			  }
			  else if((usAdcFuncFlag & 0x00C0) != 0)
			  {
				  usAdcFuncFlag &= 0x003F;
	//				FFT_Proc(3,&usAdcReadBuff4[ucAdcConvIndex][0]);
			  }
	//		  SCB_DisableDCache();
		  }
		  else
		  {
			  usAdcFuncFlag = 0;
			  ucAdcSuccess = 0;
			  if(ucSD_Open == 0)
			  {
				  ucSD_FileNum = ucDipSwVal & 0x07;
				  sprintf(SD_Path, "F%d.bin",ucSD_FileNum);
				  ret = f_open(&MyFile,SD_Path,FA_CREATE_ALWAYS | FA_WRITE);
				  if(ret == 0)
				  {
					  ucSD_Open = 1;
					  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
				  }
				  unFileSaveCnt = 0;
			  }
			  else
			  {
				  if(unFileSaveCnt < 10000)
				  {
					  ret = f_write(&MyFile,(uint8_t *)&ausAdcReadVal[0],DMA_DATA_LEN*2,(void*)&byteswritten);
					  unFileSaveCnt++;
					  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
					  if(unFileSaveCnt == 10000)
					  {
						  f_close(&MyFile);
						  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
					  }
				  }
			  }
		  }
#if 0
		  nSendCnt = 0;
		  ucUsbTxBuff[nSendCnt++] = 0x55;
		  ucUsbTxBuff[nSendCnt++] = ':';

//		  for(i=0;i<usMagPowerPeakDetectCnt[0];i++)
		  for(i=0;i<1024;i++)
		  {
//			  nPowerVal = (int) fMagPowerBuff[0][i]/100;
			  nPowerVal = usAdcReadBuff1[ucAdcConvIndex][i];
//			  nPowerVal = (int)usMagPowerPeakDetectBuff[0][i]*10;
			  nPowerVal %= 10000;
			  ucUsbTxBuff[nSendCnt++] = (nPowerVal/1000) + '0';
			  nPowerVal %= 1000;
			  ucUsbTxBuff[nSendCnt++] = (nPowerVal/100) + '0';
			  nPowerVal %= 100;
			  ucUsbTxBuff[nSendCnt++] = (nPowerVal/10) + '0';
			  ucUsbTxBuff[nSendCnt++] = (nPowerVal%10) + '0';
			  ucUsbTxBuff[nSendCnt++] = ',';

//			  ucUsbTxBuff[nSendCnt++] = (nPowerVal>>16) & 0xFF;
//			  ucUsbTxBuff[nSendCnt++] = (nPowerVal>>8) & 0xFF;
//			  ucUsbTxBuff[nSendCnt++] = (nPowerVal>>0) & 0xFF;
		  }
		  ucUsbTxBuff[nSendCnt++] = 0x0d;
		  ucUsbTxBuff[nSendCnt++] = 0x0a;
//		  ucUsbTxBuff[nSendCnt++] = 0x00;
		  Uart1SendQueue(&ucUsbTxBuff[0],nSendCnt);
#endif
	  }
	  MODBUS_RX_PROCESS();

		if ((usUart1SendCnt>0) && (ucUart1Sending==0))
	  {
		  nsend_len = Uart1ReadQueue(&ucUart1TxBuff[0]);
		  HAL_UART_Transmit_IT(&huart1, &ucUart1TxBuff[0], nsend_len);
		  ucUart1Sending = 1;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 4;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_5_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 119;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2399;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int32_t nAdcReadIndex = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	int i,j;
//	if(hadc == ADC1)
	if(ucAdcSuccess == 0)
//	if((ucAdcSuccess == 0) && (ucUart1Sending == 0))
	{
//		unAdcBuff[0] = HAL_ADC_GetValue(&hadc1);
#if 1
		SCB_InvalidateDCache_by_Addr((uint32_t *) &unAdcBuff[0], DMA_DATA_LEN);
		usAdcReadCnt=0;
		j=0;
		if(ucSD_Open)
		{
			for(i=0;i<(DMA_DATA_LEN);i++) ausAdcReadVal[i] = (int16_t)unAdcBuff[i];
		}
		else
		{
			for(i=0;i<(DMA_DATA_LEN/4);i++)
			{
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[j++]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[j++]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[j++]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[j++]);
				usAdcReadCnt++;
			}
		}
		ucAdcSuccess = 1;
		usAdcFuncFlag = 0x00FF;
		ucAdcConvIndex = ucAdcCurrIndex;
		if(ucAdcCurrIndex==0)ucAdcCurrIndex=1;
		else ucAdcCurrIndex = 0;
#else
		if(usAdcReadCnt < FFT_LENGTH)
		{
			SCB_InvalidateDCache_by_Addr((uint32_t *) &unAdcBuff[0], 16);
			if(nAdcReadIndex == 0)
			{
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[0]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[1]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[2]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[3]);
				usAdcReadCnt++;
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[4]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[5]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[6]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[7]);
				usAdcReadCnt++;
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[8]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[9]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[10]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[11]);
				usAdcReadCnt++;
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[12]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[13]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[14]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] = (int16_t)((int)unAdcBuff[15]);
				usAdcReadCnt++;
			}
			else
			{
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[0]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[1]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[2]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[3]);
				usAdcReadCnt++;
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[4]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[5]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[6]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[7]);
				usAdcReadCnt++;
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[8]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[9]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[10]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[11]);
				usAdcReadCnt++;
				usAdcReadBuff1[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[12]);
				usAdcReadBuff2[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[13]);
				usAdcReadBuff3[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[14]);
				usAdcReadBuff4[ucAdcCurrIndex][usAdcReadCnt] += (int16_t)((int)unAdcBuff[15]);
				usAdcReadCnt++;
			}
		}

		if (usAdcReadCnt >= FFT_LENGTH)
		{
			nAdcReadIndex++;
			usAdcReadCnt = 0;
			if(nAdcReadIndex >= 4)
			{
				nAdcReadIndex = 0;
				ucAdcSuccess = 1;
				usAdcFuncFlag = 0x00FF;
				ucAdcConvIndex = ucAdcCurrIndex;
				if(ucAdcCurrIndex==0)ucAdcCurrIndex=1;
				else ucAdcCurrIndex = 0;
			}
		}
#endif
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
    	usUart3RxIdleMs = 4;
        if(ucUart3RxCnt<UART_RX_BUF_MAX)
        {
        	ucUart3RxCnt++;
            HAL_UART_Receive_IT(&huart3, &ucUart3RxBuff[ucUart3RxCnt], 1);
        }
    }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		ucUart1Sending = 0;
		ucAdcSuccess = 0;
	}
	else if(huart->Instance == USART3)
	{
		ucUart3TxSucc = 1;
		usUart3TxIdleMs = 4;
		RS485_RD_MODE;
	}
}

void HAL_SYSTICK_Callback()
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == TIM7)
	{

	}
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
