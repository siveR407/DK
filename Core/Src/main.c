/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdio.h"
#include<math.h>

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
int16_t realRpm[5] = {0};
float desireRpm [5] = {0,0,0,0,0};
float desireRad [5] = {0,0,0,0,0};
float kp = 1;//kp ki kd ï¿½ï¿½ï¿½Ò½Ç¶È»ï¿½ï¿½ï¿½Öµï¿½ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½â»·ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½
float ki = 0.0;
float kd = 0.01;
float kp1 = 0.3;//kp1 ki1 kd1 ï¿½ï¿½ï¿½ï¿½ï¿½Ù¶È»ï¿½Ò²ï¿½ï¿½ï¿½ÇµÚ¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
float ki1 = 0.00003;   
float kd1 = 0.0001;
int pOut1[5]={0};
int iOut1[5]={0};
int dOut1[5]={0};
int pOut[5]={0};
int iOut[5]={0};
int dOut[5]={0};
float out[5]={0};
float error1[5]={0};
float integal[5] = {0};
float integal1[5] = {0};
float lastErr [5]= {0};
float lastErr1[5] = {0};
float realRad[5] ={0};
;//×ªï¿½ï¿½Êµï¿½ï¿½Ò²ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½39323
int16_t lastRad [5] = {0};
float initialRad [5] = {0};
int32_t flag[5] = {0};
int32_t n [5] = {0};
float givencurrent[5] = {0};
float error[5]={0};
int ID=0;;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CAN1_Filter_Init(void)
{
  CAN_FilterTypeDef CAN_FilterConfig;

  // WARN: CAN2?FilterBank??????14????????!!!!!
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  CAN_FilterConfig.FilterIdHigh = 0;
  CAN_FilterConfig.FilterIdLow = 0;

  CAN_FilterConfig.FilterMaskIdHigh = 0;
  CAN_FilterConfig.FilterMaskIdLow = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  CAN_FilterConfig.FilterActivation = ENABLE;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {
    Error_Handler();
  }   
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint16_t thisrad[4] = {0};
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204 )
  { 
		if(rx_header.StdId ==0x201){
			ID=1;
		}
		if(rx_header.StdId ==0x202){
			ID=2;
		}
		if(rx_header.StdId ==0x203){
			ID=3;
		}
		if(rx_header.StdId ==0x204){
			ID=4;
		};
    realRpm[ID] = (uint16_t)rx_data[2] << 8 | rx_data[3];//
    thisrad[ID] = (uint16_t)rx_data[0] << 8 | rx_data[1];//
    if (flag[ID] == 0)//
    {
      initialRad[ID] = thisrad[ID];
      desireRad[ID] = desireRad[ID] + initialRad[ID];
      lastRad[ID] = thisrad[ID];
      flag[ID] = 1;
    }

    if (thisrad[ID] - lastRad[ID] > 8191 / 2)//
    {
      n[ID]--;//
    }

    if (thisrad[ID] - lastRad[ID] < -8191 / 2)
    {
      n[ID]++;
    }
    lastRad[ID] = thisrad[ID];//ï¿½ï¿½Ò»ï¿½ÎµÄ½Ç¶ï¿½
    realRad [ID]= n[ID] * 8191 + thisrad[ID];//Êµï¿½ï¿½ï¿½ï¿½Òª×ªï¿½ï¿½ï¿½Ä½Ç¶ï¿½
  }
	
}

void CAN1_control_motor(int16_t current1,int16_t current2,int16_t current3,int16_t current4)
{
  CAN_TxHeaderTypeDef tx_header;
  givencurrent[1] = current1;
	givencurrent[2] = current2;
	givencurrent[3] = current3;
	givencurrent[4] = current4;
  uint32_t mailbox = 0;
  uint8_t tx_data[8] = {0};
  tx_header.StdId = 0x200;
  tx_header.DLC = 8;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_data[0] = current1 >> 8;
  tx_data[1] = current1 & 0xF;
	tx_data[2] = current2 >> 8;
  tx_data[3] = current2 & 0xF;
	tx_data[4] = current3 >> 8;
  tx_data[5] = current3 & 0xF;
	tx_data[6] = current4 >> 8;
  tx_data[7] = current4 & 0xF;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}

#define SBUS_DATA_SIZE      25      // 25×Ö½Ú

struct SBUS_t{
    uint16_t ch[16];                // 16¸ö×Ö½ÚÊý¾Ý
};

uint8_t sbus_rx_sta = 0;                // sbus ½ÓÊÕ×´Ì¬£¬0£ºÎ´Íê³É£¬1£ºÒÑÍê³ÉÒ»Ö¡½ÓÊÕ
uint8_t sbus_rx_buf[SBUS_DATA_SIZE];    // ½ÓÊÕsbusÊý¾Ý»º³åÇø
uint8_t connect_flag = 0;			// Á¬½ÓÊÇ·ñÕý³£
struct SBUS_t sbus;                     // SBUS ½á¹¹ÌåÊµÀý»¯

void SBUS_IT_Open(void)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)sbus_rx_buf, SBUS_DATA_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART3){
		if ((sbus_rx_buf[0] == 0x0F)&&sbus_rx_buf[24]==0x00)
		{
			sbus_rx_sta = 1;
		}
		else
		{
			sbus_rx_sta=0;
		}

		if(sbus_rx_sta==1)
        {
            sbus.ch[0] =((sbus_rx_buf[2]<<8)   + (sbus_rx_buf[1])) & 0x07ff;
            sbus.ch[1] =((sbus_rx_buf[3]<<5)   + (sbus_rx_buf[2]>>3)) & 0x07ff;
            sbus.ch[2] =((sbus_rx_buf[5]<<10)  + (sbus_rx_buf[4]<<2) + (sbus_rx_buf[3]>>6)) & 0x07ff;
            sbus.ch[3] =((sbus_rx_buf[6]<<7)   + (sbus_rx_buf[5]>>1)) & 0x07ff;
            sbus.ch[4] =((sbus_rx_buf[7]<<4)   + (sbus_rx_buf[6]>>4)) & 0x07ff;
            sbus.ch[5] =((sbus_rx_buf[9]<<9)   + (sbus_rx_buf[8]<<1) + (sbus_rx_buf[7]>>7)) & 0x07ff;
            sbus.ch[6] =((sbus_rx_buf[10]<<6)  + (sbus_rx_buf[9]>>2)) & 0x07ff;
            sbus.ch[7] =((sbus_rx_buf[11]<<3)  + (sbus_rx_buf[10]>>5)) & 0x07ff;
            sbus.ch[8] =((sbus_rx_buf[13]<<8)  + (sbus_rx_buf[12])) & 0x07ff;
            sbus.ch[9] =((sbus_rx_buf[14]<<5)  + (sbus_rx_buf[13]>>3)) & 0x07ff;
            sbus.ch[10]=((sbus_rx_buf[16]<<10) + (sbus_rx_buf[15]<<2) + (sbus_rx_buf[14]>>6)) & 0x07ff;
            sbus.ch[11]=((sbus_rx_buf[17]<<7)  + (sbus_rx_buf[16]>>1)) & 0x07ff;
            sbus.ch[12]=((sbus_rx_buf[18]<<4)  + (sbus_rx_buf[17]>>4)) & 0x07ff;
            sbus.ch[13]=((sbus_rx_buf[20]<<9)  + (sbus_rx_buf[19]<<1) + (sbus_rx_buf[18]>>7)) & 0x07ff;
            sbus.ch[14]=((sbus_rx_buf[21]<<6)  + (sbus_rx_buf[20]>>2)) & 0x07ff;
            sbus.ch[15]=((sbus_rx_buf[22]<<3)  + (sbus_rx_buf[21]>>5)) & 0x07ff;
            
			if(sbus_rx_buf[21] != 0x00)connect_flag = 0;
			else connect_flag = 1;
			
            sbus_rx_sta = 0;                        // ×¼±¸ÏÂÒ»´Î½ÓÊÕ
        }
        SBUS_IT_Open();
	}
}

void Drive_Motor(float vx,float vy ,float vz){
	desireRpm[4]=(vx+vy+vz*(1))*20;
	desireRpm[1]=(-vx+vy+vz*(1))*20;
	desireRpm[2]=-(vx+vy-vz*(1))*20;
	desireRpm[3]=-(-vx+vy-vz*(1))*20;
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN1_Filter_Init();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  uint32_t lasttick = HAL_GetTick();
	SBUS_IT_Open();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//
    /*error1[ID] = desireRad[ID] - realRad[ID];
    pOut1[ID] = kp1 * error1[ID];
    integal[ID] += ki1 * error1[ID];
    iOut1[ID] = integal[ID];
    dOut1[ID] = kd1 * (error1 - lastErr);
    lastErr1[ID] = error1[ID];
    desireRpm[ID] = pOut1[ID] + iOut1[ID] + dOut1[ID];*/
		float vx1 = (sbus.ch[3]-1000);
		float vy1 = (sbus.ch[1]-1000);
		float vz1 = (sbus.ch[0]-1000);
		Drive_Motor(vx1,vy1,vz1);
    error[ID] = desireRpm[ID] - realRpm[ID];
    pOut[ID] = kp * error[ID];
    integal1[ID] += ki * error[ID];
    iOut[ID] = integal1[ID];
    dOut [ID]= kd * (error[ID] - lastErr[ID]);
    lastErr[ID] = error[ID];
    out[ID] = pOut[ID] + iOut [ID]+ dOut[ID];
    if (out[ID] >= 16384)//
      out[ID] = 16384;
    if (out [ID]<= -16384)
      out[ID] = -16384;
    char buffer[30]={0};
    int givenInt=out[ID];
    //sprintf(buffer,"samples:%d,%d\n",(int)desireRad[1],(int)realRad[1]);
    //HAL_UART_Transmit(&huart1,(uint8_t *)buffer,20,1000);
    CAN1_control_motor(out[1],out[2],out[3],out[4]);
    HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
