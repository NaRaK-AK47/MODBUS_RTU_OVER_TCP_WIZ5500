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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "socket.h"
#include "wizchip_conf.h"
#include "W5500_Spi.h"
#include <stdio.h>
#include "dhcp.h"
#include "ModbusMaster.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define _MAIN_DEBUG_
#define _DHCP_DEBUG_
//////////////////////////////////////////////////
// Socket & Port number definition for Examples //
//////////////////////////////////////////////////
#define SOCK_DHCP       6
#define SOCK_SNTP		4
#define SOCK_TCPC 		5
#define TCPC_PORT 		3000
////////////////////////////////////////////////
// Shared Buffer Definition for Loopback test //
////////////////////////////////////////////////
uint8_t gDHCPBUF[RIP_MSG_SIZE];
///////////////////////////
// Network Configuration //
///////////////////////////s
wiz_NetInfo gWIZNETINFO = {
		.mac = { 0x80, 0x34, 0x28, 0x74, 0xA5, 0xCB },//MSB - LSB
		.ip ={ 192, 168, 1, 112 },
		.sn = { 255, 255, 255, 0 },
		.gw ={ 192, 168, 1, 1 },
		.dns = { 0, 0, 0, 0 },
		.dhcp = NETINFO_DHCP };
////////////////
// DHCP client//
////////////////
#define MY_MAX_DHCP_RETRY			2
uint8_t my_dhcp_retry = 0;
/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void Net_Conf();
static void Display_Net_Conf();
// Callback function : User defined DHCP functions
void my_ip_assign(void);
void my_ip_conflict(void);
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UWriteData(const char data)
{
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==RESET);

	huart2.Instance->DR=data;

}

int __io_putchar(int ch)
{
	UWriteData(ch);
	return ch;
}

void PHYStatusCheck(void)
{
	uint8_t tmp;

	do
	{
		printf("\r\nChecking Ethernet Cable Presence ...");
		ctlwizchip(CW_GET_PHYLINK, (void*) &tmp);

		if(tmp == PHY_LINK_OFF)
		{
			printf("NO Cable Connected!");
			HAL_Delay(1500);
		}
	}while(tmp == PHY_LINK_OFF);

	printf("Good! Cable got connected!");

}

void PrintPHYConf(void)
{
	wiz_PhyConf phyconf;

	ctlwizchip(CW_GET_PHYCONF, (void*) &phyconf);

	if(phyconf.by==PHY_CONFBY_HW)
	{
		printf("\n\rPHY Configured by Hardware Pins");
	}
	else
	{
		printf("\n\rPHY Configured by Registers");
	}

	if(phyconf.mode==PHY_MODE_AUTONEGO)
	{
		printf("\n\rAutonegotiation Enabled");
	}
	else
	{
		printf("\n\rAutonegotiation NOT Enabled");
	}

	if(phyconf.duplex==PHY_DUPLEX_FULL)
	{
		printf("\n\rDuplex Mode: Full");
	}
	else
	{
		printf("\n\rDuplex Mode: Half");
	}

	if(phyconf.speed==PHY_SPEED_10)
	{
		printf("\n\rSpeed: 10Mbps");
	}
	else
	{
		printf("\n\rSpeed: 100Mbps");
	}
}

extern uint16_t mbHoldingReg;
extern uint16_t mbInputReg;
extern uint8_t mbDisCoils;
extern uint8_t mbCoils;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t exit_dhcp_loop = 0;
  uint8_t run_user_applications = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("DHCP + DNS + SNTP W5500 Application!\r\n");
  W5500Init();

  ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
  PHYStatusCheck();
    PrintPHYConf();
    if(gWIZNETINFO.dhcp == NETINFO_DHCP)
      	{
      		DHCP_init(SOCK_DHCP, gDHCPBUF);
      		// if you want different action instead default ip assign, update, conflict.
      		// if cbfunc == 0, act as default.
      		reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);

      		run_user_applications = 0; 	// flag for running user's code
      	}
      	else
      	{
      		// Static
      #ifdef _MAIN_DEBUG_
      		Display_Net_Conf();
      #endif
      		run_user_applications = 1; 	// flag for running user's code
      	}


	  if(gWIZNETINFO.dhcp == NETINFO_DHCP && exit_dhcp_loop != 1)
	    	  	{
	    				switch(DHCP_run())
	    				{
	    					case DHCP_IP_ASSIGN:
	    					case DHCP_IP_CHANGED:
	    						/* If this block empty, act with default_ip_assign & default_ip_update */
	    						//
	    						// This example calls my_ip_assign in the two case.
	    						//
	    						// Add to ...
	    						//
	    						break;
	    					case DHCP_IP_LEASED:
	    						//
	    						// TODO: insert user's code here
	    						run_user_applications = 1;

	    						exit_dhcp_loop = 1;
	    						break;
	    					case DHCP_FAILED:
	    						/* ===== Example pseudo code =====  */
	    						// The below code can be replaced your code or omitted.
	    						// if omitted, retry to process DHCP
	    						my_dhcp_retry++;
	    						if(my_dhcp_retry > MY_MAX_DHCP_RETRY)
	    						{
	    							gWIZNETINFO.dhcp = NETINFO_STATIC;
	    							DHCP_stop();      // if restart, recall DHCP_init()

	    	#ifdef _MAIN_DEBUG_
	    							printf(">> DHCP %d Failed\r\n", my_dhcp_retry);
	    							Net_Conf();
	    							Display_Net_Conf();   // print out static netinfo to serial
	    	#endif
	    							my_dhcp_retry = 0;
	    						}
	    						break;
	    					default:
	    						break;
	    				}
	    	  	}
	//  mbMasterInit(&huart1);
uint8_t destip[4] = {192,168,1,5};
uint16_t destport = 5000;



if (socket(SOCK_TCPC, Sn_MR_TCP, TCPC_PORT, 0) != SOCK_TCPC){
	 	 printf("Socket not created\n\r");
	 	 while(1);
	  }

	  if( (ret = connect(SOCK_TCPC, destip, destport)) != SOCK_OK){
	 	 printf("Connection not established \n\r");
	 	 while(1);
	  }

	  uint8_t socket_io_mode=SOCK_IO_BLOCK;
	  ctlsocket(SOCK_TCPC, CS_SET_IOMODE , &socket_io_mode);//set blocking IO mode
	  uint16_t ret;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if(run_user_applications){
	  //// code for  modbus
	  		   if( mbMasterRead(R_COIL,1, 0, 3,1000) != READ_SUCCES ){
	  		   	 while (1);
	  		    }
	  //		   if( mbMasterRead(R_DIS_COIL,1, 0, 3,1000) != READ_SUCCES){
	  //		    	 while (1);
	  //		     }
	  //		   if( mbMasterRead(R_HOLD_REG,1, 0, 2,1000)!= READ_SUCCES){
	  //		    	 while (1);
	  //		     }
	  //		   if( mbMasterRead(R_INPUT_REG,1, 0, 2,1000)!= READ_SUCCES){
	  //		 		    	 while (1);
	  //		 		     }
	  //	  	    HAL_Delay(1000);
	  	  	}


	 switch(getSn_CR(SOCK_TCPC)){
	 case SOCK_ESTABLISHED:{
		 while(size != sentsize)
		 			{
		 				ret = send(SOCK_TCPC, buf+sentsize, size-sentsize); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
		 				if(ret < 0) // Send Error occurred (sent data length < 0)
		 				{
		 					close(sn); // socket close
		 					printf("Send not successful\n\r");
		 				}
		 				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
		 			}
		 break;
	 }
	 case SOCK_CLOSE_WAIT:{
		 if((ret=disconnect(SOCK_TCPC)) == SOCK_OK){
			 printf("%d:Socket Closed\r\n", sn);
		 }
		 break;
	 }
	 case SOCK_CLOSED:{
   	  close(SOCK_TCPC);
	break;
	 }
	 }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(run_user_applications){
//// code for  modbus
//		   if( mbMasterRead(R_COIL,1, 0, 3,1000) != READ_SUCCES ){
//		   	 while (1);
//		    }
//		   if( mbMasterRead(R_DIS_COIL,1, 0, 3,1000) != READ_SUCCES){
//		    	 while (1);
//		     }
//		   if( mbMasterRead(R_HOLD_REG,1, 0, 2,1000)!= READ_SUCCES){
//		    	 while (1);
//		     }
//		   if( mbMasterRead(R_INPUT_REG,1, 0, 2,1000)!= READ_SUCCES){
//		 		    	 while (1);
//		 		     }
//	  	    HAL_Delay(1000);
//	  	}


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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_SPI_ENABLE(&hspi1);
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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_W5500_Pin|CS_W5500_Pin|RS485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_W5500_Pin CS_W5500_Pin RS485_Pin */
  GPIO_InitStruct.Pin = RST_W5500_Pin|CS_W5500_Pin|RS485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Display_Net_Conf()
{
#ifdef _MAIN_DEBUG_
	uint8_t tmpstr[6] = {0,};
#endif

	ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);

#ifdef _MAIN_DEBUG_
	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	if(gWIZNETINFO.dhcp == NETINFO_DHCP) printf("\r\n===== %s NET CONF : DHCP =====\r\n",(char*)tmpstr);
		else printf("\r\n===== %s NET CONF : Static =====\r\n",(char*)tmpstr);
	printf(" MAC : %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
	printf(" IP : %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
	printf(" GW : %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
	printf(" SN : %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
	printf(" DNS IP : %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
	printf("=======================================\r\n");
#endif
}

/*******************************************************
 * @ brief Call back for ip assing & ip update from DHCP
 *******************************************************/
void my_ip_assign(void)
{
   getIPfromDHCP(gWIZNETINFO.ip);
   getGWfromDHCP(gWIZNETINFO.gw);
   getSNfromDHCP(gWIZNETINFO.sn);
   getDNSfromDHCP(gWIZNETINFO.dns);
   gWIZNETINFO.dhcp = NETINFO_DHCP;
   /* Network initialization */
   Net_Conf();      // apply from DHCP
#ifdef _MAIN_DEBUG_
   Display_Net_Conf();
   printf("DHCP LEASED TIME : %ld Sec.\r\n", getDHCPLeasetime());
   printf("\r\n");
#endif
}

/************************************
 * @ brief Call back for ip Conflict
 ************************************/
void my_ip_conflict(void)
{
#ifdef _MAIN_DEBUG_
	printf("CONFLICT IP from DHCP\r\n");
#endif
   //halt or reset or any...
   while(1); // this example is halt.
}

static void Net_Conf()
{
	/* wizchip netconf */
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
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
