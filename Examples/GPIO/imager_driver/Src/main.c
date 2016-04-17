/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.5
  * @date    29-January-2016 
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32f4xx_hal_gpio

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void load_row(uint8_t row);
static void load_col(uint8_t col);
static void config_us_delay(void);
static void config_pins(void);
static void set_col_parity(uint8_t col);
static void read_pixel(uint8_t row, uint8_t col);
//static void EXTILine0_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to use STM32F4xx GPIO HAL API to toggle PG13 
     IOs (connected to LED3 on STM32F429i-Discovery board) 
    in an infinite loop.
    To proceed, 3 steps are required: */

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();
  
  /* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
  config_us_delay();
  config_pins();
//  EXTILine0_Config();
  
  /* Infinite loop */
  while (1)
  {
    read_pixel(0, 1);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

// TODO: set up us delay properly

//void config_us_delay()
//{
//  __HAL_RCC_TIM5_CLK_ENABLE();
//  
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//  
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
//  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;
//  TIM_TimeBaseStructure.TIM_Period = UINT16_MAX; 
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//  
//  HAL_TIM_Cmd(TIM5, ENABLE);
//}

static void config_us_delay()
{
  TIM_HandleTypeDef    TimHandle;

  uint16_t uwPrescalerValue = 0;
  
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 1 MHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
              = ((SystemCoreClock /2) / 1 MHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 1000000) - 1;
  
  /* Set TIMx instance */
  TimHandle.Instance = TIM3;
   
  /* Initialize TIM3 peripheral as follows:
       + Period = UINT16_MAX (0xFFFF)
       + Prescaler = ((SystemCoreClock/2)/1000000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = UINT16_MAX;
  TimHandle.Init.Prescaler = uwPrescalerValue;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  
  assert(HAL_TIM_Base_Init(&TimHandle) == HAL_OK);
  assert(HAL_TIM_Base_Start(&TimHandle) == HAL_OK);
}

static void config_pins(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = 0x00FF;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = 0xFFFF;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void read_pixel(uint8_t row, uint8_t col)
{ 
  // Disable pixel reset while configuring pixel number
  RESET_PIX_BANK->ODR |= RESET_PIX_PIN; US_DELAY(1);
  
  set_col_parity(col);
  
  // Select pixel row and column
  load_row(row);
  load_col(col);
  
  // Pulse pixel reset to begin integration
  RESET_PIX_BANK->ODR &= ~RESET_PIX_PIN; US_DELAY(1);
  RESET_PIX_BANK->ODR |= RESET_PIX_PIN; US_DELAY(1);
  US_DELAY(10);
}

static void set_col_parity(uint8_t col)
{
  if (col % 2) {
    COL_PARITY_BANK1->ODR = 0;
    COL_PARITY_BANK2->ODR = 0;
  } else {
    COL_PARITY_BANK1->ODR = 0xFFFF;
    COL_PARITY_BANK1->ODR = 0xFFFF;
  }
}

static void load_row(uint8_t row)
{
  // Rising edge on reset_row to reset row select
  RESET_ROW_BANK->ODR |= RESET_ROW_PIN; US_DELAY(1);
  RESET_ROW_BANK->ODR &= ~RESET_ROW_PIN; US_DELAY(1);
  
  // Set row # on din_row[5:0]
  HAL_GPIO_WritePin(DIN_ROW1_BANK, DIN_ROW1_PIN, row & GPIO_PIN_0);
  HAL_GPIO_WritePin(DIN_ROW2_BANK, DIN_ROW2_PIN, row & GPIO_PIN_1);
  HAL_GPIO_WritePin(DIN_ROW3_BANK, DIN_ROW3_PIN, row & GPIO_PIN_2);
  HAL_GPIO_WritePin(DIN_ROW4_BANK, DIN_ROW4_PIN, row & GPIO_PIN_3);
  HAL_GPIO_WritePin(DIN_ROW5_BANK, DIN_ROW5_PIN, row & GPIO_PIN_4);
  HAL_GPIO_WritePin(DIN_ROW6_BANK, DIN_ROW6_PIN, row & GPIO_PIN_5);
  US_DELAY(1);
  
  // Rising edge on load_row to latch din_row
  LOAD_ROW_BANK->ODR |= LOAD_ROW_PIN; US_DELAY(1);
  LOAD_ROW_BANK->ODR &= ~LOAD_ROW_PIN; US_DELAY(1);
}

static void load_col(uint8_t col)
{
  // Rising edge on reset_col to reset col select
  RESET_COL_BANK->ODR |= RESET_COL_PIN; US_DELAY(1);
  RESET_COL_BANK->ODR &= ~RESET_COL_PIN; US_DELAY(1);
  
  // Set col # on din_col[5:0]
  HAL_GPIO_WritePin(DIN_COL1_BANK, DIN_COL1_PIN, col & GPIO_PIN_0);
  HAL_GPIO_WritePin(DIN_COL2_BANK, DIN_COL2_PIN, col & GPIO_PIN_1);
  HAL_GPIO_WritePin(DIN_COL3_BANK, DIN_COL3_PIN, col & GPIO_PIN_2);
  HAL_GPIO_WritePin(DIN_COL4_BANK, DIN_COL4_PIN, col & GPIO_PIN_3);
  HAL_GPIO_WritePin(DIN_COL5_BANK, DIN_COL5_PIN, col & GPIO_PIN_4);
  HAL_GPIO_WritePin(DIN_COL6_BANK, DIN_COL6_PIN, col & GPIO_PIN_5);
  US_DELAY(1);
  
  // Rising edge on load_col to latch din_col
  LOAD_COL_BANK->ODR |= LOAD_COL_PIN; US_DELAY(1);
  LOAD_COL_BANK->ODR &= ~LOAD_COL_PIN; US_DELAY(1);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
