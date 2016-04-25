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
#include "stdio.h"
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
ADC_HandleTypeDef    ADCHandle;
ADC_ChannelConfTypeDef chConfig;


GPIO_TypeDef *COL_ADC_BANK[] = {COLOUT_0001_BANK, COLOUT_0203_BANK, COLOUT_0405_BANK, COLOUT_0607_BANK, COLOUT_0809_BANK, COLOUT_1011_BANK, COLOUT_1213_BANK, COLOUT_1415_BANK, COLOUT_1617_BANK, COLOUT_1819_BANK, COLOUT_2021_BANK, COLOUT_2223_BANK, COLOUT_2425_BANK, COLOUT_2627_BANK, COLOUT_2829_BANK, COLOUT_3031_BANK, COLOUT_3233_BANK, COLOUT_3435_BANK, COLOUT_3637_BANK, COLOUT_3839_BANK, COLOUT_4041_BANK, COLOUT_4243_BANK, COLOUT_4445_BANK, COLOUT_4647_BANK};
uint16_t COL_ADC_PIN[] = {COLOUT_0001_PIN, COLOUT_0203_PIN, COLOUT_0405_PIN, COLOUT_0607_PIN, COLOUT_0809_PIN, COLOUT_1011_PIN, COLOUT_1213_PIN, COLOUT_1415_PIN, COLOUT_1617_PIN, COLOUT_1819_PIN, COLOUT_2021_PIN, COLOUT_2223_PIN, COLOUT_2425_PIN, COLOUT_2627_PIN, COLOUT_2829_PIN, COLOUT_3031_PIN, COLOUT_3233_PIN, COLOUT_3435_PIN, COLOUT_3637_PIN, COLOUT_3839_PIN, COLOUT_4041_PIN, COLOUT_4243_PIN, COLOUT_4445_PIN, COLOUT_4647_PIN};
ADC_TypeDef *COL_ADC[] = {COLOUT_0001_ADC, COLOUT_0203_ADC, COLOUT_0405_ADC, COLOUT_0607_ADC, COLOUT_0809_ADC, COLOUT_1011_ADC, COLOUT_1213_ADC, COLOUT_1415_ADC, COLOUT_1617_ADC, COLOUT_1819_ADC, COLOUT_2021_ADC, COLOUT_2223_ADC, COLOUT_2425_ADC, COLOUT_2627_ADC, COLOUT_2829_ADC, COLOUT_3031_ADC, COLOUT_3233_ADC, COLOUT_3435_ADC, COLOUT_3637_ADC, COLOUT_3839_ADC, COLOUT_4041_ADC, COLOUT_4243_ADC, COLOUT_4445_ADC, COLOUT_4647_ADC};
uint32_t COL_ADC_CHAN[] = {COLOUT_0001_ADC_CHAN, COLOUT_0203_ADC_CHAN, COLOUT_0405_ADC_CHAN, COLOUT_0607_ADC_CHAN, COLOUT_0809_ADC_CHAN, COLOUT_1011_ADC_CHAN, COLOUT_1213_ADC_CHAN, COLOUT_1415_ADC_CHAN, COLOUT_1617_ADC_CHAN, COLOUT_1819_ADC_CHAN, COLOUT_2021_ADC_CHAN, COLOUT_2223_ADC_CHAN, COLOUT_2425_ADC_CHAN, COLOUT_2627_ADC_CHAN, COLOUT_2829_ADC_CHAN, COLOUT_3031_ADC_CHAN, COLOUT_3233_ADC_CHAN, COLOUT_3435_ADC_CHAN, COLOUT_3637_ADC_CHAN, COLOUT_3839_ADC_CHAN, COLOUT_4041_ADC_CHAN, COLOUT_4243_ADC_CHAN, COLOUT_4445_ADC_CHAN, COLOUT_4647_ADC_CHAN};

GPIO_TypeDef *MISC_BANK[] = {DINCOL_0_BANK, DINCOL_1_BANK, DINCOL_2_BANK, DINCOL_3_BANK, DINCOL_4_BANK, DINCOL_5_BANK, DINROW_0_BANK, DINROW_1_BANK, DINROW_2_BANK, DINROW_3_BANK, DINROW_4_BANK, DINROW_5_BANK, LOAD_COL_BANK, LOAD_ROW_BANK, RESET_COL_BANK, RESET_PIX_BANK, RESET_ROW_BANK, SEL_PARITY_BANK};
uint16_t MISC_PIN[] = {DINCOL_0_PIN, DINCOL_1_PIN, DINCOL_2_PIN, DINCOL_3_PIN, DINCOL_4_PIN, DINCOL_5_PIN, DINROW_0_PIN, DINROW_1_PIN, DINROW_2_PIN, DINROW_3_PIN, DINROW_4_PIN, DINROW_5_PIN, LOAD_COL_PIN, LOAD_ROW_PIN, RESET_COL_PIN, RESET_PIX_PIN, RESET_ROW_PIN, SEL_PARITY_PIN};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void load_row(uint8_t row);
static void load_col(uint8_t col);
static void config_us_delay(void);
static void config_pins(void);
static void set_col_parity(uint8_t col);
static int16_t read_pixel(uint8_t row, uint8_t col);
static void adc_init(void);
static uint16_t adc_read(uint8_t col);
static void read_image(void);

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
  adc_init();
  
  uint16_t row_val = 16;
  while (1) {
    printf("%d\n", read_pixel(0, 15));
//    read_pixel(row_val, 19);
  }
  
//  read_image();
  
  return 0;
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

static void adc_init() {
  ADCHandle.Init.Resolution = ADC_RESOLUTION_12B;
  ADCHandle.Init.ScanConvMode = ENABLE;
  ADCHandle.Init.ContinuousConvMode = DISABLE;
  ADCHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  ADCHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADCHandle.Init.NbrOfConversion = 1;
  ADCHandle.Init.DiscontinuousConvMode = ENABLE;
  ADCHandle.Init.NbrOfDiscConversion = 1;
//  ADCHandle.Init.DMAContinuousRequests = DISABLE;
  ADCHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  
  ADCHandle.Instance = ADC1;
  assert(HAL_ADC_Init(&ADCHandle) == HAL_OK);
  
  ADCHandle.Instance = ADC2;
  assert(HAL_ADC_Init(&ADCHandle) == HAL_OK);
  
  ADCHandle.Instance = ADC3;
  assert(HAL_ADC_Init(&ADCHandle) == HAL_OK);
  
//  chConfig.Channel = ADC_CHANNEL_15;
  chConfig.Rank = 1;
  chConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  chConfig.Offset = 0;
//  assert(HAL_ADC_ConfigChannel(&ADCHandle, &chConfig) == HAL_OK);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_adc;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  /* ADC Periph clock enable */
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_ADC3_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/ 
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  for (int i = 0; i < NUM_ADC_PINS; i++) {  
    GPIO_InitStruct.Pin = COL_ADC_PIN[i];
    HAL_GPIO_Init(COL_ADC_BANK[i], &GPIO_InitStruct);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  
  // Initialize misc (i.e., non-ADC) pins as GPIOs
  for (int i = 0; i < NUM_MISC_PINS; i++) {
    GPIO_InitStructure.Pin = MISC_PIN[i];
    HAL_GPIO_Init(MISC_BANK[i], &GPIO_InitStructure);
  }
}

static void read_image()
{ 
  printf("img = [");
  for (int i = 0; i < 54; i++) {
    for (int j = 0; j < NUM_ADC_PINS * 2; j++) {
      printf("%d ", read_pixel(i, j));
    }
    printf("; ");
  }
  printf("];\n");
}

static int16_t read_pixel(uint8_t row, uint8_t col)
{ 
  int16_t reset_val, integrate_val;
  
  // Disable pixel reset while configuring pixel number
  RESET_PIX_BANK->ODR |= RESET_PIX_PIN; US_DELAY(1);
  
  set_col_parity(col);
  
  // Select pixel row and column
  load_row(row);
  load_col(col);
  
  // Pulse pixel reset to begin integration
  RESET_PIX_BANK->ODR &= ~RESET_PIX_PIN; US_DELAY(1);
  RESET_PIX_BANK->ODR |= RESET_PIX_PIN; US_DELAY(1);
  reset_val = adc_read(col);
  US_DELAY(10);
  integrate_val = adc_read(col);
  
  return reset_val - integrate_val;
}

static uint16_t adc_read(uint8_t col)
{
  ADCHandle.Instance = COL_ADC[col / 2];
  chConfig.Channel = COL_ADC_CHAN[col / 2];
  assert(HAL_ADC_ConfigChannel(&ADCHandle, &chConfig) == HAL_OK);
  
  HAL_ADC_Start(&ADCHandle);
  HAL_ADC_PollForConversion(&ADCHandle, HAL_MAX_DELAY);
  return HAL_ADC_GetValue(&ADCHandle);
}

static void set_col_parity(uint8_t col)
{
  if (col % 2) {
    SEL_PARITY_BANK->ODR |= SEL_PARITY_PIN;
  } else {
    SEL_PARITY_BANK->ODR &= ~SEL_PARITY_PIN;
  }
}

static void load_row(uint8_t row)
{
  // Rising edge on reset_row to reset row select
  RESET_ROW_BANK->ODR |= RESET_ROW_PIN; US_DELAY(1);
  RESET_ROW_BANK->ODR &= ~RESET_ROW_PIN; US_DELAY(1);
  
  // Set row # on DINROW_[5:0]
  HAL_GPIO_WritePin(DINROW_0_BANK, DINROW_0_PIN, row & GPIO_PIN_0);
  HAL_GPIO_WritePin(DINROW_1_BANK, DINROW_1_PIN, row & GPIO_PIN_1);
  HAL_GPIO_WritePin(DINROW_2_BANK, DINROW_2_PIN, row & GPIO_PIN_2);
  HAL_GPIO_WritePin(DINROW_3_BANK, DINROW_3_PIN, row & GPIO_PIN_3);
  HAL_GPIO_WritePin(DINROW_4_BANK, DINROW_4_PIN, row & GPIO_PIN_4);
  HAL_GPIO_WritePin(DINROW_5_BANK, DINROW_5_PIN, row & GPIO_PIN_5);
  US_DELAY(1);
  
  // Rising edge on load_row to latch DINROW_
  LOAD_ROW_BANK->ODR |= LOAD_ROW_PIN; US_DELAY(1);
  LOAD_ROW_BANK->ODR &= ~LOAD_ROW_PIN; US_DELAY(1);
}

static void load_col(uint8_t col)
{
  // Rising edge on reset_col to reset col select
  RESET_COL_BANK->ODR |= RESET_COL_PIN; US_DELAY(1);
  RESET_COL_BANK->ODR &= ~RESET_COL_PIN; US_DELAY(1);
  
  // Set col # on DINCOL_[5:0]
  HAL_GPIO_WritePin(DINCOL_0_BANK, DINCOL_0_PIN, col & GPIO_PIN_0);
  HAL_GPIO_WritePin(DINCOL_1_BANK, DINCOL_1_PIN, col & GPIO_PIN_1);
  HAL_GPIO_WritePin(DINCOL_2_BANK, DINCOL_2_PIN, col & GPIO_PIN_2);
  HAL_GPIO_WritePin(DINCOL_3_BANK, DINCOL_3_PIN, col & GPIO_PIN_3);
  HAL_GPIO_WritePin(DINCOL_4_BANK, DINCOL_4_PIN, col & GPIO_PIN_4);
  HAL_GPIO_WritePin(DINCOL_5_BANK, DINCOL_5_PIN, col & GPIO_PIN_5);
  US_DELAY(1);
  
  // Rising edge on load_col to latch DINCOL_
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
