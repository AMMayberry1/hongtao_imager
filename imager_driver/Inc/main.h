/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.2.5
  * @date    29-January-2016 
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"
#include "stdbool.h"
#include "assert.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PIX_ROW         0
#define PIX_COL         1

#define NUM_MISC_PINS 18

#define DINCOL_0_BANK GPIOE
#define DINCOL_0_PIN  GPIO_PIN_1
#define DINCOL_1_BANK GPIOE
#define DINCOL_1_PIN  GPIO_PIN_0
#define DINCOL_2_BANK GPIOB
#define DINCOL_2_PIN  GPIO_PIN_9
#define DINCOL_3_BANK GPIOB
#define DINCOL_3_PIN  GPIO_PIN_8
#define DINCOL_4_BANK GPIOB
#define DINCOL_4_PIN  GPIO_PIN_2
#define DINCOL_5_BANK GPIOF
#define DINCOL_5_PIN  GPIO_PIN_11

#define DINROW_0_BANK GPIOC
#define DINROW_0_PIN  GPIO_PIN_15
#define DINROW_1_BANK GPIOC
#define DINROW_1_PIN  GPIO_PIN_14
#define DINROW_2_BANK GPIOC
#define DINROW_2_PIN  GPIO_PIN_13
#define DINROW_3_BANK GPIOE
#define DINROW_3_PIN  GPIO_PIN_6
#define DINROW_4_BANK GPIOE
#define DINROW_4_PIN  GPIO_PIN_5
#define DINROW_5_BANK GPIOE
#define DINROW_5_PIN  GPIO_PIN_4

#define LOAD_COL_BANK GPIOE
#define LOAD_COL_PIN  GPIO_PIN_2

#define LOAD_ROW_BANK GPIOF
#define LOAD_ROW_PIN  GPIO_PIN_0

#define RESET_COL_BANK GPIOE
#define RESET_COL_PIN  GPIO_PIN_3

#define RESET_PIX_BANK GPIOF
#define RESET_PIX_PIN  GPIO_PIN_2

#define RESET_ROW_BANK GPIOF
#define RESET_ROW_PIN  GPIO_PIN_1

#define SEL_PARITY_BANK GPIOF
#define SEL_PARITY_PIN  GPIO_PIN_12

#define NUM_ADC_PINS 24

#define COLOUT_0001_BANK GPIOF
#define COLOUT_0001_PIN GPIO_PIN_3
#define COLOUT_0001_ADC ADC3
#define COLOUT_0001_ADC_CHAN ADC_CHANNEL_9
#define COLOUT_0203_BANK GPIOF
#define COLOUT_0203_PIN GPIO_PIN_4
#define COLOUT_0203_ADC ADC3
#define COLOUT_0203_ADC_CHAN ADC_CHANNEL_14
#define COLOUT_0405_BANK GPIOF
#define COLOUT_0405_PIN GPIO_PIN_5
#define COLOUT_0405_ADC ADC3
#define COLOUT_0405_ADC_CHAN ADC_CHANNEL_15
#define COLOUT_0607_BANK GPIOF
#define COLOUT_0607_PIN GPIO_PIN_6
#define COLOUT_0607_ADC ADC3
#define COLOUT_0607_ADC_CHAN ADC_CHANNEL_4
#define COLOUT_0809_BANK GPIOF
#define COLOUT_0809_PIN GPIO_PIN_7
#define COLOUT_0809_ADC ADC3
#define COLOUT_0809_ADC_CHAN ADC_CHANNEL_5
#define COLOUT_1011_BANK GPIOF
#define COLOUT_1011_PIN GPIO_PIN_8
#define COLOUT_1011_ADC ADC3
#define COLOUT_1011_ADC_CHAN ADC_CHANNEL_6
#define COLOUT_1213_BANK GPIOF
#define COLOUT_1213_PIN GPIO_PIN_9
#define COLOUT_1213_ADC ADC3
#define COLOUT_1213_ADC_CHAN ADC_CHANNEL_7
#define COLOUT_1415_BANK GPIOF
#define COLOUT_1415_PIN GPIO_PIN_10
#define COLOUT_1415_ADC ADC3
#define COLOUT_1415_ADC_CHAN ADC_CHANNEL_8
#define COLOUT_1617_BANK GPIOC
#define COLOUT_1617_PIN GPIO_PIN_0
#define COLOUT_1617_ADC ADC3
#define COLOUT_1617_ADC_CHAN ADC_CHANNEL_10
#define COLOUT_1819_BANK GPIOC
#define COLOUT_1819_PIN GPIO_PIN_1
#define COLOUT_1819_ADC ADC3
#define COLOUT_1819_ADC_CHAN ADC_CHANNEL_11
#define COLOUT_2021_BANK GPIOC
#define COLOUT_2021_PIN GPIO_PIN_2
#define COLOUT_2021_ADC ADC3
#define COLOUT_2021_ADC_CHAN ADC_CHANNEL_12
#define COLOUT_2223_BANK GPIOC
#define COLOUT_2223_PIN GPIO_PIN_3
#define COLOUT_2223_ADC ADC3
#define COLOUT_2223_ADC_CHAN ADC_CHANNEL_13
#define COLOUT_2425_BANK GPIOC
#define COLOUT_2425_PIN GPIO_PIN_4
#define COLOUT_2425_ADC ADC1
#define COLOUT_2425_ADC_CHAN ADC_CHANNEL_14
#define COLOUT_2627_BANK GPIOC
#define COLOUT_2627_PIN GPIO_PIN_5
#define COLOUT_2627_ADC ADC1
#define COLOUT_2627_ADC_CHAN ADC_CHANNEL_15
#define COLOUT_2829_BANK GPIOA
#define COLOUT_2829_PIN GPIO_PIN_0
#define COLOUT_2829_ADC ADC3
#define COLOUT_2829_ADC_CHAN ADC_CHANNEL_0
#define COLOUT_3031_BANK GPIOA
#define COLOUT_3031_PIN GPIO_PIN_1
#define COLOUT_3031_ADC ADC3
#define COLOUT_3031_ADC_CHAN ADC_CHANNEL_1
#define COLOUT_3233_BANK GPIOA
#define COLOUT_3233_PIN GPIO_PIN_2
#define COLOUT_3233_ADC ADC3
#define COLOUT_3233_ADC_CHAN ADC_CHANNEL_2
#define COLOUT_3435_BANK GPIOA
#define COLOUT_3435_PIN GPIO_PIN_3
#define COLOUT_3435_ADC ADC3
#define COLOUT_3435_ADC_CHAN ADC_CHANNEL_3
#define COLOUT_3637_BANK GPIOA
#define COLOUT_3637_PIN GPIO_PIN_4
#define COLOUT_3637_ADC ADC1
#define COLOUT_3637_ADC_CHAN ADC_CHANNEL_4
#define COLOUT_3839_BANK GPIOA
#define COLOUT_3839_PIN GPIO_PIN_5
#define COLOUT_3839_ADC ADC1
#define COLOUT_3839_ADC_CHAN ADC_CHANNEL_5
#define COLOUT_4041_BANK GPIOA
#define COLOUT_4041_PIN GPIO_PIN_6
#define COLOUT_4041_ADC ADC1
#define COLOUT_4041_ADC_CHAN ADC_CHANNEL_6
#define COLOUT_4243_BANK GPIOA
#define COLOUT_4243_PIN GPIO_PIN_7
#define COLOUT_4243_ADC ADC1
#define COLOUT_4243_ADC_CHAN ADC_CHANNEL_7
#define COLOUT_4445_BANK GPIOB
#define COLOUT_4445_PIN GPIO_PIN_0
#define COLOUT_4445_ADC ADC1
#define COLOUT_4445_ADC_CHAN ADC_CHANNEL_8
#define COLOUT_4647_BANK GPIOB
#define COLOUT_4647_PIN GPIO_PIN_1
#define COLOUT_4647_ADC ADC1
#define COLOUT_4647_ADC_CHAN ADC_CHANNEL_9

/* Exported macro ------------------------------------------------------------*/
#define US_DELAY(val) do { \
  for (uint16_t i = 0; i < val; i++) \
    for (uint8_t j = 0; j < 8; j++) \
      asm volatile ("nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n"); \
} while(0)

/* Exported functions ------------------------------------------------------- */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
