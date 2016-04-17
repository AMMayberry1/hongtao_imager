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

#define RESET_PIX_BANK  GPIOA
#define RESET_PIX_PIN   GPIO_PIN_6

#define RESET_ROW_BANK  GPIOA
#define RESET_ROW_PIN   GPIO_PIN_0

#define LOAD_ROW_BANK   GPIOA
#define LOAD_ROW_PIN    GPIO_PIN_7

// PA1 and PA2 aren't available for some reason
#define DIN_ROW1_BANK   GPIOB
#define DIN_ROW1_PIN    GPIO_PIN_8
#define DIN_ROW2_BANK   GPIOB
#define DIN_ROW2_PIN    GPIO_PIN_9
#define DIN_ROW3_BANK   GPIOA
#define DIN_ROW3_PIN    GPIO_PIN_3
#define DIN_ROW4_BANK   GPIOA
#define DIN_ROW4_PIN    GPIO_PIN_4
#define DIN_ROW5_BANK   GPIOA
#define DIN_ROW5_PIN    GPIO_PIN_5
#define DIN_ROW6_BANK    GPIOB
#define DIN_ROW6_PIN   GPIO_PIN_10

#define RESET_COL_BANK  GPIOB
#define RESET_COL_PIN   GPIO_PIN_0

#define LOAD_COL_BANK   GPIOB
#define LOAD_COL_PIN    GPIO_PIN_7

#define DIN_COL1_BANK   GPIOB
#define DIN_COL1_PIN    GPIO_PIN_1
#define DIN_COL2_BANK   GPIOB
#define DIN_COL2_PIN    GPIO_PIN_2
#define DIN_COL3_BANK   GPIOB
#define DIN_COL3_PIN    GPIO_PIN_3
#define DIN_COL4_BANK   GPIOB
#define DIN_COL4_PIN    GPIO_PIN_4
#define DIN_COL5_BANK   GPIOB
#define DIN_COL5_PIN    GPIO_PIN_5
#define DIN_COL6_BANK   GPIOB
#define DIN_COL6_PIN    GPIO_PIN_11

#define COL_PARITY_BANK1  GPIOC
#define COL_PARITY_BANK2  GPIOD

/* Exported macro ------------------------------------------------------------*/
#define US_DELAY(val) do { \
  for (uint16_t i = 0; i < val; i++) \
    for (uint8_t j = 0; j < 8; j++) \
      asm volatile ("nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n" "nop\n"); \
} while(0)

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
