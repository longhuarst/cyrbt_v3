/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEP_CH3_M1_Pin GPIO_PIN_2
#define STEP_CH3_M1_GPIO_Port GPIOE
#define STEP_CH3_M2_Pin GPIO_PIN_3
#define STEP_CH3_M2_GPIO_Port GPIOE
#define STEP_CH3_M3_Pin GPIO_PIN_4
#define STEP_CH3_M3_GPIO_Port GPIOE
#define STEP_CH3_EN_Pin GPIO_PIN_5
#define STEP_CH3_EN_GPIO_Port GPIOE
#define STEP_CH3_RESET_Pin GPIO_PIN_6
#define STEP_CH3_RESET_GPIO_Port GPIOE
#define STEP_CH3_DIR_Pin GPIO_PIN_13
#define STEP_CH3_DIR_GPIO_Port GPIOC
#define STEP_CH2_ST_Pin GPIO_PIN_0
#define STEP_CH2_ST_GPIO_Port GPIOF
#define STEP_CH2_M1_Pin GPIO_PIN_1
#define STEP_CH2_M1_GPIO_Port GPIOF
#define STEP_CH2_M2_Pin GPIO_PIN_2
#define STEP_CH2_M2_GPIO_Port GPIOF
#define STEP_CH2_M3_Pin GPIO_PIN_3
#define STEP_CH2_M3_GPIO_Port GPIOF
#define STEP_CH2_EN_Pin GPIO_PIN_4
#define STEP_CH2_EN_GPIO_Port GPIOF
#define STEP_CH2_RESET_Pin GPIO_PIN_5
#define STEP_CH2_RESET_GPIO_Port GPIOF
#define STEP_CH2_DIR_Pin GPIO_PIN_6
#define STEP_CH2_DIR_GPIO_Port GPIOF
#define STEP_CH1_ST_Pin GPIO_PIN_7
#define STEP_CH1_ST_GPIO_Port GPIOF
#define STEP_CH1_M1_Pin GPIO_PIN_8
#define STEP_CH1_M1_GPIO_Port GPIOF
#define STEP_CH1_M2_Pin GPIO_PIN_1
#define STEP_CH1_M2_GPIO_Port GPIOC
#define STEP_CH1_M3_Pin GPIO_PIN_2
#define STEP_CH1_M3_GPIO_Port GPIOC
#define STEP_CH1_EN_Pin GPIO_PIN_3
#define STEP_CH1_EN_GPIO_Port GPIOC
#define STEP_CH1_RESET_Pin GPIO_PIN_0
#define STEP_CH1_RESET_GPIO_Port GPIOA
#define STEP_CH1_DIR_Pin GPIO_PIN_1
#define STEP_CH1_DIR_GPIO_Port GPIOA
#define INPUT_CH0_Pin GPIO_PIN_4
#define INPUT_CH0_GPIO_Port GPIOA
#define INPUT_CH1_Pin GPIO_PIN_5
#define INPUT_CH1_GPIO_Port GPIOA
#define INPUT_CH2_Pin GPIO_PIN_6
#define INPUT_CH2_GPIO_Port GPIOA
#define INPUT_CH3_Pin GPIO_PIN_7
#define INPUT_CH3_GPIO_Port GPIOA
#define INPUT_CH4_Pin GPIO_PIN_4
#define INPUT_CH4_GPIO_Port GPIOC
#define INPUT_CH5_Pin GPIO_PIN_5
#define INPUT_CH5_GPIO_Port GPIOC
#define INPUT_CH6_Pin GPIO_PIN_0
#define INPUT_CH6_GPIO_Port GPIOB
#define INPUT_CH7_Pin GPIO_PIN_1
#define INPUT_CH7_GPIO_Port GPIOB
#define SPP_CA_RST_Pin GPIO_PIN_12
#define SPP_CA_RST_GPIO_Port GPIOF
#define SPP_CA_INT_Pin GPIO_PIN_13
#define SPP_CA_INT_GPIO_Port GPIOF
#define OLED_CS2_Pin GPIO_PIN_14
#define OLED_CS2_GPIO_Port GPIOF
#define OLED_FCO_Pin GPIO_PIN_15
#define OLED_FCO_GPIO_Port GPIOF
#define OLED_CS1_Pin GPIO_PIN_0
#define OLED_CS1_GPIO_Port GPIOG
#define OLED_DC_Pin GPIO_PIN_1
#define OLED_DC_GPIO_Port GPIOG
#define OUTPUT_CH0_Pin GPIO_PIN_7
#define OUTPUT_CH0_GPIO_Port GPIOE
#define OUTPUT_CH1_Pin GPIO_PIN_8
#define OUTPUT_CH1_GPIO_Port GPIOE
#define PWM_OUT_CH8_Pin GPIO_PIN_9
#define PWM_OUT_CH8_GPIO_Port GPIOE
#define OLED_RES_Pin GPIO_PIN_10
#define OLED_RES_GPIO_Port GPIOE
#define PWM_OUT_CH7_Pin GPIO_PIN_11
#define PWM_OUT_CH7_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_12
#define OLED_SDA_GPIO_Port GPIOE
#define PWM_OUT_CH6_Pin GPIO_PIN_13
#define PWM_OUT_CH6_GPIO_Port GPIOE
#define PWM_OUT_CH5_Pin GPIO_PIN_14
#define PWM_OUT_CH5_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_15
#define OLED_SCL_GPIO_Port GPIOE
#define OUTPUT_CH2_Pin GPIO_PIN_10
#define OUTPUT_CH2_GPIO_Port GPIOB
#define OUTPUT_CH3_Pin GPIO_PIN_11
#define OUTPUT_CH3_GPIO_Port GPIOB
#define OUTPUT_CH4_Pin GPIO_PIN_12
#define OUTPUT_CH4_GPIO_Port GPIOB
#define OUTPUT_CH5_Pin GPIO_PIN_13
#define OUTPUT_CH5_GPIO_Port GPIOB
#define OUTPUT_CH6_Pin GPIO_PIN_14
#define OUTPUT_CH6_GPIO_Port GPIOB
#define OUTPUT_CH7_Pin GPIO_PIN_15
#define OUTPUT_CH7_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_10
#define LED0_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOD
#define RS485_CH1_SHDN_Pin GPIO_PIN_14
#define RS485_CH1_SHDN_GPIO_Port GPIOD
#define Limit_SW_CH1_Pin GPIO_PIN_15
#define Limit_SW_CH1_GPIO_Port GPIOD
#define Limit_SW_CH2_Pin GPIO_PIN_2
#define Limit_SW_CH2_GPIO_Port GPIOG
#define Limit_SW_CH3_Pin GPIO_PIN_3
#define Limit_SW_CH3_GPIO_Port GPIOG
#define Limit_SW_CH4_Pin GPIO_PIN_4
#define Limit_SW_CH4_GPIO_Port GPIOG
#define Proximity_SW_CH1_Pin GPIO_PIN_5
#define Proximity_SW_CH1_GPIO_Port GPIOG
#define Proximity_SW_CH2_Pin GPIO_PIN_6
#define Proximity_SW_CH2_GPIO_Port GPIOG
#define Proximity_SW_CH3_Pin GPIO_PIN_7
#define Proximity_SW_CH3_GPIO_Port GPIOG
#define Proximity_SW_CH4_Pin GPIO_PIN_8
#define Proximity_SW_CH4_GPIO_Port GPIOG
#define PWM_OUT_CH4_Pin GPIO_PIN_6
#define PWM_OUT_CH4_GPIO_Port GPIOC
#define PWM_OUT_CH3_Pin GPIO_PIN_7
#define PWM_OUT_CH3_GPIO_Port GPIOC
#define PWM_OUT_CH2_Pin GPIO_PIN_8
#define PWM_OUT_CH2_GPIO_Port GPIOC
#define PWM_OUT_CH1_Pin GPIO_PIN_9
#define PWM_OUT_CH1_GPIO_Port GPIOC
#define RS485_CH1_RE_Pin GPIO_PIN_8
#define RS485_CH1_RE_GPIO_Port GPIOA
#define STEP_CH6_CLK_Pin GPIO_PIN_15
#define STEP_CH6_CLK_GPIO_Port GPIOA
#define STEP_CH6_ST_Pin GPIO_PIN_10
#define STEP_CH6_ST_GPIO_Port GPIOC
#define STEP_CH6_M1_Pin GPIO_PIN_11
#define STEP_CH6_M1_GPIO_Port GPIOC
#define STEP_CH6_M2_Pin GPIO_PIN_12
#define STEP_CH6_M2_GPIO_Port GPIOC
#define STEP_CH6_M3_Pin GPIO_PIN_0
#define STEP_CH6_M3_GPIO_Port GPIOD
#define STEP_CH6_EN_Pin GPIO_PIN_1
#define STEP_CH6_EN_GPIO_Port GPIOD
#define STEP_CH6_RESET_Pin GPIO_PIN_2
#define STEP_CH6_RESET_GPIO_Port GPIOD
#define STEP_CH6_DIR_Pin GPIO_PIN_3
#define STEP_CH6_DIR_GPIO_Port GPIOD
#define STEP_CH5_ST_Pin GPIO_PIN_4
#define STEP_CH5_ST_GPIO_Port GPIOD
#define STEP_CH5_M1_Pin GPIO_PIN_5
#define STEP_CH5_M1_GPIO_Port GPIOD
#define STEP_CH5_M2_Pin GPIO_PIN_6
#define STEP_CH5_M2_GPIO_Port GPIOD
#define STEP_CH5_M3_Pin GPIO_PIN_7
#define STEP_CH5_M3_GPIO_Port GPIOD
#define STEP_CH5_EN_Pin GPIO_PIN_9
#define STEP_CH5_EN_GPIO_Port GPIOG
#define STEP_CH5_RESET_Pin GPIO_PIN_10
#define STEP_CH5_RESET_GPIO_Port GPIOG
#define STEP_CH5_DIR_Pin GPIO_PIN_11
#define STEP_CH5_DIR_GPIO_Port GPIOG
#define STEP_CH4_ST_Pin GPIO_PIN_12
#define STEP_CH4_ST_GPIO_Port GPIOG
#define STEP_CH4_M1_Pin GPIO_PIN_13
#define STEP_CH4_M1_GPIO_Port GPIOG
#define STEP_CH4_M2_Pin GPIO_PIN_14
#define STEP_CH4_M2_GPIO_Port GPIOG
#define STEP_CH4_M3_Pin GPIO_PIN_15
#define STEP_CH4_M3_GPIO_Port GPIOG
#define STEP_CH1_CLK_Pin GPIO_PIN_3
#define STEP_CH1_CLK_GPIO_Port GPIOB
#define STEP_CH4_EN_Pin GPIO_PIN_4
#define STEP_CH4_EN_GPIO_Port GPIOB
#define STEP_CH4_RESET_Pin GPIO_PIN_5
#define STEP_CH4_RESET_GPIO_Port GPIOB
#define STEP_CH5_CLK_Pin GPIO_PIN_6
#define STEP_CH5_CLK_GPIO_Port GPIOB
#define STEP_CH4_CLK_Pin GPIO_PIN_7
#define STEP_CH4_CLK_GPIO_Port GPIOB
#define STEP_CH2_CLK_Pin GPIO_PIN_8
#define STEP_CH2_CLK_GPIO_Port GPIOB
#define STEP_CH3_CLK_Pin GPIO_PIN_9
#define STEP_CH3_CLK_GPIO_Port GPIOB
#define STEP_CH4_DIR_Pin GPIO_PIN_0
#define STEP_CH4_DIR_GPIO_Port GPIOE
#define STEP_CH3_ST_Pin GPIO_PIN_1
#define STEP_CH3_ST_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
