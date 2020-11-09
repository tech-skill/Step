/** 
  ******************************************************************************
  * @file    stm32f4xx_nucleo_ihm14a1.h
  * @author  STM
  * @version V1.0.0
  * @date    August 7th, 2017
  * @brief   Header for BSP driver for x-nucleo-ihm14a1 Nucleo extension board 
  *  (based on STSPIN820)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef __STM32F4XX_NUCLEO_IHM14A1_H
#define __STM32F4XX_NUCLEO_IHM14A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STM32F4XX_NUCLEO_IHM14A1
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup IHM14A1_Exported_Constants IHM14A1 Exported Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_STM32F4XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_STM32F4XX_NUCLEO Constants For STM32F4XX NUCLEO
* @{
*/  
   
/* Fault reporting------------------------------------------------------------*/ 
   
/// Interrupt line used for EN FAULT (FLAG)
#define BSP_MOTOR_CONTROL_BOARD_IRQn_EN_AND_FAULT                (EXTI15_10_IRQn)
   
/// Flag interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_PRIORITY_EN_AND_FAULT            (1)

/* reference voltage REF generation ------------------------------------------*/ 
    
/// Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_PWM_REF                          (TIM3)
   
/// Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_CHAN_PWM_REF                     (TIM_CHANNEL_2)

/// HAL Active Channel Timer used for REF
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_PWM_REF             (HAL_TIM_ACTIVE_CHANNEL_2)
   
/// Timer Clock Enable for REF
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_PWM_REF()          __TIM3_CLK_ENABLE()

/// Timer Clock Disable for REF
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_PWM_REF()         __TIM3_CLK_DISABLE()

/// REF GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AF_PWM_REF                       (GPIO_AF2_TIM3)
 
/* step clock ----------------------------------------------------------------*/   
   
/// Timer used for step clock
#define BSP_MOTOR_CONTROL_BOARD_TIM_STCK                         (TIM2)

/// Timer output for step clock        
#define BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK                  (TIMER_MAIN_OUTPUT)

/// Channel Timer used for step clock
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK                    (TIM_CHANNEL_2)
   
/// HAL Active Channel Timer used for step clock
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIM_STCK            (HAL_TIM_ACTIVE_CHANNEL_2)
   
/// Timer Clock Enable for step clock
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_TIM_STCK()         __TIM2_CLK_ENABLE()
   
/// Timer Clock Disable for step clock
#define __BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_TIM_STCK()        __TIM2_CLK_DISABLE()
   
/// Step clock global interrupt
#define BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK                    (TIM2_IRQn)

/// Step clock global interrupt priority
#define BSP_MOTOR_CONTROL_BOARD_PRIORITY_TIM_STCK                (BSP_MOTOR_CONTROL_BOARD_PRIORITY_EN_AND_FAULT + 1)

/// step clock GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AF_TIM_STCK                      (GPIO_AF1_TIM2)
   
/**
  * @}
  */

/******************************************************************************/
/* Independent platform definitions                                          */
/******************************************************************************/

/** @defgroup Constants_For_All_Nucleo_Platforms Constants For All Nucleo Platforms
* @{
*/
   
/// Timer with a main output
#define TIMER_MAIN_OUTPUT           (0)
/// Timer with a complementary output
#define TIMER_COMPLEMENTARY_OUTPUT  (1)
/// Timer without output
#define TIMER_NO_OUTPUT             (2)

/// GPIO Pin used for the STSPIN820 stby reset pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_STBY_RESET                (GPIO_PIN_9)
/// GPIO port used for the STSPIN820 reset pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_STBY_RESET               (GPIOA)

/// GPIO Pin used for the STSPIN820 en fault pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT              (GPIO_PIN_10)
/// GPIO port used for the STSPIN820 en fault pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT             (GPIOA)

/// GPIO Pin used for the STSPIN820 ref pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF                   (GPIO_PIN_7)
/// GPIO Port used for the STSPIN820 ref pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF                  (GPIOC)

/// GPIO Pin used for the STSPIN820 mode1 pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_MODE1                     (GPIO_PIN_5)
/// GPIO port used for the STSPIN820 mode1 pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_MODE1                    (GPIOA)

/// GPIO Pin used for the STSPIN820 mode2 pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_MODE2                     (GPIO_PIN_6)
/// GPIO port used for the STSPIN820 mode2 pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_MODE2                    (GPIOA)

/// GPIO Pin used for the STSPIN820 mode3 pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_MODE3                     (GPIO_PIN_7)
/// GPIO port used for the STSPIN820 mode3 pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_MODE3                    (GPIOA)

/// GPIO Pin used for the STSPIN820 decay pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_DECAY                     (GPIO_PIN_4)
/// GPIO port used for the STSPIN820 decay pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_DECAY                    (GPIOB)
   
/// GPIO Pin used for the STSPIN820 step clock pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK                  (GPIO_PIN_3)
//#define BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK                (GPIO_PIN_10)
/// GPIO Port used for the STSPIN820 step clock pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK                 (GPIOB)
//#define BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK               (GPIOB)

/// GPIO Pin used for the STSPIN820 direction pin
#define BSP_MOTOR_CONTROL_BOARD_PIN_DIR                       (GPIO_PIN_8)
//#define BSP_MOTOR_CONTROL_BOARD_PIN_DIR                     (GPIO_PIN_5)
/// GPIO port used for the STSPIN820 direction pin
#define BSP_MOTOR_CONTROL_BOARD_PORT_DIR                      (GPIOA)
//#define BSP_MOTOR_CONTROL_BOARD_PORT_DIR                    (GPIOB)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4XX_NUCLEO_IHM14A1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
