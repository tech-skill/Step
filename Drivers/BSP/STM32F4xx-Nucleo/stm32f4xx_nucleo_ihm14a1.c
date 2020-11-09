/**
  ******************************************************************************
  * @file    stm32f4xx_nucleo_ihm14a1.c
  * @author  STM
  * @version V1.0.0
  * @date    August 7th, 2017
  * @brief   BSP driver for x-nucleo-ihm14a1 Nucleo extension board 
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
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo_ihm14a1.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32F4XX_NUCLEO_IHM14A1 STM32F4XX NUCLEO IHM14A1
  * @{
  */   
    
/* Private constants ---------------------------------------------------------*/    

/** @defgroup IHM14A1_Private_Constants IHM14A1 Private Constants
  * @{
  */   
    
/// Timer Prescaler for step clocks
#define TIMER_PRESCALER (64)
    
/// MCU wait time after power bridges are enabled
#define BRIDGE_TURN_ON_DELAY                                     (10)

/// RC Filtering delay on the PWM
#define PWM_FILTER_TIME_CONSTANT                                 (5)

/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/

/** @defgroup IHM14A1_Board_Private_Variables IHM14A1 Board Private Variables
  * @{
  */
/// Timer handler for step clock
TIM_HandleTypeDef hTimerStepClock;

/// Step clock compare value
volatile uint32_t ccrValue;

/// Timer handler for REF
TIM_HandleTypeDef hTimerPwm;

/**
  * @}
  */ 

/** @defgroup IHM14A1_Board_Private_Function_Prototypes IHM14A1 Board Private Function Prototypes
  * @{
  */   
//Delay of the requested number of milliseconds   
void Stspin820_Board_Delay(uint32_t delay);
//Disable Irq
void Stspin820_Board_DisableIrq(void);
//Enable Irq
void Stspin820_Board_EnableIrq(void);
//Read the status of the Pin DECAY
uint8_t Stspin820_Board_GetDecayGpio(void);
//Initialise GPIOs used for STSPIN820
void Stspin820_Board_GpioInit(void);
//Start step clock
void Stspin820_Board_TimStckStart(void);
//Set step clock frequency and start it
void Stspin820_Board_TimStckSetFreq(uint16_t newFreq);
//Sets the frequency and duty cycle of PWM used for the reference voltage 
//generation
void Stspin820_Board_PwmRefSetFreqAndDutyCycle(uint32_t newFreq,uint8_t dutyCycle);
//Init the timer
void Stspin820_Board_TimStckInit(void);
//DeInit the timer
void Stspin820_Board_TimStckDeInit(void);
//Stop the timer
uint8_t Stspin820_Board_TimStckStop(volatile uint8_t *pToggleOdd);
//Init the reference voltage pwm
void Stspin820_Board_PwmRefInit(void);
//Start the reference voltage pwm
void Stspin820_Board_PwmRefStart(void);
//Stop the reference voltage pwm
void Stspin820_Board_PwmRefStop(void);
//Set the STSPIN820 STBY RESET pin and leave low consumption mode
void Stspin820_Board_ReleaseReset(void);
//Reset the STSPIN820 STBY RESET pin and force low consumption mode
void Stspin820_Board_Reset(void);
//Set the state of the direction GPIO
void Stspin820_Board_SetDirectionGpio(uint8_t gpioState);
//Set the state of the decay GPIO
void Stspin820_Board_SetDecayGpio(uint8_t gpioState);
//Enable the power bridges (leave the output bridges HiZ)
void Stspin820_Board_Enable(void);
//Disable the power bridges (force the output bridges HiZ)
void Stspin820_Board_Disable(void);
//Get the EN FAULT pin state
uint32_t Stspin820_Board_EN_AND_FAULT_PIN_GetState(void);
//Select the STSPIN820 mode1, mode2 and mode3 pins levels
uint8_t Stspin820_Board_SetModePins(uint8_t modePin1Level,\
  uint8_t modePin2Level,\
  uint8_t modePin3Level);
//Select Full Step mode
void Stspin820_Board_SetFullStep(void);
//Step clock compare value initialization
void Stspin820_Board_TimStckCompareInit(void);
/**
  * @}
  */

/** @defgroup IHM14A1_Board_Private_Functions IHM14A1 Board Private Functions
  * @{
  */   

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void Stspin820_Board_Delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/******************************************************//**
 * @brief This function disable the interruptions
 * @retval None
 **********************************************************/
void Stspin820_Board_DisableIrq(void)
{
  __disable_irq();
}

/******************************************************//**
 * @brief This function enable the interruptions
 * @retval None
 **********************************************************/
void Stspin820_Board_EnableIrq(void)
{
  __enable_irq();
}

/******************************************************//**
 * @brief  Read the digital level of GPIO decay pin
 * @retval Value of the decay pin
 **********************************************************/
uint8_t Stspin820_Board_GetDecayGpio(void)
{
    uint8_t statusPin;
    
    if(HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PORT_DECAY,\
      BSP_MOTOR_CONTROL_BOARD_PIN_DECAY) != GPIO_PIN_RESET)
    {
      statusPin = 1;
    }
    else
    {
      statusPin = 0;
    }
    
    return statusPin;
    
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the STSPIN820
 * @retval None
  **********************************************************/
void Stspin820_Board_GpioInit(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();  

  /* Configure STSPIN820 - EN pin --------------------------------------------*/
  /* When this pin is set low, it is configured just before as                */
  /* GPIO_MODE_OUTPUT_PP with GPIO_NOPULL                                     */
  /* When this pin is set high, it is just after configured for FAULT         */
  /* as GPIO_MODE_IT_FALLING with GPIO_PULLUP                                 */
  Stspin820_Board_Disable();
  
  /* Set Priority of External Line Interrupt used for the FAULT interrupt*/ 
  HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_IRQn_EN_AND_FAULT,\
    BSP_MOTOR_CONTROL_BOARD_PRIORITY_EN_AND_FAULT,\
    0);
    
  /* Enable the External Line Interrupt used for the FAULT interrupt*/
  HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_EN_AND_FAULT);  

  /* Configure STSPIN820 - MODE1 pin -----------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_MODE1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_MODE1, &GPIO_InitStruct);

  /* Configure STSPIN820 - MODE2 pin -----------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_MODE2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_MODE2, &GPIO_InitStruct);

  /* Configure STSPIN820 - MODE3 pin -----------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_MODE3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_MODE3, &GPIO_InitStruct);
  
  /* Configure STSPIN820 - DIR pin -------------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_DIR;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_DIR, &GPIO_InitStruct);
  
  /* Configure STSPIN820 - DECAY pin -------------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_DECAY;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_DECAY, &GPIO_InitStruct);
  
  /* Configure STSPIN820 - STBY/RESET pin ------------------------------------*/
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_STBY_RESET;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_STBY_RESET, &GPIO_InitStruct);
  
  Stspin820_Board_Reset();

}

/******************************************************//**
 * @brief  Starts the step clock
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void Stspin820_Board_TimStckStart()
{
  /* Clear pending timer interrupt */
  if (BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK == TIM_CHANNEL_1)
  {
    __HAL_TIM_CLEAR_IT(&hTimerStepClock, TIM_IT_CC1);
  }
  else if (BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK == TIM_CHANNEL_2)
  {
    __HAL_TIM_CLEAR_IT(&hTimerStepClock, TIM_IT_CC2);
  }
  else if (BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK == TIM_CHANNEL_3)
  {
    __HAL_TIM_CLEAR_IT(&hTimerStepClock, TIM_IT_CC3);
  }
  else if (BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK == TIM_CHANNEL_4)
  {
    __HAL_TIM_CLEAR_IT(&hTimerStepClock, TIM_IT_CC4);
  }
  /* Start timer interrupts */
#if (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_MAIN_OUTPUT)
  if ((hTimerStepClock.Instance->CCER &\
        (TIM_OCPOLARITY_LOW<<BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK)) == 0)
  {
    hTimerStepClock.Instance->CCER |=\
      (TIM_OCPOLARITY_LOW<<BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
  }
  else
  {
    hTimerStepClock.Instance->CCER &=\
      ~(TIM_OCPOLARITY_LOW<<BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
  }
  HAL_TIM_OC_Start_IT(&hTimerStepClock,\
    BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);  
#endif /* (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_MAIN_OUTPUT) */

#if (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_COMPLEMENTARY_OUTPUT)
  if ((hTimerStepClock.Instance->CCER &\
        (TIM_OCNPOLARITY_LOW<<BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK)) == 0)
  {
    hTimerStepClock.Instance->CCER |=\
      (TIM_OCNPOLARITY_LOW<<BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
  }
  else
  {
    hTimerStepClock.Instance->CCER &=\
      ~(TIM_OCNPOLARITY_LOW<<BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
  }
  HAL_TIMEx_OCN_Start_IT(&hTimerStepClock,\
    BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
#endif /* (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_COMPLEMENTARY_OUTPUT) */

#if (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_NO_OUTPUT)
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK,\
    BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK,\
        GPIO_PIN_RESET);
  HAL_TIM_OC_Start_IT(&hTimerStepClock,\
    BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
#endif /* (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_NO_OUTPUT) */
}
                                                     
/******************************************************//**
 * @brief Step clock compare value initialization
 * @retval None
 **********************************************************/
void Stspin820_Board_TimStckCompareInit(void)
{
  ccrValue = hTimerStepClock.Instance->CNT;
}

/******************************************************//**
 * @brief  Sets the frequency of step clock
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void Stspin820_Board_TimStckSetFreq(uint16_t newFreq)
{
  ccrValue += (HAL_RCC_GetSysClockFreq()/\
                           (TIMER_PRESCALER * 2 * (uint32_t)newFreq));
  __HAL_TIM_SetCompare(&hTimerStepClock,\
    BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK,\
    ccrValue);
}

/******************************************************//**
 * @brief  Starts the PWM used for the reference voltage generation
 * @retval None
 **********************************************************/
void Stspin820_Board_PwmRefStart(void)
{
  HAL_TIM_PWM_Start_IT(&hTimerPwm, BSP_MOTOR_CONTROL_BOARD_CHAN_PWM_REF);
  HAL_Delay(5*PWM_FILTER_TIME_CONSTANT);
}
                                                  
/******************************************************//**
 * @brief  Sets the frequency and duty cycle of PWM used for the reference
 * voltage generation
 * @param[in] newFreq in Hz
 * @param[in] dutyCycle 0 - 100%
 * @retval None
 **********************************************************/
void Stspin820_Board_PwmRefSetFreqAndDutyCycle(uint32_t newFreq, uint8_t dutyCycle)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/newFreq) - 1;
  uint16_t pulseLength;
  TIM_HandleTypeDef *pHTim;
  
  pHTim = &hTimerPwm;
  pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_PWM_REF;
  
  if (dutyCycle > 100) dutyCycle = 100;
  pulseLength = (uint16_t)((period * (100-(uint32_t)dutyCycle)) / 100);
  
  __HAL_TIM_SetAutoreload(pHTim, period);
  __HAL_TIM_SetCompare(pHTim, BSP_MOTOR_CONTROL_BOARD_CHAN_PWM_REF, pulseLength);
}

/******************************************************//**
 * @brief  Initialises the PWM used for the reference voltage generation
 * @retval None
 **********************************************************/
void Stspin820_Board_PwmRefInit(void)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_HandleTypeDef *pHTim;

  pHTim = &hTimerPwm;
  
  pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_PWM_REF;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.Prescaler = 0;
  pHTim->Init.Period = 0;
  pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.Pulse = 0;

  HAL_TIM_PWM_Init(pHTim);
  HAL_TIM_PWM_ConfigChannel(pHTim, &sConfigOC, BSP_MOTOR_CONTROL_BOARD_CHAN_PWM_REF);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(pHTim, &sMasterConfig);
}

/******************************************************//**
 * @brief  Initialises the timer used for the step clock
 * @retval None
 **********************************************************/
void Stspin820_Board_TimStckInit(void)
{
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_HandleTypeDef *pHTim;
  
  pHTim = &hTimerStepClock;
  pHTim->Instance = BSP_MOTOR_CONTROL_BOARD_TIM_STCK;
  pHTim->Init.CounterMode = TIM_COUNTERMODE_UP;
  pHTim->Init.Prescaler = TIMER_PRESCALER -1;
  if ((pHTim->Instance != TIM2) && (pHTim->Instance != TIM5))
  {
    pHTim->Init.Period = 0xFFFF;
  }
  else
  {
    pHTim->Init.Period = 0xFFFFFFFF;
  }
  pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(pHTim);
  
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; 
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.Pulse = 0;
  if (pHTim->Instance != TIM4)
  {
    /* Setting the OCMode to TIM_OCMODE_FORCED_ACTIVE ensures that on the */
    /* first interrupt occuring in the toggle mode, a rising edge will occur */
    sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
    HAL_TIM_OC_ConfigChannel(pHTim, &sConfigOC, BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  }
  else
  {
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
  }
  HAL_TIM_OC_ConfigChannel(pHTim, &sConfigOC, BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(pHTim, &sMasterConfig);
}

/******************************************************//**
 * @brief  DeInitialises the timer used for the step clock
 * @retval None
 **********************************************************/
void Stspin820_Board_TimStckDeInit(void)
{
  HAL_TIM_OC_DeInit(&hTimerStepClock);
}

/******************************************************//**
 * @brief  Stops the timer
 * @param[in] pToggleOdd pointer to the volatile toggleOdd variable
 * @retval 1 if OK, 0 if STCK pin is high (forbidden configuration) 
 **********************************************************/
uint8_t Stspin820_Board_TimStckStop(volatile uint8_t *pToggleOdd)
{
  __disable_irq();
  if (*pToggleOdd == 1)
  {
    __enable_irq();
    return 1;
  }
  if (HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK,\
    BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK) != 0)
  {
    __enable_irq();
    return 0;
  }
#if (BSP_MOTOR_CONTROL_BOARD_OUTPUT_TIM_STCK == TIMER_COMPLEMENTARY_OUTPUT)
  HAL_TIMEx_OCN_Stop_IT(&hTimerStepClock, BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
#else
  HAL_TIM_OC_Stop_IT(&hTimerStepClock, BSP_MOTOR_CONTROL_BOARD_CHAN_TIM_STCK);
#endif
  __enable_irq();
  /* DeInitialize the step clock timer */
  Stspin820_Board_TimStckDeInit();
  return 1;
}

/******************************************************//**
 * @brief  Stops the PWM used for the reference voltage generation 
 * @retval None
 **********************************************************/
void Stspin820_Board_PwmRefStop(void)
{
  HAL_TIM_PWM_Stop_IT(&hTimerPwm, BSP_MOTOR_CONTROL_BOARD_CHAN_PWM_REF);
}

/******************************************************//**
 * @brief  Releases the STSPIN820 reset (pin set to High)
 * @retval None
 **********************************************************/
void Stspin820_Board_ReleaseReset(void)
{ 
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_STBY_RESET,\
    BSP_MOTOR_CONTROL_BOARD_PIN_STBY_RESET,\
    GPIO_PIN_SET);
}

/******************************************************//**
 * @brief  Resets the STSPIN820 (reset pin set to low)
 * @retval None
 **********************************************************/
void Stspin820_Board_Reset(void)
{
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_STBY_RESET,\
    BSP_MOTOR_CONTROL_BOARD_PIN_STBY_RESET,\
    GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief  Set the GPIO used for the decay mode
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void Stspin820_Board_SetDecayGpio(uint8_t gpioState)
{
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_DECAY,\
    BSP_MOTOR_CONTROL_BOARD_PIN_DECAY,\
    (GPIO_PinState)gpioState);
}
                   
/******************************************************//**
 * @brief  Set the GPIO used for the direction
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void Stspin820_Board_SetDirectionGpio(uint8_t gpioState)
{
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_DIR,\
    BSP_MOTOR_CONTROL_BOARD_PIN_DIR,\
    (GPIO_PinState)gpioState);
}

/******************************************************//**
 * @brief Disable the power bridges (Force the output bridges HiZ)
 * the IHM14A1 board
 * @retval None
 **********************************************************/
void Stspin820_Board_Disable(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure the GPIO connected to EN pin as an output */
  GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT, &GPIO_InitStruct);
  __disable_irq();
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT,\
    BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT,\
    GPIO_PIN_RESET);
  __HAL_GPIO_EXTI_CLEAR_IT(BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT);
  __enable_irq();
}

/******************************************************//**
 * @brief Enable the power bridges (leave the output bridges HiZ)
 * @retval None
 **********************************************************/
void Stspin820_Board_Enable(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if (HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT,\
    BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT) == 0)
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT,\
      BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT,\
      GPIO_PIN_SET);
    HAL_Delay(BRIDGE_TURN_ON_DELAY);
    /* Configure the GPIO connected to EN pin to take interrupt */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT, &GPIO_InitStruct);
    __HAL_GPIO_EXTI_CLEAR_IT(BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT);
    HAL_NVIC_ClearPendingIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_EN_AND_FAULT);
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_EN_AND_FAULT);
  }
}

/******************************************************//**
 * @brief  Returns the EN FAULT pin state.
 * @retval The EN FAULT pin value.
 **********************************************************/
uint32_t Stspin820_Board_EN_AND_FAULT_PIN_GetState(void)
{
  return HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PORT_EN_AND_FAULT, BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT);
}

/******************************************************//**
 * @brief Select the STSPIN820 mode1, mode2 and mode3 pins levels.
 * @param[in] modePin1Level level of the mode1 gpio (0 low, 1+ high)
 * @param[in] modePin2Level level of the mode2 gpio (0 low, 1+ high)
 * @param[in] modePin3Level level of the mode3 gpio (0 low, 1+ high)
 * @retval 1
 **********************************************************/
uint8_t Stspin820_Board_SetModePins(uint8_t modePin1Level,\
  uint8_t modePin2Level,\
  uint8_t modePin3Level)
{  
  
  if (modePin1Level != 0)
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE1,\
      BSP_MOTOR_CONTROL_BOARD_PIN_MODE1,\
      GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE1,\
      BSP_MOTOR_CONTROL_BOARD_PIN_MODE1,\
      GPIO_PIN_RESET);
  }

  if (modePin2Level != 0)
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE2,\
      BSP_MOTOR_CONTROL_BOARD_PIN_MODE2,\
      GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE2,\
      BSP_MOTOR_CONTROL_BOARD_PIN_MODE2,\
      GPIO_PIN_RESET);
  }
  
  if (modePin3Level != 0)
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE3,\
      BSP_MOTOR_CONTROL_BOARD_PIN_MODE3,\
      GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE3,\
      BSP_MOTOR_CONTROL_BOARD_PIN_MODE3,\
      GPIO_PIN_RESET);
  }
  
  return 1;
}

/******************************************************//**
 * @brief Select Full Step mode
 * @retval None
 **********************************************************/
void Stspin820_Board_SetFullStep(void)
{
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE1,\
    BSP_MOTOR_CONTROL_BOARD_PIN_MODE1,\
    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE2,\
    BSP_MOTOR_CONTROL_BOARD_PIN_MODE2,\
    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PORT_MODE3,\
    BSP_MOTOR_CONTROL_BOARD_PIN_MODE3,\
    GPIO_PIN_RESET);
}

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
