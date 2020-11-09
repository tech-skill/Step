/**
  ******************************************************************************
  * @file    stspin820.c
  * @author  STM
  * @version V1.0.0
  * @date    August 7th, 2017
  * @brief   STSPIN820 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2017 STMicroelectronics
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
#include "stspin820.h"

/** @addtogroup BSP
  * @{
  */   
   
/** @defgroup STSPIN820 STSPIN820
  * @{
  */   

/* Private constants ---------------------------------------------------------*/
    
/* Private variables ---------------------------------------------------------*/

/** @defgroup Stspin820_Private_Variables Stspin820 Private Variables
  * @{
  */

/// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
/// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);
static uint16_t stspin820DriverInstance = 0;
static uint8_t  stspin820NumberOfDevices = 0;
static volatile uint8_t toggleOdd;

/// STSPIN820 Device Paramaters structure
deviceParams_t devicePrm;

/**
  * @}
  */
    
/* Private Types  ------------------------------------------------------------*/    
/* Private function prototypes -----------------------------------------------*/

/** @defgroup Stspin820_Private_Functions Stspin820 Private Functions
  * @{
  */  
void Stspin820_ApplySpeed(uint8_t pwmId, uint16_t newSpeed);
void Stspin820_ApplyTorque(uint8_t deviceId, motorTorqueMode_t torqueMode);
void Stspin820_ComputeSpeedProfile(uint8_t deviceId, uint32_t nbSteps);
void Stspin820_FlagInterruptHandler(void);
void Stspin820_SetDeviceParamsOtherValues(void);
void Stspin820_SetDeviceParamsToGivenValues(Stspin820_Init_t* initDevicePrm);
void Stspin820_SetDeviceParamsToPredefinedValues(void);
void Stspin820_StartMovement(uint8_t deviceId);
void Stspin820_StepClockHandler(uint8_t deviceId);
/**
  * @}
  */ 

/** @defgroup Stspin820_Exported_Variables Stspin820 Exported Variables
  * @{
  */

/// STSPIN820 motor driver functions pointer structure 
motorDrv_t   stspin820Drv = 
{
  Stspin820_Init,                      //void (*Init)(void*);
  Stspin820_ReadId,                    //uint16_t (*ReadID)(void);
  Stspin820_AttachErrorHandler,        //void (*AttachErrorHandler)(void (*callback)(uint16_t));
  Stspin820_AttachFlagInterrupt,       //void (*AttachFlagInterrupt)(void (*callback)(void));
  0,                                   //void (*AttachBusyInterrupt)(void (*callback)(void));
  Stspin820_FlagInterruptHandler,      //void (*FlagInterruptHandler)(void);
  Stspin820_GetAcceleration,           //uint16_t (*GetAcceleration)(uint8_t);
  Stspin820_GetCurrentSpeed,           //uint16_t (*GetCurrentSpeed)(uint8_t);
  Stspin820_GetDeceleration,           //uint16_t (*GetDeceleration)(uint8_t);
  Stspin820_GetDeviceState,            //motorState_t(*GetDeviceState)(uint8_t);
  Stspin820_GetFwVersion,              //uint32_t (*GetFwVersion)(void);
  Stspin820_GetMark,                   //int32_t (*GetMark)(uint8_t);
  Stspin820_GetMaxSpeed,               //uint16_t (*GetMaxSpeed)(uint8_t);
  Stspin820_GetMinSpeed,               //uint16_t (*GetMinSpeed)(uint8_t);
  Stspin820_GetPosition,               //int32_t (*GetPosition)(uint8_t);
  Stspin820_GoHome,                    //void (*GoHome)(uint8_t);
  Stspin820_GoMark,                    //void (*GoMark)(uint8_t);
  Stspin820_GoTo,                      //void (*GoTo)(uint8_t, int32_t);
  Stspin820_HardStop,                  //void (*HardStop)(uint8_t);
  Stspin820_Move,                      //void (*Move)(uint8_t, motorDir_t, uint32_t );
  0,                                   //void (*ResetAllDevices)(void);
  Stspin820_Run,                       //void (*Run)(uint8_t, motorDir_t);
  Stspin820_SetAcceleration,           //bool(*SetAcceleration)(uint8_t ,uint16_t );
  Stspin820_SetDeceleration,           //bool(*SetDeceleration)(uint8_t , uint16_t );
  Stspin820_SetHome,                   //void (*SetHome)(uint8_t);
  Stspin820_SetMark,                   //void (*SetMark)(uint8_t);
  Stspin820_SetMaxSpeed,               //bool (*SetMaxSpeed)(uint8_t, uint16_t );
  Stspin820_SetMinSpeed,               //bool (*SetMinSpeed)(uint8_t, uint16_t );
  Stspin820_SoftStop,                  //bool (*SoftStop)(uint8_t);
  Stspin820_StepClockHandler,          //void (*StepClockHandler)(uint8_t deviceId);
  Stspin820_WaitWhileActive,           //void (*WaitWhileActive)(uint8_t);
  Stspin820_Disable,                   //void (*CmdDisable)(uint8_t);
  Stspin820_Enable,                    //void (*CmdEnable)(uint8_t);
  0,                                   //uint32_t (*CmdGetParam)(uint8_t, uint32_t);
  0,                                   //uint16_t (*CmdGetStatus)(uint8_t);
  0,                                   //void (*CmdNop)(uint8_t);
  0,                                   //void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
  0,                                   //uint16_t (*ReadStatusRegister)(uint8_t);
  0,                                   //void (*ReleaseReset)(void);
  0,                                   //void (*Reset)(void);
  Stspin820_SetStepMode,               //bool (*SelectStepMode)(uint8_t deviceId, motorStepMode_t);
  Stspin820_SetDirection,              //void (*SetDirection)(uint8_t, motorDir_t);
  Stspin820_GoToDir,                   //void (*CmdGoToDir)(uint8_t, motorDir_t, int32_t);
  0,                                   //uint8_t (*CheckBusyHw)(void);
  Stspin820_CheckStatusHw,             //uint8_t (*CheckStatusHw)(void);
  0,                                   //void (*CmdGoUntil)(uint8_t, motorAction_t, motorDir_t, uint32_t);
  0,                                   //void (*CmdHardHiZ)(uint8_t);
  0,                                   //void (*CmdReleaseSw)(uint8_t, motorAction_t, motorDir_t);
  Stspin820_PutDeviceInStandby,        //void (*CmdResetDevice)(uint8_t);
  0,                                   //void (*CmdResetPos)(uint8_t);
  0,                                   //void (*CmdRun)(uint8_t, motorDir_t, uint32_t);
  0,                                   //void (*CmdSoftHiZ)(uint8_t);
  0,                                   //void (*CmdStepClock)(uint8_t, motorDir_t);
  0,                                   //void (*FetchAndClearAllStatus)(void);
  0,                                   //uint16_t (*GetFetchedStatus)(uint8_t);
  Stspin820_GetNbDevices,              //uint8_t (*GetNbDevices)(void);
  0,                                   //bool (*IsDeviceBusy)(uint8_t);
  0,                                   //void (*SendQueuedCommands)(void);
  0,                                   //void (*QueueCommands)(uint8_t, uint8_t, int32_t);
  0,                                   //void (*WaitForAllDevicesNotBusy)(void);
  Stspin820_ErrorHandler,              //void (*ErrorHandler)(uint16_t);
  0,                                   //void (*BusyInterruptHandler)(void);
  0,                                   //void (*CmdSoftStop)(uint8_t);
  0,                                   //void (*StartStepClock)(uint16_t);
  0,                                   //void (*StopStepClock)(void);
  0,                                   //void (*SetDualFullBridgeConfig)(uint8_t);
  Stspin820_VrefPwmGetFreq,            //uint32_t (*GetBridgeInputPwmFreq)(uint8_t);
  Stspin820_VrefPwmSetFreq,            //void (*SetBridgeInputPwmFreq)(uint8_t, uint32_t);
  Stspin820_SetStopMode,               //void (*SetStopMode)(uint8_t, motorStopMode_t);
  Stspin820_GetStopMode,               //motorStopMode_t (*GetStopMode)(uint8_t);
  Stspin820_SetDecayMode,              //void (*SetDecayMode)(uint8_t, motorDecayMode_t);
  Stspin820_GetDecayMode,              //motorDecayMode_t (*GetDecayMode)(uint8_t);
  Stspin820_GetStepMode,               //motorStepMode_t (*GetStepMode)(uint8_t);
  Stspin820_GetDirection,              //motorDir_t (*GetDirection)(uint8_t);
  Stspin820_ExitDeviceFromStandby,     //void (*ExitDeviceFromReset)(uint8_t);
  Stspin820_SetTorque,                 //void (*SetTorque)(uint8_t, motorTorqueMode_t, uint8_t);
  Stspin820_GetTorque,                 //uint8_t (*GetTorque)(uint8_t, motorTorqueMode_t);
  0,                                   //void (*SetRefFreq)(uint8_t, uint32_t);
  0,                                   //uint32_t (*GetRefFreq)(uint8_t);
  0,                                   //void (*SetRefDc)(uint8_t, uint8_t);
  0,                                   //uint8_t (*GetRefDc)(uint8_t);
  Stspin820_SetNbDevices,              //bool (*SetNbDevices)(uint8_t);
  0,                                   //bool (*SetAnalogValue)(uint8_t, uint32_t, float);
  0,                                   //float (*GetAnalogValue)(uint8_t, uint32_t);
  Stspin820_SetTorqueBoostEnable,      //void (*SetTorqueBoostEnable) (uint8_t, bool);
  Stspin820_GetTorqueBoostEnable,      //bool (*GetTorqueBoostEnable) (uint8_t);
  Stspin820_SetTorqueBoostThreshold,   //void (*SetTorqueBoostThreshold) (uint8_t, uint16_t);
  Stspin820_GetTorqueBoostThreshold    //uint16_t (*GetTorqueBoostThreshold) (uint8_t);
};

/**
  * @}
  */ 

/** @defgroup STSPIN820_Library_Functions STSPIN820 Library Functions
  * @{
  */

/******************************************************//**
 * @brief Return motor handle (pointer to the STSPIN820 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
motorDrv_t* Stspin820_GetMotorHandle(void)
{
  return (&stspin820Drv);
}

/******************************************************//**
 * @brief Start the STSPIN820 library
 * @param[in] pInit pointer to the initialization data
 * @retval None
 **********************************************************/
void Stspin820_Init(void* pInit)
{
  stspin820DriverInstance++;
  
  /* Initialise the GPIOs */
  Stspin820_Board_GpioInit();

  /* Initialise the timer used for the step clock and ------------------------*/
  /* the PWM for the reference voltage generation ----------------------------*/
  Stspin820_Board_TimStckInit();
  Stspin820_Board_PwmRefInit();

  if (pInit == 0)
  {
    /* Set all context variables to the predefined values */
    /* from stspin820_target_config.h */
    Stspin820_SetDeviceParamsToPredefinedValues();
  }
  else
  {
    Stspin820_SetDeviceParamsToGivenValues((Stspin820_Init_t*) pInit);
  }
}

/******************************************************//**
 * @brief Read id
 * @retval Id of the stspin820 Driver Instance
 **********************************************************/
uint16_t Stspin820_ReadId(void)
{
  return (stspin820DriverInstance);
}

/******************************************************//**
 * @brief Attach a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void Stspin820_AttachErrorHandler(void (*callback)(uint16_t))
{
  errorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief Attach a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal pre-warning or 
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void Stspin820_AttachFlagInterrupt(void (*callback)(void))
{
  flagInterruptCallback = (void (*)())callback;
}

/******************************************************//**
 * @brief Check if STSPIN820 has a fault by reading EN pin position.
 * @retval One if STSPIN820 has EN pin down, otherwise zero
 **********************************************************/
uint8_t Stspin820_CheckStatusHw(void)
{
	if(!Stspin820_Board_EN_AND_FAULT_PIN_GetState())
  {
    return 0x01;
  }
  else
  {
    return 0x00;
  }
}

/******************************************************//**
 * @brief Disable the power bridges (leave the output bridges HiZ)
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_Disable(uint8_t deviceId)
{
  Stspin820_Board_Disable();
}

/******************************************************//**
 * @brief Enable the power bridges
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_Enable(uint8_t deviceId)
{
  Stspin820_Board_Enable();
}

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void Stspin820_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    (void) errorHandlerCallback(error);
  }
  else   
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief Exit STSPIN820 device from standby (low power consumption)
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_ExitDeviceFromStandby(uint8_t deviceId)
{
  uint32_t sequencerPosition = devicePrm.sequencerPosition;
  
  /* Exit standby and set step mode */
  /* Disable step clock */
  if (Stspin820_Board_TimStckStop(&toggleOdd) == 0)
  {
    Stspin820_ErrorHandler(STSPIN820_ERROR_STEP_CLOCK);
  }

  /* Let the PWM REF and bridges enabled at least for DISABLE_DELAY time */
  /* after the last step clock rising edge triggering the last step */
  Stspin820_Board_Delay(DISABLE_DELAY);
    /* Set reference voltage to 0 */
  Stspin820_SetTorque(0, CURRENT_TORQUE, 0);

  /* Disable power bridges */
  Stspin820_Board_Disable();
  
  devicePrm.commandExecuted = NO_CMD;
  devicePrm.stepsToTake = 0;  
  devicePrm.speed = 0;

  /* Reset the microstepping sequencer position */
  devicePrm.sequencerPosition = 0;

  /* Reset current and mark positions */
  devicePrm.currentPosition = 0; 
  devicePrm.markPosition = 0;
  
  Stspin820_SetStepMode(deviceId, devicePrm.stepMode);
  Stspin820_Board_ReleaseReset();
  Stspin820_Board_Delay(AFTER_STANDBY_EXIT_DEAD_TIME);
  
  Stspin820_SetHome(deviceId);
  Stspin820_SetMark(deviceId);
  
  if (devicePrm.sequencerPosition != 0)
  {
    /* Set direction to FORWARD to ensure the HW sequencer is increased at */
    /* each step clock rising edge */
    Stspin820_SetDirection(0, FORWARD);
    /* Going out of standby */
    devicePrm.motionState = STANDBYTOINACTIVE;
    /* Initialize the step clock timer */
    Stspin820_Board_TimStckInit();
    /* Program the step clock */    
    Stspin820_Board_TimStckCompareInit();
    Stspin820_Board_TimStckSetFreq(STSPIN820_MAX_STCK_FREQ);
    toggleOdd = 0;
    Stspin820_Board_TimStckStart();
    while (devicePrm.sequencerPosition != 0);
    while (toggleOdd!=0);
    devicePrm.sequencerPosition = sequencerPosition;    
  }
  
  devicePrm.motionState = INACTIVE;
}

/******************************************************//**
 * @brief Return the acceleration of the specified device
 * @param[in] deviceId Unused parameter
 * @retval Acceleration in pps^2
 **********************************************************/
uint16_t Stspin820_GetAcceleration(uint8_t deviceId)
{                                                  
  return (devicePrm.acceleration);
}            

/******************************************************//**
 * @brief Return the current speed of the specified device
 * @param[in] deviceId Unused parameter
 * @retval Speed in pps
 **********************************************************/
uint16_t Stspin820_GetCurrentSpeed(uint8_t deviceId)
{
  return devicePrm.speed;
}

/******************************************************//**
 * @brief Return the decay mode of the specified device
 * @param[in] deviceId Unused parameter
 * @retval Decay Mode State (SLOW or MIXED)
 **********************************************************/
motorDecayMode_t Stspin820_GetDecayMode(uint8_t deviceId)
{
  motorDecayMode_t status;
  if (Stspin820_Board_GetDecayGpio() != 0) 
  {
    status = SLOW_DECAY;
  }
  else
  {
    status = MIXED_DECAY;
  }
  
  return status;
}

/******************************************************//**
 * @brief Return the deceleration of the specified device
 * @param[in] deviceId Unused parameter
 * @retval Deceleration in pps^2
 **********************************************************/
uint16_t Stspin820_GetDeceleration(uint8_t deviceId)
{                                                  
  return (devicePrm.deceleration);
}          

/******************************************************//**
 * @brief Return the device state
 * @param[in] deviceId Unused parameter
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 **********************************************************/
motorState_t Stspin820_GetDeviceState(uint8_t deviceId)
{
  return devicePrm.motionState;
}

/******************************************************//**
 * @brief Get the motor current direction
 * @param[in] deviceId Unused parameter
 * @retval direction
 **********************************************************/
motorDir_t Stspin820_GetDirection(uint8_t deviceId)
{
  return devicePrm.direction;
}

/******************************************************//**
 * @brief Return the FW version of the library
 * @retval Stspin220_FW_VERSION
 **********************************************************/
uint32_t Stspin820_GetFwVersion(void)
{
  return (STSPIN820_FW_VERSION);
}

/******************************************************//**
 * @brief Return the mark position of the specified device
 * @param[in] deviceId Unused parameter
 * @retval mark position value
 **********************************************************/
int32_t Stspin820_GetMark(uint8_t deviceId)
{
  return devicePrm.markPosition;
}

/******************************************************//**
 * @brief Return the max speed of the specified device
 * @param[in] deviceId Unused parameter
 * @retval maxSpeed in pps
 **********************************************************/
uint16_t Stspin820_GetMaxSpeed(uint8_t deviceId)
{                                                  
  return (devicePrm.maxSpeed);
}

/******************************************************//**
 * @brief Return the min speed of the specified device
 * @param[in] deviceId Unused parameter
 * @retval minSpeed in pps
 **********************************************************/
uint16_t Stspin820_GetMinSpeed(uint8_t deviceId)
{                                                  
  return (devicePrm.minSpeed);
}                                                     

/******************************************************//**
 * @brief  Returns the number of devices
 * @retval number of devices
 **********************************************************/
uint8_t Stspin820_GetNbDevices(void)
{
  return (stspin820NumberOfDevices);
}

/******************************************************//**
 * @brief Return the current position value of the specified device
 * @param[in] deviceId Unused parameter
 * @retval current position value
 **********************************************************/
int32_t Stspin820_GetPosition(uint8_t deviceId)
{
  return devicePrm.currentPosition;
}

/******************************************************//**
 * @brief Get the motor step mode
 * @param[in] deviceId Unused parameter
 * @retval step mode
 **********************************************************/
motorStepMode_t Stspin820_GetStepMode(uint8_t deviceId)
{
  return devicePrm.stepMode;
}

/******************************************************//**
 * @brief Get the selected stop mode
 * @param[in] deviceId Unused parameter
 * @retval the selected stop mode
 **********************************************************/
motorStopMode_t Stspin820_GetStopMode(uint8_t deviceId)
{
  return devicePrm.stopMode;
}

/******************************************************//**
 * @brief Get the torque of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] torqueMode torque mode
 * @retval the torqueValue in % (from 0 to 100)
 * @note
 **********************************************************/
uint8_t Stspin820_GetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode)
{
  uint8_t torqueValue = 0;
  switch(torqueMode)
  {
    case ACC_TORQUE:
      torqueValue = devicePrm.accelTorque;
      break;
    case DEC_TORQUE:
      torqueValue = devicePrm.decelTorque;
      break;
    case RUN_TORQUE:
      torqueValue = devicePrm.runTorque;
      break;
    case HOLD_TORQUE:
      torqueValue = devicePrm.holdTorque;
      break;
    case CURRENT_TORQUE:
      torqueValue = devicePrm.currentTorque;
      break;
    default:
      break;
  }
  return torqueValue;
}

/******************************************************//**
 * @brief Get the torque boost feature status
 * @param[in] deviceId Unused parameter
 * @retval TRUE if enabled, FALSE if disabled
 **********************************************************/
bool Stspin820_GetTorqueBoostEnable(uint8_t deviceId)
{
  return devicePrm.torqueBoostEnable;
}

/******************************************************//**
 * @brief Get the torque boost threshold
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @retval the torque boost threshold above which the step mode is
 * changed to full step
 **********************************************************/
uint16_t Stspin820_GetTorqueBoostThreshold(uint8_t deviceId)
{
  return devicePrm.torqueBoostSpeedThreshold;
}

/******************************************************//**
 * @brief Request the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_GoHome(uint8_t deviceId)
{
  deviceId = 0;
  
  Stspin820_GoTo(deviceId, 0);
} 
  
/******************************************************//**
 * @brief Request the motor to move to the mark position 
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_GoMark(uint8_t deviceId)
{
  deviceId = 0;
  
	Stspin820_GoTo(deviceId, devicePrm.markPosition);
}

/******************************************************//**
 * @brief Request the motor to move to the specified position 
 * @param[in] deviceId Unused parameter
 * @param[in] targetPosition absolute position in steps
 * @retval None 
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL                   : Full step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step
 * STEP_MODE_1_32                   : 1/32 step
 * STEP_MODE_1_128                  : 1/128 step
 * STEP_MODE_1_256                  : 1/256 step
 * @note The 1/64 step mode is not allowed
 **********************************************************/
void Stspin820_GoTo(uint8_t deviceId, int32_t targetPosition)
{
  motorDir_t direction;
  deviceId = 0;
  
  /* Exit from standby if needed */
  if (devicePrm.motionState == STANDBY)
  {
    Stspin820_ExitDeviceFromStandby(deviceId);
  }
  /* Deactivate motor if needed */
  else if (devicePrm.motionState != INACTIVE)
  { 
    Stspin820_HardHiZ(deviceId);
  }
  
  if (targetPosition > devicePrm.currentPosition)
  {
    devicePrm.stepsToTake = targetPosition -\
                                      devicePrm.currentPosition;
    if (devicePrm.stepsToTake < (STSPIN820_POSITION_RANGE>>1))
    {
      direction = FORWARD;
    }
    else
    {
      direction = BACKWARD;
      devicePrm.stepsToTake = STSPIN820_POSITION_RANGE -\
                                        devicePrm.stepsToTake;
    }
  }
  else
  {
    devicePrm.stepsToTake = devicePrm.currentPosition -\
                                      targetPosition;
    if (devicePrm.stepsToTake < (STSPIN820_POSITION_RANGE>>1))
    {
      direction = BACKWARD;
    }
    else
    {
      direction = FORWARD; 
      devicePrm.stepsToTake = STSPIN820_POSITION_RANGE -\
                                        devicePrm.stepsToTake;
    }
  }
  
  if (devicePrm.stepsToTake != 0) 
  {
    devicePrm.commandExecuted = MOVE_CMD;
    
    /* Direction setup */
    Stspin820_SetDirection(deviceId, direction);
    
    Stspin820_ComputeSpeedProfile(deviceId, devicePrm.stepsToTake);
    
    /* Motor activation */
    Stspin820_StartMovement(deviceId);
  }
}

/******************************************************//**
 * @brief Move the motor to the absolute position
 * @param[in] deviceId Unused parameter
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] targetPosition 32 bit signed value position
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL                   : step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step
 * STEP_MODE_1_32                   : 1/32 step
 * STEP_MODE_1_128                  : 1/128 step
 * STEP_MODE_1_256                  : 1/256 step
 * @note The 1/64 step mode is not allowed
 **********************************************************/
void Stspin820_GoToDir(uint8_t deviceId, motorDir_t direction, int32_t targetPosition)
{
  deviceId = 0;
  
  /* Exit from standby if needed */
  if (devicePrm.motionState == STANDBY)
  {
    Stspin820_ExitDeviceFromStandby(deviceId);
  }
  /* Deactivate motor if needed */
  else if (devicePrm.motionState != INACTIVE)
  { 
    Stspin820_HardHiZ(deviceId);
  }
  
  if (direction != BACKWARD)
  {
    if (targetPosition > devicePrm.currentPosition)
    {
      devicePrm.stepsToTake = targetPosition -\
                                        devicePrm.currentPosition;
    }
    else
    {
      devicePrm.stepsToTake = STSPIN820_POSITION_RANGE +\
                                       (targetPosition -\
                                        devicePrm.currentPosition);
    }
  }
  else
  {
    if (targetPosition > devicePrm.currentPosition)
    {
      devicePrm.stepsToTake = STSPIN820_POSITION_RANGE +\
                                        (devicePrm.currentPosition -\
                                         targetPosition);
    }
    else
    {
      devicePrm.stepsToTake = devicePrm.currentPosition -\
                                        targetPosition;
    }
  }

  if (devicePrm.stepsToTake != 0) 
  {
    devicePrm.commandExecuted = MOVE_CMD;
    
    /* Direction setup */
    Stspin820_SetDirection(deviceId, direction);
    
    Stspin820_ComputeSpeedProfile(deviceId, devicePrm.stepsToTake);
    
    /* Motor activation */
    Stspin820_StartMovement(deviceId);
  }  
}

/******************************************************//**
 * @brief  Immediatly stop the motor and disable the power bridge
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_HardHiZ(uint8_t deviceId) 
{
  /* Set inactive state */
  devicePrm.motionState = INACTIVE;
  
  /* Disable step clock */
  if (Stspin820_Board_TimStckStop(&toggleOdd) == 0)
  {
    Stspin820_ErrorHandler(STSPIN820_ERROR_STEP_CLOCK);
  }

  /* Let the PWM REF and bridges enabled at least for DISABLE_DELAY time */
  /* after the last step clock rising edge triggering the last step */
  Stspin820_Board_Delay(DISABLE_DELAY);
  
  /* Set reference voltage to 0 */
  Stspin820_SetTorque(0, CURRENT_TORQUE, 0);

  /* Disable power bridges */
  Stspin820_Board_Disable();
  
  /* Comeback to nominal step mode */
  if (devicePrm.stepModeLatched != devicePrm.stepMode)
  {
    motorStepMode_t StepMode = devicePrm.stepModeLatched;
    Stspin820_SetStepMode(0, StepMode);
    devicePrm.stepMode = devicePrm.stepModeLatched;
  }

  devicePrm.commandExecuted = NO_CMD;
  devicePrm.stepsToTake = 0;  
  devicePrm.speed = 0;
}

/******************************************************//**
 * @brief  Immediatly stop the motor
 * and either set holding torque when stop mode is HOLD_MODE,
 * or call Stspin820_HardHiz function when stop mode is HIZ_MODE,
 * or call Stspin820_PutDeviceInStandby function when stop mode is STANDBY_MODE
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_HardStop(uint8_t deviceId) 
{
  deviceId = 0;
  
  if (devicePrm.stopMode == HOLD_MODE)
  {
    /* Set inactive state */
    devicePrm.motionState = INACTIVE;

    /* Disable step clock */
    if (Stspin820_Board_TimStckStop(&toggleOdd) == 0)
    {
      Stspin820_ErrorHandler(STSPIN820_ERROR_STEP_CLOCK);
    }
    
    /* Set holding torque */
    Stspin820_ApplyTorque(deviceId, HOLD_TORQUE);
 
    /* Comeback to nominal step mode */
    if (devicePrm.stepModeLatched != devicePrm.stepMode)
    {
      motorStepMode_t StepMode = devicePrm.stepModeLatched;
      Stspin820_SetStepMode(0, StepMode);
      devicePrm.stepMode = devicePrm.stepModeLatched;
    }    
    
    devicePrm.commandExecuted = NO_CMD;
    devicePrm.stepsToTake = 0;  
    devicePrm.speed = 0;
  }
  else if (devicePrm.stopMode == HIZ_MODE)
  {
    Stspin820_HardHiZ(deviceId);
  }
  else if (devicePrm.stopMode == STANDBY_MODE)
  {
    Stspin820_PutDeviceInStandby(deviceId);
  }
}

/******************************************************//**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] deviceId Unused parameter
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void Stspin820_Move(uint8_t deviceId, motorDir_t direction, uint32_t stepCount)
{
  deviceId = 0;
  
  /* Exit from standby if needed */
  if (devicePrm.motionState == STANDBY)
  {
    Stspin820_ExitDeviceFromStandby(deviceId);
  }
  /* Deactivate motor if needed */
  else if (devicePrm.motionState != INACTIVE)
  {
    Stspin820_HardHiZ(deviceId);
  }
  
  if (stepCount != 0) 
  {
    devicePrm.stepsToTake = stepCount;    
    devicePrm.commandExecuted = MOVE_CMD;
    
    /* Direction setup */
    Stspin820_SetDirection(deviceId, direction);
    
    Stspin820_ComputeSpeedProfile(deviceId, stepCount);
    
    /* Motor activation */
    Stspin820_StartMovement(deviceId);
  }  
}

/******************************************************//**
 * @brief Put STSPIN820 device in standby (low power consumption)
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_PutDeviceInStandby(uint8_t deviceId)
{
  /* Stop movement */
  Stspin820_HardHiZ(deviceId);
  
  /* Enter standby */
	Stspin820_Board_Reset();
  
  devicePrm.motionState = STANDBY;
}

/******************************************************//**
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the device acceleration.
 * @param[in] deviceId Unused parameter
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 **********************************************************/
void Stspin820_Run(uint8_t deviceId, motorDir_t direction)
{
  /* Exit from standby if needed */
  if (devicePrm.motionState == STANDBY)
  {
    Stspin820_ExitDeviceFromStandby(deviceId);
  }
  /* Deactivate motor if needed */
  else if (devicePrm.motionState != INACTIVE)
  {
    Stspin820_HardHiZ(deviceId);
  }
  
	/* Direction setup */
	Stspin820_SetDirection(deviceId,direction);
	devicePrm.commandExecuted = RUN_CMD;
	/* Motor activation */
	Stspin820_StartMovement(deviceId); 
}

/******************************************************//**
 * @brief  Changes the acceleration of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed if the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool Stspin820_SetAcceleration(uint8_t deviceId,uint16_t newAcc)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newAcc != 0)&&
      (((devicePrm.motionState & INACTIVE) == INACTIVE)||
       (devicePrm.commandExecuted == RUN_CMD)))
  {
    devicePrm.acceleration = newAcc;
    cmdExecuted = TRUE;
  }    
  return cmdExecuted;
}            

/******************************************************//**
 * @brief  Specifies the decay mode 
 * @param[in] deviceId Unused parameter
 * @param[in] decay SLOW_DECAY or MIXED_DECAY
 * @retval true if the command is successfully executed, else false
 **********************************************************/
void Stspin820_SetDecayMode(uint8_t deviceId, motorDecayMode_t decay)
{ 
  if (decay == SLOW_DECAY)
  {
    Stspin820_Board_SetDecayGpio(1);
  }
  else if (decay == MIXED_DECAY)
  {
    Stspin820_Board_SetDecayGpio(0);
  }
}

/******************************************************//**
 * @brief  Changes the deceleration of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed if the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool Stspin820_SetDeceleration(uint8_t deviceId, uint16_t newDec)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newDec != 0)&& 
      (((devicePrm.motionState & INACTIVE) == INACTIVE)||
       (devicePrm.commandExecuted == RUN_CMD)))
  {
    devicePrm.deceleration = newDec;
    cmdExecuted = TRUE;
  }      
  return cmdExecuted;
}

/******************************************************//**
 * @brief  Specifies the direction 
 * @param[in] deviceId Unused parameter
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is applied if the device 
 * is in INACTIVE or STANDBY state or if the device is 
 * executing a run command
 * @retval None
 **********************************************************/
void Stspin820_SetDirection(uint8_t deviceId, motorDir_t dir)
{ 
  if ((devicePrm.motionState == INACTIVE)||\
      (devicePrm.motionState == STANDBY))
  {
    devicePrm.direction = dir;
    Stspin820_Board_SetDirectionGpio(dir);
  }
  else if ((devicePrm.commandExecuted&RUN_CMD)!=0)
  {
    devicePrm.commandExecuted|=STSPIN820_DIR_CHANGE_BIT_MASK;
  }
}

/******************************************************//**
 * @brief  Set current position to be the Home position
 * (current position set to 0)
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_SetHome(uint8_t deviceId)
{
  devicePrm.currentPosition = 0;
}
 
/******************************************************//**
 * @brief  Set current position to be the Mark position 
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_SetMark(uint8_t deviceId)
{
  devicePrm.markPosition = devicePrm.currentPosition;
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] newMaxSpeed New max speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool Stspin820_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newMaxSpeed >= STSPIN820_MIN_STCK_FREQ)&&\
      ((newMaxSpeed <= STSPIN820_MAX_STCK_FREQ)||\
       ((devicePrm.torqueBoostEnable != FALSE)&&\
        ((newMaxSpeed>>Stspin820_GetStepMode(deviceId))<= STSPIN820_MAX_STCK_FREQ)))&&\
      (devicePrm.minSpeed <= newMaxSpeed) &&\
      (((devicePrm.motionState & INACTIVE) == INACTIVE)||\
      (devicePrm.commandExecuted == RUN_CMD)))
  {
    devicePrm.maxSpeed = newMaxSpeed;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}                                                     

/******************************************************//**
 * @brief  Changes the min speed of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] newMinSpeed New min speed  to apply in pps
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command).
 **********************************************************/
bool Stspin820_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed)
{                                                  
  bool cmdExecuted = FALSE;
  if ((newMinSpeed >= STSPIN820_MIN_STCK_FREQ)&&
      (newMinSpeed <= STSPIN820_MAX_STCK_FREQ) &&
      (newMinSpeed <= devicePrm.maxSpeed) && 
      (((devicePrm.motionState & INACTIVE) == INACTIVE)||
       (devicePrm.commandExecuted == RUN_CMD)))
  {
    devicePrm.minSpeed = newMinSpeed;
    cmdExecuted = TRUE;
  }  
  return cmdExecuted;
}

/******************************************************//**
 * @brief  Sets the number of devices to be used 
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of 
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool Stspin820_SetNbDevices(uint8_t nbDevices)
{
  if (nbDevices <= MAX_NUMBER_OF_DEVICES)
  {
    stspin820NumberOfDevices = nbDevices;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/******************************************************//**
 * @brief Set the stepping mode 
 * @param[in] deviceId Unused parameter
 * @param[in] stepMode from full step to 1/256 microstep
 * as specified in enum motorStepMode_t
 * 1/64 microstep mode not supported by STSPIN820
 * @retval true if the command is successfully executed, else false
 **********************************************************/
bool Stspin820_SetStepMode(uint8_t deviceId, motorStepMode_t stepMode)
{
  /* Store step mode */
  devicePrm.stepMode = stepMode;
  devicePrm.stepModeLatched = stepMode;
  
  /* Set the mode pins to the levels corresponding to the selected step mode */
  switch (stepMode)
  {
    case STEP_MODE_FULL:
      Stspin820_Board_SetFullStep();
      break;
    case STEP_MODE_HALF:
      Stspin820_Board_SetModePins(1, 0, 0);
      break;    
    case STEP_MODE_1_4:
      Stspin820_Board_SetModePins(0, 1, 0);
      break;        
    case STEP_MODE_1_8:
      Stspin820_Board_SetModePins(1, 1, 0);
      break;
    case STEP_MODE_1_16:
      Stspin820_Board_SetModePins(0, 0, 1);
      break;   
    case STEP_MODE_1_32:
      Stspin820_Board_SetModePins(1, 0, 1);
      break;   
    case STEP_MODE_1_128:
      Stspin820_Board_SetModePins(0, 1, 1);
      break;  
    case STEP_MODE_1_256:
      Stspin820_Board_SetModePins(1, 1, 1);
      break;
    default:
      return FALSE;
  }

  return TRUE;
  
}

/******************************************************//**
 * @brief Select the mode to stop the motor.
 * @param[in] deviceId Unused parameter
 * @param[in] stopMode HOLD_MODE to let power bridge enabled
 * @retval None
 **********************************************************/
void Stspin820_SetStopMode(uint8_t deviceId, motorStopMode_t stopMode)
{
  devicePrm.stopMode = stopMode;
}

/******************************************************//**
 * @brief Set the torque of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] torqueMode Torque mode as specified in enum motorTorqueMode_t
 * @param[in] torqueValue in % (from 0 to 100)
 * @retval None
 * @note
 **********************************************************/
void Stspin820_SetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode, uint8_t torqueValue)
{
  devicePrm.updateTorque = TRUE;
  if (torqueValue>100) torqueValue = 100;
  switch(torqueMode)
  {
    case ACC_TORQUE:
      devicePrm.accelTorque = torqueValue;
      break;
    case DEC_TORQUE:
      devicePrm.decelTorque = torqueValue;
      break;
    case RUN_TORQUE:
      devicePrm.runTorque = torqueValue;
      break;
    case HOLD_TORQUE:
      devicePrm.holdTorque = torqueValue;
      if (devicePrm.motionState != INACTIVE)
      {
        break;
      }
    case CURRENT_TORQUE:
      devicePrm.currentTorque = torqueValue;
      Stspin820_Board_PwmRefSetFreqAndDutyCycle(devicePrm.refPwmFreq,torqueValue);
    default:
      devicePrm.updateTorque = FALSE;
      break; //ignore error
  }
}

/******************************************************//**
 * @brief Enable or disable the torque boost feature
 * @param[in] deviceId Unused parameter
 * @param[in] enable true to enable torque boost, false to disable
 * @retval None
 **********************************************************/
void Stspin820_SetTorqueBoostEnable(uint8_t deviceId, bool enable)
{
  devicePrm.torqueBoostEnable = enable;
}

/******************************************************//**
 * @brief Set the torque boost threshold
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] speedThreshold speed threshold above which the step mode is
 * changed to full step
 * @retval None
 **********************************************************/
void Stspin820_SetTorqueBoostThreshold(uint8_t deviceId, uint16_t speedThreshold)
{
  devicePrm.torqueBoostSpeedThreshold = speedThreshold;
}

/******************************************************//**
 * @brief  Stops the motor by using the device deceleration
 * @param[in] deviceId Unused parameter
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed if the device is in INACTIVE,
 * STANDBYTOINACTIVE or STANDBY state.
 **********************************************************/
bool Stspin820_SoftStop(uint8_t deviceId)
{	
  bool cmdExecuted = FALSE;
  if ((devicePrm.motionState & INACTIVE) != INACTIVE)
  {
    devicePrm.commandExecuted |= STSPIN820_SOFT_STOP_BIT_MASK;
    cmdExecuted = TRUE;
  }
  return (cmdExecuted);
}

/******************************************************//**
 * @brief Get the frequency of REF PWM of the specified device
 * @param[in] deviceId Unused parameter
 * @retval the frequency of REF PWM in Hz
 * @note
 **********************************************************/
uint32_t Stspin820_VrefPwmGetFreq(uint8_t deviceId)
{
  return devicePrm.refPwmFreq;
}

/******************************************************//**
 * @brief Set the frequency of REF PWM of the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] newFreq in Hz
 * @retval None
 * @note
 **********************************************************/
void Stspin820_VrefPwmSetFreq(uint8_t deviceId, uint32_t newFreq)
{ 
  devicePrm.refPwmFreq = newFreq;
  Stspin820_Board_PwmRefSetFreqAndDutyCycle(newFreq,devicePrm.currentTorque);
}

/******************************************************//**
 * @brief  Locks until the device state becomes Inactive
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_WaitWhileActive(uint8_t deviceId)
 {
  /* Wait while motor is running */
  while (((Stspin820_GetDeviceState(deviceId)&INACTIVE)!=INACTIVE)||\
   (((Stspin820_GetDeviceState(deviceId)&INACTIVE)==INACTIVE)&&(toggleOdd!=0)));
}

/**
  * @}
  */

/** @addtogroup Stspin820_Private_Functions
  * @{
  */  
/******************************************************//**
 * @brief Updates the current speed of the device
 * @param[in] deviceId Unused parameter
 * @param[in] newSpeed in pps
 * @retval None
 **********************************************************/
void Stspin820_ApplySpeed(uint8_t deviceId, uint16_t newSpeed)
{
  
  if (devicePrm.torqueBoostEnable != FALSE)
  {
    if (devicePrm.stepMode > STEP_MODE_1_256)
    {
      Stspin820_ErrorHandler(STSPIN820_ERROR_APPLY_SPEED);
    }
    if (devicePrm.stepMode != STEP_MODE_FULL)
    {
      if (((newSpeed>>devicePrm.stepModeLatched)>\
           devicePrm.torqueBoostSpeedThreshold)&&\
          (((devicePrm.commandExecuted & STSPIN820_MOVE_BIT_MASK) != MOVE_CMD) ||\
           ((devicePrm.stepsToTake-devicePrm.relativePos)>=\
            (1<<devicePrm.stepModeLatched))))
      {         
        if ((devicePrm.sequencerPosition & 0xFF) == 0x80)
       {
          Stspin820_Board_SetFullStep();
          devicePrm.stepMode = STEP_MODE_FULL;
          devicePrm.accu >>= devicePrm.stepModeLatched;
          newSpeed >>= devicePrm.stepModeLatched;
       }
      }
    }
    else if (((newSpeed <= devicePrm.torqueBoostSpeedThreshold) &&\
              (devicePrm.stepModeLatched != STEP_MODE_FULL))||\
             (((devicePrm.commandExecuted & STSPIN820_MOVE_BIT_MASK) == MOVE_CMD)&&\
               ((devicePrm.stepsToTake-devicePrm.relativePos)<=\
                (1<<devicePrm.stepModeLatched))))
    {
      Stspin820_SetStepMode(0, devicePrm.stepModeLatched);
      devicePrm.stepMode = devicePrm.stepModeLatched;
      devicePrm.accu <<= devicePrm.stepModeLatched;
      newSpeed <<= devicePrm.stepModeLatched;
    }
  }
  else if (devicePrm.stepMode != devicePrm.stepModeLatched)
  {
    //torqueBoostEnable has just been disabled
    Stspin820_SetStepMode(0, devicePrm.stepModeLatched);
    devicePrm.stepMode = devicePrm.stepModeLatched;
    devicePrm.accu <<= devicePrm.stepModeLatched;
    newSpeed <<= devicePrm.stepModeLatched;
  }
  
  if (newSpeed < STSPIN820_MIN_STCK_FREQ)
  {
    newSpeed = STSPIN820_MIN_STCK_FREQ;  
  }
  if (newSpeed > STSPIN820_MAX_STCK_FREQ)
  {
    newSpeed = STSPIN820_MAX_STCK_FREQ;
  }
  
  devicePrm.speed = newSpeed;
  Stspin820_Board_TimStckSetFreq(newSpeed);

}

/******************************************************//**
 * @brief Apply the set torque to the specified device
 * @param[in] deviceId Unused parameter
 * @param[in] torqueMode torque mode
 * @retval None
 * @note
 **********************************************************/
void Stspin820_ApplyTorque(uint8_t deviceId, motorTorqueMode_t torqueMode)
{
  uint8_t torqueValue = 0;
  devicePrm.updateTorque = FALSE;
  switch(torqueMode)
  {
    case ACC_TORQUE:
      devicePrm.currentTorque = devicePrm.accelTorque;
      break;
    case DEC_TORQUE:
      devicePrm.currentTorque = devicePrm.decelTorque;
      break;
    case RUN_TORQUE:
      devicePrm.currentTorque = devicePrm.runTorque;
      break;
    case HOLD_TORQUE:
      devicePrm.currentTorque = devicePrm.holdTorque;
      break;
    case CURRENT_TORQUE:
      break;
    default:
      return; //ignore error
  }
  torqueValue = devicePrm.currentTorque;
  Stspin820_Board_PwmRefSetFreqAndDutyCycle(devicePrm.refPwmFreq,torqueValue);
}

/******************************************************//**
 * @brief  Computes the speed profile according to the number of steps to move
 * @param[in] deviceId Unused parameter
 * @param[in] nbSteps number of steps to perform
 * @retval None
 * @note Using the acceleration and deceleration of the device,
 * this function determines the duration in steps of the acceleration,
 * steady and deceleration phases.
 * If the total number of steps to perform is big enough, a trapezoidal move
 * is performed (i.e. there is a steady phase where the motor runs at the maximum
 * speed.
 * Else, a triangular move is performed (no steady phase: the maximum speed is never
 * reached.
 **********************************************************/
void Stspin820_ComputeSpeedProfile(uint8_t deviceId, uint32_t nbSteps)
{
  uint32_t reqAccSteps; 
	uint32_t reqDecSteps;
   
  /* compute the number of steps to get the targeted speed */
  uint16_t minSpeed = devicePrm.minSpeed;
  reqAccSteps = (devicePrm.maxSpeed - minSpeed);
  reqAccSteps *= (devicePrm.maxSpeed + minSpeed);
  reqDecSteps = reqAccSteps;
  reqAccSteps /= (uint32_t)devicePrm.acceleration;
  reqAccSteps /= 2;

  /* compute the number of steps to stop */
  reqDecSteps /= (uint32_t)devicePrm.deceleration;
  reqDecSteps /= 2;

	if(( reqAccSteps + reqDecSteps ) > nbSteps)
	{	
    /* Triangular move  */
    /* reqDecSteps = (Pos * Dec) /(Dec+Acc) */
    uint32_t dec = devicePrm.deceleration;
    uint32_t acc = devicePrm.acceleration;
    
    reqDecSteps =  ((uint32_t) dec * nbSteps) / (acc + dec);
    if (reqDecSteps > 1)
    {
      reqAccSteps = reqDecSteps - 1;
      if(reqAccSteps == 0)
      {
        reqAccSteps = 1;
      }      
    }
    else
    {
      reqAccSteps = 0;
    }
    devicePrm.endAccPos = reqAccSteps;
    devicePrm.startDecPos = reqDecSteps;
	}
	else
	{	 
    /* Trapezoidal move */
    /* accelerating phase to endAccPos */
    /* steady phase from  endAccPos to startDecPos */
    /* decelerating from startDecPos to stepsToTake*/
    devicePrm.endAccPos = reqAccSteps;
    devicePrm.startDecPos = nbSteps - reqDecSteps - 1;
	}
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void Stspin820_FlagInterruptHandler(void)
{
  if (flagInterruptCallback != 0)
  {
    flagInterruptCallback();
  }
}

/******************************************************//**
 * @brief  Set the parameters of the device whose values are not defined in
 * stspin820_target_config.h
 * @retval None
 **********************************************************/
void Stspin820_SetDeviceParamsOtherValues(void)
{
  uint16_t tmp;

  devicePrm.accu = 0;
  devicePrm.currentPosition = 0;
  devicePrm.sequencerPosition = 0;
  devicePrm.endAccPos = 0;
  devicePrm.relativePos = 0;
  devicePrm.startDecPos = 0;
  devicePrm.stepsToTake = 0;
  devicePrm.updateTorque = FALSE;
  devicePrm.speed = 0;
  devicePrm.commandExecuted = NO_CMD;
  devicePrm.direction = FORWARD;
  tmp = devicePrm.minSpeed;
  if (((devicePrm.torqueBoostEnable != FALSE)&&\
       (devicePrm.torqueBoostSpeedThreshold>STSPIN820_MAX_STCK_FREQ))||\
      (tmp>devicePrm.maxSpeed))
  {
    Stspin820_ErrorHandler(STSPIN820_ERROR_INIT);
  }
}

/******************************************************//**
 * @brief  Set the parameters of the device to values of initDevicePrm structure
 * @param pInitDevicePrm structure containing values to initialize the device 
 * parameters
 * @retval None
 **********************************************************/
void Stspin820_SetDeviceParamsToGivenValues(Stspin820_Init_t* pInitDevicePrm)
{
  devicePrm.motionState = STANDBY;;

  if (Stspin820_SetAcceleration(0,pInitDevicePrm->acceleration)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_ACCELERATION); 
  if (Stspin820_SetDeceleration(0,pInitDevicePrm->deceleration)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_DECELERATION);
  if (Stspin820_SetMaxSpeed(0,pInitDevicePrm->maxSpeed)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_MAX_SPEED);
  if (Stspin820_SetMinSpeed(0,pInitDevicePrm->minSpeed)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_MIN_SPEED);
 
  Stspin820_VrefPwmSetFreq(0,pInitDevicePrm->vrefPwmFreq);
  Stspin820_SetTorque(0,ACC_TORQUE,pInitDevicePrm->accelTorque);
  Stspin820_SetTorque(0,DEC_TORQUE,pInitDevicePrm->decelTorque);
  Stspin820_SetTorque(0,RUN_TORQUE,pInitDevicePrm->runTorque);
  Stspin820_SetTorque(0,HOLD_TORQUE,pInitDevicePrm->holdTorque);
  devicePrm.torqueBoostEnable = pInitDevicePrm->torqueBoostEnable;
  devicePrm.torqueBoostSpeedThreshold = pInitDevicePrm->torqueBoostSpeedThreshold;
  Stspin820_SetStopMode(0,pInitDevicePrm->stopMode);

  Stspin820_SetDeviceParamsOtherValues();
  
  /* Eventually deactivate motor */
  if ((devicePrm.motionState != INACTIVE)&&\
      (devicePrm.motionState != STANDBY))
  {
    Stspin820_HardHiZ(0);
  }

  /* Enter standby */
  Stspin820_Board_Reset();
  
  /* Reset the microstepping sequencer position */
  devicePrm.sequencerPosition = 0;

  /* Reset current and mark positions */
  devicePrm.currentPosition = 0; 
  devicePrm.markPosition = 0;
  
  /* Set predefined step mode */
  Stspin820_SetStepMode(0, pInitDevicePrm->stepMode);
  
  /* Wait */
  Stspin820_Board_Delay(SELECT_STEP_MODE_DELAY);
  
  /* Exit standby */
  Stspin820_Board_ReleaseReset();
  
    /* Let a delay after reset release*/
  Stspin820_Board_Delay(AFTER_STANDBY_EXIT_DEAD_TIME);
  
}

/******************************************************//**
 * @brief  Sets the parameters of the device to predefined values
 * from stspin820_target_config.h
 * @retval None
 **********************************************************/
void Stspin820_SetDeviceParamsToPredefinedValues(void)
{
  devicePrm.motionState = STANDBY;

  if (Stspin820_SetAcceleration(0,STSPIN820_CONF_PARAM_ACC)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_ACCELERATION);
  if (Stspin820_SetDeceleration(0,STSPIN820_CONF_PARAM_DEC)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_DECELERATION);
  if (Stspin820_SetMaxSpeed(0,STSPIN820_CONF_PARAM_RUNNING_SPEED)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_MAX_SPEED);
  if (Stspin820_SetMinSpeed(0,STSPIN820_CONF_PARAM_MIN_SPEED)==FALSE) Stspin820_ErrorHandler(STSPIN820_ERROR_SET_MIN_SPEED);

  Stspin820_VrefPwmSetFreq(0,STSPIN820_CONF_PARAM_REF_PWM_FREQUENCY);
  Stspin820_SetTorque(0,ACC_TORQUE,STSPIN820_CONF_PARAM_ACC_TORQUE);
  Stspin820_SetTorque(0,DEC_TORQUE,STSPIN820_CONF_PARAM_DEC_TORQUE);
  Stspin820_SetTorque(0,RUN_TORQUE,STSPIN820_CONF_PARAM_RUNNING_TORQUE);
  Stspin820_SetTorque(0,HOLD_TORQUE,STSPIN820_CONF_PARAM_HOLDING_TORQUE);
  devicePrm.torqueBoostEnable = STSPIN820_CONF_PARAM_TORQUE_BOOST_EN;
  devicePrm.torqueBoostSpeedThreshold = STSPIN820_CONF_PARAM_TORQUE_BOOST_TH;
  Stspin820_SetStopMode(0,STSPIN820_CONF_PARAM_AUTO_HIZ_STOP);

  Stspin820_SetDeviceParamsOtherValues();
  
    /* Eventually deactivate motor */
  if ((devicePrm.motionState != INACTIVE)&&\
      (devicePrm.motionState != STANDBY))
  {
    Stspin820_HardHiZ(0);
  }

  /* Enter standby */
  Stspin820_Board_Reset();
  
  /* Reset the microstepping sequencer position */
  devicePrm.sequencerPosition = 0;

  /* Reset current and mark positions */
  devicePrm.currentPosition = 0; 
  devicePrm.markPosition = 0;
  
  /* Set predefined step mode */
  Stspin820_SetStepMode(0, STSPIN820_CONF_PARAM_STEP_MODE);
    
  /* Wait */
  Stspin820_Board_Delay(SELECT_STEP_MODE_DELAY);
  
  /* Exit standby */
  Stspin820_Board_ReleaseReset();
  
  /* Let a delay after reset release*/
  Stspin820_Board_Delay(AFTER_STANDBY_EXIT_DEAD_TIME);
}

/******************************************************//**
 * @brief Initialises the bridge parameters to start the movement
 * and enable the power bridge
 * @param[in] deviceId Unused parameter
 * @retval None
 **********************************************************/
void Stspin820_StartMovement(uint8_t deviceId)  
{
  deviceId = 0;
  
  /* Enable STSPIN820 powerstage */
  Stspin820_Enable(deviceId);
  toggleOdd = 0;
  devicePrm.accu = 0;
  devicePrm.relativePos = 0;  
  if ((devicePrm.endAccPos == 0)&&\
      (devicePrm.commandExecuted != RUN_CMD))
  {
    devicePrm.motionState = DECELERATING;
    Stspin820_ApplyTorque(deviceId, DEC_TORQUE);
  }
  else
  {
    devicePrm.motionState = ACCELERATING;
    Stspin820_ApplyTorque(deviceId, ACC_TORQUE);
  }
  Stspin820_Board_PwmRefStart();
  /* Initialize the step clock timer */
  Stspin820_Board_TimStckInit();
  /* Program the step clock */
  Stspin820_Board_TimStckCompareInit();
  Stspin820_ApplySpeed(deviceId, devicePrm.minSpeed);
  Stspin820_Board_TimStckStart();
}

/******************************************************//**
 * @brief  Handles the device state machine at each pulse
 * @param[in] deviceId Unused parameter
 * @retval None
 * @note Must only be called by the timer ISR
 **********************************************************/
void Stspin820_StepClockHandler(uint8_t deviceId)
{
  uint32_t stepModeShift = devicePrm.stepModeLatched - devicePrm.stepMode;
  uint16_t tmp;
  deviceId = 0;
  
  if (devicePrm.motionState == STANDBYTOINACTIVE)
  {
    if (toggleOdd != 0)
    {
      toggleOdd = 0;
      if (devicePrm.sequencerPosition == 0)
      {
        if (Stspin820_Board_TimStckStop(&toggleOdd) == 0)
        {
          Stspin820_ErrorHandler(STSPIN820_ERROR_STEP_CLOCK);
        }
        return;
      }      
    }
    else
    {
      toggleOdd = 1;
      tmp = (1 << (STEP_MODE_1_256-devicePrm.stepMode));
      devicePrm.sequencerPosition -= tmp;
    }
    Stspin820_Board_TimStckSetFreq(STSPIN820_MAX_STCK_FREQ);
    return;
  }  
  
  if (toggleOdd == 0)
  {
    toggleOdd = 1;
  }
  else
  {
    toggleOdd = 0;
    /* Incrementation of the relative position */
    devicePrm.relativePos += (1 << stepModeShift);

    /* Incrementation of the current position */
    if (devicePrm.direction != BACKWARD)
    {
      devicePrm.currentPosition += (1 << stepModeShift);
      tmp = (1 << (STEP_MODE_1_256-devicePrm.stepMode));
      devicePrm.sequencerPosition += tmp;
      if (devicePrm.sequencerPosition >= (SEQUENCER_MAX_VALUE+1))
      {
        devicePrm.sequencerPosition -= (SEQUENCER_MAX_VALUE+1);
      }
    }
    else
    {
      devicePrm.currentPosition -= (1 << stepModeShift);
      tmp = (1 << (STEP_MODE_1_256-devicePrm.stepMode));
      devicePrm.sequencerPosition -= tmp;
      if (devicePrm.sequencerPosition < 0)
      {
        devicePrm.sequencerPosition += (SEQUENCER_MAX_VALUE+1);
      }
    }

    switch (devicePrm.motionState) 
    {
      case ACCELERATING: 
      {
          uint32_t relPos = devicePrm.relativePos;
          uint32_t endAccPos = devicePrm.endAccPos;
          uint16_t speed = devicePrm.speed;
          uint32_t acc = ((uint32_t)devicePrm.acceleration << 16)>>stepModeShift;
        
          if (((devicePrm.commandExecuted&(STSPIN820_SOFT_STOP_BIT_MASK|STSPIN820_DIR_CHANGE_BIT_MASK))!=0)||\
              ((devicePrm.commandExecuted==MOVE_CMD)&&(relPos>=devicePrm.startDecPos)))
          {
            devicePrm.motionState = DECELERATING;
            devicePrm.accu = 0;
            /* Apply decelerating torque */
            Stspin820_ApplyTorque(deviceId, DEC_TORQUE);
          }
          else if ((speed>=(devicePrm.maxSpeed>>stepModeShift))||\
                   ((devicePrm.commandExecuted==MOVE_CMD)&&(relPos >= endAccPos)))
          {
            devicePrm.motionState = STEADY;
            /* Apply running torque */
            Stspin820_ApplyTorque(deviceId, RUN_TORQUE);
          }
          else
          {
            bool speedUpdated = FALSE;
            /* Go on accelerating */
            if (speed==0) speed =1;
            devicePrm.accu += acc / speed;
            while (devicePrm.accu>=(0X10000L))
            {
              devicePrm.accu -= (0X10000L);
              speed +=1;
              speedUpdated = TRUE;
            }
          
            if (speedUpdated)
            {
              if (speed>(devicePrm.maxSpeed>>stepModeShift))
              {
                speed = devicePrm.maxSpeed>>stepModeShift;
              }    
              devicePrm.speed = speed;
            }
            
            if (devicePrm.updateTorque!=FALSE)
            {
              /* Apply accelerating torque */
              Stspin820_ApplyTorque(deviceId, ACC_TORQUE);              
            }
          }
          break;
      }
      case STEADY: 
      {
        uint16_t maxSpeed = devicePrm.maxSpeed>>stepModeShift;
        uint32_t relativePos = devicePrm.relativePos;
        if (devicePrm.updateTorque!=FALSE)
        {
          /* Apply accelerating torque */
          Stspin820_ApplyTorque(deviceId, RUN_TORQUE);
        }
        if  (((devicePrm.commandExecuted&(STSPIN820_SOFT_STOP_BIT_MASK|STSPIN820_DIR_CHANGE_BIT_MASK))!=0)||\
             ((devicePrm.commandExecuted==MOVE_CMD)&&\
              (relativePos>=(devicePrm.startDecPos)))||\
             ((devicePrm.commandExecuted==RUN_CMD)&&\
              (devicePrm.speed>maxSpeed)))
        {
          devicePrm.motionState = DECELERATING;
          devicePrm.accu = 0;
          /* Apply decelerating torque */
          Stspin820_ApplyTorque(deviceId, DEC_TORQUE);
        }
        else if ((devicePrm.commandExecuted==RUN_CMD)&&(devicePrm.speed<maxSpeed))
        {
          devicePrm.motionState = ACCELERATING;
          devicePrm.accu = 0;
          /* Apply accelerating torque */
          Stspin820_ApplyTorque(deviceId, ACC_TORQUE);
        }
        break;
      }
      case DECELERATING: 
      {
        uint32_t relativePos = devicePrm.relativePos;
        uint16_t speed = devicePrm.speed;
        uint32_t dec = ((uint32_t)devicePrm.deceleration << 16)>>stepModeShift;
        if ((((devicePrm.commandExecuted&(STSPIN820_SOFT_STOP_BIT_MASK|STSPIN820_DIR_CHANGE_BIT_MASK))!=0)&&\
             (speed<=(devicePrm.minSpeed>>stepModeShift)))||\
            ((devicePrm.commandExecuted==MOVE_CMD)&&(relativePos>=devicePrm.stepsToTake)))
        {
          /* Motion process complete */
          if ((devicePrm.commandExecuted&STSPIN820_DIR_CHANGE_BIT_MASK)!=0)
          {
            devicePrm.commandExecuted&=~STSPIN820_DIR_CHANGE_BIT_MASK;
            if (devicePrm.direction==BACKWARD) devicePrm.direction=FORWARD;
            else devicePrm.direction=BACKWARD;
            Stspin820_Board_SetDirectionGpio(devicePrm.direction);
            if ((devicePrm.commandExecuted&STSPIN820_SOFT_STOP_BIT_MASK)==0)
            {
              devicePrm.motionState = ACCELERATING;
              devicePrm.accu = 0;
              /* Apply accelerating torque */
              Stspin820_ApplyTorque(deviceId, ACC_TORQUE);
              break;
            }
          }
          if (devicePrm.stopMode==HOLD_MODE)
          {
            Stspin820_HardStop(deviceId);
          }
          else if (devicePrm.stopMode==STANDBY_MODE)
          {
            Stspin820_PutDeviceInStandby(deviceId);
          }
          else
          {
            Stspin820_HardHiZ(deviceId);
          }
        }
        else if ((devicePrm.commandExecuted==RUN_CMD)&&
                 (speed<=(devicePrm.maxSpeed>>stepModeShift)))
        {
          devicePrm.motionState = STEADY;
          /* Apply running torque */
          Stspin820_ApplyTorque(deviceId, RUN_TORQUE);
        }
        else
        {
          /* Go on decelerating */
          if (speed>(devicePrm.minSpeed>>stepModeShift))
          {
            bool speedUpdated = FALSE;
            if (speed==0) speed =1;
            devicePrm.accu += dec / speed;
            while (devicePrm.accu>=(0X10000L))
            {
              devicePrm.accu -= (0X10000L);
              if (speed>1)
              {  
                speed -=1;
              }
              speedUpdated = TRUE;
            }
          
            if (speedUpdated)
            {
              if (speed<(devicePrm.minSpeed>>stepModeShift))
              {
                speed = devicePrm.minSpeed>>stepModeShift;
              }  
              devicePrm.speed = speed;
            }
            
            if (devicePrm.updateTorque!=FALSE)
            {
              /* Apply decelerating torque */
              Stspin820_ApplyTorque(deviceId, DEC_TORQUE);
            }
          }
        }
        break;
      }
      default: 
      {
        break;
      }
    }
  }
  if ((devicePrm.motionState & INACTIVE) != INACTIVE)
  {
    Stspin820_ApplySpeed(deviceId, devicePrm.speed);
  }
  else
  {
    if (Stspin820_Board_TimStckStop(&toggleOdd) == 0)
    {
      Stspin820_ErrorHandler(STSPIN820_ERROR_STEP_CLOCK);
    }
  }
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
