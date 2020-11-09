/******************************************************//**
  * @file    stspin820.h 
  * @author  STM
  * @version V1.0.0
  * @date    August 7th, 2017
  * @brief   Header for STSPIN820 driver (fully integrated microstepping motor driver)
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN820_H
#define __STSPIN820_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stspin820_target_config.h"
#include "motor.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STSPIN820
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup Stspin820_Exported_Constants Stspin820 Exported Constants
  * @{
  */   
/// Current FW major version
#define STSPIN820_FW_MAJOR_VERSION (uint8_t)(1)
/// Current FW minor version
#define STSPIN820_FW_MINOR_VERSION (uint8_t)(0)
/// Current FW patch version
#define STSPIN820_FW_PATCH_VERSION (uint8_t)(0)
/// Current FW version
#define STSPIN820_FW_VERSION       (uint32_t)((STSPIN820_FW_MAJOR_VERSION<<16)|\
                                              (STSPIN820_FW_MINOR_VERSION<<8)|\
                                              (STSPIN820_FW_PATCH_VERSION))

/// Max position
#define STSPIN820_MAX_POSITION           (0x7FFFFFFF)

/// Min position
#define STSPIN820_MIN_POSITION           (0x80000000)

/// Position range
#define STSPIN820_POSITION_RANGE         ((uint32_t)(STSPIN820_MAX_POSITION -\
                                                        STSPIN820_MIN_POSITION))
/// STSPIN820 error base number
#define STSPIN820_ERROR_BASE             (0xA000)

/// run bit mask
#define STSPIN820_RUN_BIT_MASK           (0x01)

/// move bit mask
#define STSPIN820_MOVE_BIT_MASK          (0x02)

/// soft stop bit mask
#define STSPIN820_SOFT_STOP_BIT_MASK     (0x04)
   
/// direction change bit mask
#define STSPIN820_DIR_CHANGE_BIT_MASK    (0x08)

/// Maximum frequency of the step clock frequency in Hz
#define STSPIN820_MAX_STCK_FREQ          (10000)

/// Minimum frequency of the step clock frequency in Hz
#define STSPIN820_MIN_STCK_FREQ          (8)

/// Minimum duration of standby 
#define STANDBY_MIN_DURATION             (1)
    
/// Dead time after standby exit
#define AFTER_STANDBY_EXIT_DEAD_TIME     (1)

/// Reset delay to select step mode
#define SELECT_STEP_MODE_DELAY           (1)

/// PWM REF and bridges disable delay
#define DISABLE_DELAY                    (1)

/// Microstepping sequencer maximum value
#define SEQUENCER_MAX_VALUE              (uint16_t)(0x3FF)

/**
  * @}
  */

/* Exported Variables --------------------------------------------------------*/

/** @addtogroup Stspin820_Exported_Variables
  * @{
  */
extern motorDrv_t   stspin820Drv;
/**
  * @}
  */
     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup Stspin820_Exported_Types Stspin820 Exported Types
  * @{
  */
    
/** @defgroup Error_Types Error Types
  * @{
  */
/// Errors
typedef enum {
  STSPIN820_ERROR_SET_HOME         = STSPIN820_ERROR_BASE,      /// Error while setting home position
  STSPIN820_ERROR_SET_MAX_SPEED    = STSPIN820_ERROR_BASE + 1,  /// Error while setting max speed
  STSPIN820_ERROR_SET_MIN_SPEED    = STSPIN820_ERROR_BASE + 2,  /// Error while setting min speed
  STSPIN820_ERROR_SET_ACCELERATION = STSPIN820_ERROR_BASE + 3,  /// Error while setting acceleration
  STSPIN820_ERROR_SET_DECELERATION = STSPIN820_ERROR_BASE + 4,  /// Error while setting decelaration
  STSPIN820_ERROR_MCU_OSC_CONFIG   = STSPIN820_ERROR_BASE + 5,  /// Error while configuring mcu oscillator
  STSPIN820_ERROR_MCU_CLOCK_CONFIG = STSPIN820_ERROR_BASE + 6,  /// Error while configuring mcu clock
  STSPIN820_ERROR_POSITION         = STSPIN820_ERROR_BASE + 7,  /// Unexpected current position (wrong number of steps)
  STSPIN820_ERROR_SPEED            = STSPIN820_ERROR_BASE + 8,  /// Unexpected current speed
  STSPIN820_ERROR_INIT             = STSPIN820_ERROR_BASE + 9,  /// Unexpected number of devices or unexpected value for predefined parameter
  STSPIN820_ERROR_SET_DIRECTION    = STSPIN820_ERROR_BASE + 10, /// Error while setting direction
  STSPIN820_ERROR_SET_STEP_MODE    = STSPIN820_ERROR_BASE + 11, /// Attempt to set an unsupported step mode
  STSPIN820_ERROR_APPLY_SPEED      = STSPIN820_ERROR_BASE + 12, /// Error while applying speed
  STSPIN820_ERROR_SET_TORQUE       = STSPIN820_ERROR_BASE + 13, /// Error while setting torque
  STSPIN820_ERROR_STEP_CLOCK       = STSPIN820_ERROR_BASE + 14  /// Error related to step clock
}errorTypes_t;
/**
  * @}
  */

/** @defgroup Device_Commands Device Commands
  * @{
  */
/// Device commands 
typedef enum {
  NO_CMD              = 0x00, 
  RUN_CMD             = (STSPIN820_RUN_BIT_MASK),
  MOVE_CMD            = (STSPIN820_MOVE_BIT_MASK),
} deviceCommand_t;
/**
  * @}
  */


/** @defgroup Device_Parameters Device Parameters
  * @{
  */

/// Device Parameters Structure Type
typedef struct {
    /// accumulator used to store speed increase smaller than 1 pps
    volatile uint32_t accu;           
    /// Position in microstep according to current step mode
    volatile int32_t currentPosition;
    /// Position of sequencer
    volatile int16_t sequencerPosition;
    /// mark position in microstep (motor position control mode)
    volatile int32_t markPosition;
    /// position in microstep at the end of the accelerating phase
    volatile uint32_t endAccPos;      
    /// nb of in microstep performed from the beggining of the goto or the move command 
    volatile uint32_t relativePos;    
    /// position in microstep step at the start of the decelerating phase
    volatile uint32_t startDecPos;    
    /// nb of microstep steps to perform for the goto or move commands
    uint32_t stepsToTake;
    
    /// constant speed phase torque value (%)
    volatile uint8_t runTorque;
    /// acceleration phase torque value (%)
    volatile uint8_t accelTorque;
    /// deceleration phase torque value (%)
    volatile uint8_t decelTorque;
    /// holding phase torque value (%)
    volatile uint8_t holdTorque;
    /// current selected torque value
    volatile uint8_t currentTorque;
    /// torque update
    volatile bool updateTorque;
    /// PWM frequency used to generate REF voltage
    volatile uint32_t refPwmFreq;
    /// torque boost enable
    volatile bool torqueBoostEnable;
    /// torque boost speed threshold
    volatile uint16_t torqueBoostSpeedThreshold;
    
    /// acceleration in pps^2 
    volatile uint16_t acceleration;
    /// deceleration in pps^2
    volatile uint16_t deceleration;
    /// max speed in pps (speed use for goto or move command)
    volatile uint16_t maxSpeed;
    /// min speed in pps
    volatile uint16_t minSpeed;
    /// current speed in pps
    volatile uint16_t speed;
    
    /// command under execution
    volatile deviceCommand_t commandExecuted; 
    /// FORWARD or BACKWARD direction
    volatile motorDir_t direction;                 
    /// current state of the device
    volatile motorState_t motionState;
    /// current step mode
    volatile motorStepMode_t stepMode;
    /// latched step mode
    motorStepMode_t stepModeLatched;
    /// current stop mode
    motorStopMode_t stopMode;
    
}deviceParams_t; 

/// Motor driver initialization structure definition  
typedef struct
{
  /// acceleration in pps^2
  uint16_t acceleration;
  /// deceleration in pps^2
  uint16_t deceleration;
  /// max speed in pps (speed use for goto or move command)
  uint16_t maxSpeed;
  /// min speed in pps
  uint16_t minSpeed;
  /// acceleration phase torque value (%)
  uint8_t accelTorque;
  /// deceleration phase torque value (%)
  uint8_t decelTorque;
  /// constant speed phase torque value (%)
  uint8_t runTorque;
  /// holding phase torque value (%)
  uint8_t holdTorque;
  /// torque boost enable
  bool torqueBoostEnable;
  /// torque boost speed threshold
  uint16_t torqueBoostSpeedThreshold;
  /// step mode
  motorStepMode_t stepMode;
  /// stop mode
  motorStopMode_t stopMode;
  /// PWM frequency used to generate REF voltage
  uint32_t vrefPwmFreq;
} Stspin820_Init_t;
    
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup Stspin820_Exported_Functions Stspin820 Exported Functions
  * @{
  */   
motorDrv_t* Stspin820_GetMotorHandle(void);                      //Return handle of the motor driver handle
void Stspin820_Init(void* pInit);                                 //Start the STSPIN820 library
uint16_t Stspin820_ReadId(void);                                 //Read Id to get driver instance
void Stspin820_AttachErrorHandler(void (*callback)(uint16_t));   //Attach a user callback to the error handler
void Stspin820_AttachFlagInterrupt(void (*callback)(void));      //Attach a user callback to the flag Interrupt
uint8_t Stspin820_CheckStatusHw(void);                           //Check if STSPIN820 has a fault by reading EN pin position
void Stspin820_Disable(uint8_t deviceId);                        //Disable the power stage of the specified device
void Stspin820_Enable(uint8_t deviceId);                         //Enable the power stage of the specified device
void Stspin820_ErrorHandler(uint16_t error);                     //Error handler which calls the user callback
void Stspin820_ExitDeviceFromStandby(uint8_t deviceId);          //Exit STSPIN820 device from standby 
uint16_t Stspin820_GetAcceleration(uint8_t deviceId);            //Return the acceleration in pps^2
uint16_t Stspin820_GetCurrentSpeed(uint8_t deviceId);            //Return the current speed in pps
motorDecayMode_t Stspin820_GetDecayMode(uint8_t deviceId);       //Return the device decay mode
uint16_t Stspin820_GetDeceleration(uint8_t deviceId);            //Return the deceleration in pps^2
motorState_t Stspin820_GetDeviceState(uint8_t deviceId);         //Return the device state
motorDir_t Stspin820_GetDirection(uint8_t deviceId);             //Get the motor current direction
uint32_t Stspin820_GetFwVersion(void);                           //Return the FW version
int32_t Stspin820_GetMark(uint8_t deviceId);                     //Return the mark position 
uint16_t Stspin820_GetMaxSpeed(uint8_t deviceId);                //Return the max speed in pps
uint16_t Stspin820_GetMinSpeed(uint8_t deviceId);                //Return the min speed in pps
uint8_t Stspin820_GetNbDevices(void);                            //Return the nupber of devices
int32_t Stspin820_GetPosition(uint8_t deviceId);                 //Return the ABS_POSITION (32b signed)
motorStepMode_t Stspin820_GetStepMode(uint8_t deviceId);         //Get the motor step mode
motorStopMode_t Stspin820_GetStopMode(uint8_t deviceId);         //Get the selected mode to stop the motor
uint8_t Stspin820_GetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode);
bool Stspin820_GetTorqueBoostEnable(uint8_t deviceId);           //Get the torque boost feature status
uint16_t Stspin820_GetTorqueBoostThreshold(uint8_t deviceId);    //Get the torque boost threshold
void Stspin820_GoHome(uint8_t deviceId);                         //Move to the home position
void Stspin820_GoMark(uint8_t deviceId);                         //Move to the Mark position
void Stspin820_GoTo(uint8_t deviceId, int32_t targetPosition);   //Go to the specified position
void Stspin820_GoToDir(uint8_t deviceId,\
  motorDir_t direction,\
  int32_t targetPosition);                                       //Go to the specified position using the specified direction
void Stspin820_HardStop(uint8_t deviceId);                       //Stop the motor and keep holding torque
void Stspin820_HardHiZ(uint8_t deviceId);                        //Stop the motor and disable the power bridge
void Stspin820_Move(uint8_t deviceId,                            //Move the motor of the specified number of steps
                motorDir_t direction,
                uint32_t stepCount);
void Stspin820_PutDeviceInStandby(uint8_t deviceId);              //Put STSPIN820 device in standby (low power consumption)
void Stspin820_Run(uint8_t deviceId, motorDir_t direction);       //Run the motor 
bool Stspin820_SetAcceleration(uint8_t deviceId,uint16_t newAcc); //Set the acceleration in pps^2
bool Stspin820_SetDeceleration(uint8_t deviceId,uint16_t newDec); //Set the deceleration in pps^2
void Stspin820_SetDecayMode(uint8_t deviceId,\
 motorDecayMode_t decay);                                         //Set the STSPIN820 decay mode pin
void Stspin820_SetDirection(uint8_t deviceId,                     //Set the STSPIN820 direction pin
                        motorDir_t direction);
void Stspin820_SetHome(uint8_t deviceId);                         //Set current position to be the home position
void Stspin820_SetMark(uint8_t deviceId);                         //Set current position to be the Markposition
bool Stspin820_SetMaxSpeed(uint8_t deviceId,uint16_t newMaxSpeed);//Set the max speed in pps
bool Stspin820_SetMinSpeed(uint8_t deviceId,uint16_t newMinSpeed);//Set the min speed in pps
bool Stspin820_SetNbDevices(uint8_t nbDevices);
bool Stspin820_SetStepMode(uint8_t deviceId,                      
  motorStepMode_t stepMode);                                      // Step mode selection
void Stspin820_SetStopMode(uint8_t deviceId,\
  motorStopMode_t stopMode);                                      //Select the mode to stop the motor
void Stspin820_SetTorque(uint8_t deviceId,\
  motorTorqueMode_t torqueMode,\
  uint8_t torqueValue);
void Stspin820_SetTorqueBoostEnable(uint8_t deviceId, bool enable); // Enable or disable the torque boost feature
void Stspin820_SetTorqueBoostThreshold(uint8_t deviceId, uint16_t speedThreshold); //Set the torque boost threshold
bool Stspin820_SoftStop(uint8_t deviceId);                        //Progressively stop the motor by using the device deceleration and set deceleration torque
uint32_t Stspin820_VrefPwmGetFreq(uint8_t deviceId);              //Get the frequency of REF PWM of the specified device
void Stspin820_VrefPwmSetFreq(uint8_t deviceId, uint32_t newFreq);//Set the frequency of REF PWM of the specified device
void Stspin820_WaitWhileActive(uint8_t deviceId);                 //Wait for the device state becomes Inactive
/**
  * @}
  */

/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */
///Delay of the requested number of milliseconds
extern void Stspin820_Board_Delay(uint32_t delay);
///Disable the power bridges (leave the output bridges HiZ)    
extern void Stspin820_Board_Disable(void);
///Disable Irq
extern void Stspin820_Board_DisableIrq(void);
//Get the EN FAULT pin state
extern uint32_t Stspin820_Board_EN_AND_FAULT_PIN_GetState(void);
///Enable the power bridges (leave the output bridges HiZ)
extern void Stspin820_Board_Enable(void); 
///Enable Irq
extern void Stspin820_Board_EnableIrq(void);
///Initialise GPIOs used for STSPIN820
extern void Stspin820_Board_GpioInit(void);
///Init the reference voltage pwm
extern void Stspin820_Board_PwmRefInit(void); 
///Set the frequency and duty cycle of PWM used for the reference voltage generation
extern void Stspin820_Board_PwmRefSetFreqAndDutyCycle(uint32_t newFreq, uint8_t dutyCycle);
///Start the reference voltage pwm
extern void Stspin820_Board_PwmRefStart(void);
///Stop the reference voltage pwm
extern void Stspin820_Board_PwmRefStop(void);
///Reset the STSPIN820 reset pin
extern void Stspin820_Board_ReleaseReset(void);
///Set the STSPIN820 reset pin 
extern void Stspin820_Board_Reset(void);
///Set decay GPIO
extern void Stspin820_Board_SetDecayGpio(uint8_t gpioState);
///Get decay GPIO
extern uint8_t Stspin820_Board_GetDecayGpio(void);
///Set direction GPIO
extern void Stspin820_Board_SetDirectionGpio(uint8_t gpioState);
///Select Full Step mode
extern void Stspin820_Board_SetFullStep(void);
///Select the STSPIN820 mode1, mode2, and mode3 pins levels
extern bool Stspin820_Board_SetModePins(uint8_t modePin1Level,\
  uint8_t modePin2Level,\
  uint8_t modePin3Level);
///Step clock compare value initialization
extern void Stspin820_Board_TimStckCompareInit(void);
///DeInit the timer
extern void Stspin820_Board_TimStckDeInit(void);
///Init the timer
extern void Stspin820_Board_TimStckInit(void);
///Set step clock frequency
extern void Stspin820_Board_TimStckSetFreq(uint16_t newFreq);
///Start step clock
extern void Stspin820_Board_TimStckStart(void);
///Stop the timer
extern uint8_t Stspin820_Board_TimStckStop(volatile uint8_t *pToggleOdd);
///Unselect Full Step mode
extern void Stspin820_Board_UnsetFullStep(void);
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

#endif /* #ifndef __STSPIN820_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
