/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes
#include <stdio.h>
#include <stdlib.h> // for abs()
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "comms.h"

#ifdef VARIANT_KiSC
#include "kisc-hoverboard-protocol.h"
#include "new-protocol.h"
#endif

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern UART_HandleTypeDef huart2;

extern int16_t batVoltage;
extern uint8_t backwardDrive;
extern uint8_t buzzerCount;             // global variable for the buzzer counts. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern volatile uint32_t timeoutCntGen; // global counter for general timeout counter
extern volatile uint8_t  timeoutFlgGen; // global flag for general timeout counter
extern volatile uint32_t main_loop_counter;

//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------
// Matlab defines - from auto-code generation
//---------------
RT_MODEL rtM_Left_;                     /* Real-time model */
RT_MODEL rtM_Right_;                    /* Real-time model */
RT_MODEL *const rtM_Left  = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

extern P rtP_Left;                      /* Block parameters (auto storage) */
DW       rtDW_Left;                     /* Observable states */
ExtU     rtU_Left;                      /* External inputs */
ExtY     rtY_Left;                      /* External outputs */

P        rtP_Right;                     /* Block parameters (auto storage) */
DW       rtDW_Right;                    /* Observable states */
ExtU     rtU_Right;                     /* External inputs */
ExtY     rtY_Right;                     /* External outputs */
//---------------

uint8_t  inIdx      = 0;
uint8_t  inIdx_prev = 0;

#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)
InputStruct input1[INPUTS_NR] = { {0, 0, 0, PRI_INPUT1}, {0, 0, 0, AUX_INPUT1} };
InputStruct input2[INPUTS_NR] = { {0, 0, 0, PRI_INPUT2}, {0, 0, 0, AUX_INPUT2} };
#else
InputStruct input1[INPUTS_NR] = { {0, 0, 0, PRI_INPUT1} };
InputStruct input2[INPUTS_NR] = { {0, 0, 0, PRI_INPUT2} };
#endif

int16_t  speedAvg;                      // average measured speed
int16_t  speedAvgAbs;                   // average measured speed in absolute
uint8_t  timeoutFlgADC    = 0;          // Timeout Flag for ADC Protection:    0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
uint8_t  timeoutFlgSerial = 0;          // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

uint8_t  ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t  ctrlModReq    = CTRL_MOD_REQ;  // Final control mode request 

uint16_t VirtAddVarTab[NB_OF_VAR] = {1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009,
                                     1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018};


//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static int16_t INPUT_MAX;             // [-] Input target maximum limitation
static int16_t INPUT_MIN;             // [-] Input target minimum limitation


#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
  static uint8_t  cur_spd_valid  = 0;
  static uint8_t  inp_cal_valid  = 0;
#endif


#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
static uint8_t  rx_buffer_L[SERIAL_BUFFER_SIZE];      // USART Rx DMA circular buffer
static uint32_t rx_buffer_L_len = ARRAY_LEN(rx_buffer_L);
#endif
#if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
static uint16_t timeoutCntSerial_L = SERIAL_TIMEOUT;  // Timeout counter for Rx Serial command
static uint8_t  timeoutFlgSerial_L = 0;               // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
#endif

#if defined(CONTROL_SERIAL_USART2)
volatile  KiSCCommand    KiSC_Command;
volatile  KiSCSettings   KiSC_Settings = {5, 200, 127, 1}; // Default settings// Max 4 AMP, MAX 200 rpm, eBrake auf 127

static SerialCommand  commandL;
static SerialCommand  commandL_raw;
static uint32_t commandL_len = sizeof(commandL);
  #ifdef CONTROL_IBUS
  static uint16_t ibusL_captured_value[IBUS_NUM_CHANNELS];
  #endif
#endif

#ifdef VARIANT_HOVERCAR
static uint8_t brakePressed;
#endif

#if defined(CRUISE_CONTROL_SUPPORT) || (defined(STANDSTILL_HOLD_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ != SPD_MODE))
uint8_t cruiseCtrlAcv = 0;
uint8_t standstillAcv = 0;
#endif

/* =========================== Initialization Functions =========================== */

void BLDC_Init(void) {
  /* Set BLDC controller parameters */ 
  rtP_Left.b_angleMeasEna       = 0;            // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
  rtP_Left.z_selPhaCurMeasABC   = 0;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
  rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP_Left.b_diagEna            = DIAG_ENA;
  rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP_Left.b_fieldWeakEna       = FIELD_WEAK_ENA; 
  rtP_Left.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP_Left.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)

  rtP_Right                     = rtP_Left;     // Copy the Left motor parameters to the Right motor parameters
  rtP_Right.z_selPhaCurMeasABC  = 1;            // Right motor measured current phases {Blue, Yellow} = {iB, iC} -> do NOT change

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam        = &rtP_Left;
  rtM_Left->dwork               = &rtDW_Left;
  rtM_Left->inputs              = &rtU_Left;
  rtM_Left->outputs             = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam       = &rtP_Right;
  rtM_Right->dwork              = &rtDW_Right;
  rtM_Right->inputs             = &rtU_Right;
  rtM_Right->outputs            = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);
}

void Input_Lim_Init(void) {     // Input Limitations - ! Do NOT touch !
  if (rtP_Left.b_fieldWeakEna || rtP_Right.b_fieldWeakEna) {
    INPUT_MAX = MAX( 1000, FIELD_WEAK_HI);
    INPUT_MIN = MIN(-1000,-FIELD_WEAK_HI);
  } else {
    INPUT_MAX =  1000;
    INPUT_MIN = -1000;
  }
}

void Input_Init(void) {
  UART2_Init();
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer_L, sizeof(rx_buffer_L));
  UART_DisableRxErrors(&huart2);

  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    uint16_t writeCheck, readVal;
    HAL_FLASH_Unlock();
    EE_Init();            /* EEPROM Init */
    EE_ReadVariable(VirtAddVarTab[0], &writeCheck);
    if (writeCheck == FLASH_WRITE_KEY) {

      EE_ReadVariable(VirtAddVarTab[1] , &readVal); rtP_Left.i_max = rtP_Right.i_max = (int16_t)readVal;
      EE_ReadVariable(VirtAddVarTab[2] , &readVal); rtP_Left.n_max = rtP_Right.n_max = (int16_t)readVal;
      for (uint8_t i=0; i<INPUTS_NR; i++) {
        EE_ReadVariable(VirtAddVarTab[ 3+8*i] , &readVal); input1[i].typ = (uint8_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 4+8*i] , &readVal); input1[i].min = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 5+8*i] , &readVal); input1[i].mid = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 6+8*i] , &readVal); input1[i].max = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 7+8*i] , &readVal); input2[i].typ = (uint8_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 8+8*i] , &readVal); input2[i].min = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[ 9+8*i] , &readVal); input2[i].mid = (int16_t)readVal;
        EE_ReadVariable(VirtAddVarTab[10+8*i] , &readVal); input2[i].max = (int16_t)readVal;
      
        printf("Limits Input1: TYP:%i MIN:%i MID:%i MAX:%i\r\nLimits Input2: TYP:%i MIN:%i MID:%i MAX:%i\r\n",
          input1[i].typ, input1[i].min, input1[i].mid, input1[i].max,
          input2[i].typ, input2[i].min, input2[i].mid, input2[i].max);
      }
    } else {

      for (uint8_t i=0; i<INPUTS_NR; i++) {
        if (input1[i].typDef == 3) {  // If Input type defined is 3 (auto), identify the input type based on the values from config.h
          input1[i].typ = checkInputType(input1[i].min, input1[i].mid, input1[i].max);
        } else {
          input1[i].typ = input1[i].typDef;
        }
        if (input2[i].typDef == 3) {
          input2[i].typ = checkInputType(input2[i].min, input2[i].mid, input2[i].max);
        } else {
          input2[i].typ = input2[i].typDef;
        }
        printf("Limits Input1: TYP:%i MIN:%i MID:%i MAX:%i\r\nLimits Input2: TYP:%i MIN:%i MID:%i MAX:%i\r\n",
          input1[i].typ, input1[i].min, input1[i].mid, input1[i].max,
          input2[i].typ, input2[i].min, input2[i].mid, input2[i].max);
      }
    }
    HAL_FLASH_Lock();
  #endif
}

/**
  * @brief  Disable Rx Errors detection interrupts on UART peripheral (since we do not want DMA to be stopped)
  *         The incorrect data will be filtered based on the START_FRAME and checksum.
  * @param  huart: UART handle.
  * @retval None
  */
#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
void UART_DisableRxErrors(UART_HandleTypeDef *huart)
{  
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);    /* Disable PE (Parity Error) interrupts */  
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);     /* Disable EIE (Frame error, noise error, overrun error) interrupts */
}
#endif


/* =========================== General Functions =========================== */

void poweronMelody(void) {
    buzzerCount = 0;  // prevent interraction with beep counter
    for (int i = 8; i >= 0; i--) {
      buzzerFreq = (uint8_t)i;
      HAL_Delay(100);
    }
    buzzerFreq = 0;
}

void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) {
    buzzerCount   = cnt;
    buzzerFreq    = freq;
    buzzerPattern = pattern;
}

void beepLong(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    HAL_Delay(500);
    buzzerFreq = 0;
}

void beepShort(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    HAL_Delay(100);
    buzzerFreq = 0;
}

void beepShortMany(uint8_t cnt, int8_t dir) {
    if (dir >= 0) {   // increasing tone
      for(uint8_t i = 2*cnt; i >= 2; i=i-2) {
        beepShort(i + 3);
      }
    } else {          // decreasing tone
      for(uint8_t i = 2; i <= 2*cnt; i=i+2) {
        beepShort(i + 3);
      }
    }
}

void calcAvgSpeed(void) {
    // Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
    speedAvg = 0;
    #if defined(MOTOR_LEFT_ENA)
      #if defined(INVERT_L_DIRECTION)
        speedAvg -= rtY_Left.n_mot;
      #else
        speedAvg += rtY_Left.n_mot;
      #endif
    #endif
    #if defined(MOTOR_RIGHT_ENA)
      #if defined(INVERT_R_DIRECTION)
        speedAvg += rtY_Right.n_mot;
      #else
        speedAvg -= rtY_Right.n_mot;
      #endif

      // Average only if both motors are enabled
      #if defined(MOTOR_LEFT_ENA)
        speedAvg /= 2;
      #endif  
    #endif

    // Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
    if (SPEED_COEFFICIENT & (1 << 16)) {
      speedAvg    = -speedAvg;
    } 
    speedAvgAbs   = abs(speedAvg);
}

 /*
 * Auto-calibration of the ADC Limits
 * This function finds the Minimum, Maximum, and Middle for the ADC input
 * Procedure:
 * - press the power button for more than 5 sec and release after the beep sound
 * - move the potentiometers freely to the min and max limits repeatedly
 * - release potentiometers to the resting postion
 * - press the power button to confirm or wait for the 20 sec timeout
 * The Values will be saved to flash. Values are persistent if you flash with platformio. To erase them, make a full chip erase.
 */
void adcCalibLim(void) {
}
 /*
 * Update Maximum Motor Current Limit (via ADC1) and Maximum Speed Limit (via ADC2)
 * Procedure:
 * - press the power button for more than 5 sec and immediatelly after the beep sound press one more time shortly
 * - move and hold the pots to a desired limit position for Current and Speed
 * - press the power button to confirm or wait for the 10 sec timeout
 */
void updateCurSpdLim(void) {
  calcAvgSpeed();
  if (speedAvgAbs > 5) {    // do not enter this mode if motors are spinning
    return;
  }

#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)


  int32_t  input1_fixdt = input1[inIdx].raw << 16;
  int32_t  input2_fixdt = input2[inIdx].raw << 16;
  uint16_t cur_factor;    // fixdt(0,16,16)
  uint16_t spd_factor;    // fixdt(0,16,16)
  uint16_t cur_spd_timeout = 0;
  cur_spd_valid = 0;

  // Wait for the power button press
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && cur_spd_timeout++ < 2000) {  // 10 sec timeout
    readInputRaw();
    filtLowPass32(input1[inIdx].raw, FILTER, &input1_fixdt);
    filtLowPass32(input2[inIdx].raw, FILTER, &input2_fixdt);
    HAL_Delay(5);
  }
  // Calculate scaling factors
  cur_factor = CLAMP((input1_fixdt - (input1[inIdx].min << 16)) / (input1[inIdx].max - input1[inIdx].min), 6553, 65535);    // ADC1, MIN_cur(10%) = 1.5 A 
  spd_factor = CLAMP((input2_fixdt - (input2[inIdx].min << 16)) / (input2[inIdx].max - input2[inIdx].min), 3276, 65535);    // ADC2, MIN_spd(5%)  = 50 rpm
      
  if (input1[inIdx].typ != 0){
    // Update current limit
    rtP_Left.i_max = rtP_Right.i_max  = (int16_t)((I_MOT_MAX * A2BIT_CONV * cur_factor) >> 12);    // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid   = 1;  // Mark update to be saved in Flash at shutdown
  }

  if (input2[inIdx].typ != 0){
    // Update speed limit
    rtP_Left.n_max = rtP_Right.n_max  = (int16_t)((N_MOT_MAX * spd_factor) >> 12);                 // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid  += 2;  // Mark update to be saved in Flash at shutdown
  }

#endif
}

 /*
 * Standstill Hold Function
 * This function uses Cruise Control to provide an anti-roll functionality at standstill.
 * Only available and makes sense for FOC VOLTAGE or FOC TORQUE mode.
 * 
 * Input:  none
 * Output: standstillAcv
 */
void standstillHold(void) {
#ifdef VARIANT_KiSC
  #if defined(STANDSTILL_HOLD_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ != SPD_MODE)
    if (!rtP_Left.b_cruiseCtrlEna) {                                  // If Stanstill in NOT Active -> try Activation
      if (((input1[inIdx].cmd < 20 || input2[inIdx].cmd < 20) && speedAvgAbs < 30) // Check if Brake is pressed AND measured speed is small
          || (input2[inIdx].cmd < 20 && speedAvgAbs < 5)) {           // OR Throttle is small AND measured speed is very small
        rtP_Left.n_cruiseMotTgt   = 0;
        rtP_Right.n_cruiseMotTgt  = 0;
        rtP_Left.b_cruiseCtrlEna  = 1;
        rtP_Right.b_cruiseCtrlEna = 1;
        standstillAcv = 1;
      } 
    }
    else {                                                            // If Stanstill is Active -> try Deactivation
      if (input1[inIdx].cmd > 20 && input2[inIdx].cmd > 20 && !cruiseCtrlAcv) { // Check if Brake is released AND Throttle is pressed AND no Cruise Control
        rtP_Left.b_cruiseCtrlEna  = 0;
        rtP_Right.b_cruiseCtrlEna = 0;
        standstillAcv = 0;
      }
    }
  #endif
#else  
  #if defined(STANDSTILL_HOLD_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ != SPD_MODE)
    if (!rtP_Left.b_cruiseCtrlEna) {                                  // If Stanstill in NOT Active -> try Activation
      if (((input1[inIdx].cmd > 50 || input2[inIdx].cmd < -50) && speedAvgAbs < 30) // Check if Brake is pressed AND measured speed is small
          || (input2[inIdx].cmd < 20 && speedAvgAbs < 5)) {           // OR Throttle is small AND measured speed is very small
        rtP_Left.n_cruiseMotTgt   = 0;
        rtP_Right.n_cruiseMotTgt  = 0;
        rtP_Left.b_cruiseCtrlEna  = 1;
        rtP_Right.b_cruiseCtrlEna = 1;
        standstillAcv = 1;
      } 
    }
    else {                                                            // If Stanstill is Active -> try Deactivation
      if (input1[inIdx].cmd < 20 && input2[inIdx].cmd > 50 && !cruiseCtrlAcv) { // Check if Brake is released AND Throttle is pressed AND no Cruise Control
        rtP_Left.b_cruiseCtrlEna  = 0;
        rtP_Right.b_cruiseCtrlEna = 0;
        standstillAcv = 0;
      }
    }
  #endif
#endif  
}

 /*
 * Electric Brake Function
 * In case of TORQUE mode, this function replaces the motor "freewheel" with a constant braking when the input torque request is 0.
 * This is useful when a small amount of motor braking is desired instead of "freewheel".
 * 
 * Input: speedBlend = fixdt(0,16,15), reverseDir = {0, 1}
 * Output: input2.cmd (Throtle) with brake component included
 */
void electricBrake(uint16_t speedBlend, uint8_t reverseDir) {
  #if defined(ELECTRIC_BRAKE_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ == TRQ_MODE)
    int16_t brakeVal;

    // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving) 
    if (speedAvg > 0) {
      brakeVal = (int16_t)((-ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    } else {
      brakeVal = (int16_t)(( ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    }

    // Check if direction is reversed
    if (reverseDir) {
      brakeVal = -brakeVal;
    }

    // Calculate the new input2.cmd with brake component included
    if (input2[inIdx].cmd >= 0 && input2[inIdx].cmd < ELECTRIC_BRAKE_THRES) {
      input2[inIdx].cmd = MAX(brakeVal, ((ELECTRIC_BRAKE_THRES - input2[inIdx].cmd) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (input2[inIdx].cmd >= -ELECTRIC_BRAKE_THRES && input2[inIdx].cmd < 0) {
      input2[inIdx].cmd = MIN(brakeVal, ((ELECTRIC_BRAKE_THRES + input2[inIdx].cmd) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (input2[inIdx].cmd >= ELECTRIC_BRAKE_THRES) {
      input2[inIdx].cmd = MAX(brakeVal, ((input2[inIdx].cmd - ELECTRIC_BRAKE_THRES) * INPUT_MAX) / (INPUT_MAX - ELECTRIC_BRAKE_THRES));
    } else {  // when (input2.cmd < -ELECTRIC_BRAKE_THRES)
      input2[inIdx].cmd = MIN(brakeVal, ((input2[inIdx].cmd + ELECTRIC_BRAKE_THRES) * INPUT_MIN) / (INPUT_MIN + ELECTRIC_BRAKE_THRES));
    }
  #endif
}

 /*
 * Cruise Control Function
 * This function activates/deactivates cruise control.
 * 
 * Input: button (as a pulse)
 * Output: cruiseCtrlAcv
 */

#ifdef ESP32_USART_CONTROL
void cruiseControlEnable(uint8_t bEnable) {
    if (bEnable) {
        if (!cruiseCtrlAcv) {
            cruiseControl(1);
        }
    } else {
        if (cruiseCtrlAcv) {
            cruiseControl(1);
        }
    }
}

#endif
void cruiseControl(uint8_t button) {
  #ifdef CRUISE_CONTROL_SUPPORT
    if (button && !rtP_Left.b_cruiseCtrlEna) {                          // Cruise control activated
      rtP_Left.n_cruiseMotTgt   = rtY_Left.n_mot;
      rtP_Right.n_cruiseMotTgt  = rtY_Right.n_mot;
      rtP_Left.b_cruiseCtrlEna  = 1;
      rtP_Right.b_cruiseCtrlEna = 1;
      cruiseCtrlAcv = 1;
      beepShortMany(2, 1);                                              // 200 ms beep delay. Acts as a debounce also.
    } else if (button && rtP_Left.b_cruiseCtrlEna && !standstillAcv) {  // Cruise control deactivated if no Standstill Hold is active
      rtP_Left.b_cruiseCtrlEna  = 0;
      rtP_Right.b_cruiseCtrlEna = 0;
      cruiseCtrlAcv = 0;
      beepShortMany(2, -1);
    }
  #endif
}

 /*
 * Check Input Type
 * This function identifies the input type: 0: Disabled, 1: Normal Pot, 2: Middle Resting Pot
 */
int checkInputType(int16_t min, int16_t mid, int16_t max){

  int type = 0;  

  int16_t threshold = 200;

  if ((min / threshold) == (max / threshold) || (mid / threshold) == (max / threshold) || min > max || mid > max) {
    type = 0;
  } else {
    if ((min / threshold) == (mid / threshold)){
      type = 1;
    } else {
      type = 2;
    }
  }

  return type;
}



/* =========================== Input Functions =========================== */

 /*
 * Calculate Input Command
 * This function realizes dead-band around 0 and scales the input between [out_min, out_max]
 */
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max) {
  switch (in->typ){
    case 1: // Input is a normal pot
      in->cmd = CLAMP(MAP(in->raw, in->min, in->max, 0, out_max), 0, out_max);
      break;
    case 2: // Input is a mid resting pot
      if( in->raw > in->mid - in->dband && in->raw < in->mid + in->dband ) {
        in->cmd = 0;
      } else if(in->raw > in->mid) {
        in->cmd = CLAMP(MAP(in->raw, in->mid + in->dband, in->max, 0, out_max), 0, out_max);
      } else {
        in->cmd = CLAMP(MAP(in->raw, in->mid - in->dband, in->min, 0, out_min), out_min, 0);
      }
      break;
    default: // Input is ignored
      in->cmd = 0;
      break;
  }
}

 /*
 * Function to read the Input Raw values from various input devices
 */
void readInputRaw(void) {
    #if defined(CONTROL_SERIAL_USART2)
    if (inIdx == CONTROL_SERIAL_USART2) {
        input1[inIdx].cmd = commandL.left.pwm;
        input2[inIdx].cmd = commandL.right.pwm;
        if (commandL.cruiseCtrlAcv) {
          cruiseControlEnable(1);
        } else {
          cruiseControlEnable(0);
        }
    }
    #endif
}

 /*
 * Function to handle the ADC, UART and General timeout (Nunchuk, PPM, PWM)
 */
void handleTimeout(void) {
    #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
      if (timeoutCntSerial_L++ >= SERIAL_TIMEOUT) {     // Timeout qualification
        timeoutFlgSerial_L = 1;                         // Timeout detected
        timeoutCntSerial_L = SERIAL_TIMEOUT;            // Limit timout counter value
      } else {                                          // No Timeout
      }
      #if (defined(CONTROL_SERIAL_USART2) && CONTROL_SERIAL_USART2 == 0) || (defined(SIDEBOARD_SERIAL_USART2) && SIDEBOARD_SERIAL_USART2 == 0 && !defined(VARIANT_HOVERBOARD))
        timeoutFlgSerial = timeoutFlgSerial_L;          // Report Timeout only on the Primary Input
      #endif

    #endif

    // In case of timeout bring the system to a Safe State
    if (timeoutFlgADC || timeoutFlgSerial || timeoutFlgGen) {
      ctrlModReq  = OPEN_MODE;                                          // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way
      input1[inIdx].cmd  = 0;
      input2[inIdx].cmd  = 0;
    } else {
      ctrlModReq  = ctrlModReqRaw;                                      // Follow the Mode request
    }

    // Beep in case of Input index change
    if (inIdx && !inIdx_prev) {                                         // rising edge
      beepShort(8);
    } else if (!inIdx && inIdx_prev) {                                  // falling edge
      beepShort(18);
    }
}

 /*
 * Function to calculate the command to the motors. This function also manages:
 * - timeout detection
 * - MIN/MAX limitations and deadband
 */
void readCommand(void) {
    readInputRaw();

    #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
      calcInputCmd(&input1[inIdx], INPUT_MIN, INPUT_MAX);
      #if !defined(VARIANT_SKATEBOARD)
        calcInputCmd(&input2[inIdx], INPUT_MIN, INPUT_MAX);
      #else
        calcInputCmd(&input2[inIdx], INPUT_BRK, INPUT_MAX);
      #endif
    #endif

    handleTimeout();
}


void KiSC_process_command(char *data, KiSCCommand *cmd) {
    switch (data[2]) {
      case HOVER_CMD_MOTORCTRL:
        cmd->left.mode = (data[7] & 12) >> 2;
        cmd->left.type = (data[7] & 48) >> 4;
        cmd->right.mode = (data[10] & 12) >> 2;
        cmd->right.type = (data[10] & 48) >> 4;
        cmd->left.target = (int16_t)(data[5] << 8 | (data[6]));
        cmd->right.target = (int16_t)(data[8] << 8 | (data[9]));
        cmd->left.enable = data[7] & 1;
        cmd->right.enable = data[10] & 1;
        cmd->left.direction = (data[7] & 2) >> 1;
        cmd->right.direction = (data[10] & 2) >> 1;
        break;
      case HOVER_CMD_POWER:
        if (data[4] == 0xAA) {
          beepShort(4);                     // make 2 beeps indicating the motor enable
          beepShort(6); 
          beepShort(6); 
          HAL_Delay(100);
          poweroff();
        }

        break;
      case HOVER_CMD_BUZZER:
        beepCount(1, data[4], data[5]);
        break;
      case HOVER_CMD_SETTINGS:
        KiSC_Settings.maxCurrrent   = (int16_t)(data[4] << 8 | (data[5]));
        KiSC_Settings.maxSpeed      = (int16_t)(data[6] << 8 | (data[7]));
        KiSC_Settings.eBrake        = data[8];
        KiSC_Settings.parkingbrake  = data[9];

//       max_speed = MULTI_MODE_DRIVE_M1_MAX;
//      rate = MULTI_MODE_DRIVE_M1_RATE;
//      rtP_Left.n_max = rtP_Right.n_max = MULTI_MODE_M1_N_MOT_MAX << 4;
//      rtP_Left.i_max = rtP_Right.i_max = (MULTI_MODE_M1_I_MOT_MAX * A2BIT_CONV) << 4;
        
        break;
    }
}

/*
 * Check for new data received on USART2 with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart_process_data(char *data, int16_t len) {
    static uint8_t ptr_raw[SERIAL_BUFFER_SIZE];
    static uint8_t inComingPrev = 0;
    uint8_t inComing;
    static uint8_t idx = 0;
    static uint8_t *ptr = (uint8_t *)&commandL_raw;
    uint16_t bufStartFrame = 0;
// void usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx)
    while (len--) {
        inComing = *data++;
        bufStartFrame	= ((uint16_t)(inComingPrev) << 8) | inComing;       // Construct the start frame
        if (bufStartFrame == HOVER_VALID_HEADER) {  // Initialize if new data is detected
           ptr       = (uint8_t *)&ptr_raw;
          *ptr++    = inComingPrev;
          *ptr++    = inComing;
          idx     = 2;
        } else if (idx >= 2) {  // Save the new received data
          *ptr++    = inComing;
          idx++;
        }
        if (idx >= 4) {
          uint8_t len = ptr_raw[3];
          if (idx >= len) {
            uint8_t cmd = ptr_raw[2];
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < len - 1; i++) {
              checksum ^= ptr_raw[i];
            }
            if (checksum == ptr_raw[len - 1]) {
              timeoutFlgSerial_L = 0;         // Clear timeout flag
              timeoutCntSerial_L = 0;         // Reset timeout counter
              KiSC_process_command((SerialCommand *)ptr_raw, &KiSC_Command);
//              usart_process_command((SerialCommand *)ptr_raw, &commandL, 2);
            }
            idx = 0;

          }
        }
        // Check if we reached the end of the package
      if (idx > SERIAL_BUFFER_SIZE) {
        idx = 0;
        bufStartFrame = 0x0000;
        // Check if the checksum is correct
       // if (commandL_raw.checksum == calculateCommandChecksum(commandL_raw)) {
//            usart_process_command(&commandL_raw, &commandL, 2);
//        timeoutFlgSerial_L = 0;         // Clear timeout flag
//        timeoutCntSerial_L = 0;         // Reset timeout counter

        //}
    }

        inComingPrev = inComing;
    }

}

void usart2_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
//    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
    pos = rx_buffer_L_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
//            void usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx)

            usart_process_data(&rx_buffer_L[old_pos], pos - old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&rx_buffer_L[old_pos], ARRAY_LEN(rx_buffer_L) - old_pos);
            if (pos > 0) {
                usart_process_data(&rx_buffer_L[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
}
/*
 * Check for new data received on USART3 with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart3_rx_check(void)
{
}

/*
 * Process command Rx data
 * - if the command_in data is valid (correct START_FRAME and checksum) copy the command_in to command_out
 */
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
void usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx)
{
  uint16_t checksum;
  if (command_in->start == VALID_HEADER) {
      checksum = VALID_HEADER;
//    checksum = calculateCommandChecksum(*command_in);
    if (command_in->checksum == checksum) {
      *command_out = *command_in;
      if (usart_idx == 2) {             // Sideboard USART2
        #ifdef CONTROL_SERIAL_USART2
        timeoutFlgSerial_L = 0;         // Clear timeout flag
        timeoutCntSerial_L = 0;         // Reset timeout counter
        #endif
      }
      }
    } else {
    }

  }


#endif

/* =========================== Poweroff Functions =========================== */

 /*
 * Save Configuration to Flash
 * This function makes sure data is not lost after power-off
 */
void saveConfig() {
  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    if (inp_cal_valid || cur_spd_valid) {

      HAL_FLASH_Unlock();
      EE_WriteVariable(VirtAddVarTab[0] , (uint16_t)FLASH_WRITE_KEY);
      EE_WriteVariable(VirtAddVarTab[1] , (uint16_t)rtP_Left.i_max);
      EE_WriteVariable(VirtAddVarTab[2] , (uint16_t)rtP_Left.n_max);
      for (uint8_t i=0; i<INPUTS_NR; i++) {
        EE_WriteVariable(VirtAddVarTab[ 3+8*i] , (uint16_t)input1[i].typ);
        EE_WriteVariable(VirtAddVarTab[ 4+8*i] , (uint16_t)input1[i].min);
        EE_WriteVariable(VirtAddVarTab[ 5+8*i] , (uint16_t)input1[i].mid);
        EE_WriteVariable(VirtAddVarTab[ 6+8*i] , (uint16_t)input1[i].max);
        EE_WriteVariable(VirtAddVarTab[ 7+8*i] , (uint16_t)input2[i].typ);
        EE_WriteVariable(VirtAddVarTab[ 8+8*i] , (uint16_t)input2[i].min);
        EE_WriteVariable(VirtAddVarTab[ 9+8*i] , (uint16_t)input2[i].mid);
        EE_WriteVariable(VirtAddVarTab[10+8*i] , (uint16_t)input2[i].max);
      }
      HAL_FLASH_Lock();
    }
  #endif 
}


void poweroff(void) {
  enable = 0;
/*  
  buzzerCount = 0;  // prevent interraction with beep counter
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }*/
  saveConfig();
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
  while(1) {}
}


void poweroffPressCheck(void) {
  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      uint16_t cnt_press = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        HAL_Delay(10);
        if (cnt_press++ == 5 * 100) { beepShort(5); }
      }

      if (cnt_press > 8) enable = 0;

      if (cnt_press >= 5 * 100) {                         // Check if press is more than 5 sec
        HAL_Delay(1000);
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {  // Double press: Adjust Max Current, Max Speed
          while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
          beepLong(8);
          updateCurSpdLim();
          beepShort(5);
        } else {                                          // Long press: Calibrate ADC Limits
          #ifdef AUTO_CALIBRATION_ENA
          beepLong(16); 
          adcCalibLim();
          beepShort(5);
          #endif
        }
      } else if (cnt_press > 8) {                         // Short press: power off (80 ms debounce)
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
          printf("Powering off, button has been pressed\r\n");
        #endif
        beepShort(4);                     // make 2 beeps indicating the motor enable
        beepShort(6); HAL_Delay(100);

      poweroff();
      }
    }
#endif    
}



/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
  * Max:  32767.99998474121
  * Min: -32768
  * Res:  1.52587890625e-05
  * 
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  * 
  * Example: 
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;  
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}
  // Old filter
  // Inputs:       u     = int16
  // Outputs:      y     = fixdt(1,32,20)
  // Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  // yint = (int16_t)(y >> 20); // the integer output is the fixed-point ouput shifted by 20 bits
  // void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
  //   int32_t tmp;  
  //   tmp = (int16_t)(u << 4) - (*y >> 16);  
  //   tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection  
  //   *y  = coef * tmp + (*y);
  // }


  /* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,16,4)
  * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
  */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
  int16_t q0;
  int16_t q1;

  q0 = (u << 4)  - *y;

  if (q0 > rate) {
    q0 = rate;
  } else {
    q1 = -rate;
    if (q0 < q1) {
      q0 = q1;
    }
  }

  *y = q0 + *y;
}


  /* mixerFcn(rtu_speed, rtu_steer, &rty_speedR, &rty_speedL); 
  * Inputs:       rtu_speed, rtu_steer                  = fixdt(1,16,4)
  * Outputs:      rty_speedR, rty_speedL                = int16_t
  * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
  */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR, int16_t *rty_speedL) {
    int16_t prodSpeed;
    int16_t prodSteer;
    int32_t tmp;

    prodSpeed   = (int16_t)((rtu_speed * (int16_t)SPEED_COEFFICIENT) >> 14);
    prodSteer   = (int16_t)((rtu_steer * (int16_t)STEER_COEFFICIENT) >> 14);

    tmp         = prodSpeed - prodSteer;  
    tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
    *rty_speedR = (int16_t)(tmp >> 4);        // Convert from fixed-point to int 
    *rty_speedR = CLAMP(*rty_speedR, INPUT_MIN, INPUT_MAX);

    tmp         = prodSpeed + prodSteer;
    tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
    *rty_speedL = (int16_t)(tmp >> 4);        // Convert from fixed-point to int
    *rty_speedL = CLAMP(*rty_speedL, INPUT_MIN, INPUT_MAX);
}



/* =========================== Multiple Tap Function =========================== */

  /* multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x)
  * This function detects multiple tap presses, such as double tapping, triple tapping, etc.
  * Inputs:       u = int16_t (input signal); timeNow = uint32_t (current time)  
  * Outputs:      x->b_multipleTap (get the output here)
  */
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x) {
  uint8_t 	b_timeout;
  uint8_t 	b_hyst;
  uint8_t 	b_pulse;
  uint8_t 	z_pulseCnt;
  uint8_t   z_pulseCntRst;
  uint32_t 	t_time; 

  // Detect hysteresis
  if (x->b_hysteresis) {
    b_hyst = (u > MULTIPLE_TAP_LO);
  } else {
    b_hyst = (u > MULTIPLE_TAP_HI);
  }

  // Detect pulse
  b_pulse = (b_hyst != x->b_hysteresis);

  // Save time when first pulse is detected
  if (b_hyst && b_pulse && (x->z_pulseCntPrev == 0)) {
    t_time = timeNow;
  } else {
    t_time = x->t_timePrev;
  }

  // Create timeout boolean
  b_timeout = (timeNow - t_time > MULTIPLE_TAP_TIMEOUT);

  // Create pulse counter
  if ((!b_hyst) && (x->z_pulseCntPrev == 0)) {
    z_pulseCnt = 0U;
  } else {
    z_pulseCnt = b_pulse;
  }

  // Reset counter if we detected complete tap presses OR there is a timeout
  if ((x->z_pulseCntPrev >= MULTIPLE_TAP_NR) || b_timeout) {
    z_pulseCntRst = 0U;
  } else {
    z_pulseCntRst = x->z_pulseCntPrev;
  }
  z_pulseCnt = z_pulseCnt + z_pulseCntRst;

  // Check if complete tap presses are detected AND no timeout
  if ((z_pulseCnt >= MULTIPLE_TAP_NR) && (!b_timeout)) {
    x->b_multipleTap = !x->b_multipleTap;	// Toggle output
  }

  // Update states
  x->z_pulseCntPrev = z_pulseCnt;
  x->b_hysteresis 	= b_hyst;
  x->t_timePrev 	  = t_time;
}


