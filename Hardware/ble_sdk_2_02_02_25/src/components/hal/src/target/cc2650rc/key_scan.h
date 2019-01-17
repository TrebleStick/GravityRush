/******************************************************************************

 @file  key_scan.h

 @brief This file contains the key driver for a key matrix.

 Group: WCS, LPC, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2005-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

#ifndef  KEY_SCAN_H
#define  KEY_SCAN_H

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "board.h"
#include "kcb.h"

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */
#define KEY_CODE_NOKEY              0xFF

#define KEY_NUM_ROWS                11
#define KEY_NUM_COLUMNS             3

#define KEY_REPEATED_THRESHOLD      10

/******************************************************************************
 * TYPEDEFS
 */
typedef void (*KeyEvtCBack_t)(uint8 key);
typedef void (*KeyTimerEvtCBack_t)(void);
typedef void (*ShiftRegTimerEvtCBack_t)(void);

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * FUNCTIONS - API
 */

/*
 * Initialize the Key Service
 */
extern void KeyInit(void);

/*
 * Configure the Key Service
 */
extern void KeyConfig(KeyEvtCBack_t key_cback, uint16_t initialKeyRepeatInterval,
                      uint16_t debounceTime, uint16_t pollRate);

/******************************************************************************
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
