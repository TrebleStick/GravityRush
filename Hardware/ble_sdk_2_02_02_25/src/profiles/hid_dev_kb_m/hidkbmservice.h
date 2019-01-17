/******************************************************************************

 @file  hidkbmservice.h

 @brief This file contains the HID service for a keyboard/mouse.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2011-2018, Texas Instruments Incorporated
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

#ifndef HIDKBMSERVICE_H
#define HIDKBMSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Number of HID reports defined in the service
#define HID_NUM_REPORTS          9

// HID Report IDs for the service
#define HID_RPT_ID_MOUSE_IN      1  // Mouse input report ID
#define HID_RPT_ID_KEY_IN        2  // Keyboard input report ID
#define HID_RPT_ID_CC_IN         3  // Consumer Control input report ID
#define HID_RPT_ID_LED_OUT       0  // LED output report ID
#define HID_RPT_ID_FEATURE       0  // Feature report ID

// HID feature flags
#define HID_KBD_FLAGS            HID_FLAGS_REMOTE_WAKE

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      HidKbM_AddService
 *
 * @brief   Initializes the HID service for keyboard/mouse by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t HidKbM_AddService( void );

/*********************************************************************
 * @fn      HidKbM_SetParameter
 *
 * @brief   Set a HID KbM parameter.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   len - length of data to right.
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  GATT status code.
 */
extern uint8 HidKbM_SetParameter( uint8 id, uint8 type, uint16 uuid, uint8 len, void *pValue );

/*********************************************************************
 * @fn      HidKbM_GetParameter
 *
 * @brief   Get a HID KbM parameter.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   pLen - length of data to be read.
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  GATT status code.
 */
extern uint8 HidKbM_GetParameter( uint8 id, uint8 type, uint16 uuid, uint8 *pLen, void *pValue );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HIDKBMSERVICE_H */
