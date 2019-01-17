/******************************************************************************

 @file  nvocop.h

 @brief NV definitions for CC26xx devices - On-Chip One-Page Flash Memory

 Group: WCS, LPC, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2014-2018, Texas Instruments Incorporated
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
#ifndef NVOCOP_H
#define NVOCOP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_board_cfg.h"
#include "../nvintf.h"

//*****************************************************************************
// Constants and definitions
//*****************************************************************************

#ifndef SNV_FIRST_PAGE
// One page reserved for this module
#define SNV_FIRST_PAGE  0x1E
#endif
  
#ifndef FLASH_PAGE_SIZE
// Common page size for all CC26xx devices
#define FLASH_PAGE_SIZE  HAL_FLASH_PAGE_SIZE
#endif

#ifndef FLASH_WORD_SIZE
#define FLASH_WORD_SIZE HAL_FLASH_WORD_SIZE
#endif

// Maximum length of a single NV item
#define MAX_NV_LEN     (FLASH_PAGE_SIZE / 4)

// Maximum ID parameters
#define MAX_NV_SYSID   0x003F  //  6 bits
#define MAX_NV_ITEMID  0x03FF  // 10 bits
#define MAX_NV_SUBID   0x03FF  // 10 bits

//*****************************************************************************
// Macros
//*****************************************************************************

//*****************************************************************************
// Typedefs
//*****************************************************************************

//*****************************************************************************
// Functions
//*****************************************************************************

extern uint8 NVOCOP_initNV( void *param );
extern uint8 NVOCOP_compactNV( uint16 minAvail );
extern uint8 NVOCOP_readItem( NVINTF_itemID_t nv_id, uint16 offset, uint16 len,
                              void *pBuf );
extern uint8 NVOCOP_writeItem( NVINTF_itemID_t nv_id, uint16 len, void *pBuf );

//*****************************************************************************
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* NVOCOP_H */

