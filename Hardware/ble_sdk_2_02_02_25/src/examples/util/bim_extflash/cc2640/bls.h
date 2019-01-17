/******************************************************************************

 @file  bls.h

 @brief Boot Loader storage abstraction

 Group: WCS, BTS
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
#ifndef BLS_H
#define BLS_H

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * Initialize storage driver.
   *
   * @return Zero when successful.
   */
  extern int BLS_init(void);

  /**
   * Close the storage driver
   */
  extern void BLS_close(void);

  /**
   * Read storage content
   *
   * @return Zero when successful.
   */
  extern int BLS_read(size_t offset, size_t length, uint8_t *buf);

  /**
   * Erase storage sectors corresponding to the range.
   *
   * @return Zero when successful.
   */
  extern int BLS_erase(size_t offset, size_t length);

  /**
   * Write to storage sectors.
   *
   * @return Zero when successful.
   */
  extern int BLS_write(size_t offset, size_t length, const uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* BLS_H */
