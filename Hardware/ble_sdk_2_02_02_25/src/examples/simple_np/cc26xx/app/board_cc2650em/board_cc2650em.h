/******************************************************************************

 @file  board_cc2650em.h

 @brief CC2650EM_7ID Board Specific header file.
                The project options should point to this file if this is the
                CC2650EM you are developing code for.

        The CC2650 header file should be included in an application as follows:
        @code
        #include <Board.h>
        @endcode

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2015-2018, Texas Instruments Incorporated
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

#ifndef __CC2650EM_7ID_H__
#define __CC2650EM_7ID_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Symbol by generic Board.c to include the correct kit specific Board.c
 *  ==========================================================================*/
#define CC2650EM_7ID


/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */
/* Leds */
#define Board_LED_ON                        1               /* LEDs on SmartRF06 EB are active high */
#define Board_LED_OFF                       0
#define Board_DK_LED1                       IOID_25         /* P2.11 */
#define Board_DK_LED2                       IOID_27         /* P2.13 */

/* MRDY/SRDY Board */
#define Board_MRDY                          IOID_19          /* P1.10  */
#define Board_SRDY                          IOID_12          /* P1.12  */

/* Button Board */
#define Board_KEY_SELECT                    IOID_11         /* P1.14 */
#define Board_KEY_UP                        IOID_19         /* P1.10 */
#define Board_KEY_DOWN                      IOID_12         /* P1.12 */
#define Board_KEY_LEFT                      IOID_15         /* P1.6  */
#define Board_KEY_RIGHT                     IOID_18         /* P1.8  */

/* LCD  Board */
#define Board_LCD_MODE                      IOID_4          /* P1.11 */
#define Board_LCD_RST                       IOID_5          /* P1.13 */
#define Board_LCD_CSN                       IOID_14         /* P1.17 */

/* UART Board */
#define Board_UART_RX                       IOID_2          /* P1.7  */
#define Board_UART_TX                       IOID_3          /* P1.9  */
#define Board_UART_CTS                      IOID_0          /* P1.3  */
#define Board_UART_RTS                      IOID_21         /* P2.18 */

/* SPI Board */
#define Board_SPI0_MISO                     IOID_8          /* P1.20 */
#define Board_SPI0_MOSI                     IOID_9          /* P1.18 */
#define Board_SPI0_CLK                      IOID_10         /* P1.16 */
#define Board_SPI0_CSN                      PIN_UNASSIGNED  /* P1.14, separate CSn for LCD, SDCARD, and ACC */
#define Board_SPI1_MISO                     IOID_24         /* RF2.10 for testing only */
#define Board_SPI1_MOSI                     IOID_23         /* RF2.5  for testing only */
#define Board_SPI1_CLK                      IOID_30         /* RF2.12 for testing only */
#define Board_SPI1_CSN                      PIN_UNASSIGNED  /* RF2.6  for testing only */

/* Ambient Light Sensor */
#define Board_ALS_OUT                       IOID_23         /* P2.5 */
#define Board_ALS_PWR                       IOID_26         /* P2.6 */

/* Accelerometer */
#define Board_ACC_PWR                       IOID_20         /* P2.8 */
#define Board_ACC_CSN                       IOID_24         /* P2.10 */

/* SD Card */
#define Board_SDCARD_CSN                    IOID_30         /* P2.12 */

/* Power Board */
#define Board_3V3_EN                        IOID_13         /* P1.15 */

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC2650DK_7ID_SPI0
#define Board_SPI1                  CC2650DK_7ID_SPI1
/* Generic UART instance identifiers */
#define Board_UART                  CC2650DK_7ID_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC2650DK_7ID_CRYPTO0
/* Generic TRNG instance identiifer */
#define Board_TRNG                  CC2650DK_7ID_TRNG0

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650DK_7ID_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650DK_7ID_CryptoName {
    CC2650DK_7ID_CRYPTO0 = 0,
    CC2650DK_7ID_CRYPTOCOUNT
} CC2650DK_7ID_CryptoName;

/*!
 *  @def    CC2650DK_7ID_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650DK_7ID_SPIName {
    CC2650DK_7ID_SPI0 = 0,
    CC2650DK_7ID_SPI1,
    CC2650DK_7ID_SPICOUNT
} CC2650DK_7ID_SPIName;

/*!
 *  @def    CC2650DK_7ID_TRNGName
 *  @brief  Enum of TRNG names on the CC2650 dev board
 */
typedef enum CC2650DK_7ID_TRNGName {
    CC2650DK_7ID_TRNG0 = 0,
    CC2650DK_7ID_TRNGCOUNT
} CC2650DK_7ID_TRNGName;

/*!
 *  @def    CC2650DK_7ID_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650DK_7ID_UARTName {
    CC2650DK_7ID_UART0 = 0,
    CC2650DK_7ID_UARTCOUNT
} CC2650DK_7ID_UARTName;

/*!
 *  @def    CC2650DK_7ID_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650DK_7ID_UdmaName {
    CC2650DK_7ID_UDMA0 = 0,
    CC2650DK_7ID_UDMACOUNT
} CC2650DK_7ID_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2650EM_H__ */
