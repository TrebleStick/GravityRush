/*******************************************************************************

 @file  board.c

 @brief This file is a simple gateway to include the appropriate board.h file
        which is located in a corresponding subdirectory relative to this file.
        In this way, all projects look the same and only need to include this
        board.h. To change a board, the project only need update the board
        define. Presently, the following board targets are possible:

        BOOSTXL_CC2650MA
        CC2650_LAUNCHXL
        CC2650DK_4XS
        CC2650DK_5XD
        CC2650DK_7ID
        CC2650RC
        CC2650STK
        CC1310_LAUNCHXL
        CC1310DK_4XD
        CC1310DK_5XD
        CC1310DK_7XD
        CC1350_LAUNCHXL
        CC1350STK

        If a project needs to change the board defined for a particular target,
        they can modify the board.h located in the corresponding board
        subdirectory.

        NOTE: THIS FILE SHOULD ONLY BE CHANGED TO ADD A NEW BOARD/TARGET!

 Group: WCS, LPC, BTS
 Target Device: CC2650, CC2640

 *******************************************************************************
 
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

 *******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 ******************************************************************************/

#if defined(CC2650DK_7ID) || defined(CC2650DK_5XD) || defined(CC2650DK_4XS)
    #include "./cc2650em/cc2650em_board.c"
#elif defined(CC2650STK)
    #include "./cc2650st/cc2650st_board.c"
#elif defined(CC2650RC)
    #include "./cc2650rc/cc2650rc_board.c"
#elif defined(CC2650_LAUNCHXL)
    #include "./cc2650lp/cc2650lp_board.c"
#elif defined(BOOSTXL_CC2650MA)
    #include "./cc2650bp/cc2650bp_board.c"
#elif defined(CC1310DK_7XD) || defined(CC1310DK_5XD) || defined(CC1310DK_4XD)
    #include "./cc1310em/cc1310em_board.c"
#elif defined(CC1310_LAUNCHXL)
    #include "./cc1310lp/cc1310lp_board.c"
#elif defined(CC1350_LAUNCHXL)
    #include "./cc1350lp/cc1350lp_board.c"
#elif defined(CC1350STK)
    #include "./cc1350st/cc1350st_board.c"
#elif defined(USE_FPGA)
    #include "./cc2650fpga/cc2650fpga_board.c"
#else // unknown board
    #error "***ERROR*** Invalid Board Specified! Please see board.h for options."
#endif
