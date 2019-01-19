/***************************************************************************//**
 *   @file   Communication.h
 *   @brief  Header file of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 812
*******************************************************************************/
/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>

#ifndef _57XXCOMMUNICATION_H_
#define _57XXCOMMUNICATION_H_

#define AD57XX_RESET_PIN 7
#define AD57XX_CLR_PIN 6
#define AD57XX_LDAC_PIN 5
#define AD57XX_SYNC_PIN 4

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

/*! Initializes the SPI communication peripheral. */
uint8_t AD57XX_COMM_Init(uint8_t lsbFirst,
                       uint32_t clockFreq,
                       uint8_t clockPol,
                       uint8_t clockEdg);

/*! Reads data from SPI. */
uint8_t AD57XX_Read(uint8_t slaveDeviceId,
                       uint8_t* data,
                       uint8_t bytesNumber);

/*! Writes data to SPI. */
uint8_t AD57XX_Write(uint8_t slaveDeviceId,
                        uint8_t* data,
                        uint8_t bytesNumber);

#endif // _57XXCOMMUNICATION_H_
