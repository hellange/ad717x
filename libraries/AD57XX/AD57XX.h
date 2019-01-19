/***************************************************************************//**
 *   @file   AD57XX.h
 *   @brief  Header file of AD57XX Driver.
 *   @author Dan Nechita
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
 *   SVN Revision: 781
*******************************************************************************/
#ifndef __AD57XX_H__
#define __AD57XX_H__

#include <stdint.h>
#include "AD57XX_Comm.h"

/******************************************************************************/
/******************************** AD57XX **************************************/
/******************************************************************************/
//#define DEBUGG

/* Supported devices */
#define AD5760      0
#define AD5780      1
#define AD5781      2
#define AD5790      3
#define AD5791      4

/* SPI slave device ID */
#define AD57XX_SLAVE_ID         1

/* AD57XX Register Map */
#define AD57XX_NOP                 0  // No operation (NOP).
#define AD57XX_REG_DAC             1  // DAC register.
#define AD57XX_REG_CTRL            2  // Control register.
#define AD57XX_REG_CLR_CODE        3  // Clearcode register.
#define AD57XX_CMD_WR_SOFT_CTRL    4  // Software control register.

/* Input Shift Register Bit definition. */
#define AD57XX_READ (1ul << 23)
#define AD57XX_WRITE (0ul << 23)
#define AD57XX_ADDR_REG(x) (((uint32_t)(x) & 0x7ul) << 20)

/* DAC Register Bit definition. */
#define AD57XX_DAC_DATA(x)         ((uint32_t)(x) & 0x0FFFFFul)

/* Clearcode Register Bit definition. */
#define AD57XX_CLR_CODE_DATA(x)    ((uint32_t)(x) & 0x0FFFFFul)

/* Control Register Bit definition. */
#define AD57XX_CTRL_LINCOMP(x)  (((x) & 0xFul) << 6)  //  only for AD5781, AD5791.
#define AD57XX_CTRL_SDODIS      (1ul << 5) // SDO pin enable/disable control.
#define AD57XX_CTRL_BIN2SC      (1ul << 4) // DAC register coding selection.
#define AD57XX_CTRL_DACTRI      (1ul << 3) // DAC tristate control.
#define AD57XX_CTRL_OPGND       (1ul << 2) // Output ground clamp control.
#define AD57XX_CTRL_RBUF        (1ul << 1) // Output amplifier setup.

/* #define AD57XX_CTRL_LINCOMP(x) options. */
#define AD57XX_LINCOMP_10V_SPAN     0x0ul // used by AD5781 and AD5791
#define AD57XX_LINCOMP_12V_SPAN     0x9ul // used by AD5791
#define AD57XX_LINCOMP_16V_SPAN     0xAul // used by AD5791
#define AD57XX_LINCOMP_19V_SPAN     0xBul // used by AD5791
#define AD57XX_LINCOMP_20V_SPAN     0xCul // used by AD5781 and AD5791

/* Software Control Register Bit definition. */
#define AD57XX_SOFT_CTRL_RESET      (1ul << 2) // RESET function.
#define AD57XX_SOFT_CTRL_CLR        (1ul << 1) // CLR function.
#define AD57XX_SOFT_CTRL_LDAC       (1ul << 0) // LDAC function.

/* DAC OUTPUT STATES */
#define AD5791_OUT_NORMAL       0x00ul
#define AD5791_OUT_CLAMPED_6K   0x01ul
#define AD5791_OUT_TRISTATE     0x02ul

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! Initializes the communication with the device. */
uint8_t AD57XX_Init(uint8_t deviceVersion);

/*! Writes data into a register. */
void AD57XX_SetRegisterValue(uint8_t registerAddress,
                             uint32_t registerValue);

/*! Reads the value of a register. */
uint32_t AD57XX_GetRegisterValue(uint8_t registerAddress);

/*! The part is placed in normal mode or its output is clamped to the ground. */
void AD57XX_EnableOutput(uint8_t state);

/*! Writes to the DAC register. */
void AD57XX_SetDacValue(int32_t value);

/*! Sets the clear code. */
void AD57XX_SetClearCode(uint32_t clrCode);

/*! Asserts RESET, CLR and LDAC in a software manner. */
void AD57XX_SoftInstruction(uint8_t instructionBit);

/*! Writes to Control Register. */
void AD57XX_Setup(uint32_t setupWord);

/* LDAC */
void AD57XX_LDAC_OUT(void);  //       pinMode(AD57XX_LDAC_PIN, OUTPUT)
void AD57XX_LDAC_LOW(void);  //      digitalWrite(AD57XX_LDAC_PIN, LOW)
void AD57XX_LDAC_HIGH(void); //      digitalWrite(AD57XX_LDAC_PIN, HIGH)

/* CLR */
void AD57XX_CLR_OUT(void);   //      pinMode(AD57XX_CLR_PIN, OUTPUT)
void AD57XX_CLR_LOW(void);   //     digitalWrite(AD57XX_CLR_PIN, LOW)
void AD57XX_CLR_HIGH(void);	  //     digitalWrite(AD57XX_CLR_PIN, HIGH)

/* RESET */
void AD57XX_RESET_OUT(void);  //     pinMode(AD57XX_RESET_PIN, OUTPUT)
void AD57XX_RESET_LOW(void); //      digitalWrite(AD57XX_RESET_PIN, LOW)
void AD57XX_RESET_HIGH(void); //     digitalWrite(AD57XX_RESET_PIN, HIGH)


#endif	// __AD57XX_H__
