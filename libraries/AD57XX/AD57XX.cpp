/***************************************************************************//**
 *   @file   AD57XX.c
 *   @brief  Implementation of AD57XX Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "Arduino.h"
#include <stdint.h>
#include "AD57XX_Comm.h"		// Communication definitions.
#include "AD57XX.h"			    // AD57XX definitions.

/*****************************************************************************/
/************************ Variables Definitions ******************************/
/*****************************************************************************/
uint8_t deviceBitNumber = 20;

/**************************************************************************//**
 * @brief Initializes the communication with the device.
 *
 * @param deviceVersion - The name of one of the supported devices.
 *                        Example: AD5760
 *                                 AD5780
 *                                 AD5781
 *                                 AD5790
 *                                 AD5791
 *
 * @return status - Result of the initialization procedure.
 *					Example: 0x0 - SPI peripheral was not initialized.
 *				  			 0x1 - SPI peripheral is initialized.
*******************************************************************************/
uint8_t AD57XX_Init(uint8_t deviceVersion)
{  
    uint8_t status = 0;

    /* GPIO configuration. */
    AD57XX_CLR_OUT();    
    AD57XX_LDAC_OUT();
    AD57XX_RESET_OUT();
    AD57XX_CLR_HIGH();
    AD57XX_LDAC_HIGH();
    AD57XX_RESET_HIGH();    
    status = AD57XX_COMM_Init(0,        // Transfer format.
                      1000000,  // SPI clock frequency.
                      1,        // SPI clock polarity.
                      1);       // SPI clock edge.
    switch(deviceVersion)
    {
     case AD5760:
         deviceBitNumber = 16;
         break;
    case AD5780:
         deviceBitNumber = 18;
         break;
    case AD5781:
         deviceBitNumber = 18;
         break;
    case AD5790:
         deviceBitNumber = 20;
         break;
    case AD5791:
         deviceBitNumber = 20;
         break;       
    }
    
    return(status);
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 *                          Example: AD57XX_REG_DAC
 *                                   AD57XX_REG_CTRL
 *                                   AD57XX_REG_CLR_CODE
 *                                   AD57XX_CMD_WR_SOFT_CTRL
 *                                   AD57XX_NOP
 * @param registerValue - Value of the register.
 *
 * @return None.
*******************************************************************************/
void AD57XX_SetRegisterValue(uint8_t registerAddress,
                             uint32_t registerValue)
{
    uint8_t writeCommand[3] = {0, 0, 0};
    uint32_t spiWord         = 0;
    
    /*spiWord = AD57XX_RW_(0) | AD57XX_ADDR_REG(registerAddress) | registerValue;
    writeCommand[0] = (spiWord & 0xFF0000) >> 16;
    writeCommand[1] = (spiWord & 0x00FF00) >> 8;
    writeCommand[2] = (spiWord & 0x0000FF);
*/
    spiWord = AD57XX_WRITE | AD57XX_ADDR_REG(registerAddress) | (registerValue & 0xFFFFF);
    writeCommand[0] = (spiWord >> 16) & 0x0000FF;
    writeCommand[1] = (spiWord >> 8 ) & 0x0000FF;
    writeCommand[2] = (spiWord >> 0 ) & 0x0000FF;
    
    #ifdef DEBUGG
    Serial.print("SET ");
    Serial.print(registerAddress, HEX);
    Serial.print(":");
    Serial.print(registerValue, HEX);
    Serial.print(" ");
    Serial.print(spiWord, HEX);
    Serial.print(" ");
    Serial.print(writeCommand[0], HEX);
    Serial.print(" ");
    Serial.print(writeCommand[1], HEX);
    Serial.print(" ");
    Serial.println(writeCommand[2], HEX);
    #endif // DEBUGG

    AD57XX_Write(AD57XX_SLAVE_ID, writeCommand, 3);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *                          Example: AD57XX_REG_DAC
 *                                   AD57XX_REG_CTRL
 *                                   AD57XX_REG_CLR_CODE
 *                                   AD57XX_CMD_WR_SOFT_CTRL
 *                                   AD57XX_NOP
 *
 * @return readData - Value of the register.
*******************************************************************************/
uint32_t AD57XX_GetRegisterValue(uint8_t registerAddress)
{
    uint8_t registerWord[3] = {0, 0, 0}; 
    uint32_t readData        = 0;
   
    registerWord[0] = (uint32_t)(AD57XX_READ | AD57XX_ADDR_REG(registerAddress)) >> 16;

    #ifdef DEBUGG
    Serial.print("GET ");
    Serial.print(registerAddress, HEX);
    Serial.print(" ");
    Serial.print(registerWord[0], HEX);
    Serial.print(" ");
    Serial.print(registerWord[1], HEX);
    Serial.print(" ");
    Serial.print(registerWord[2], HEX);
    #endif // DEBUGG

    AD57XX_Write(AD57XX_SLAVE_ID, registerWord, 3);  
    registerWord[0] = 0;
    registerWord[1] = 0;
    registerWord[2] = 0;
    AD57XX_Read(AD57XX_SLAVE_ID, registerWord, 3);
    readData = ((uint32_t)registerWord[0] << 16) | 
               ((uint32_t)registerWord[1] << 8)  |
                (uint32_t)registerWord[2];
    #ifdef DEBUGG
    Serial.print(" ");
    Serial.println(readData, HEX);
    #endif // DEBUGG
    return readData;
}

/***************************************************************************//**
 * @brief The part is placed in normal mode or its output is clamped to the 
 *        ground.
 *
 * @param state - Enables/disables the output.
 *                Example: 1 - Enables output.
 *                         0 - Disables output.
 *
 * @return None.
*******************************************************************************/
void AD57XX_EnableOutput(uint8_t state)
{
    uint32_t oldControl = 0;
    uint32_t newControl = 0;
    uint32_t controlControl = 0;
    
    oldControl = AD57XX_GetRegisterValue(AD57XX_REG_CTRL);
    oldControl = AD57XX_GetRegisterValue(AD57XX_REG_CTRL);
    Serial.print("stat1 ");
    Serial.println(oldControl, BIN);
    oldControl = oldControl & ~(AD57XX_CTRL_SDODIS
                                | AD57XX_CTRL_BIN2SC
                                | AD57XX_CTRL_DACTRI
                                | AD57XX_CTRL_OPGND
                                | AD57XX_CTRL_RBUF); // Clears OPGND bit.
    newControl = oldControl | AD57XX_CTRL_OPGND * (!state);
    Serial.print("stat2 ");
    Serial.println(newControl, BIN);
    AD57XX_SetRegisterValue(AD57XX_REG_CTRL, newControl);
    controlControl = AD57XX_GetRegisterValue(AD57XX_REG_CTRL);
    controlControl = AD57XX_GetRegisterValue(AD57XX_REG_CTRL);
    Serial.print("stat3 ");
    Serial.println(controlControl, BIN);
}

/***************************************************************************//**
 * @brief Writes to the DAC register.
 *
 * @param value - The value to be written to DAC.
 *
 * @return None.
*******************************************************************************/
void AD57XX_SetDacValue(int32_t value)
{
    AD57XX_LDAC_LOW();
    //AD57XX_SetRegisterValue(AD57XX_REG_DAC, AD57XX_DAC_DATA(value << (20 - deviceBitNumber)));
    AD57XX_SetRegisterValue(AD57XX_REG_DAC, value << (20 - deviceBitNumber));
    AD57XX_LDAC_HIGH();
}

/***************************************************************************//**
 * @brief Sets the clear code.
 *
 * @param clrCode - Clear code.
 *
 * @return None.
*******************************************************************************/
void AD57XX_SetClearCode(uint32_t clrCode)
{
    AD57XX_SetRegisterValue(AD57XX_REG_CLR_CODE, AD57XX_CLR_CODE_DATA(clrCode << (20 - deviceBitNumber)));
}

/***************************************************************************//**
 * @brief Asserts RESET, CLR and LDAC in a software manner.
 *
 * @param instructionBit - A Software Control Register bit.
 *                         Example: AD57XX_SOFT_CTRL_LDAC  - Load DAC
 *                                  AD57XX_SOFT_CTRL_CLR   - Clear
 *                                  AD57XX_SOFT_CTRL_RESET - Reset
 *
 * @return None.
*******************************************************************************/
void AD57XX_SoftInstruction(uint8_t instructionBit)
{    
    AD57XX_SetRegisterValue(AD57XX_CMD_WR_SOFT_CTRL, instructionBit);
}

/***************************************************************************//**
 * @brief Writes to Control Register.
 *
 * @param setupWord - Is a 24-bit value that sets or clears the Control Register
 *                    bits.
 *                    Example: AD57XX_CTRL_BIN2SC | AD57XX_CTRL_SDODIS - sets
 *                             the DAC register to use offset binary coding and 
 *                             disables SDO pin(tristated).     
 *
 * @return None.
*******************************************************************************/
void AD57XX_Setup(uint32_t setupWord)
{
    AD57XX_SetRegisterValue(AD57XX_REG_CTRL, setupWord);
}

/* LDAC */
void AD57XX_LDAC_OUT(void){
    pinMode(AD57XX_LDAC_PIN, OUTPUT);
}

void AD57XX_LDAC_LOW(void){
    digitalWrite(AD57XX_LDAC_PIN, LOW);
}

void AD57XX_LDAC_HIGH(void){
    digitalWrite(AD57XX_LDAC_PIN, HIGH);
}

/* CLR */
void AD57XX_CLR_OUT(void){
    pinMode(AD57XX_CLR_PIN, OUTPUT);
}

void AD57XX_CLR_LOW(void){
    digitalWrite(AD57XX_CLR_PIN, LOW);
}

void AD57XX_CLR_HIGH(void){
    digitalWrite(AD57XX_CLR_PIN, HIGH);
}

/* RESET */
void AD57XX_RESET_OUT(void){
    pinMode(AD57XX_RESET_PIN, OUTPUT);
}

void AD57XX_RESET_LOW(void){
    digitalWrite(AD57XX_RESET_PIN, LOW);
}

void AD57XX_RESET_HIGH(void){
    digitalWrite(AD57XX_RESET_PIN, HIGH);
}
