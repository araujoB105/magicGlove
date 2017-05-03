/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//*****************************************************************************
//
// usci_a_spi.c - Driver for the usci_a_spi Module.
//
//*****************************************************************************

//!                  MSP430F5529LP
//!                 -----------------
//!                |                 |
//!                |                 |
//!                |                 |
//!                |                 |
//!                |             P3.3|-> Data Out (UCB0SIMO)
//!                |                 |
//!                |             P3.4|<- Data In (UCB0SOMI)
//!                |                 |
//!                |             P2.7|-> Serial Clock Out (UCB0CLK)

//*****************************************************************************
//
//! \addtogroup usci_a_spi_api usci_a_spi
//! @{
//
//*****************************************************************************

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_USCI_Ax__
#include "usci_a_spi.h"
#include "gpio.h"
#include "ucs.h"
#include <assert.h>

bool USCI_A_SPI_initMaster(uint16_t baseAddress,
                           USCI_A_SPI_initMasterParam *param)
{
    //Disable the USCI Module
    HWREG8(baseAddress + OFS_UCAxCTL1) |= UCSWRST;

    //Reset OFS_UCAxCTL0 values
    HWREG8(baseAddress + OFS_UCAxCTL0) &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB +
                                            UCMST + UCMODE_3 + UCSYNC);

    //Reset OFS_UCAxCTL1 values
    HWREG8(baseAddress + OFS_UCAxCTL1) &= ~(UCSSEL_3);

    //Select Clock
    HWREG8(baseAddress + OFS_UCAxCTL1) |= param->selectClockSource;

    HWREG16(baseAddress + OFS_UCAxBRW) =
        (uint16_t)(param->clockSourceFrequency / param->desiredSpiClock);

    /*
     * Configure as SPI master mode.
     * Clock phase select, polarity, msb
     * UCMST = Master mode
     * UCSYNC = Synchronous mode
     * UCMODE_0 = 3-pin SPI
     */
    HWREG8(baseAddress + OFS_UCAxCTL0) |= (
        param->msbFirst +
        param->clockPhase +
        param->clockPolarity +
        UCMST +
        UCSYNC +
        UCMODE_0
        );
    //No modulation
    HWREG8(baseAddress + OFS_UCAxMCTL) = 0;

    return (STATUS_SUCCESS);
}

void USCI_A_SPI_changeMasterClock(uint16_t baseAddress,
                                  USCI_A_SPI_changeMasterClockParam *param)
{
    //Disable the USCI Module
    HWREG8(baseAddress + OFS_UCAxCTL1) |= UCSWRST;

    HWREG8(baseAddress + OFS_UCAxBRW) =
        (uint16_t)(param->clockSourceFrequency / param->desiredSpiClock);

    //Reset the UCSWRST bit to enable the USCI Module
    HWREG8(baseAddress + OFS_UCAxCTL1) &= ~(UCSWRST);
}

bool USCI_A_SPI_initSlave(uint16_t baseAddress,
                          uint8_t msbFirst,
                          uint8_t clockPhase,
                          uint8_t clockPolarity)
{
    //Disable USCI Module
    HWREG8(baseAddress + OFS_UCAxCTL1) |= UCSWRST;

    //Reset OFS_UCAxCTL0 register
    HWREG8(baseAddress + OFS_UCAxCTL0) &= ~(UCMSB +
                                            UC7BIT +
                                            UCMST +
                                            UCCKPL +
                                            UCCKPH +
                                            UCMODE_3
                                            );

    //Clock polarity, phase select, msbFirst, SYNC, Mode0
    HWREG8(baseAddress + OFS_UCAxCTL0) |= (clockPhase +
                                           clockPolarity +
                                           msbFirst +
                                           UCSYNC +
                                           UCMODE_0
                                           );

    return (STATUS_SUCCESS);
}

void USCI_A_SPI_transmitData(uint16_t baseAddress,
                             uint8_t transmitData)
{
    HWREG8(baseAddress + OFS_UCAxTXBUF) = transmitData;
}

uint8_t USCI_A_SPI_receiveData(uint16_t baseAddress)
{
    return (HWREG8(baseAddress + OFS_UCAxRXBUF));
}

void USCI_A_SPI_enableInterrupt(uint16_t baseAddress,
                                uint8_t mask)
{
    HWREG8(baseAddress + OFS_UCAxIE) |= mask;
}

void USCI_A_SPI_disableInterrupt(uint16_t baseAddress,
                                 uint8_t mask)
{
    HWREG8(baseAddress + OFS_UCAxIE) &= ~mask;
}

uint8_t USCI_A_SPI_getInterruptStatus(uint16_t baseAddress,
                                      uint8_t mask)
{
    return (HWREG8(baseAddress + OFS_UCAxIFG) & mask);
}

void USCI_A_SPI_clearInterrupt(uint16_t baseAddress,
                               uint8_t mask)
{
    HWREG8(baseAddress + OFS_UCAxIFG) &= ~mask;
}

void USCI_A_SPI_enable(uint16_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG8(baseAddress + OFS_UCAxCTL1) &= ~(UCSWRST);
}

void USCI_A_SPI_disable(uint16_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG8(baseAddress + OFS_UCAxCTL1) |= UCSWRST;
}

uint32_t USCI_A_SPI_getReceiveBufferAddressForDMA(uint16_t baseAddress)
{
    return (baseAddress + OFS_UCAxRXBUF);
}

uint32_t USCI_A_SPI_getTransmitBufferAddressForDMA(uint16_t baseAddress)
{
    return (baseAddress + OFS_UCAxTXBUF);
}

uint8_t USCI_A_SPI_isBusy(uint16_t baseAddress)
{
    //Return the bus busy status.
    return (HWREG8(baseAddress + OFS_UCAxSTAT) & UCBUSY);
}

bool initMasterSPI_A(uint32_t SPICLK){
    //P3.3,4, P2.7 option select
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P3,
       GPIO_PIN3 + GPIO_PIN4
       );

   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P2,
       GPIO_PIN7
       );

   bool returnValue;
   //Initialize Master
   USCI_A_SPI_initMasterParam param = {0};
   param.selectClockSource = USCI_A_SPI_CLOCKSOURCE_SMCLK;
   param.clockSourceFrequency = UCS_getSMCLK();
   param.desiredSpiClock = SPICLK;
   param.msbFirst = USCI_A_SPI_MSB_FIRST;
   param.clockPhase = USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
   param.clockPolarity = USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
   returnValue = USCI_A_SPI_initMaster(USCI_A0_BASE, &param);

   //Enable SPI module
   USCI_A_SPI_enable(USCI_A0_BASE);

   //Enable Receive interrupt
   USCI_A_SPI_clearInterrupt(USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);
   USCI_A_SPI_enableInterrupt(USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);

   if(STATUS_FAIL == returnValue)
   {
       return false;
   }
   return true;
}

bool initSlaveSPI_A(){

    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);

    while(GPIO_INPUT_PIN_LOW == GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7)){
        ;
    }

   //P3.0,1,2 option select
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P3,
       GPIO_PIN3 + GPIO_PIN4
       );

   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P2,
       GPIO_PIN7
       );

   bool returnValue;
   returnValue = USCI_A_SPI_initSlave(USCI_A0_BASE,
                                      USCI_A_SPI_MSB_FIRST,
                                      USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                                      USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
                                      );

   //Enable SPI module
   USCI_A_SPI_enable(USCI_A0_BASE);

   //Enable Receive interrupt
   USCI_A_SPI_clearInterrupt(USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);
   USCI_A_SPI_enableInterrupt(USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);

   if(STATUS_FAIL == returnValue)
   {
       return false;
   }
   return true;
}

bool transmitSPI_A(uint8_t transmitData){
    //USCI_A0 TX buffer ready?
    while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE,
                                         USCI_A_SPI_TRANSMIT_INTERRUPT))
    {
        ;
    }

    //Transmit Data to slave
    USCI_A_SPI_transmitData(USCI_A0_BASE, transmitData);

    return true;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for usci_a_spi_api
//! @}
//
//*****************************************************************************
