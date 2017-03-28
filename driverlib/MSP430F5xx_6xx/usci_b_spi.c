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
// usci_b_spi.c - Driver for the usci_b_spi Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup usci_b_spi_api usci_b_spi
//! @{
//
//*****************************************************************************

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_USCI_Bx__
#include "usci_b_spi.h"

#include <assert.h>

bool USCI_B_SPI_initMaster(uint16_t baseAddress,
                           USCI_B_SPI_initMasterParam *param)
{
    //Disable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCSWRST;

    //Reset OFS_UCBxCTL0 values
    HWREG8(baseAddress + OFS_UCBxCTL0) &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB +
                                            UCMST + UCMODE_3 + UCSYNC);

    //Reset OFS_UCBxCTL1 values
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~(UCSSEL_3);

    //Select Clock
    HWREG8(baseAddress + OFS_UCBxCTL1) |= param->selectClockSource;

    HWREG16(baseAddress + OFS_UCBxBRW) =
        (uint16_t)(param->clockSourceFrequency / param->desiredSpiClock);

    /*
     * Configure as SPI master mode.
     * Clock phase select, polarity, msb
     * UCMST = Master mode
     * UCSYNC = Synchronous mode
     * UCMODE_0 = 3-pin SPI
     */
    HWREG8(baseAddress + OFS_UCBxCTL0) |= (
        param->msbFirst +
        param->clockPhase +
        param->clockPolarity +
        UCMST +
        UCSYNC +
        UCMODE_0
        );

    return (STATUS_SUCCESS);
}

void USCI_B_SPI_changeMasterClock(uint16_t baseAddress,
                                  USCI_B_SPI_changeMasterClockParam *param)
{
    //Disable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCSWRST;

    HWREG8(baseAddress + OFS_UCBxBRW) =
        (uint16_t)(param->clockSourceFrequency / param->desiredSpiClock);

    //Reset the UCSWRST bit to enable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~(UCSWRST);
}

bool USCI_B_SPI_initSlave(uint16_t baseAddress,
                          uint8_t msbFirst,
                          uint8_t clockPhase,
                          uint8_t clockPolarity)
{
    //Disable USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCSWRST;

    //Reset OFS_UCBxCTL0 register
    HWREG8(baseAddress + OFS_UCBxCTL0) &= ~(UCMSB +
                                            UC7BIT +
                                            UCMST +
                                            UCCKPL +
                                            UCCKPH +
                                            UCMODE_3
                                            );

    //Clock polarity, phase select, msbFirst, SYNC, Mode0
    HWREG8(baseAddress + OFS_UCBxCTL0) |= (clockPhase +
                                           clockPolarity +
                                           msbFirst +
                                           UCSYNC +
                                           UCMODE_0
                                           );

    return (STATUS_SUCCESS);
}

void USCI_B_SPI_transmitData(uint16_t baseAddress,
                             uint8_t transmitData)
{
    HWREG8(baseAddress + OFS_UCBxTXBUF) = transmitData;
}

uint8_t USCI_B_SPI_receiveData(uint16_t baseAddress)
{
    return (HWREG8(baseAddress + OFS_UCBxRXBUF));
}

void USCI_B_SPI_enableInterrupt(uint16_t baseAddress,
                                uint8_t mask)
{
    HWREG8(baseAddress + OFS_UCBxIE) |= mask;
}

void USCI_B_SPI_disableInterrupt(uint16_t baseAddress,
                                 uint8_t mask)
{
    HWREG8(baseAddress + OFS_UCBxIE) &= ~mask;
}

uint8_t USCI_A_SPI_getInterruptStatus(uint16_t baseAddress,
                                      uint8_t mask)
{
    return (HWREG8(baseAddress + OFS_UCBxIFG) & mask);
}

void USCI_B_SPI_clearInterrupt(uint16_t baseAddress,
                               uint8_t mask)
{
    HWREG8(baseAddress + OFS_UCBxIFG) &= ~mask;
}

void USCI_B_SPI_enable(uint16_t baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) &= ~(UCSWRST);
}

void USCI_B_SPI_disable(uint16_t baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG8(baseAddress + OFS_UCBxCTL1) |= UCSWRST;
}

uint32_t USCI_B_SPI_getReceiveBufferAddressForDMA(uint16_t baseAddress)
{
    return (baseAddress + OFS_UCBxRXBUF);
}

uint32_t USCI_B_SPI_getTransmitBufferAddressForDMA(uint16_t baseAddress)
{
    return (baseAddress + OFS_UCBxTXBUF);
}

uint8_t USCI_B_SPI_isBusy(uint16_t baseAddress)
{
    //Return the bus busy status.
    return (HWREG8(baseAddress + OFS_UCBxSTAT) & UCBUSY);
}

bool initMasterSPI_B(uint32_t SPICLK){
    //P3.0,1,2 option select
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P3,
       GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2
       );

   //Initialize Master
   USCI_B_SPI_initMasterParam param = {0};
   param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
   param.clockSourceFrequency = UCS_getSMCLK();
   param.desiredSpiClock = SPICLK;
   param.msbFirst = USCI_B_SPI_MSB_FIRST;
   param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
   param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
   returnValue = USCI_B_SPI_initMaster(USCI_B0_BASE, &param);

   //Enable SPI module
   USCI_B_SPI_enable(USCI_B0_BASE);

   //Enable Receive interrupt
   USCI_B_SPI_clearInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);
   USCI_B_SPI_enableInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);

   if(STATUS_FAIL == returnValue)
   {
       return false;
   }
   return true;
}

bool initSlaveSPI_B(){

    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN2);

    while(GPIO_INPUT_PIN_LOW == GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN2)){
        ;
    }

   //P3.0,1,2 option select
   GPIO_setAsPeripheralModuleFunctionInputPin(
       GPIO_PORT_P3,
       GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2
       );

   returnValue = USCI_B_SPI_initSlave(USCI_B0_BASE,
                                      USCI_B_SPI_MSB_FIRST,
                                      USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                                      USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
                                      );

   //Enable SPI module
   USCI_B_SPI_enable(USCI_B0_BASE);

   //Enable Receive interrupt
   USCI_B_SPI_clearInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);
   USCI_B_SPI_enableInterrupt(USCI_B0_BASE, USCI_B_SPI_RECEIVE_INTERRUPT);

   if(STATUS_FAIL == returnValue)
   {
       return false;
   }
   return true;
}

bool transmitSPI_B(uint8_t transmitData){
    //USCI_A0 TX buffer ready?
    while(!USCI_B_SPI_getInterruptStatus(USCI_B0_BASE,
                                         USCI_B_SPI_TRANSMIT_INTERRUPT))
    {
        ;
    }

    //Transmit Data to slave
    USCI_B_SPI_transmitData(USCI_B0_BASE, transmitData);

    return true;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for usci_b_spi_api
//! @}
//
//*****************************************************************************
