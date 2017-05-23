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

#include "driverlib.h"
#include "bmp280.h"


//******************************************************************************
//!
//!   Empty Project that includes driverlib
//!
//******************************************************************************
void main(void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    transmitCounter = 0;
    transmitLength = 0;
    receiveCount = 0;

    I2C_InitMaster();

    s32 temp1 = bmp280_data_readout_template();
    s32 temp2 = bmp280_data_readout_template();

}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_B0_VECTOR)))
#endif
void USCI_B0_ISR(void)
{
    switch(__even_in_range(UCB0IV,12))
    {
    case USCI_I2C_UCTXIFG:
    {
        //Check TX byte counter
        if(transmitCounter < transmitLength)
        {
            //Initiate send of character from Master to Slave
            USCI_B_I2C_masterSendMultiByteNext(USCI_B0_BASE,
                                               transmitData[transmitCounter]
                                               );

            //Increment TX byte counter
            transmitCounter++;
        }
        else
        {
            //Initiate stop only
            USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);

            //Clear master interrupt status
            USCI_B_I2C_clearInterrupt(USCI_B0_BASE,
                                      USCI_B_I2C_TRANSMIT_INTERRUPT);

            //Exit LPM0 on interrupt return
            __bic_SR_register_on_exit(LPM0_bits);
        }
        break;
    }
    case USCI_I2C_UCRXIFG:
    {
        //Decrement RX byte counter
        receiveCount--;

        if(receiveCount)
        {
            if(receiveCount == 1)
            {
                //Initiate end of reception -> Receive byte with NAK
                *receiveBufferPointer++ =
                    USCI_B_I2C_masterReceiveMultiByteFinish(
                        USCI_B0_BASE
                        );
            }
            else
            {
                //Keep receiving one byte at a time
                *receiveBufferPointer++ = USCI_B_I2C_masterReceiveMultiByteNext(
                    USCI_B0_BASE
                    );
            }
        }
        else
        {
            //Receive last byte
            *receiveBufferPointer = USCI_B_I2C_masterReceiveMultiByteNext(
                USCI_B0_BASE
                );
            __bic_SR_register_on_exit(LPM0_bits);
        }
        break;
    }
    }
}
