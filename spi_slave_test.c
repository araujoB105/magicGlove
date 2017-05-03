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
#include "./driverlib/MSP430F5xx_6xx/driverlib.h"

//*****************************************************************************
//!   SPI slave talks to SPI master using 3-wire mode. Data is received
//!   from master and data from slave is then transmitted back to master.
//!   USCI RX ISR is used to handle communication, CPU normally in LPM4.
//!   Prior to initial data exchange, master pulses slaves RST for complete
//!   reset.
//!   ACLK = ~32.768kHz, MCLK = SMCLK = DCO ~ 1048kHz
//!
//!   Use with SPI Master Incremented Data code example.  If the slave is in
//!   debug mode, the reset signal from the master will conflict with slave's
//!   JTAG; to work around, use IAR's "Release JTAG on Go" on slave device.  If
//!   breakpoints are set in slave RX ISR, master must stopped also to avoid
//!   overrunning slave RXBUF.
//!
//!
//!                 -----------------
//!            /|\ |                 |
//!             |  |                 |
//!                |                 |
//!                |                 |
//!                |             P3.0|-> Data Out (UCB0SIMO)
//!                |                 |
//!                |             P3.1|<- Data In (UCB0SOMI)
//!                |                 |
//!                |             P3.2|<- Serial Clock Out (UCB0CLK)
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SPI peripheral
//! - GPIO Port peripheral (for SPI pins)
//! - UCB0SIMO
//! - UCB0SOMI
//! - UCB0CLK
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - USCI_A0_VECTOR
//!
//
//*****************************************************************************

uint8_t transmitData = 0x01, receiveData = 0x00;

void main(void)
{
    WDT_A_hold(WDT_A_BASE);
    initSlaveSPI_A();
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    //Enter LPM4, enable interrupts
    __bis_SR_register(LPM4_bits+GIE);
   }

//******************************************************************************
//
//This is the USCI_B0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void USCI_A0_ISR(void)
{
    static int count=0;
    switch(__even_in_range(UCA0IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
        //USCI_A0 TX buffer ready?

        //Receive data from master
        receiveData = USCI_A_SPI_receiveData(USCI_A0_BASE);
        /*if (receiveData==0xFF){
           GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }*/

        if (receiveData==0xAB){
            count++;


            //Transmit data to master
            if (count==100){
                GPIO_toggleOutputOnPin(
                                    GPIO_PORT_P1,
                                    GPIO_PIN0
                                            );
                count=0;
            }
        }

        transmitSPI_A(0xCC);
        break;

    default: break;
    }
}
