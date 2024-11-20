/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */
#include "spi.h"

void spiBegin(void)
{
    // Configure USCI_B1 for SPI operation
    UCB1CTLW0 = UCSWRST;                    // **Put state machine in reset**
    //Clock Polarity: The inactive state is high
    //MSB First, 8-bit, Master, 3-pin mode, Synchronous, SMCLK
    UCB1CTLW0 |= UCCKPH | UCMSB | UCSYNC | UCMST | UCSSEL__SMCLK;
    UCB1BRW = 0x20;                         // /2
    UCB1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
}

void spiCsHigh(unsigned int pin)
{
    CS_OUT |= pin;
}
void spiCsLow(unsigned int pin)
{
    CS_OUT &= ~pin;
}

unsigned char spiWriteRead(unsigned char val)
{
    unsigned char rt = 0;

    UCB1TXBUF = val;
    while ((UCB1STAT & UCBUSY))
        ;
    rt = UCB1RXBUF;
    while ((UCB1STAT & UCBUSY))
        ;

    return rt;
}

