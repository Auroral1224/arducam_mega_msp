/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */

//                   MSP430FR5994
//                 -----------------
//            /|\ |                 |
//             |  |                 |
//             ---|RST              |
//                |                 |
//                |             P5.0|-> Data Out (UCB1SIMO)
//                |                 |
//                |             P5.1|<- Data In (UCB1SOMI)
//                |                 |
//                |             P5.2|-> Serial Clock Out (UCB1CLK)
//                |                 |
//                |             P5.3|-> Slave Chip Select (GPIO)

#ifndef  __SPI_H
#define  __SPI_H

#include <msp430.h>

#define CS_OUT    P5OUT
#define CS_DIR    P5DIR
#define CS_PIN    BIT3

void spiBegin(void);
void spiCsHigh(unsigned int pin);
void spiCsLow(unsigned int pin);
unsigned char spiWriteRead(unsigned char val);
#endif /*__SPI_H*/
