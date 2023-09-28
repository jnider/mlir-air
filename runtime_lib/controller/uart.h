/******************************************************************************
* Copyright (C) 2017 - 2021 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(uint64_t BaseAddress);
uint32_t uart_IsReceiveData(void);
void uart_SendByte(uint8_t Data);
uint8_t uart_RecvByte(void);

#endif // UART_H
