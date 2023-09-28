/******************************************************************************
* Copyright (C) 2017 - 2021 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

#include "platform.h"
#include "uart.h"

#define XUARTPSV_UARTDR_OFFSET 	0x0000U	/**< Data */
#define XUARTPSV_UARTRSR_OFFSET	0x0004U	/**< Receive Status Register/Error Clear */

#define XUARTPSV_UARTFR_OFFSET 	0x0018U	/**< Flag Register */
#define XUARTPSV_UARTFR_RI 	0x00000100U	/**< Ring indicator */
#define XUARTPSV_UARTFR_TXFE	0x00000080U	/**< Transmit FIFO empty */
#define XUARTPSV_UARTFR_RXFF	0x00000040U	/**< Receive FIFO full */
#define XUARTPSV_UARTFR_TXFF	0x00000020U	/**< Transmit FIFO full */
#define XUARTPSV_UARTFR_RXFE	0x00000010U	/**< Receive FIFO empty */
#define XUARTPSV_UARTFR_BUSY	0x00000008U	/**< UART Busy */
#define XUARTPSV_UARTFR_DCD 	0x00000004U	/**< Data carrier detect */
#define XUARTPSV_UARTFR_DSR	0x00000002U	/**<  Data set ready */
#define XUARTPSV_UARTFR_CTS 	0x00000001U	/**< Clear to send */

#define XUARTPSV_UARTILPR_OFFSET	0x0020U	/**< IrDA Low-Power Counter */
#define XUARTPSV_UARTIBRD_OFFSET	0x0024U	/**< Integer Baud Rate */
#define XUARTPSV_UARTFBRD_OFFSET	0x0028U	/**< Fractional Baud Rate */
#define XUARTPSV_UARTLCR_OFFSET	0x002CU	/**< Line Control */
#define XUARTPSV_UARTCR_OFFSET 	0x0030U	/**< Control */
#define XUARTPSV_UARTIFLS_OFFSET	0x0034U	/**< Interrupt FIFO Level Select */
#define XUARTPSV_UARTIMSC_OFFSET	0x0038U	/**< Interrupt Mask Set/Clear */
#define XUARTPSV_UARTRIS_OFFSET	0x003CU	/**< Raw interrupt Status */
#define XUARTPSV_UARTMIS_OFFSET	0x0040U	/**< Mask interrupt Status */
#define XUARTPSV_UARTICR_OFFSET	0x0044U	/**< Interrupt Clear */
#define XUARTPSV_UARTDMACR_OFFSET	0x0048U	/**< DMA Control */

static uint64_t base;

void uart_init(uint64_t BaseAddress)
{
  base = BaseAddress;
}

uint32_t uart_IsReceiveData(void)
{
  return !((IO_READ32(base + XUARTPSV_UARTFR_OFFSET) &
      XUARTPSV_UARTFR_RXFE) == XUARTPSV_UARTFR_RXFE);
}

uint32_t uart_IsTransmitFull(void)
{
  return ((IO_READ32(base + XUARTPSV_UARTFR_OFFSET) &
    XUARTPSV_UARTFR_TXFF) == XUARTPSV_UARTFR_TXFF);
}

void uart_SendByte(uint8_t Data)
{
	while (uart_IsTransmitFull());

	IO_WRITE32(base + XUARTPSV_UARTDR_OFFSET, Data);
}

uint8_t uart_RecvByte(void)
{
	uint32_t RecievedByte;

	while (!uart_IsReceiveData());

	RecievedByte = IO_READ32(base + XUARTPSV_UARTDR_OFFSET);
	return (uint8_t)RecievedByte;
}
