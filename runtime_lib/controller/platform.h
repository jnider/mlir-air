//===- platform.h -----------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include "hsa.h"
#include <cstdint>

#define NPI_BASE 0xF70A0000UL
#define UART_BASE 0xFF000000UL

#define DRAM_1_BASE 0x000800000000ULL
#define DRAM_1_SIZE 0x000380000000ULL

#ifdef ARM_CONTROLLER
#define CDMA_BASE 0x0000A4000000UL
#else
#define CDMA_BASE 0x000044000000UL
#endif // ARM_CONTROLLER

#define AIE_BASE 0x020000000000ULL
#define AIE_CSR_SIZE 0x000100000000ULL

#define ernic_0_base 0x0000020100080000UL
#define ernic_1_base 0x00000201000C0000UL
#define shmem_base 0x020100000000ULL

#define IO_READ32(addr) *((volatile uint32_t *)(addr))
#define IO_WRITE32(addr, val) *((volatile uint32_t *)(addr)) = val

#define HIGH_ADDR(addr) ((addr & 0xffffffff00000000ULL) >> 32)
#define LOW_ADDR(addr) (addr & 0x00000000ffffffffULL)

void init_platform();
void cleanup_platform();

/*
        Return the base address of the interface data structures
*/
uint64_t get_base_address(void);

void packet_set_active(hsa_agent_dispatch_packet_t *pkt, bool b);

#endif
