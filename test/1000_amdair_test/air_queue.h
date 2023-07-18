//===- air_queue.h ---------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2020-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef ACDC_QUEUE_H
#define ACDC_QUEUE_H

typedef struct queue_s {

  // HSA-like interface
  uint32_t type;
  uint32_t features;
  uint64_t base_address;
  volatile uint64_t doorbell;
  uint32_t size;
  uint32_t reserved0;
  uint64_t id;

  // implementation detail
  uint64_t read_index;
  uint64_t write_index;
  uint64_t last_doorbell;

  uint64_t base_address_paddr;
  uint64_t base_address_vaddr;

} __attribute__((packed, aligned(__alignof__(uint64_t)))) queue_t;

#endif
