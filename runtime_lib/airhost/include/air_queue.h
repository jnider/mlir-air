//===- air_queue.h ---------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2020-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef ACDC_QUEUE_H
#define ACDC_QUEUE_H

#include "hsa/hsa.h"

#include <stdint.h>

// Default BRAM layout is as follows:
// 1 system page
// 1 queue descriptor page, with 16 queue descriptors
// N (=9 for now, 1 for each BP and 1 for ARM) pages for queue ring buffers
// Remaining pages for signals, doorbells, etc

// Define the number of HSA packets we can have in a queue
// as well as the number of pages used for different purposes in BRAM
#define MB_QUEUE_SIZE 64
#define NUM_SYSTEM_PAGES 1
#define NUM_QUEUE_STRUCT_PAGES 1
#define NUM_QUEUE_BUFFER_PAGES 9
#define NUM_SIGNAL_PAGES 16
#define NUM_DOORBELL_PAGES 1

// Should be no need to change below this line
#define MB_PAGE_SIZE 0x1000

#define MB_SHMEM_QUEUE_STRUCT_OFFSET NUM_SYSTEM_PAGES * MB_PAGE_SIZE
#define MB_SHMEM_QUEUE_STRUCT_SIZE NUM_QUEUE_STRUCT_PAGES * MB_PAGE_SIZE

#define MB_SHMEM_QUEUE_BUFFER_OFFSET MB_SHMEM_QUEUE_STRUCT_OFFSET + MB_SHMEM_QUEUE_STRUCT_SIZE
#define MB_SHMEM_QUEUE_BUFFER_SIZE NUM_QUEUE_BUFFER_PAGES * MB_PAGE_SIZE

// Area of memory that can be used for signals.
// A controller will initialize these to zero.
#define MB_SHMEM_SIGNAL_OFFSET MB_SHMEM_QUEUE_BUFFER_OFFSET + MB_SHMEM_QUEUE_BUFFER_SIZE
#define MB_SHMEM_SIGNAL_SIZE NUM_SIGNAL_PAGES * MB_PAGE_SIZE

#define MB_SHMEM_DOORBELL_OFFSET MB_SHMEM_SIGNAL_OFFSET + MB_SHMEM_SIGNAL_SIZE
#define MB_SHMEM_DOORBELL_SIZE NUM_DOORBELL_PAGES * MB_PAGE_SIZE

// Note below that "__attribute__((packed))" also asserts that the whole
// structure is unaligned in some compilers.  This helps to silence errors from
// -waddress-of-packed-struct

typedef struct dispatch_packet_s {

  // HSA-like interface
  volatile uint16_t header;
  volatile uint16_t type;
  uint32_t reserved0;
  uint64_t return_address;
  uint64_t arg[4];
  uint64_t reserved1;
  uint64_t completion_signal;

} __attribute__((packed, aligned(__alignof__(uint64_t)))) dispatch_packet_t;

typedef struct barrier_and_packet_s {

  // HSA-like interface
  volatile uint16_t header;
  uint16_t reserved0;
  uint32_t reserved1;
  uint64_t dep_signal[5];
  uint64_t reserved2;
  uint64_t completion_signal;

} __attribute__((packed, aligned(__alignof__(uint64_t)))) barrier_and_packet_t;

typedef struct barrier_or_packet_s {

  // HSA-like interface
  volatile uint16_t header;
  uint16_t reserved0;
  uint32_t reserved1;
  uint64_t dep_signal[5];
  uint64_t reserved2;
  uint64_t completion_signal;

} __attribute__((packed, aligned(__alignof__(uint64_t)))) barrier_or_packet_t;

typedef struct queue_s {

  // HSA-like interface
  uint32_t type;
  uint32_t features;
  uint64_t base_address;
  uint64_t* doorbell;
  uint64_t* doorbell_vaddr;
  uint32_t size;
  uint32_t reserved0;
  uint64_t id;

  // implementation detail
  uint64_t read_index;
  uint64_t write_index;
  uint64_t last_doorbell;

  uint64_t base_address_paddr;
  uint64_t base_address_vaddr;
  uint64_t reserved[21];

} __attribute__((packed, aligned(__alignof__(uint64_t)))) queue_t;

typedef struct signal_s {
  uint64_t handle;
} signal_t;

typedef uint64_t signal_value_t;

namespace {

inline uint64_t queue_add_write_index(queue_t *q, uint64_t v) {
  auto r = q->write_index;
  q->write_index = r + v;
  return r;
}

inline uint64_t queue_add_read_index(queue_t *q, uint64_t v) {
  auto r = q->read_index;
  q->read_index = r + v;
  return r;
}

inline uint64_t queue_load_read_index(queue_t *q) { return q->read_index; }

inline uint64_t queue_load_write_index(queue_t *q) { return q->write_index; }

inline uint64_t queue_paddr_from_index(queue_t *q, uint64_t idx) {
  return q->base_address + idx;
}

inline bool packet_get_active(dispatch_packet_t *pkt) {
  return pkt->reserved1 & 0x1;
}

inline void packet_set_active(dispatch_packet_t *pkt, bool b) {
  pkt->reserved1 = (pkt->reserved1 & ~0x1) | b;
}

inline void initialize_packet(dispatch_packet_t *pkt) {
  pkt->header = HSA_PACKET_TYPE_INVALID;
  // pkt->type = AIR_PKT_TYPE_INVALID;
}

inline hsa_status_t signal_create(signal_value_t initial_value,
                                  uint32_t num_consumers, void *consumers,
                                  signal_t *signal) {
  // auto s = (signal_value_t*)malloc(sizeof(signal_value_t));
  //*s = initial_value;
  signal->handle = (uint64_t)initial_value;
  return HSA_STATUS_SUCCESS;
}

inline hsa_status_t signal_destroy(signal_t signal) {
  // free((void*)signal.handle);
  return HSA_STATUS_SUCCESS;
}

inline void signal_store_release(signal_t *signal, signal_value_t value) {
  signal->handle = (uint64_t)value;
}

inline signal_value_t signal_wait_acquire(volatile signal_t *signal,
                                          hsa_signal_condition_t condition,
                                          signal_value_t compare_value,
                                          uint64_t timeout_hint,
                                          hsa_wait_state_t wait_state_hint) {
  signal_value_t ret = 0;
  uint64_t timeout = timeout_hint;
  do {
    ret = signal->handle;
    if (ret == compare_value)
      return compare_value;
  } while (timeout--);
  return ret;
}

inline void signal_subtract_acq_rel(signal_t *signal, signal_value_t value) {
  signal->handle = signal->handle - (uint64_t)value;
  // uint64_t i;
  // memcpy((void*)&i, (void*)signal, sizeof(signal_value_t));
  // i = i - (uint64_t)value;
  // memcpy((void*)signal, (void*)&i, sizeof(signal_value_t));
}

} // namespace

#endif
