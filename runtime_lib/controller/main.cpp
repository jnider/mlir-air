//===- main.cpp -------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2020-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdio.h>

#include "aie.h"
#include "amd_hsa.h"
#include "debug.h"
#include "hsa_csr.h"
#include "hsa_ext_air.h"
#include "memory.h"
#include "unistd.h"

extern "C" {

// Right now, only ARM can control ERNICs
#ifdef ARM_CONTROLLER
#include "pcie-ernic-defines.h"
#endif

#if !defined(ARM_CONTROLLER)
#include "pvr.h"
#endif
}

#include "airbin.h"
#include "cdma.h"
#include "platform.h"

#include "shell.h"

#ifdef ARM_CONTROLLER
#define XAIE_ADDR_ARRAY_OFF 0
#else
#define XAIE_ADDR_ARRAY_OFF 0x800ULL
#endif // ARM_CONTROLLER

#define ALIGN(_x, _size) (((_x) + (_size - 1)) & ~(_size - 1))

#define LOGICAL_HERD_DMAS 16

#define INVLD_COUNT_TIMEOUT 100

bool setup;

inline uint64_t mymod(uint64_t a) {
  uint64_t result = a;
  while (result >= AQL_QUEUE_NUM_ENTRIES) {
    result -= AQL_QUEUE_NUM_ENTRIES;
  }
  return result;
}

bool packet_get_active(hsa_agent_dispatch_packet_t *pkt) {
  return pkt->reserved2 & 0x1;
}

void packet_set_active(hsa_agent_dispatch_packet_t *pkt, bool b) {
  pkt->reserved2 = (pkt->reserved2 & ~0x1) | b;
}

/*
  read 32 bit value from specified address
*/
static inline uint32_t in32(uint64_t Addr) {
  uint32_t Value = IO_READ32(Addr);
  return Value;
}

/*
  write 32 bit value to specified address
*/
static inline void out32(uint64_t Addr, uint32_t Value) {
  IO_WRITE32(Addr, Value);
}

uint64_t get_base_address(void) { return shmem_base; }

void lock_uart(uint32_t id) {
// ARM has seperate UART so doesn't use lock
#ifndef ARM_CONTROLLER
  XMutex_Lock(xmutex_ptr, XPAR_MUTEX_0_UART_LOCK, id);
#endif
}

void unlock_uart(uint32_t id) {
// ARM has seperate UART so doesn't use lock
#ifndef ARM_CONTROLLER
  XMutex_Unlock(xmutex_ptr, XPAR_MUTEX_0_UART_LOCK, id);
#endif
}

void complete_agent_dispatch_packet(hsa_agent_dispatch_packet_t *pkt) {
  // completion phase
  packet_set_active(pkt, false);
  pkt->header = HSA_PACKET_TYPE_INVALID;
  pkt->type = AIR_PKT_TYPE_INVALID;
  hsa_signal_subtract_scacq_screl(pkt->completion_signal, 1);
}

void complete_barrier_packet(void *pkt) {
  hsa_barrier_and_packet_t *p = (hsa_barrier_and_packet_t *)(pkt);
  // completion phase
  p->header = HSA_PACKET_TYPE_INVALID;
  hsa_signal_subtract_scacq_screl(p->completion_signal, 1);
}

void handle_packet_device_initialize(hsa_agent_dispatch_packet_t *pkt) {
  packet_set_active(pkt, true);
  xaie_device_init();
}

void handle_packet_segment_initialize(hsa_agent_dispatch_packet_t *pkt) {
  setup = true;
  packet_set_active(pkt, true);

  // Address mode here is absolute range
  if (((pkt->arg[0] >> 48) & 0xf) == AIR_ADDRESS_ABSOLUTE_RANGE) {
    uint16_t start_row = (pkt->arg[0] >> 16) & 0xff;
    uint16_t num_rows = (pkt->arg[0] >> 24) & 0xff;
    uint16_t start_col = (pkt->arg[0] >> 32) & 0xff;
    uint16_t num_cols = (pkt->arg[0] >> 40) & 0xff;

    uint16_t segment_id = pkt->arg[1] & 0xffff;

    // TODO more checks on segment dimensions
    if (start_row == 0)
      start_row++;
    xaie_segment_init(start_col, num_cols, start_row, num_rows);
    air_printf("Initialized segment %d at (%d, %d) of size (%d,%d)\r\n",
               segment_id, start_col, start_row, num_cols, num_rows);
  } else {
    air_printf("Unsupported address type 0x%04lX for segment initialize\r\n",
               (pkt->arg[0] >> 48) & 0xf);
  }
}

void handle_packet_get_capabilities(hsa_agent_dispatch_packet_t *pkt,
                                    uint32_t mb_id) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint64_t *addr = (uint64_t *)(pkt->return_address);

  lock_uart(mb_id);
  air_printf("Writing to 0x%lx\n\r", (uint64_t)addr);
  unlock_uart(mb_id);
  // We now write a capabilities structure to the address we were just passed
  // We've already done this once - should we just cache the results?
#if defined(ARM_CONTROLLER)
  int user1 = 1;
  int user2 = 0;

#else
  pvr_t pvr;
  microblaze_get_pvr(&pvr);
  int user1 = MICROBLAZE_PVR_USER1(pvr);
  int user2 = MICROBLAZE_PVR_USER2(pvr);
#endif
  addr[0] = (uint64_t)mb_id;        // region id
  addr[1] = (uint64_t)user1;        // num regions
  addr[2] = (uint64_t)(user2 >> 8); // region controller firmware version
  addr[3] = 16L;                    // cores per region
  addr[4] = 32768L;                 // Total L1 data memory per core
  addr[5] = 8L;                     // Number of L1 data memory banks
  addr[6] = 16384L;                 // L1 program memory per core
  addr[7] = 0L;                     // L2 data memory per region
}

void handle_packet_get_info(hsa_agent_dispatch_packet_t *pkt, uint32_t mb_id) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint64_t attribute = (pkt->arg[0]);
  uint64_t *addr =
      (uint64_t *)(&pkt->return_address); // FIXME when we can use a VA

#if defined(ARM_CONTROLLER)
  int user1 = 1;
  int user2 = 0;
#else
  pvr_t pvr;
  microblaze_get_pvr(&pvr);
  int user1 = MICROBLAZE_PVR_USER1(pvr);
  int user2 = MICROBLAZE_PVR_USER2(pvr);
#endif
  char name[] = "ACDC";
  char vend[] = "AMD";

  // TODO change this to use pkt->return_address
  switch (attribute) {
  case AIR_AGENT_INFO_NAME:
    strcpy((char *)addr, name);
    break;
  case AIR_AGENT_INFO_VENDOR_NAME:
    strcpy((char *)addr, vend);
    break;
  case AIR_AGENT_INFO_CONTROLLER_ID:
    *addr = (uint64_t)mb_id; // region id
    break;
  case AIR_AGENT_INFO_FIRMWARE_VER:
    *addr = (uint64_t)(user2 >> 8); // region controller firmware version
    break;
  case AIR_AGENT_INFO_NUM_REGIONS:
    *addr = (uint64_t)user1; // num regions
    break;
  case AIR_AGENT_INFO_HERD_SIZE: // cores per region
    *addr = xaie_herd_get_num_cols() * xaie_herd_get_num_rows();
    break;
  case AIR_AGENT_INFO_HERD_ROWS:
    *addr = xaie_herd_get_num_rows(); // rows of cores
    break;
  case AIR_AGENT_INFO_HERD_COLS:
    *addr = xaie_herd_get_num_cols(); // cols of cores
    break;
  case AIR_AGENT_INFO_TILE_DATA_MEM_SIZE:
    *addr = 32768L; // total L1 data memory per core
    break;
  case AIR_AGENT_INFO_TILE_PROG_MEM_SIZE:
    *addr = 16384L; // L1 program memory per core
    break;
  case AIR_AGENT_INFO_L2_MEM_SIZE: // L2 memory per region (cols * 256k)
    *addr = 262144L * xaie_herd_get_num_cols();
    break;
  default:
    *addr = 0;
    break;
  }
}

#ifdef ARM_CONTROLLER

/* Used for the device controller to poll on an
incoming RDMA SEND, and copy the payload to some
buffer in memory */
void handle_packet_rdma_post_recv(hsa_agent_dispatch_packet_t *pkt) {

  // Need to do this before processing the packet
  packet_set_active(pkt, true);

  // Parsing the packet
  uint64_t local_physical_address = pkt->arg[0];
  uint8_t qpid = pkt->arg[1] & 0xFF;
  // uint8_t  tag                  = (pkt->arg[1] >> 8)  & 0xFF; // Currently
  // don't use tag
  uint32_t length = (pkt->arg[1] >> 16) & 0xFFFF;
  uint8_t ernic_sel = (pkt->arg[1] >> 48) & 0xFF;

  // Pointing to the proper ERNIC
  volatile uint32_t *ernic_csr = NULL;
  if (ernic_sel == 0) {
    ernic_csr = (volatile uint32_t *)ernic_0_base;
  } else {
    ernic_csr = (volatile uint32_t *)ernic_1_base;
  }

  // Can print without grabbing the lock because running on the ARM in the ARM
  // which has sole use of the UART (BPs use JTAG UART)
  air_printf("Packet:\r\n");
  air_printf("\tlocal_physical_address: 0x%lx\r\n", local_physical_address);
  air_printf("\tlength: 0x%x\r\n", length);
  air_printf("\tqpid: 0x%x\r\n", qpid);
  air_printf("\ternic_sel: 0x%x\r\n", ernic_sel);

  // Determine base address of the RQ to read the RQE
  uint32_t rq_base_address_low = ernic_csr[ERNIC_QP_ADDR(qpid, RQBAi)];
  uint32_t rq_base_address_high = ernic_csr[ERNIC_QP_ADDR(qpid, RQBAMSBi)];
  uint64_t rq_base_address = (((uint64_t)rq_base_address_high) << 32) |
                             ((uint64_t)rq_base_address_low);
  air_printf("\trq_base_address: 0x%lx\r\n", rq_base_address);

  // Wait for RQPIDB to be greater than RQCIDB
  uint32_t rq_ci_db = ernic_csr[ERNIC_QP_ADDR(qpid, RQCIi)];
  uint32_t rq_pi_db = ernic_csr[ERNIC_QP_ADDR(qpid, STATRQPIDBi)];
  air_printf("Polling onon rq_pi_db to be greater than 0x%x. Read: 0x%x\r\n",
             rq_ci_db, rq_pi_db);
  while (rq_pi_db <= rq_ci_db) {
    rq_pi_db = ernic_csr[ERNIC_QP_ADDR(qpid, STATRQPIDBi)];
  }
  air_printf("Observed aSEND. Copying to local buffer\r\n");

  // Copy what RQ PIDB is pointing at to local_physical_address
  void *rqe = (void *)(rq_base_address + (rq_pi_db - 1) * RQE_SIZE);
  air_printf("rqe is at %p and copying to 0x%lx\r\n", rqe,
             local_physical_address);
  memcpy((size_t *)local_physical_address, (size_t *)rqe, length);

  // Increment RQ CIDB so it knows that it can overwrite it
  ernic_csr[ERNIC_QP_ADDR(qpid, RQCIi)] = rq_ci_db + 1;
}

/* Used for the device controller to post an RDMA WQE
to the ERNIC. This can be used to initiate either
one-sided or two-sided communication. */
void handle_packet_rdma_post_wqe(hsa_agent_dispatch_packet_t *pkt) {

  // Need to do this before processing the packet
  packet_set_active(pkt, true);

  // Parsing the packet
  uint64_t remote_virtual_address = pkt->arg[0];
  uint64_t local_physical_address = pkt->arg[1];
  uint32_t length = pkt->arg[2] & 0xFFFF;
  uint8_t op = (pkt->arg[2] >> 32) & 0xFF;
  uint8_t key = (pkt->arg[2] >> 40) & 0xFF;
  uint8_t qpid = (pkt->arg[2] >> 48) & 0xFF;
  uint8_t ernic_sel = (pkt->arg[2] >> 56) & 0xFF;

  // Pointing to the proper ERNIC
  volatile uint32_t *ernic_csr = NULL;
  if (ernic_sel == 0) {
    ernic_csr = (volatile uint32_t *)ernic_0_base;
  } else {
    ernic_csr = (volatile uint32_t *)ernic_1_base;
  }

  // Can print without grabbing the lock because running on the ARM in the ARM
  // which has sole use of the UART (BPs use JTAG UART)
  air_printf("Packet:\r\n");
  air_printf("\tremote_virtual_address: 0x%lx\r\n", remote_virtual_address);
  air_printf("\tlocal_physical_address: 0x%lx\r\n", local_physical_address);
  air_printf("\tlength: 0x%x\r\n", length);
  air_printf("\top: 0x%x\r\n", op);
  air_printf("\tkey: 0x%x\r\n", key);
  air_printf("\tqpid: 0x%x\r\n", qpid);
  air_printf("\ternic_sel: 0x%x\r\n", ernic_sel);

  uint32_t sq_base_address_low = ernic_csr[ERNIC_QP_ADDR(qpid, SQBAi)];
  uint32_t sq_base_address_high = ernic_csr[ERNIC_QP_ADDR(qpid, SQBAMSBi)];
  uint64_t sq_base_address = (((uint64_t)sq_base_address_high) << 32) |
                             ((uint64_t)sq_base_address_low);
  air_printf("\tsq_base_address: 0x%lx\r\n", sq_base_address);

  // Read the doorbell to determine where to put the WQE
  uint32_t sq_pi_db = ernic_csr[ERNIC_QP_ADDR(qpid, SQPIi)];
  air_printf("\tsq_pi_db: 0x%x\r\n", sq_pi_db);

  // Write the WQE to the SQ
  struct pcie_ernic_wqe *wqe =
      &(((struct pcie_ernic_wqe *)(sq_base_address))[sq_pi_db]);
  air_printf("Starting writing WQE to %p\r\n", wqe);
  wqe->wrid = 0xe0a6 & 0x0000FFFF; // Just hardcoding the ID for now
  wqe->laddr_lo = (uint32_t)(local_physical_address & 0x00000000FFFFFFFF);
  wqe->laddr_hi = (uint32_t)(local_physical_address >> 32);
  wqe->length = length;
  wqe->op = op & 0x000000FF;
  wqe->offset_lo = (uint32_t)(remote_virtual_address & 0x00000000FFFFFFFF);
  wqe->offset_hi = (uint32_t)(remote_virtual_address >> 32);
  wqe->rtag = key;
  wqe->send_data_dw_0 = 0;
  wqe->send_data_dw_1 = 0;
  wqe->send_data_dw_2 = 0;
  wqe->send_data_dw_3 = 0;
  wqe->immdt_data = 0;
  wqe->reserved_1 = 0;
  wqe->reserved_2 = 0;
  wqe->reserved_3 = 0;
  air_printf("Done writing WQE\r\n");

  // Ring the doorbell
  ernic_csr[ERNIC_QP_ADDR(qpid, SQPIi)] = sq_pi_db + 1;

  // Poll on the completion
  uint32_t cq_ci_db = ernic_csr[ERNIC_QP_ADDR(qpid, CQHEADi)];
  while (cq_ci_db != (sq_pi_db + 1)) {
    air_printf("Polling on on CQHEADi to be 0x%x. Read: 0x%x\r\n", sq_pi_db + 1,
               cq_ci_db);
    cq_ci_db = ernic_csr[ERNIC_QP_ADDR(qpid, CQHEADi)];
  }
}
#endif

void handle_packet_read_write_aie_reg32(hsa_agent_dispatch_packet_t *pkt,
                                        bool is_write) {

  packet_set_active(pkt, true);
  uint64_t address = pkt->arg[0];
  uint32_t value = pkt->arg[1] & 0xFFFFFFFF;

  if (address > AIE_CSR_SIZE) {
    printf("[ERROR] read32/write32 packets provided address of size 0x%lx. "
           "Window is only 4GB\n",
           address);
  }

  if (is_write) {
    out32(AIE_BASE + address, value);
  } else {
    pkt->arg[2] = in32(AIE_BASE + address);
  }
  packet_set_active(pkt, false);
  --pkt->completion_signal.handle;
}

/*
 Load an AIRBIN from DRAM into tiles

 The AIRBIN is loaded into device memory (usually from the host)
 arg[0]: a table specifying an offset and length of each section. The parameter
 is a user virtual address.  The offsets in the table are relative to the start
 of the table.

 A CDMA descriptor chain is created to load these sections into tile
 memory. This is necessary because the tile memory is not directly accessible
 by the host.
*/
void handle_packet_load_airbin(hsa_agent_dispatch_packet_t *pkt) {
  uint32_t idx = 0;
  uint64_t src, dest, tile;
  uint16_t col, row, starting_col;
  uint16_t start_col = 0xFFFF;
  uint16_t start_row = 0xFFFF;
  uint16_t end_col = 0;
  uint16_t end_row = 0;
  uint64_t table_va = pkt->arg[0];
  uint16_t target_col = pkt->arg[1] & 0xFFFF;
  uint64_t entry_iter = 0;
  airbin_table_entry *entry =
      (airbin_table_entry *)translate_virt_to_phys(table_va);

  air_printf("Loading AIRBIN to col %u from 0x%lx\r\n", target_col,
             (uint64_t)entry);
  packet_set_active(pkt, true);

  // reset our shim DMA
  xaie_shim_dma_init(target_col);

  cdma_sg_init();

  // parse airbin table and create CDMA descriptor chain to load the data
  while (entry->size) {
    col = GET_COLUMN(entry->addr);

    // Marking the first column
    if (entry_iter == 0) {
      starting_col = col;
      entry_iter++;
    }

    // Shifting the design over to the target column + offset
    col = col - starting_col + target_col;

    row = GET_ROW(entry->addr);
    tile = getTileAddr(col, row);
    src = translate_virt_to_phys(table_va + entry->offset);
    dest = tile | (entry->addr & 0xFFFFFF);
    air_printf("Entry: src=0x%lx dest=0x%lx size=%x\r\n", src, dest,
               entry->size);
    cdma_sg_set(idx++, dest, src, entry->size);

    if (col < start_col)
      start_col = col;
    if (row < start_row)
      start_row = row;
    if (col > end_col)
      end_col = col;
    if (row > end_row)
      end_row = row;

    entry++;
  }

  // roll index back so it refers to the last valid descriptor
  idx--;

  // reset shim tile
  xaie_reset_shim(start_col);

  // put AIE cores in reset
  for (uint16_t c = start_col; c <= end_col; c++) {
    for (uint16_t r = start_row; r <= end_row; r++) {
      // row 0 is reserved for addressing components within the shim
      if (r == 0)
        continue;
      air_printf("Putting core (%u, %u) in reset\r\n", c, r);
      aie_tile_reset(c, r);
    }

    // reset the column
    aie_reset_column(c);
  }

  // copy
  uint32_t ret = cdma_sg_start_sync(0, idx);
  if (ret) {
    printf("Error 0x%x in CDMA\r\n", ret);
    return;
  }
  air_printf("DMA done\r\n");

  // start the AIE cores
  aie_start_cores(start_col, end_col, start_row, end_row);

  air_printf("AIE started\r\n");
}

void handle_packet_sg_cdma(hsa_agent_dispatch_packet_t *pkt) {
  // packet is in active phase
  packet_set_active(pkt, true);
  volatile uint32_t *cdmab = (volatile uint32_t *)(CDMA_BASE);
  uint32_t start_row = (pkt->arg[3] >> 0) & 0xff;
  uint32_t num_rows = (pkt->arg[3] >> 8) & 0xff;
  uint32_t start_col = (pkt->arg[3] >> 16) & 0xff;
  uint32_t num_cols = (pkt->arg[3] >> 24) & 0xff;
  for (uint c = start_col; c < start_col + num_cols; c++) {
    for (uint r = start_row; r < start_row + num_rows; r++) {
      out32(getTileAddr(c, r) + 0x00032000, 0x2);
      air_printf("Done resetting col %d row %d.\n\r", c, r);
    }
    air_printf("Resetting column %d.\n\r", c);
    aie_reset_column(c);
  }
  air_printf("CDMA reset.\n\r");
  cdmab[0] |= 0x4;
  cdmab[0] &= 0x4;
  while (cdmab[0] & 0x4)
    ;
  air_printf("CDMA start.\n\r");
  uint64_t daddr = (pkt->arg[0]);
  uint64_t saddr = (pkt->arg[1]);
  uint32_t bytes = (pkt->arg[2]);
  air_printf("CMDA daddr 0x%016lx saddr 0x%016lx\n\r", daddr, saddr);
  cdmab[0] = 0x0;          // unset SG mode
  if (bytes >= 0xffffff) { // SG
    cdmab[0] = 0x8;        // set SG mode
    cdmab[2] = saddr & 0xffffffff;
    cdmab[3] = saddr >> 32;
    cdmab[5] = daddr >> 32;
    cdmab[4] = daddr & 0xffffffff;
  } else {
    cdmab[6] = saddr & 0xffffffff;
    cdmab[7] = saddr >> 32;
    cdmab[8] = daddr & 0xffffffff;
    cdmab[9] = daddr >> 32;
    cdmab[10] = bytes;
  }
  int cnt = 100;
  while (!(cdmab[1] & 2) && cnt--)
    air_printf("SG CDMA wait... %x\n\r", cdmab[1]);
  for (uint c = start_col; c < start_col + num_cols; c++) {
    for (uint r = start_row; r <= start_row + num_rows; r++) {
      wait_for_locks_sync(c, r);
      aie_tile_enable(c, r);
    }
  }
  air_printf("CDMA done!\n\r");
}

void handle_packet_cdma(hsa_agent_dispatch_packet_t *pkt) {
  // packet is in active phase
  packet_set_active(pkt, true);
  uint32_t start_row = (pkt->arg[3] >> 0) & 0xff;
  uint32_t num_rows = (pkt->arg[3] >> 8) & 0xff;
  uint32_t start_col = (pkt->arg[3] >> 16) & 0xff;
  uint32_t num_cols = (pkt->arg[3] >> 24) & 0xff;
  uint32_t op = (pkt->arg[3] >> 32) & 0xff;
  if (op == 2) {
    for (uint16_t c = start_col; c < start_col + num_cols; c++) {
      for (uint16_t r = start_row; r < start_row + num_rows; r++) {
        uint32_t st = aie_get_core_status(c, r);
        air_printf("Status col %d row %d. 0x%x\n\r", c, r, st & 0x3);
        if ((0x3 & st) != 0x2) {
          aie_tile_reset(c, r);
          air_printf("Done resetting col %d row %d.\n\r", c, r);
        }
      }
    }
  }
  if (op == 1) {
    for (uint8_t c = start_col; c < start_col + num_cols; c++) {
      air_printf("Resetting column %u.\n\r", c);
      aie_reset_column(c);
      air_printf("Done resetting column %u.\n\r", c);
    }
  }
  volatile uint32_t *cdmab = (volatile uint32_t *)(CDMA_BASE);
  uint32_t status = cdmab[1];
  air_printf("CMDA raw %x idle %x\n\r", status, status & 2);
  uint64_t daddr = (pkt->arg[0]);
  uint64_t saddr = (pkt->arg[1]);
  uint32_t bytes = (pkt->arg[2]);
  air_printf("CMDA dst %lx src %lx\n\r", daddr, saddr);
  cdmab[0] = 0x0; // unset SG mode
  cdmab[6] = saddr & 0xffffffff;
  cdmab[7] = saddr >> 32;
  cdmab[8] = daddr & 0xffffffff;
  cdmab[9] = daddr >> 32;
  cdmab[10] = bytes;
  while (!(cdmab[1] & 2))
    air_printf("CMDA wait...\n\r");
  if (op == 2) {
    for (uint c = start_col; c < start_col + num_cols; c++) {
      for (uint r = start_row; r <= start_row + num_rows; r++) {
        wait_for_locks_sync(c, r);
        aie_tile_enable(c, r);
      }
    }
  }
}

void handle_packet_xaie_lock(hsa_agent_dispatch_packet_t *pkt) {
  // packet is in active phase
  packet_set_active(pkt, true);

  uint32_t num_cols =
      (((pkt->arg[0] >> 48) & 0xf) == AIR_ADDRESS_HERD_RELATIVE_RANGE)
          ? ((pkt->arg[0] >> 40) & 0xff)
          : 1;
  uint32_t num_rows =
      (((pkt->arg[0] >> 48) & 0xf) == AIR_ADDRESS_HERD_RELATIVE_RANGE)
          ? ((pkt->arg[0] >> 24) & 0xff)
          : 1;
  uint32_t start_col = (pkt->arg[0] >> 32) & 0xff;
  uint32_t start_row = (pkt->arg[0] >> 16) & 0xff;
  uint32_t lock_id = pkt->arg[1];
  uint32_t acqrel = pkt->arg[2];
  uint32_t val = pkt->arg[3];
  for (uint32_t col = 0; col < num_cols; col++) {
    for (uint32_t row = 0; row < num_rows; row++) {
      if (acqrel == 0)
        xaie_lock_acquire_nb(xaie_herd_get_col_start() + start_col + col,
                             xaie_herd_get_row_start() + start_row + row,
                             lock_id, val);
      else
        xaie_lock_release(xaie_herd_get_col_start() + start_col + col,
                          xaie_herd_get_row_start() + start_row + row, lock_id,
                          val);
    }
  }
}

#ifdef ARM_CONTROLLER
void handle_packet_xaie_status(hsa_agent_dispatch_packet_t *pkt,
                               uint32_t type) {
  printf("Reading status! %d %lu %lu\r\n", type, pkt->arg[0], pkt->arg[1]);
  if (type == 1) {
    mlir_aie_print_shimdma_status(pkt->arg[0]);
  } else if (type == 2) {
    mlir_aie_print_dma_status(pkt->arg[0], pkt->arg[1]);
  } else if (type == 3) {
    mlir_aie_print_tile_status(pkt->arg[0], pkt->arg[1]);
  }
}
#endif

void handle_packet_hello(hsa_agent_dispatch_packet_t *pkt, uint32_t mb_id) {
  packet_set_active(pkt, true);

  uint64_t say_what = pkt->arg[0];
  lock_uart(mb_id);
  printf("MB %d : HELLO %08X\n\r", mb_id, (uint32_t)say_what);
  unlock_uart(mb_id);
}

typedef struct staged_nd_memcpy_s {
  uint32_t valid;
  hsa_agent_dispatch_packet_t *pkt;
  uint64_t paddr[3];
  uint32_t index[3];
} staged_nd_memcpy_t; // about 48B therefore @ 64 slots ~3kB

// GLOBAL storage for 'in progress' ND memcpy work
// NOTE 4 slots per shim DMA
staged_nd_memcpy_t staged_nd_slot[NUM_DMAS * 4];

void nd_dma_put_checkpoint(hsa_agent_dispatch_packet_t **pkt, uint32_t slot,
                           uint32_t idx_4d, uint32_t idx_3d, uint32_t idx_2d,
                           uint64_t pad_3d, uint64_t pad_2d, uint64_t pad_1d) {
  staged_nd_slot[slot].pkt = *pkt;
  staged_nd_slot[slot].paddr[0] = pad_1d;
  staged_nd_slot[slot].paddr[1] = pad_2d;
  staged_nd_slot[slot].paddr[2] = pad_3d;
  staged_nd_slot[slot].index[0] = idx_2d;
  staged_nd_slot[slot].index[1] = idx_3d;
  staged_nd_slot[slot].index[2] = idx_4d;
}

void nd_dma_get_checkpoint(hsa_agent_dispatch_packet_t **pkt, uint32_t slot,
                           uint32_t &idx_4d, uint32_t &idx_3d, uint32_t &idx_2d,
                           uint64_t &pad_3d, uint64_t &pad_2d,
                           uint64_t &pad_1d) {
  *pkt = staged_nd_slot[slot].pkt;
  pad_1d = staged_nd_slot[slot].paddr[0];
  pad_2d = staged_nd_slot[slot].paddr[1];
  pad_3d = staged_nd_slot[slot].paddr[2];
  idx_2d = staged_nd_slot[slot].index[0];
  idx_3d = staged_nd_slot[slot].index[1];
  idx_4d = staged_nd_slot[slot].index[2];
}

int do_packet_nd_memcpy(uint32_t slot) {
  hsa_agent_dispatch_packet_t *a_pkt;
  uint64_t paddr_3d;
  uint64_t paddr_2d;
  uint64_t paddr_1d;
  uint32_t index_4d;
  uint32_t index_3d;
  uint32_t index_2d;
  nd_dma_get_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d, paddr_3d,
                        paddr_2d, paddr_1d);

  uint16_t channel = (a_pkt->arg[0] >> 24) & 0x00ff;
  uint16_t col = (a_pkt->arg[0] >> 32) & 0x00ff;
  uint16_t direction = (a_pkt->arg[0] >> 60) & 0x000f;
  uint32_t length_1d = (a_pkt->arg[2] >> 0) & 0xffffffff;
  uint32_t length_2d = (a_pkt->arg[2] >> 32) & 0x0000ffff;
  uint32_t stride_2d = (a_pkt->arg[2] >> 48) & 0x0000ffff;
  uint32_t length_3d = (a_pkt->arg[3] >> 0) & 0x0000ffff;
  uint32_t stride_3d = (a_pkt->arg[3] >> 16) & 0x0000ffff;
  uint32_t length_4d = (a_pkt->arg[3] >> 32) & 0x0000ffff;
  uint32_t stride_4d = (a_pkt->arg[3] >> 48) & 0x0000ffff;
  uint32_t outstanding = 0;

  air_printf(
      "%s: col=%u dir=%u chan=%u paddr=0x%lx 4d stride=%u length=%u\r\n",
      __func__, col, direction, channel, paddr_1d, stride_4d, length_4d);
  air_printf(
      "  3d stride=%u length=%u, 2d stride=%u length=%u, 1d length=%u\r\n",
      stride_3d, length_3d, stride_2d, length_2d, length_1d);

  for (; index_4d < length_4d; index_4d++) {
    for (; index_3d < length_3d; index_3d++) {
      for (; index_2d < length_2d; index_2d++) {
        outstanding = xaie_shim_dma_get_outstanding(getTileAddr(col, 0),
                                                    direction, channel);
        air_printf("\n\rND start shim DMA %u %u [%u][%u][%u] paddr=0x%lx\r\n",
                   direction, channel, index_4d, index_3d, index_2d, paddr_1d);
        if (outstanding >= 4) { // NOTE What is proper 'stalled' threshold?
          nd_dma_put_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d,
                                paddr_3d, paddr_2d, paddr_1d);
          return 1;
        } else {
          xaie_shim_dma_push_bd(getTileAddr(col, 0), direction, channel, col,
                                paddr_1d, length_1d);
        }
        paddr_1d += stride_2d;
      }
      index_2d = 0;
      paddr_2d += stride_3d;
      if (index_3d + 1 < length_3d)
        paddr_1d = paddr_2d;
      else
        paddr_1d = paddr_3d + stride_4d;
    }
    index_3d = 0;
    paddr_3d += stride_4d;
    paddr_2d = paddr_3d;
  }

  // Wait check idle
  int wait_idle_ret =
      xaie_shim_dma_wait_idle(getTileAddr(col, 0), direction, channel);

  // If return 1 we timed out, BDs waiting on other BDs. Put checkpoint and
  // return 1
  if (wait_idle_ret) {
    nd_dma_put_checkpoint(&a_pkt, slot, index_4d, index_3d, index_2d, paddr_3d,
                          paddr_2d, paddr_1d);
  }

  return wait_idle_ret;
}

int do_packet_memcpy(uint32_t slot) {
  if (slot >= NUM_SHIM_DMAS * 4) {
    return 0;
  } else {
    return do_packet_nd_memcpy(slot);
  }
}

int stage_packet_nd_memcpy(hsa_agent_dispatch_packet_t *pkt, uint32_t slot,
                           uint32_t memory_space) {
  air_printf("stage_packet_nd_memcpy %d\n\r", slot);
  if (staged_nd_slot[slot].valid) {
    air_printf("STALL: ND Memcpy Slot %d Busy!\n\r", slot);
    return 2;
  }
  packet_set_active(pkt, true);

  uint64_t paddr = translate_virt_to_phys(pkt->arg[1]);
  air_printf("ND_MEMCPY: Got physical address 0x%lx\r\n", paddr);

  if (memory_space == 2) {
    nd_dma_put_checkpoint(&pkt, slot, 0, 0, 0, paddr, paddr, paddr);
    staged_nd_slot[slot].valid = 1;
    return 0;
  } else {
    air_printf("NOT SUPPORTED: Cannot program memory space %d DMAs\n\r",
               memory_space);
    return 1;
  }
}

void handle_agent_dispatch_packet(amd_queue_t *amd_queue, uint32_t mb_id,
                                  int queue_id) {
  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_agent_dispatch_packet_t *pkt_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t *>(
          hsa_csr->queue_bufs[queue_id]));
  hsa_agent_dispatch_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  int last_slot = 0;
  int max_slot = 4 * NUM_DMAS - 1;

  int num_active_packets = 1;
  int packets_processed = 0;

  do {
    // Looped back because ND memcpy failed to finish on the first try.
    // No other packet type will not finish on first try.
    if (num_active_packets > 1) {
      // NOTE assume we are coming from a stall, that's why we RR.

      // INFO:
      // 1)  check for valid staged packets that aren't the previous
      // 2a)  FOUND process packet here
      // 2b) !FOUND get next packet && check invalid
      // 3b) goto packet_op
      int slot = last_slot;
      bool stalled = true;
      bool active = false;
      do {
        slot = (slot == max_slot) ? 0 : slot + 1; // TODO better heuristic
        if (slot == last_slot)
          break;
        air_printf("RR check slot: %d\n\r", slot);
        if (staged_nd_slot[slot].valid) {
          hsa_agent_dispatch_packet_t *a_pkt = staged_nd_slot[slot].pkt;
          uint16_t channel = (a_pkt->arg[0] >> 24) & 0x00ff;
          uint16_t col = (a_pkt->arg[0] >> 32) & 0x00ff;
          // uint16_t logical_col  = (a_pkt->arg[0] >> 32) & 0x00ff;
          uint16_t direction = (a_pkt->arg[0] >> 60) & 0x000f;
          // uint16_t col          = mappedShimDMA[logical_col];
          stalled = (xaie_shim_dma_get_outstanding(getTileAddr(col, 0),
                                                   direction, channel) >= 4);
          active = packet_get_active(a_pkt);
        } else {
          stalled = true;
          active = false;
        }
        air_printf("RR slot: %d - valid %d stalled %d active %d\n\r", slot,
                   staged_nd_slot[slot].valid, stalled, active);
      } while (!staged_nd_slot[slot].valid || stalled || !active);

      if (slot == last_slot) { // Begin get next packet
        local_read_index++;
        pkt = &pkt_buf[local_read_index % amd_queue->hsa_queue.size];
        air_printf("HELLO NEW PACKET IN FLIGHT!\n\r");
        if (((pkt->header) & 0xff) != HSA_PACKET_TYPE_AGENT_DISPATCH) {
          local_read_index--;
          pkt = &pkt_buf[local_read_index % amd_queue->hsa_queue.size];
          air_printf("WARN: Found invalid HSA packet inside peek loop!\n\r");
          // TRICKY weird state where we didn't find a new packet but RR won't
          // let us retry. So advance last_slot.
          last_slot =
              (slot == max_slot) ? 0 : slot + 1; // TODO better heuristic
          continue;
        } else
          goto packet_op;
      } // End get next packet

      // FOUND ND packet process here
      last_slot = slot;
      int ret = do_packet_memcpy(slot);
      if (ret)
        continue;
      else {
        num_active_packets--;
        staged_nd_slot[slot].valid = 0;
        complete_agent_dispatch_packet(staged_nd_slot[slot].pkt);
        packets_processed++;
        continue;
      }
    }

  packet_op:
    auto op = pkt->type & 0xffff;
    // air_printf("Op is %04X\n\r",op);
    switch (op) {
    case AIR_PKT_TYPE_INVALID:
    default:
      air_printf("WARN: invalid air pkt type\n\r");
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_DEVICE_INITIALIZE:
      handle_packet_device_initialize(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_SEGMENT_INITIALIZE:
      handle_packet_segment_initialize(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_CONFIGURE:
      handle_packet_sg_cdma(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

#ifdef ARM_CONTROLLER
    case AIR_PKT_TYPE_POST_RDMA_WQE:
      handle_packet_rdma_post_wqe(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
    case AIR_PKT_TYPE_READ_AIE_REG32:
      handle_packet_read_write_aie_reg32(pkt, false);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_WRITE_AIE_REG32:
      handle_packet_read_write_aie_reg32(pkt, true);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_POST_RDMA_RECV:
      handle_packet_rdma_post_recv(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
#endif

    case AIR_PKT_TYPE_AIRBIN:
      // hard-coded column number for now
      handle_packet_load_airbin(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_HELLO:
      handle_packet_hello(pkt, mb_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_GET_CAPABILITIES:
      handle_packet_get_capabilities(pkt, mb_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_GET_INFO:
      handle_packet_get_info(pkt, mb_id);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;
    case AIR_PKT_TYPE_XAIE_LOCK:
      handle_packet_xaie_lock(pkt);
      complete_agent_dispatch_packet(pkt);
      packets_processed++;
      break;

    case AIR_PKT_TYPE_ND_MEMCPY: // Only arrive here the first try.
      uint16_t memory_space = (pkt->arg[0] >> 16) & 0x00ff;
      uint16_t channel = (pkt->arg[0] >> 24) & 0x00ff;
      uint16_t direction = (pkt->arg[0] >> 60) & 0x000f;
      uint16_t col = (pkt->arg[0] >> 32) & 0x00ff;
      uint32_t slot = channel + get_slot(col, memory_space);
      if (direction == SHIM_DMA_S2MM)
        slot += XAIEDMA_SHIM_CHNUM_S2MM0;
      else
        slot += XAIEDMA_SHIM_CHNUM_MM2S0;
      int ret = stage_packet_nd_memcpy(pkt, slot, memory_space);
      if (ret == 0) {
        last_slot = slot;
        if (do_packet_memcpy(slot)) {
          num_active_packets++;
          break;
        } // else completed the packet in the first try
      } else if (ret == 2)
        break; // slot busy, retry.
      staged_nd_slot[slot].valid = 0;
      complete_agent_dispatch_packet(
          pkt); // this is correct for the first try or invalid stage
      packets_processed++;
      break;

    } // switch
  } while (num_active_packets > 1);
  lock_uart(mb_id);
  air_printf("Completing: %d packets processed.\n\r", packets_processed);
  unlock_uart(mb_id);
  *rd_id += packets_processed;
}

void handle_barrier_and_packet(amd_queue_t *amd_queue, uint32_t mb_id,
                               int queue_id) {

  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_barrier_and_packet_t *pkt_buf(
      reinterpret_cast<hsa_barrier_and_packet_t *>(
          hsa_csr->queue_bufs[queue_id]));
  hsa_barrier_and_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  // TODO complete functionality with VAs
  hsa_signal_t s0 = pkt->dep_signal[0];
  hsa_signal_t s1 = pkt->dep_signal[1];
  hsa_signal_t s2 = pkt->dep_signal[2];
  hsa_signal_t s3 = pkt->dep_signal[3];
  hsa_signal_t s4 = pkt->dep_signal[4];

  // lock_uart(mb_id);
  // for (int i = 0; i < 5; i++)
  //  air_printf("MB %d : dep_signal[%d] @ %p\n\r",mb_id,i,(uint64_t
  //  *)(pkt->dep_signal[i]));
  // unlock_uart(mb_id);

  while (hsa_signal_wait_scacquire(s0, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 ||
         hsa_signal_wait_scacquire(s1, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 ||
         hsa_signal_wait_scacquire(s2, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 ||
         hsa_signal_wait_scacquire(s3, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 ||
         hsa_signal_wait_scacquire(s4, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0) {
    lock_uart(mb_id);
    air_printf("MB %d : barrier AND packet completion signal timeout!\n\r",
               mb_id);
    for (int i = 0; i < 5; i++)
      air_printf("MB %d : dep_signal[%d] = %lu\r\n", mb_id, i,
                 pkt->dep_signal[i].handle);
    unlock_uart(mb_id);
  }

  complete_barrier_packet(pkt);
  *rd_id += 1;
}

void handle_barrier_or_packet(amd_queue_t *amd_queue, uint32_t mb_id,
                              int queue_id) {

  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  uint64_t local_read_index = amd_queue->read_dispatch_id;
  hsa_barrier_or_packet_t *pkt_buf(reinterpret_cast<hsa_barrier_or_packet_t *>(
      hsa_csr->queue_bufs[queue_id]));
  hsa_barrier_or_packet_t *pkt(
      &pkt_buf[local_read_index % amd_queue->hsa_queue.size]);

  // TODO complete functionality with VAs
  hsa_signal_t s0 = pkt->dep_signal[0];
  hsa_signal_t s1 = pkt->dep_signal[1];
  hsa_signal_t s2 = pkt->dep_signal[2];
  hsa_signal_t s3 = pkt->dep_signal[3];
  hsa_signal_t s4 = pkt->dep_signal[4];

  // lock_uart(mb_id);
  // for (int i = 0; i < 5; i++)
  //  air_printf("MB %d : dep_signal[%d] @ %p\n\r",mb_id,i,(uint64_t
  //  *)(pkt->dep_signal[i]));
  // unlock_uart(mb_id);

  while (hsa_signal_wait_scacquire(s0, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 &&
         hsa_signal_wait_scacquire(s1, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 &&
         hsa_signal_wait_scacquire(s2, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 &&
         hsa_signal_wait_scacquire(s3, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0 &&
         hsa_signal_wait_scacquire(s4, HSA_SIGNAL_CONDITION_EQ, 0, 0x80000,
                                   HSA_WAIT_STATE_ACTIVE) != 0) {

    lock_uart(mb_id);
    air_printf("MB %d : barrier OR packet completion signal timeout!\n\r",
               mb_id);
    for (int i = 0; i < 5; i++)
      air_printf("MB %d : dep_signal[%d] = %lu\r\n", mb_id, i,
                 pkt->dep_signal[i].handle);
    unlock_uart(mb_id);
  }

  complete_barrier_packet(pkt);
  *rd_id += 1;
}

int main() {

  // Initializing our platform
  init_platform();

  hsa_csr_init();
  hsa_csr_print();

#if defined(ARM_CONTROLLER)
  // Xil_DCacheDisable();

  int err = mlir_aie_init_device();
  if (err)
    printf("ERROR initializing device.\n\r");

  // Setting the number of agents in the system
  // int user1 = 1; // number of controllers
  int user2 = 0;

  int mb_id = user2 & 0xff;
  int maj = (user2 >> 24) & 0xff;
  int min = (user2 >> 16) & 0xff;
  int ver = (user2 >> 8) & 0xff;
#else
  pvr_t pvr;
  microblaze_get_pvr(&pvr);
  uint32_t user2 = MICROBLAZE_PVR_USER2(pvr);
  uint32_t mb_id = user2 & 0xff;
  uint32_t maj = (user2 >> 24) & 0xff;
  uint32_t min = (user2 >> 16) & 0xff;
  uint32_t ver = (user2 >> 8) & 0xff;
#endif

  lock_uart(mb_id);
#if defined(ARM_CONTROLLER)
  printf("ARM %d of %d firmware %d.%d.%d created on %s at %s GMT\n\r",
         mb_id + 1, hsa_csr->num_aql_queues, maj, min, ver, __DATE__, __TIME__);
#else
  printf("MB %d of %d firmware %d.%d.%d created on %s at %s GMT\n\r", mb_id + 1,
         hsa_csr->num_aql_queues, maj, min, ver, __DATE__, __TIME__);
#endif
  printf("(c) Copyright 2020-2022 AMD, Inc. All rights reserved.\n\r");
  unlock_uart(mb_id);

  setup = false;
  lock_uart(mb_id);
  unlock_uart(mb_id);

  bool done(false);

  int admin_queue_id(0);
  amd_queue_t *admin_queue(hsa_csr->amd_aql_queues[admin_queue_id]);
  admin_queue->hsa_queue.size = 64;
  volatile uint64_t *admin_doorbell(
      reinterpret_cast<uint64_t *>(hsa_csr->doorbells[admin_queue_id]));
  volatile uint64_t *admin_rd_id(&admin_queue->read_dispatch_id);
  volatile uint64_t *admin_wr_id(&admin_queue->write_dispatch_id);
  hsa_agent_dispatch_packet_t *admin_queue_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t *>(
          hsa_csr->queue_bufs[admin_queue_id]));
  hsa_agent_dispatch_packet_t *admin_pkt(nullptr);
  uint64_t admin_last_doorbell(std::numeric_limits<uint64_t>::max());

  int hqd_id(1);
  amd_queue_t *amd_queue(hsa_csr->amd_aql_queues[hqd_id]);
  volatile uint64_t *doorbell(
      reinterpret_cast<uint64_t *>(hsa_csr->doorbells[hqd_id]));
  volatile uint64_t *rd_id(&amd_queue->read_dispatch_id);
  volatile uint64_t *wr_id(&amd_queue->write_dispatch_id);
  hsa_agent_dispatch_packet_t *queue_buf(
      reinterpret_cast<hsa_agent_dispatch_packet_t *>(
          hsa_csr->queue_bufs[hqd_id]));
  hsa_agent_dispatch_packet_t *aql_pkt(nullptr);
  uint64_t last_doorbell(std::numeric_limits<uint64_t>::max());

  *doorbell = std::numeric_limits<uint64_t>::max();
  *rd_id = 0;
  *wr_id = 0;

  *admin_doorbell = std::numeric_limits<uint64_t>::max();
  *admin_rd_id = 0;
  *admin_wr_id = 0;

  air_printf("Starting packet processing loop\n\r");
  while (!done) {
    if (*admin_doorbell + 1 > *admin_rd_id) {
      ++admin_last_doorbell;
      admin_pkt = &admin_queue_buf[*admin_rd_id % 64];
      uint32_t type(static_cast<uint32_t>(admin_pkt->header) & 0xffU);

      switch (type) {
      case HSA_PACKET_TYPE_AGENT_DISPATCH:
        handle_agent_dispatch_packet(admin_queue, mb_id, admin_queue_id);
        break;
      default:
        air_printf("Unsupported admin queue packet type: %u\n\r", type);
        ++(*admin_rd_id);
        break;
      }
    }

    if (*doorbell + 1 > *rd_id) {
      ++last_doorbell;
      aql_pkt = &queue_buf[*rd_id % amd_queue->hsa_queue.size];
      uint32_t type(static_cast<uint32_t>(aql_pkt->header) & 0xffU);
      uint32_t func(static_cast<uint32_t>(aql_pkt->type) & 0xffffU);

      air_printf("Doorbell rung %lu\n\r", *doorbell);
      air_printf("Packet type %u, func type %u, pkt data %lx\n\r", type, func,
                 aql_pkt->arg[0]);
      air_printf("queue heap addr %lx\n\r",
                 hsa_csr->queue_dram_cpu_va[hqd_id]);

      uint32_t invalid_count = 0;
      while (type == HSA_PACKET_TYPE_INVALID) {
        aql_pkt = &queue_buf[*rd_id % amd_queue->hsa_queue.size];
        type = aql_pkt->header & 0xff;
        type = static_cast<uint32_t>(aql_pkt->header) & 0xffU;
        func = static_cast<uint32_t>(aql_pkt->type) & 0xffffU;

        // TODO: Come back to this for the multi-prodcer queue as we can hit
        // this
        invalid_count++;
        if (invalid_count > INVLD_COUNT_TIMEOUT) {
          printf("[WARNING] We are stuck in an invalid packet and timed out. "
                 "Breaking\r\n");
          printf("\theader: 0x%x\r\n", aql_pkt->header);
          printf("\ttype: 0x%x\r\n", type);
          printf("\tfunc: 0x%x\r\n", func);
          printf("\trd_id: 0x%lx\r\n", *rd_id);
          printf("\tdoorbell: 0x%lx\r\n", *doorbell);
          break;
        }
      }

      switch (type) {
      case HSA_PACKET_TYPE_AGENT_DISPATCH:
        air_printf("Dispatching agent dispatch packet\n\r");
        handle_agent_dispatch_packet(amd_queue, mb_id, hqd_id);
        break;
      case HSA_PACKET_TYPE_BARRIER_AND:
        air_printf("Executing barrier and packet\r\n");
        handle_barrier_and_packet(amd_queue, mb_id, hqd_id);
        break;
      case HSA_PACKET_TYPE_BARRIER_OR:
        air_printf("Executing barrier or packet\r\n");
        handle_barrier_or_packet(amd_queue, mb_id, hqd_id);
        break;
      // We are already handling the invalid packet above
      case HSA_PACKET_TYPE_INVALID:
        break;
      default:
        air_printf("Unsupported packet type\n\r");
        ++(*rd_id);
        break;
      }
    }
    shell();
  }

  cleanup_platform();

  return 0;
}
