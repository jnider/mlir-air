//===- aie.h -------------------------------------------------*- C++ -*-===//
//
// Copyright (C) 2020-2022, Xilinx Inc.
// Copyright (C) 2022, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//

#ifndef _AIE_H
#define _AIE_H

#include <stdint.h>

/*
 * Tile address format:
 * --------------------------------------------
 * |                7 bits  5 bits   18 bits  |
 * --------------------------------------------
 * | Array offset | Column | Row | Tile addr  |
 * --------------------------------------------
 */
#define AIE_TILE_WIDTH 18
#define AIE_ROW_WIDTH 5
#define AIE_COLUMN_WIDTH 7

#define AIE_ROW_SHIFT (AIE_TILE_WIDTH)
#define AIE_COLUMN_SHIFT (AIE_TILE_WIDTH + AIE_ROW_WIDTH)
#define AIE_ARRAY_SHIFT (AIE_TILE_WIDTH + AIE_ROW_WIDTH + AIE_COLUMN_WIDTH)
#define AIE_TILE_MASK ((1 << AIE_TILE_WIDTH) - 1)
#define AIE_ROW_MASK ((1 << AIE_ROW_WIDTH) - 1)
#define AIE_COLUMN_MASK ((1 << AIE_COLUMN_WIDTH) - 1)

#define GET_COLUMN(_addr) (((_addr) >> AIE_COLUMN_SHIFT) & AIE_COLUMN_MASK)
#define GET_ROW(_addr) (((_addr) >> AIE_ROW_SHIFT) & AIE_ROW_MASK)
#define GET_TILE(_addr) ((_addr)&AIE_TILE_MASK)

#define NUM_SHIM_DMAS 16
#define NUM_COL_DMAS 4
#define NUM_DMAS (NUM_SHIM_DMAS + NUM_COL_DMAS)

// direction
#define SHIM_DMA_S2MM 0
#define SHIM_DMA_MM2S 1

// channel numbers
#define XAIEDMA_SHIM_CHNUM_S2MM0 0U
#define XAIEDMA_SHIM_CHNUM_S2MM1 1U
#define XAIEDMA_SHIM_CHNUM_MM2S0 2U
#define XAIEDMA_SHIM_CHNUM_MM2S1 3U

struct HerdConfig {
  uint16_t row_start;
  uint16_t num_rows;
  uint16_t col_start;
  uint16_t num_cols;
};

int mlir_aie_init_device(void);
int mlir_aie_reinit_device(void);
void xaie_device_init(void);
void xaie_array_reset(void);

uint64_t getTileAddr(uint16_t ColIdx, uint16_t RowIdx);
int xaie_lock_acquire_nb(uint16_t col, uint16_t row, uint32_t lock_id, uint32_t val);
int xaie_lock_release(uint16_t col, uint16_t row, uint32_t lock_id, uint32_t val);
void wait_for_locks_sync(uint16_t col, uint16_t row);
int XAie_Finish(void);

void aie_tile_reset(int col, int row);
void aie_tile_enable(int col, int row);
void aie_start_cores(uint16_t start_col, uint16_t end_col, uint16_t start_row, uint16_t end_row);
uint32_t aie_get_core_status(uint16_t c, uint16_t r);
void aie_reset_column(uint16_t col_idx);

void xaie_shim_dma_init(uint16_t col);
int xaie_shim_dma_push_bd(uint64_t TileAddr, int direction, int channel,
                          uint16_t col, uint64_t addr, uint32_t len);
void xaie_reset_shim(uint16_t col);
uint32_t xaie_shim_dma_get_outstanding(uint64_t TileAddr, int direction,
                                       int channel);
int xaie_shim_dma_wait_idle(uint64_t TileAddr, int direction, int channel);

void mlir_aie_print_dma_status(int col, int row);
void mlir_aie_print_shimdma_status(uint16_t col);
void mlir_aie_print_tile_status(int col, int row);

uint32_t get_slot(uint16_t col, uint16_t space);

void xaie_segment_init(uint16_t start_col, uint16_t num_cols,
                       uint16_t start_row, uint16_t num_rows);
uint16_t xaie_herd_get_num_cols(void);
uint16_t xaie_herd_get_num_rows(void);
uint16_t xaie_herd_get_col_start(void);
uint16_t xaie_herd_get_row_start(void);

#endif // _AIE_H
