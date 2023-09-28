#include "aie.h"
#include "platform.h"
#include <stdint.h>
#include <stdio.h>

#ifdef DEBUG_AIE
#define PRINTF printf
#else
#define PRINTF(...)
#endif // DEBUG_CDMA

// The NPI registers we use to reset the array
constexpr auto NPI_MASK_REG = 0x0;
constexpr auto NPI_VAL_REG = 0x4;
constexpr auto NPI_LOCK_REG = 0xC;

#define SHIM_DMA_NUM_BDS 16
#define NUM_SHIM_DMA_S2MM_CHANNELS 2
#define NUM_SHIM_DMA_MM2S_CHANNELS 2

// AIE (ME) registers
#define REG_AIE_DMA_BD_ADDR_A(_idx) (0x1D000 + (0x20 * _idx))
#define REG_AIE_DMA_BD_ADDR_B(_idx) (0x1D004 + (0x20 * _idx))
#define AIE_DMA_BD_ADDR_LOCK (0xFUL << 22)
#define AIE_DMA_BD_ADDR_ENA_REL (1UL << 21)
#define AIE_DMA_BD_ADDR_REL_VAL (1UL << 20)
#define AIE_DMA_BD_ADDR_USE_REL_VAL (1UL << 19)
#define AIE_DMA_BD_ADDR_ENA_ACQ (1UL << 18)
#define AIE_DMA_BD_ADDR_ACQ_VAL (1UL << 17)
#define AIE_DMA_BD_ADDR_USE_ACQ_VAL (1UL << 16)
#define AIE_DMA_BD_ADDR_BASE (0x1FFFUL << 0)

#define REG_AIE_DMA_BD_2D_X(_idx) (0x1D008 + (0x20 * _idx))
#define REG_AIE_DMA_BD_2D_Y(_idx) (0x1D00C + (0x20 * _idx))
#define REG_AIE_DMA_BD_PKT(_idx) (0x1D010 + (0x20 * _idx))
#define AIE_DMA_BD_PKT_TYPE (0x3UL << 12)
#define AIE_DMA_BD_PKT_ID (0x1FUL << 0)

#define REG_AIE_DMA_BD_IS(_idx) (0x1D014 + (0x20 * _idx))
#define REG_AIE_DMA_BD_CTL(_idx) (0x1D018 + (0x20 * _idx))
#define AIE_DMA_BD_CTL_VALID (1UL << 31)
#define AIE_DMA_BD_CTL_ENA_AB (1UL << 30)
#define AIE_DMA_BD_CTL_ENA_FIFO (3UL << 28)
#define AIE_DMA_BD_CTL_ENA_PKT (1UL << 27)
#define AIE_DMA_BD_CTL_ENA_ILV (1UL << 26)
#define AIE_DMA_BD_CTL_ILV_CNT (0xFFUL << 18)
#define AIE_DMA_BD_CTL_USE_NEXT (1UL << 17)
#define AIE_DMA_BD_CTL_NEXT (0xFUL << 13)
#define AIE_DMA_BD_CTL_LEN (0x1FFFUL << 0)

#define REG_AIE_LOCK_RELEASE_0(_idx) (0x1E020 + (0x80 * _idx))
#define REG_AIE_CORE_CTL 0x00032000
#define REG_AIE_CORE_STATUS 0x00032004

// NoC (shim) registers
#define REG_SHIM_DMA_BD_ADDR(_idx) (0x1D000 + (0x14 * _idx))
#define REG_SHIM_DMA_BD_BUF_LEN(_idx) (0x1D004 + (0x14 * _idx))
#define REG_SHIM_DMA_BD_CTRL(_idx) (0x1D008 + (0x14 * _idx))
#define SHIM_DMA_BD_CTRL_VALID (1 << 0)

#define REG_SHIM_DMA_BD_AXI_CFG(_idx) (0x1D00C + (0x14 * _idx))
#define REG_SHIM_DMA_BD_PKT(_idx) (0x1D010 + (0x14 * _idx))
#define REG_SHIM_DMA_CTRL(_chan) (0x1D140 + (0x8 * _chan))
#define REG_SHIM_DMA_START_QUEUE(_chan) (0x1D144 + (0x8 * _chan))

#define REG_SHIM_DMA_S2MM_STATUS (0x1D160)
#define SHIM_DMA_CURR_BD_SHIFT 16
#define SHIM_DMA_CURR_BD_WIDTH 4
#define SHIM_DMA_CURR_BD_MASK ((1 << SHIM_DMA_CURR_BD_WIDTH) - 1)
#define SHIM_DMA_QUEUE_SIZE_SHIFT 6
#define SHIM_DMA_QUEUE_SIZE_WIDTH 3
#define SHIM_DMA_QUEUE_SIZE_MASK ((1 << SHIM_DMA_QUEUE_SIZE_WIDTH) - 1)
#define SHIM_DMA_STATUS_SHIFT 0
#define SHIM_DMA_STATUS_WIDTH 2
#define SHIM_DMA_STATUS_MASK ((1 << SHIM_DMA_STATUS_WIDTH) - 1)
#define SHIM_DMA_STALLED_SHIFT 4
#define SHIM_DMA_STALLED_WIDTH 1
#define SHIM_DMA_STALLED_MASK 1
#define GET_SHIM_DMA(_field, _reg, _ch)                                        \
  ((_reg) >>                                                                   \
       (SHIM_DMA_##_field##_SHIFT + (SHIM_DMA_##_field##_WIDTH * (_ch))) &     \
   SHIM_DMA_##_field##_MASK)

#define REG_SHIM_DMA_MM2S_STATUS (0x1D164)

#define REG_AIE_COL_RESET 0x00036048
#define REG_SHIM_RESET_ENA 0x0003604C

#define REG_AIE_CORE_CTL_RESET (1U << 1)
#define REG_AIE_CORE_CTL_ENABLE (1U << 0)

// number of DMA descriptors
#define AIE_NUM_BD 16

// supported geometry
#define XAIE_NUM_ROWS 8
#define XAIE_NUM_COLS 50

static uint32_t last_bd[8] = {0};
static uint8_t shim_dma_cols[NUM_SHIM_DMAS] = {2,  3,  6,  7,  10, 11, 18, 19,
                                               26, 27, 34, 35, 42, 43, 46, 47};
static uint8_t col_dma_cols[NUM_COL_DMAS] = {7, 8, 9, 10};
static HerdConfig HerdCfgInst;

static uint32_t maskpoll32(uint64_t Addr, uint32_t Mask, uint32_t Value,
                           uint32_t TimeOut) {
  uint32_t Ret = 1;

  uint32_t Count = 10 + TimeOut;

  while (Count > 0U) {
    if ((IO_READ32(Addr) & Mask) == Value) {
      Ret = 0;
      break;
    }
    Count--;
  }

  return Ret;
}

void wait_for_locks_sync(uint16_t col, uint16_t row) {
  for (int l = 0; l < 16; l++)
    maskpoll32(getTileAddr(col, row) + REG_AIE_LOCK_RELEASE_0(l), 0x1, 0x1, 0);
}

uint32_t get_slot(uint16_t col, uint16_t space) {
  if (space == 2) {
    for (uint16_t i = 0; i < NUM_SHIM_DMAS; i++) {
      if (col == shim_dma_cols[i]) {
        return i * 4;
      }
    }
  } else if (space == 1) {
    for (uint16_t i = 0; i < NUM_COL_DMAS; i++) {
      if (col == col_dma_cols[i]) {
        return i * 4 + NUM_SHIM_DMAS * 4;
      }
    }
  }
  return 0;
}

// This should be called after enabling the proper
// shims to be reset via the mask
void xaie_strobe_shim_reset() {

  // Getting a pointer to NPI
  auto *npib = (volatile uint32_t *)(NPI_BASE);

  PRINTF("Starting shim reset\r\n");

  // Unlocking NPI
  npib[NPI_LOCK_REG >> 2] = 0xF9E8D7C6;

  // Performing reset
  npib[NPI_MASK_REG >> 2] = 0x08000000;
  npib[NPI_VAL_REG >> 2] = 0x080381B1;
  npib[NPI_MASK_REG >> 2] = 0x08000000;
  npib[NPI_VAL_REG >> 2] = 0x000381B1;

  // Locking NPI
  npib[NPI_LOCK_REG >> 2] = 0x12341234;
  PRINTF("Done with shim reset\r\n");
}

int RequestTiles() {
  /*
    int RC;
          RC = _XAie_PrivilegeSetPartProtectedRegs(XAIE_ENABLE);
          RC = DevInst->DevOps->RequestTiles(Args);
          _XAie_PrivilegeSetPartProtectedRegs(XAIE_DISABLE);
  */

  return 0;
}
/*
  Calculate the address of an AIE tile
*/
uint64_t getTileAddr(uint16_t ColIdx, uint16_t RowIdx) {
#ifdef ARM_CONTROLLER
  uint64_t my_ta = (uint64_t)(AIE_BASE | (ColIdx << AIE_COLUMN_SHIFT) |
                              (RowIdx << AIE_ROW_SHIFT));
  return my_ta;
#else
  uint64_t TileAddr = 0;
  uint64_t ArrOffset = XAIE_ADDR_ARRAY_OFF;

#ifdef XAIE_BASE_ARRAY_ADDR_OFFSET
  ArrOffset = XAIE_BASE_ARRAY_ADDR_OFFSET;
#endif

  /*
   * Tile address format:
   * --------------------------------------------
   * |                7 bits  5 bits   18 bits  |
   * --------------------------------------------
   * | Array offset | Column | Row | Tile addr  |
   * --------------------------------------------
   */
  TileAddr = (uint64_t)((ArrOffset << XAIEGBL_TILE_ADDR_ARR_SHIFT) |
                        (ColIdx << XAIEGBL_TILE_ADDR_COL_SHIFT) |
                        (RowIdx << XAIEGBL_TILE_ADDR_ROW_SHIFT));

  return TileAddr;
#endif
}

int XAie_Finish(void) { return 0; }

int mlir_aie_init_device(void) {
  int RC = 0;

  RC = RequestTiles();
  if (RC != 0) {
    PRINTF("Failed to request tiles.\n\r");
    return -1;
  }

  return 0;
}

int mlir_aie_reinit_device(void) {
  int RC = 0;

  RC = RequestTiles();
  if (RC != 0) {
    PRINTF("Failed to request tiles.\n\r");
    return -1;
  }

  return 0;
}

int xaie_lock_release(uint16_t col, uint16_t row, uint32_t lock_id,
                      uint32_t val) {
  uint64_t Addr = getTileAddr(col, row);
  uint64_t LockOfst = 0x0001E020;
  if (row != 0)
    LockOfst = 0x0001E020 + 0x10 * (val & 0x1);
  else {
    switch (col % 4) {
    case 0:
    case 1:
      LockOfst = 0x00014020 + 0x10 * (val & 0x1);
      break;
    default:
      LockOfst = 0x00014020 + 0x10 * (val & 0x1);
      break;
    }
  }
  maskpoll32(Addr + LockOfst + 0x80 * lock_id, 0x1, 0x1, 0);
  return 1;
}

int xaie_lock_acquire_nb(uint16_t col, uint16_t row, uint32_t lock_id,
                         uint32_t val) {
  uint64_t Addr = getTileAddr(col, row);
  uint64_t LockOfst = 0x0001E060;
  if (row != 0)
    LockOfst = 0x0001E060 + 0x10 * (val & 0x1);
  else {
    switch (col % 4) {
    case 0:
    case 1:
      LockOfst = 0x00014060 + 0x10 * (val & 0x1);
      break;
    default:
      LockOfst = 0x00014060 + 0x10 * (val & 0x1);
      break;
    }
  }
  uint8_t lock_ret = 0;
  uint32_t loop = 0;
  while ((!lock_ret) && (loop < 512)) {
    lock_ret = maskpoll32(Addr + LockOfst + 0x80 * lock_id, 0x1, 0x1, 100);
    loop++;
  }
  if (loop == 512) {
    PRINTF("Acquire [%d, %d, %d] value %d time-out\n\r", col, row, lock_id,
           val);
    return 0;
  }
  return 1;
}

/*
  Put a tile into reset
*/
void aie_tile_reset(int col, int row) {
  uint64_t tileAddr = getTileAddr(col, row);
  IO_WRITE32(tileAddr + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_RESET);
}

/*
  Take a tile out of reset
*/
void aie_tile_enable(int col, int row) {
  uint64_t tileAddr = getTileAddr(col, row);
  IO_WRITE32(tileAddr + REG_AIE_CORE_CTL, REG_AIE_CORE_CTL_ENABLE);
}

void aie_start_cores(uint16_t start_col, uint16_t end_col, uint16_t start_row,
                     uint16_t end_row) {
  for (uint16_t c = start_col; c <= end_col; c++) {
    for (uint16_t r = start_row; r <= end_row; r++) {
      // row 0 is reserved for addressing components within the shim
      if (r == 0)
        continue;

      // reset locks
      // TODO: this can be parallelized
      for (int l = 0; l < 16; l++) {
        maskpoll32(getTileAddr(c, r) + REG_AIE_LOCK_RELEASE_0(l), 0x1, 0x1, 0);
      }

      // bring cores out of reset
      PRINTF("Enabling core (%u, %u)\r\n", c, r);
      aie_tile_enable(c, r);
    }
  }
}

uint32_t aie_get_core_status(uint16_t c, uint16_t r) {
  return IO_READ32(getTileAddr(c, r) + REG_AIE_CORE_STATUS);
}

static const char *decode_dma_state(uint32_t state) {
  switch (state) {
  case 0:
    return "idle";
  case 1:
    return "starting";
  case 2:
    return "running";
  }
  return "unknown";
}

void mlir_aie_print_dma_status(int col, int row) {
  uint64_t tileAddr = getTileAddr(col, row);

  uint32_t dma_s2mm0_control = IO_READ32(tileAddr + 0x0001DE00);
  uint32_t dma_s2mm1_control = IO_READ32(tileAddr + 0x0001DE08);
  uint32_t dma_mm2s0_control = IO_READ32(tileAddr + 0x0001DE10);
  uint32_t dma_mm2s1_control = IO_READ32(tileAddr + 0x0001DE18);
  uint32_t dma_s2mm_status = IO_READ32(tileAddr + 0x0001DF00);
  uint32_t dma_mm2s_status = IO_READ32(tileAddr + 0x0001DF10);

  uint32_t s2mm_ch0_running = dma_s2mm_status & 0x3;
  uint32_t s2mm_ch1_running = (dma_s2mm_status >> 2) & 0x3;
  uint32_t mm2s_ch0_running = dma_mm2s_status & 0x3;
  uint32_t mm2s_ch1_running = (dma_mm2s_status >> 2) & 0x3;

  printf("DMA [%d, %d] tile addr=0x%lx\r\n", col, row, tileAddr);
  printf("  mm2s (0=%s 1=%s) status=%08X ctrl0=%02X ctrl1=%02X\r\n",
         decode_dma_state(mm2s_ch0_running), decode_dma_state(mm2s_ch1_running),
         dma_mm2s_status, dma_mm2s0_control, dma_mm2s1_control);
  printf("  s2mm (0=%s 1=%s) status=%08X ctrl0=%02X ctrl1=%02X\r\n",
         decode_dma_state(s2mm_ch0_running), decode_dma_state(s2mm_ch1_running),
         dma_s2mm_status, dma_s2mm0_control, dma_s2mm1_control);

  printf("Descriptors:\r\n");
  for (uint32_t bd = 0; bd < AIE_NUM_BD; bd++) {
    uint32_t dma_bd_addr_a = IO_READ32(tileAddr + REG_AIE_DMA_BD_ADDR_A(bd));
    uint32_t dma_bd_control = IO_READ32(tileAddr + REG_AIE_DMA_BD_CTL(bd));
    if (dma_bd_control & AIE_DMA_BD_CTL_VALID) {
      printf("BD %d valid\n\r", bd);
      uint32_t current_s2mm_ch0 = (dma_s2mm_status >> 16) & 0xf;
      uint32_t current_s2mm_ch1 = (dma_s2mm_status >> 20) & 0xf;
      uint32_t current_mm2s_ch0 = (dma_mm2s_status >> 16) & 0xf;
      uint32_t current_mm2s_ch1 = (dma_mm2s_status >> 20) & 0xf;

      if (s2mm_ch0_running && bd == current_s2mm_ch0) {
        printf(" * Current BD for s2mm channel 0\n\r");
      }
      if (s2mm_ch1_running && bd == current_s2mm_ch1) {
        printf(" * Current BD for s2mm channel 1\n\r");
      }
      if (mm2s_ch0_running && bd == current_mm2s_ch0) {
        printf(" * Current BD for mm2s channel 0\n\r");
      }
      if (mm2s_ch1_running && bd == current_mm2s_ch1) {
        printf(" * Current BD for mm2s channel 1\n\r");
      }

      if (dma_bd_control & AIE_DMA_BD_CTL_ENA_PKT) {
        uint32_t dma_packet = IO_READ32(tileAddr + REG_AIE_DMA_BD_PKT(bd));
        printf("   Packet mode: %02lX\n\r", dma_packet & AIE_DMA_BD_PKT_ID);
      }
      int words_to_transfer = 1 + (dma_bd_control & AIE_DMA_BD_CTL_LEN);
      int base_address = dma_bd_addr_a & AIE_DMA_BD_ADDR_BASE;
      printf("   Transfering %d 32 bit words to/from %06X\n\r",
             words_to_transfer, base_address);

      printf("   ");
      for (int w = 0; w < 7; w++) {
        uint32_t tmpd = IO_READ32(tileAddr + (base_address << 2) + (w * 4));
        printf("%08X ", tmpd);
      }
      printf("\n\r");
      if (dma_bd_addr_a & AIE_DMA_BD_ADDR_ENA_ACQ) {
        uint32_t lock_id = (dma_bd_addr_a >> 22) & 0xf;
        printf("   Acquires lock %d ", lock_id);
        if (dma_bd_addr_a & 0x10000)
          printf("with value %d ", (dma_bd_addr_a >> 17) & 0x1);

        printf("currently ");
        uint32_t locks = IO_READ32(tileAddr + 0x0001EF00);
        uint32_t two_bits = (locks >> (lock_id * 2)) & 0x3;
        if (two_bits) {
          uint32_t acquired = two_bits & 0x1;
          uint32_t value = two_bits & 0x2;
          if (acquired)
            printf("Acquired ");
          printf(value ? "1" : "0");
        } else
          printf("0");
        printf("\n\r");
      }
      if (dma_bd_control & 0x30000000) { // FIFO MODE
        int FIFO = (dma_bd_control >> 28) & 0x3;
        uint32_t dma_fifo_counter = IO_READ32(tileAddr + 0x0001DF20);
        printf("   Using FIFO Cnt%d : %08X\n\r", FIFO, dma_fifo_counter);
      }
      uint32_t nextBd = ((dma_bd_control >> 13) & 0xF);
      uint32_t useNextBd = ((dma_bd_control >> 17) & 0x1);
      printf("   Next BD: %d %s\r\n", nextBd,
             (useNextBd == 0) ? "(unused)" : "(used)");
    }
  }
}

/*
  The shim tile is always row 0
*/
void mlir_aie_print_shimdma_status(uint16_t col) {
  uint64_t tileAddr = getTileAddr(col, 0);
  uint32_t s2mm_status = IO_READ32(tileAddr + REG_SHIM_DMA_S2MM_STATUS);
  uint32_t mm2s_status = IO_READ32(tileAddr + REG_SHIM_DMA_MM2S_STATUS);

  printf("Shim DMA [%u]\r\n", col);
  printf("S2MM\r\n");
  for (uint8_t channel = 0; channel < NUM_SHIM_DMA_S2MM_CHANNELS; channel++) {
    printf("   [channel %u] start_bd=%u queue_size=%u curr_bd=%u status=%u "
           "stalled=%u\r\n",
           channel, IO_READ32(tileAddr + REG_SHIM_DMA_START_QUEUE(channel)),
           GET_SHIM_DMA(QUEUE_SIZE, s2mm_status, channel),
           GET_SHIM_DMA(CURR_BD, s2mm_status, channel),
           GET_SHIM_DMA(STATUS, s2mm_status, channel),
           GET_SHIM_DMA(STALLED, s2mm_status, channel));
  }
  printf("MM2S\r\n");
  for (uint8_t channel = 0; channel < NUM_SHIM_DMA_MM2S_CHANNELS; channel++) {
    printf("   [channel %u] start_bd=%u queue_size=%u curr_bd=%u status=%u "
           "stalled=%u\r\n",
           channel, IO_READ32(tileAddr + REG_SHIM_DMA_START_QUEUE(channel)),
           GET_SHIM_DMA(QUEUE_SIZE, mm2s_status, channel),
           GET_SHIM_DMA(CURR_BD, mm2s_status, channel),
           GET_SHIM_DMA(STATUS, mm2s_status, channel),
           GET_SHIM_DMA(STALLED, mm2s_status, channel));
  }

  printf("Descriptors:\r\n");
  for (int bd = 0; bd < 16; bd++) {
    uint64_t bd_addr_a = IO_READ32(tileAddr + REG_SHIM_DMA_BD_ADDR(bd));
    uint32_t dma_bd_buffer_length =
        IO_READ32(tileAddr + REG_SHIM_DMA_BD_BUF_LEN(bd));
    uint32_t dma_bd_control = IO_READ32(tileAddr + REG_SHIM_DMA_BD_CTRL(bd));

    printf("[%02d] ", bd);
    if (dma_bd_control & SHIM_DMA_BD_CTRL_VALID)
      printf("valid ");

    int words_to_transfer = dma_bd_buffer_length;
    uint64_t base_address = (uint64_t)bd_addr_a +
                            ((uint64_t)((dma_bd_control >> 16) & 0xFFFF) << 32);
    printf("   Transferring %d 32 bit words to/from %08lX\n\r",
           words_to_transfer, base_address);

    int use_next_bd = ((dma_bd_control >> 15) & 0x1);
    int next_bd = ((dma_bd_control >> 11) & 0xF);
    int lockID = ((dma_bd_control >> 7) & 0xF);
    int enable_lock_release = ((dma_bd_control >> 6) & 0x1);
    int lock_release_val = ((dma_bd_control >> 5) & 0x1);
    int use_release_val = ((dma_bd_control >> 4) & 0x1);
    int enable_lock_acquire = ((dma_bd_control >> 3) & 0x1);
    int lock_acquire_val = ((dma_bd_control >> 2) & 0x1);
    int use_acquire_val = ((dma_bd_control >> 1) & 0x1);

    printf("next=%d, use_next=%d ", next_bd, use_next_bd);
    printf("lock: %d, acq(en: %d, val: %d, use: %d), rel(en: %d, val: %d, "
           "use: %d)\r\n",
           lockID, enable_lock_acquire, lock_acquire_val, use_acquire_val,
           enable_lock_release, lock_release_val, use_release_val);
  }
}

/// Print the status of a core represented by the given tile, at the given
/// coordinates.
void mlir_aie_print_tile_status(int col, int row) {
  uint32_t trace_status;
  uint32_t status, coreTimerLow, PC, LR, SP, locks, R0, R4;
  uint64_t tileAddr = getTileAddr(col, row);

  status = IO_READ32(tileAddr + REG_AIE_CORE_STATUS);
  coreTimerLow = IO_READ32(tileAddr + 0x0340F8);
  PC = IO_READ32(tileAddr + 0x00030280);
  LR = IO_READ32(tileAddr + 0x000302B0);
  SP = IO_READ32(tileAddr + 0x000302A0);
  locks = IO_READ32(tileAddr + 0x0001EF00);
  trace_status = IO_READ32(tileAddr + 0x000140D8);
  R0 = IO_READ32(tileAddr + 0x00030000);
  R4 = IO_READ32(tileAddr + 0x00030040);

  printf("Core [%d, %d] addr is 0x%08lX\n\r", col, row, tileAddr);
  printf("Core [%d, %d] status is 0x%08X, timer is %u, PC is 0x%08X, locks are "
         "%08X, LR is %08X, SP is %08X, R0 is %08X,R4 is %08X\n\r",
         col, row, status, coreTimerLow, PC, locks, LR, SP, R0, R4);
  printf("Core [%d, %d] trace status is %08X\n\r", col, row, trace_status);

  for (int lock = 0; lock < 16; lock++) {
    uint32_t two_bits = (locks >> (lock * 2)) & 0x3;
    if (two_bits) {
      printf("Lock %d: ", lock);
      uint32_t acquired = two_bits & 0x1;
      uint32_t value = two_bits & 0x2;
      if (acquired)
        printf("Acquired ");
      printf(value ? "1" : "0");
      printf("\n\r");
    }
  }

  const char *core_status_strings[] = {"Enabled",
                                       "In Reset",
                                       "Memory Stall S",
                                       "Memory Stall W",
                                       "Memory Stall N",
                                       "Memory Stall E",
                                       "Lock Stall S",
                                       "Lock Stall W",
                                       "Lock Stall N",
                                       "Lock Stall E",
                                       "Stream Stall S",
                                       "Stream Stall W",
                                       "Stream Stall N",
                                       "Stream Stall E",
                                       "Cascade Stall Master",
                                       "Cascade Stall Slave",
                                       "Debug Halt",
                                       "ECC Error",
                                       "ECC Scrubbing",
                                       "Error Halt",
                                       "Core Done"};
  printf("Core Status: ");
  for (int i = 0; i <= 20; i++) {
    if ((status >> i) & 0x1)
      printf("%s ", core_status_strings[i]);
  }
  printf("\r\n");
}

int xaie_shim_dma_push_bd(uint64_t TileAddr, int direction, int channel,
                          uint16_t col, uint64_t addr, uint32_t len) {
  uint32_t shimDMAchannel = channel; // Need
  uint32_t status_register_offset;
  uint32_t status_mask_shift;
  uint32_t control_register_offset;
  uint32_t start_queue_register_offset;
  uint32_t start_queue_size_mask_shift;

  if (direction == SHIM_DMA_S2MM) {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_S2MM0;
    status_register_offset = 0x1d160;
    if (channel == 0) {
      status_mask_shift = 0;
      control_register_offset = 0x1d140;
      start_queue_register_offset = 0x1d144;
      start_queue_size_mask_shift = 6;
    } else {
      status_mask_shift = 2;
      control_register_offset = 0x1d148;
      start_queue_register_offset = 0x1d14c;
      start_queue_size_mask_shift = 9;
    }
    PRINTF("\n\r  S2MM Shim DMA %d start channel %d\n\r", col, shimDMAchannel);
    // PRINTF("\n\r  S2MM Shim DMA %d start channel %d\n\r",
    // mappedShimDMA[dma], shimDMAchannel);
  } else {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_MM2S0;
    status_register_offset = 0x1d164;
    if (channel == 0) {
      status_mask_shift = 0;
      control_register_offset = 0x1d150;
      start_queue_register_offset = 0x1d154;
      start_queue_size_mask_shift = 6;
    } else {
      status_mask_shift = 2;
      control_register_offset = 0x1d158;
      start_queue_register_offset = 0x1d15c;
      start_queue_size_mask_shift = 9;
    }
    PRINTF("\n\r  MM2S Shim DMA %d start channel %d\n\r", col, shimDMAchannel);
    // PRINTF("\n\r  MM2S Shim DMA %d start channel %d\n\r",
    // mappedShimDMA[dma], shimDMAchannel);
  }

  uint32_t start_bd = 4 * shimDMAchannel; // shimDMAchannel<<2;
  uint32_t outstanding = (IO_READ32(TileAddr + status_register_offset) >>
                          start_queue_size_mask_shift) &
                         0b111;
  // If outstanding >=4, we're in trouble!!!!
  // Theoretically this should never occur due to check in do_packet_nd_memcpy
  if (outstanding >= 4) { // NOTE had this at 3? // What is proper 'stalled'
                          // threshold? if (outstanding >=4)
    PRINTF("\n\r *** BD OVERFLOW in shimDMA channel %d *** \n\r",
           shimDMAchannel);
    bool waiting = true;
    while (waiting) {
      outstanding = (IO_READ32(TileAddr + status_register_offset) >>
                     start_queue_size_mask_shift) &
                    0b111;
      waiting = (outstanding > 3); // NOTE maybe >= 3
      PRINTF("*** Stalled in shimDMA channel %d outstanding = %d *** \n\r",
             shimDMAchannel, outstanding + 1);
    } // WARNING this can lead to an endless loop
  }
  PRINTF("Outstanding pre : %d\n\r", outstanding);
  // uint32_t bd = start_bd+outstanding;// + 0; // HACK
  int slot = channel;
  slot += ((col % 2) == 1) ? 4 : 0;
  if (direction == SHIM_DMA_S2MM)
    slot += XAIEDMA_SHIM_CHNUM_S2MM0;
  else
    slot += XAIEDMA_SHIM_CHNUM_MM2S0;
  uint32_t bd = start_bd + last_bd[slot];
  last_bd[slot] = (last_bd[slot] == 3) ? 0 : last_bd[slot] + 1;

  // Mark the BD as invalid
  IO_WRITE32(TileAddr + REG_SHIM_DMA_BD_CTRL(bd), 0);

  // Set the registers directly ...
  IO_WRITE32(TileAddr + REG_SHIM_DMA_BD_ADDR(bd), LOW_ADDR(addr));

  // change length in bytes to 32 bit words
  IO_WRITE32(TileAddr + REG_SHIM_DMA_BD_BUF_LEN(bd), len >> 2);

  uint32_t control = (HIGH_ADDR(addr) << 16) | SHIM_DMA_BD_CTRL_VALID;
  IO_WRITE32(TileAddr + REG_SHIM_DMA_BD_CTRL(bd), control);
  IO_WRITE32(TileAddr + REG_SHIM_DMA_BD_AXI_CFG(bd),
             0x410); // Burst len [10:9] = 2 (16)
                     // QoS [8:5] = 0 (best effort)
                     // Secure bit [4] = 1 (set)

  IO_WRITE32(TileAddr + REG_SHIM_DMA_BD_PKT(bd), 0);

  // Check if the channel is running or not
  uint32_t precheck_status =
      (IO_READ32(TileAddr + status_register_offset) >> status_mask_shift) &
      0b11;
  if (precheck_status == 0b00) {
    // Stream traffic can run, we can issue AXI-MM, and the channel is enabled
    PRINTF("Enabling shim DMA [%u] channel %u\r\n", col, channel);
    IO_WRITE32(TileAddr + control_register_offset, 0x1);
  }

  // Now push into the queue
  PRINTF("Pushing bd %u into 0x%lx\r\n", bd,
         TileAddr + start_queue_register_offset);
  IO_WRITE32(TileAddr + start_queue_register_offset, bd);

#if CHATTY
  outstanding = (IO_READ32(TileAddr + status_register_offset) >>
                 start_queue_size_mask_shift) &
                0b111;
  PRINTF("Outstanding post: %d\n\r", outstanding);
  PRINTF("bd pushed as bd %u\r\n", bd);

  if (direction == SHIM_DMA_S2MM) {
    PRINTF("  End of S2MM Shim DMA %d start channel %d\n\r", col,
           shimDMAchannel);
  } else {
    PRINTF("  End of MM2S Shim DMA %d start channel %d\n\r", col,
           shimDMAchannel);
  }
#endif

  return 1;
}

int xaie_shim_dma_wait_idle(uint64_t TileAddr, int direction, int channel) {
  uint32_t shimDMAchannel = channel;
  uint32_t status_register_offset;
  uint32_t status_mask_shift;
  if (channel == 0) {
    status_mask_shift = 0;
  } else {
    status_mask_shift = 2;
  }
  if (direction == SHIM_DMA_S2MM) {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_S2MM0;
    status_register_offset = 0x1d160;
  } else {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_MM2S0;
    status_register_offset = 0x1d164;
  }

  // Will timeout if shim is busy
  uint32_t timeout_count = 0;
  uint32_t timeout_val = 100;
  while ((IO_READ32(TileAddr + status_register_offset) >> status_mask_shift) &
         0b11) {
    if (timeout_count >= timeout_val) {
      PRINTF("[WARNING] xaie_shim_dma_wait_idle timed out\r\n");
      return 1;
    }
    timeout_count++;
  }

  return 0;
}

uint32_t xaie_shim_dma_get_outstanding(uint64_t TileAddr, int direction,
                                       int channel) {
  uint32_t shimDMAchannel = channel;
  uint32_t status_register_offset;
  uint32_t start_queue_size_mask_shift;
  if (channel == 0) {
    start_queue_size_mask_shift = 6;
  } else {
    start_queue_size_mask_shift = 9;
  }
  if (direction == SHIM_DMA_S2MM) {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_S2MM0;
    status_register_offset = 0x1d160;
  } else {
    shimDMAchannel += XAIEDMA_SHIM_CHNUM_MM2S0;
    status_register_offset = 0x1d164;
  }
  uint32_t outstanding = (IO_READ32(TileAddr + status_register_offset) >>
                          start_queue_size_mask_shift) &
                         0b111;
  return outstanding;
}

/*
  Reset all of the ME tiles in the specified column
*/
void aie_reset_column(uint16_t col_idx) {
  PRINTF("Resetting column %u\r\n", col_idx);
  IO_WRITE32(getTileAddr(col_idx, 0) + REG_AIE_COL_RESET,
             1); // 1 == ResetEnable
  IO_WRITE32(getTileAddr(col_idx, 0) + REG_AIE_COL_RESET,
             0); // 0 == ResetDisable
}

/*
  Invalidate all BDs by writing to their buffer control register
*/
void xaie_shim_dma_init(uint16_t col) {
  uint64_t tileAddr = getTileAddr(col, 0);
  // Disable all channels
  for (uint8_t ch = 0; ch < 4; ch++) {
    IO_WRITE32(tileAddr + REG_SHIM_DMA_CTRL(ch), 0);
  }
  for (uint8_t bd = 0; bd < SHIM_DMA_NUM_BDS; bd++) {
    IO_WRITE32(tileAddr + REG_SHIM_DMA_BD_CTRL(bd), 0);
  }
}

/*
  Reset a shim tile
*/
void xaie_reset_shim(uint16_t col) {
  PRINTF("Resetting shim tile %u\r\n", col);
  for (uint16_t c = 0; c < XAIE_NUM_COLS; c++) {
    uint32_t val = (c == col) ? 1 : 0;
    IO_WRITE32(getTileAddr(c, 0) + REG_SHIM_RESET_ENA, val);
  }

  xaie_strobe_shim_reset();

  IO_WRITE32(getTileAddr(col, 0) + REG_SHIM_RESET_ENA, 0);
}

void xaie_device_init(void) {

  PRINTF("Initializing device...\r\n");

  // First, resetting the entire device
  xaie_array_reset();

#ifdef ARM_CONTROLLER
  int err = mlir_aie_reinit_device();
  if (err)
    printf("ERROR initializing device.\n\r");
#endif

  for (int c = 0; c < NUM_SHIM_DMAS; c++) {
    xaie_shim_dma_init(shim_dma_cols[c]);
  }

  // Turning the shim_reset_enable bit low for every column so they don't get
  // reset when we perform a global shim reset
  for (int col = 0; col < XAIE_NUM_COLS; col++) {
    IO_WRITE32(getTileAddr(col, 0) + 0x0003604C, 0);
  }
}

void xaie_array_reset() {

  // Getting a pointer to NPI
  auto *npib = (volatile uint32_t *)(NPI_BASE);

  // Performing array reset sequence
  PRINTF("Starting array reset\r\n");

  // Unlocking NPI
  npib[NPI_LOCK_REG >> 2] = 0xF9E8D7C6;

  // Performing reset
  npib[NPI_MASK_REG >> 2] = 0x04000000;
  npib[NPI_VAL_REG >> 2] = 0x040381B1;
  npib[NPI_MASK_REG >> 2] = 0x04000000;
  npib[NPI_VAL_REG >> 2] = 0x000381B1;

  // Locking NPI
  npib[NPI_LOCK_REG >> 2] = 0x12341234;
  PRINTF("Done with array reset\r\n");
}

// Initialize one segment with lower left corner at (col_start, row_start)
void xaie_segment_init(uint16_t start_col, uint16_t num_cols,
                       uint16_t start_row, uint16_t num_rows) {
  HerdCfgInst.col_start = start_col;
  HerdCfgInst.num_cols = num_cols;
  HerdCfgInst.row_start = start_row;
  HerdCfgInst.num_rows = num_rows;
#ifdef ARM_CONTROLLER

  // Performing the shim reset
  PRINTF("Performing shim reset; start_col=%u num_cols=%u\r\n", start_col,
         num_cols);
  for (uint16_t c = start_col; c < start_col + num_cols; c++) {
    IO_WRITE32(getTileAddr(c, 0) + REG_SHIM_RESET_ENA, 1);
  }

  xaie_strobe_shim_reset();

  for (uint16_t c = start_col; c < start_col + num_cols; c++) {
    IO_WRITE32(getTileAddr(c, 0) + REG_SHIM_RESET_ENA, 0);
  }

  // Performing the column reset
  PRINTF("Performing col reset\r\n");
  for (uint16_t c = start_col; c < start_col + num_cols; c++)
    aie_reset_column(c);

#endif
}

uint16_t xaie_herd_get_num_cols(void) { return HerdCfgInst.num_cols; }

uint16_t xaie_herd_get_num_rows(void) { return HerdCfgInst.num_rows; }

uint16_t xaie_herd_get_col_start(void) { return HerdCfgInst.col_start; }

uint16_t xaie_herd_get_row_start(void) { return HerdCfgInst.row_start; }
