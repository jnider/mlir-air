#include "interrupt.h"
#include "platform.h"
#include <stdio.h>

#ifdef DEBUG_INT
#define PRINTF printf
#else
#define PRINTF(...)
#endif // DEBUG_INT

// A72 CPU interface
#define GICC_CTLR 0x0000000000
#define GICC_PMR 0x0000000004
#define GICC_BPR 0x0000000008
#define GICC_IAR 0x000000000C
#define GICC_EOIR 0x0000000010
#define GICC_RPR  0x0000000014
#define GICC_HPPIR  0x0000000018
#define GICC_ABPR 0x000000001C
#define GICC_AIAR 0x0000000020
#define GICC_AEOIR  0x0000000024
#define GICC_AHPPIR 0x0000000028
#define GICC_APR0 0x00000000D0
#define GICC_NSAPR0 0x00000000E0
#define GICC_IIDR 0x00000000FC
#define GICC_DIR  0x0000001000

// Distributor
#define GICD_CTLR 0x00
#define GICD_TYPER 0x04

static uint64_t controllerBase;
static uint64_t distributorBase;

int gic_init(uint64_t controllerAddr, uint64_t distributorAddr)
{
  controllerBase = controllerAddr;
  distributorBase = distributorAddr;

  // Program the priority mask of the CPU
  IO_WRITE32(controllerBase + GICC_PMR, 0xF0);

  // enable FIQ and secure domain interrupts
  IO_WRITE32(controllerBase + GICC_CTLR, 0x07);

  uint32_t val = IO_READ32(distributorBase + GICD_TYPER);
  PRINTF("%s %u\r\n", __func__, val);

  return 0;
}

extern "C" void SynchronousInterrupt(void)
{
}

extern "C" void IRQInterrupt(void)
{
}

extern "C" void FIQInterrupt(void)
{
}

extern "C" void SErrorInterrupt(void)
{
}
