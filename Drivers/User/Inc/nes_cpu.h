#ifndef NES_CPU_H
#define NES_CPU_H

#include <stdint.h>

typedef uint8_t (*NES_CPU_ReadFn)(void *user, uint16_t address);
typedef void (*NES_CPU_WriteFn)(void *user, uint16_t address, uint8_t value);

typedef struct
{
  uint16_t pc;
  uint8_t a;
  uint8_t x;
  uint8_t y;
  uint8_t sp;
  uint8_t p;
  uint8_t nmi_pending;
  uint8_t irq_pending;
  uint8_t jammed;
  uint8_t last_opcode;
  uint16_t last_pc;
  uint32_t total_cycles;
  NES_CPU_ReadFn read;
  NES_CPU_WriteFn write;
  void *user;
} NES_CPU;

void NES_CPU_Init(NES_CPU *cpu, NES_CPU_ReadFn read_fn,
                  NES_CPU_WriteFn write_fn, void *user);
void NES_CPU_Reset(NES_CPU *cpu);
void NES_CPU_RequestNmi(NES_CPU *cpu);
void NES_CPU_RequestIrq(NES_CPU *cpu);
void NES_CPU_ClearIrq(NES_CPU *cpu);
int32_t NES_CPU_Run(NES_CPU *cpu, int32_t cycle_budget);

#endif /* NES_CPU_H */
