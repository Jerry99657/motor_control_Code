#include "nes_cpu.h"

#include <stddef.h>
#include <string.h>

/*
 * Small native-C Ricoh 2A03/6502 interpreter for the GCC build.  The two
 * reference projects supplied with this board use a Keil ARMASM core, whose
 * syntax cannot be linked by arm-none-eabi-gcc.  This implementation keeps
 * their scanline execution model while avoiding toolchain-specific assembly.
 * Decimal mode is intentionally ignored because the NES 2A03 has no BCD ALU.
 */

#define F_C 0x01U
#define F_Z 0x02U
#define F_I 0x04U
#define F_D 0x08U
#define F_B 0x10U
#define F_U 0x20U
#define F_V 0x40U
#define F_N 0x80U

static const uint8_t s_cycles[256] =
{
  7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6,
  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6,
  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6,
  2,5,2,8,4,4,6,6,2,4,3,7,4,4,7,7,
  6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6,
  2,5,2,8,4,4,6,6,2,4,4,7,4,4,7,7,
  2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
  2,6,2,6,4,4,4,4,2,5,2,5,5,5,5,5,
  2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
  2,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,
  2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
  2,5,2,8,4,4,6,6,2,4,3,7,4,4,7,7,
  2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7
};

static uint8_t rd(NES_CPU *c, uint16_t a)
{
  return c->read(c->user, a);
}

static void wr(NES_CPU *c, uint16_t a, uint8_t v)
{
  c->write(c->user, a, v);
}

static uint8_t fetch8(NES_CPU *c)
{
  uint8_t v = rd(c, c->pc);
  c->pc++;
  return v;
}

static uint16_t fetch16(NES_CPU *c)
{
  uint16_t lo = fetch8(c);
  return (uint16_t)(lo | ((uint16_t)fetch8(c) << 8));
}

static uint16_t read16(NES_CPU *c, uint16_t a)
{
  uint16_t lo = rd(c, a);
  return (uint16_t)(lo | ((uint16_t)rd(c, (uint16_t)(a + 1U)) << 8));
}

static void push(NES_CPU *c, uint8_t v)
{
  wr(c, (uint16_t)(0x0100U | c->sp), v);
  c->sp--;
}

static uint8_t pull(NES_CPU *c)
{
  c->sp++;
  return rd(c, (uint16_t)(0x0100U | c->sp));
}

static void set_nz(NES_CPU *c, uint8_t v)
{
  c->p = (uint8_t)((c->p & (uint8_t)~(F_N | F_Z)) |
                   ((v == 0U) ? F_Z : 0U) | (v & F_N));
}

static uint16_t addr_zp(NES_CPU *c) { return fetch8(c); }
static uint16_t addr_zpx(NES_CPU *c) { return (uint8_t)(fetch8(c) + c->x); }
static uint16_t addr_zpy(NES_CPU *c) { return (uint8_t)(fetch8(c) + c->y); }
static uint16_t addr_abs(NES_CPU *c) { return fetch16(c); }

static uint16_t addr_absx(NES_CPU *c, uint8_t *cross)
{
  uint16_t base = fetch16(c);
  uint16_t a = (uint16_t)(base + c->x);
  *cross = ((base ^ a) & 0xFF00U) ? 1U : 0U;
  return a;
}

static uint16_t addr_absy(NES_CPU *c, uint8_t *cross)
{
  uint16_t base = fetch16(c);
  uint16_t a = (uint16_t)(base + c->y);
  *cross = ((base ^ a) & 0xFF00U) ? 1U : 0U;
  return a;
}

static uint16_t addr_indx(NES_CPU *c)
{
  uint8_t zp = (uint8_t)(fetch8(c) + c->x);
  return (uint16_t)(rd(c, zp) | ((uint16_t)rd(c, (uint8_t)(zp + 1U)) << 8));
}

static uint16_t addr_indy(NES_CPU *c, uint8_t *cross)
{
  uint8_t zp = fetch8(c);
  uint16_t base = (uint16_t)(rd(c, zp) |
                            ((uint16_t)rd(c, (uint8_t)(zp + 1U)) << 8));
  uint16_t a = (uint16_t)(base + c->y);
  *cross = ((base ^ a) & 0xFF00U) ? 1U : 0U;
  return a;
}

static void op_adc(NES_CPU *c, uint8_t v)
{
  uint16_t sum = (uint16_t)c->a + v + ((c->p & F_C) ? 1U : 0U);
  uint8_t result = (uint8_t)sum;
  c->p = (uint8_t)((c->p & (uint8_t)~(F_C | F_V)) |
                   ((sum > 0xFFU) ? F_C : 0U) |
                   (((~(c->a ^ v) & (c->a ^ result) & 0x80U) != 0U) ? F_V : 0U));
  c->a = result;
  set_nz(c, c->a);
}

static void op_sbc(NES_CPU *c, uint8_t v)
{
  op_adc(c, (uint8_t)~v);
}

static void op_cmp(NES_CPU *c, uint8_t reg, uint8_t v)
{
  uint16_t d = (uint16_t)reg - v;
  c->p = (uint8_t)((c->p & (uint8_t)~F_C) | ((reg >= v) ? F_C : 0U));
  set_nz(c, (uint8_t)d);
}

static uint8_t op_asl(NES_CPU *c, uint8_t v)
{
  c->p = (uint8_t)((c->p & (uint8_t)~F_C) | ((v & 0x80U) ? F_C : 0U));
  v <<= 1;
  set_nz(c, v);
  return v;
}

static uint8_t op_lsr(NES_CPU *c, uint8_t v)
{
  c->p = (uint8_t)((c->p & (uint8_t)~F_C) | ((v & 1U) ? F_C : 0U));
  v >>= 1;
  set_nz(c, v);
  return v;
}

static uint8_t op_rol(NES_CPU *c, uint8_t v)
{
  uint8_t carry = (c->p & F_C) ? 1U : 0U;
  c->p = (uint8_t)((c->p & (uint8_t)~F_C) | ((v & 0x80U) ? F_C : 0U));
  v = (uint8_t)((v << 1) | carry);
  set_nz(c, v);
  return v;
}

static uint8_t op_ror(NES_CPU *c, uint8_t v)
{
  uint8_t carry = (c->p & F_C) ? 0x80U : 0U;
  c->p = (uint8_t)((c->p & (uint8_t)~F_C) | ((v & 1U) ? F_C : 0U));
  v = (uint8_t)((v >> 1) | carry);
  set_nz(c, v);
  return v;
}

static uint8_t branch(NES_CPU *c, uint8_t condition)
{
  int8_t rel = (int8_t)fetch8(c);
  uint16_t old_pc;
  if (condition == 0U)
  {
    return 0U;
  }
  old_pc = c->pc;
  c->pc = (uint16_t)(c->pc + rel);
  return (uint8_t)(1U + ((((old_pc ^ c->pc) & 0xFF00U) != 0U) ? 1U : 0U));
}

static void interrupt_nmi(NES_CPU *c)
{
  push(c, (uint8_t)(c->pc >> 8));
  push(c, (uint8_t)c->pc);
  push(c, (uint8_t)((c->p & (uint8_t)~F_B) | F_U));
  c->p |= F_I;
  c->pc = read16(c, 0xFFFAU);
}

static void interrupt_irq(NES_CPU *c)
{
  push(c, (uint8_t)(c->pc >> 8));
  push(c, (uint8_t)c->pc);
  push(c, (uint8_t)((c->p & (uint8_t)~F_B) | F_U));
  c->p |= F_I;
  c->pc = read16(c, 0xFFFEU);
}

void NES_CPU_Init(NES_CPU *cpu, NES_CPU_ReadFn read_fn,
                  NES_CPU_WriteFn write_fn, void *user)
{
  if (cpu == NULL)
  {
    return;
  }
  memset(cpu, 0, sizeof(*cpu));
  cpu->read = read_fn;
  cpu->write = write_fn;
  cpu->user = user;
}

void NES_CPU_Reset(NES_CPU *cpu)
{
  if ((cpu == NULL) || (cpu->read == NULL) || (cpu->write == NULL))
  {
    return;
  }
  cpu->a = 0U;
  cpu->x = 0U;
  cpu->y = 0U;
  cpu->sp = 0xFDU;
  cpu->p = F_I | F_U;
  cpu->nmi_pending = 0U;
  cpu->irq_pending = 0U;
  cpu->jammed = 0U;
  cpu->last_opcode = 0U;
  cpu->total_cycles = 7U;
  cpu->pc = read16(cpu, 0xFFFCU);
  cpu->last_pc = cpu->pc;
}

void NES_CPU_RequestNmi(NES_CPU *cpu)
{
  if (cpu != NULL)
  {
    cpu->nmi_pending = 1U;
  }
}

void NES_CPU_RequestIrq(NES_CPU *cpu)
{
  if (cpu != NULL)
  {
    cpu->irq_pending = 1U;
  }
}

void NES_CPU_ClearIrq(NES_CPU *cpu)
{
  if (cpu != NULL)
  {
    cpu->irq_pending = 0U;
  }
}

int32_t NES_CPU_Run(NES_CPU *c, int32_t cycle_budget)
{
  int32_t used = 0;
  uint8_t page;
  uint8_t op;
  uint8_t v;
  uint16_t a;
  uint16_t t;
  uint8_t extra;

  if ((c == NULL) || (c->read == NULL) || (c->write == NULL) ||
      (cycle_budget <= 0) || (c->jammed != 0U))
  {
    return 0;
  }

  while ((used < cycle_budget) && (c->jammed == 0U))
  {
    if (c->nmi_pending != 0U)
    {
      c->nmi_pending = 0U;
      interrupt_nmi(c);
      used += 7;
      c->total_cycles += 7U;
      continue;
    }

    if ((c->irq_pending != 0U) && ((c->p & F_I) == 0U))
    {
      interrupt_irq(c);
      used += 7;
      c->total_cycles += 7U;
      continue;
    }

    c->last_pc = c->pc;
    op = fetch8(c);
    c->last_opcode = op;
    page = 0U;
    extra = 0U;
    a = 0U;
    v = 0U;

    switch (op)
    {
      case 0x00: /* BRK */
        c->pc++;
        push(c, (uint8_t)(c->pc >> 8)); push(c, (uint8_t)c->pc);
        push(c, (uint8_t)(c->p | F_B | F_U)); c->p |= F_I;
        c->pc = read16(c, 0xFFFEU); break;
      case 0x01: c->a |= rd(c, addr_indx(c)); set_nz(c,c->a); break;
      case 0x05: c->a |= rd(c, addr_zp(c)); set_nz(c,c->a); break;
      case 0x06: a=addr_zp(c); wr(c,a,op_asl(c,rd(c,a))); break;
      case 0x08: push(c,(uint8_t)(c->p|F_B|F_U)); break;
      case 0x09: c->a |= fetch8(c); set_nz(c,c->a); break;
      case 0x0A: c->a=op_asl(c,c->a); break;
      case 0x0D: c->a |= rd(c,addr_abs(c)); set_nz(c,c->a); break;
      case 0x0E: a=addr_abs(c); wr(c,a,op_asl(c,rd(c,a))); break;
      case 0x10: extra=branch(c,(c->p&F_N)==0U); break;
      case 0x11: c->a |= rd(c,addr_indy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x15: c->a |= rd(c,addr_zpx(c)); set_nz(c,c->a); break;
      case 0x16: a=addr_zpx(c); wr(c,a,op_asl(c,rd(c,a))); break;
      case 0x18: c->p&=(uint8_t)~F_C; break;
      case 0x19: c->a |= rd(c,addr_absy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x1D: c->a |= rd(c,addr_absx(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x1E: a=addr_absx(c,&page); wr(c,a,op_asl(c,rd(c,a))); break;

      case 0x20: a=fetch16(c); t=(uint16_t)(c->pc-1U); push(c,(uint8_t)(t>>8)); push(c,(uint8_t)t); c->pc=a; break;
      case 0x21: c->a &= rd(c,addr_indx(c)); set_nz(c,c->a); break;
      case 0x24: v=rd(c,addr_zp(c)); c->p=(uint8_t)((c->p&~(F_N|F_V|F_Z))|(v&(F_N|F_V))|((c->a&v)?0U:F_Z)); break;
      case 0x25: c->a &= rd(c,addr_zp(c)); set_nz(c,c->a); break;
      case 0x26: a=addr_zp(c); wr(c,a,op_rol(c,rd(c,a))); break;
      case 0x28: c->p=(uint8_t)((pull(c)&~F_B)|F_U); break;
      case 0x29: c->a &= fetch8(c); set_nz(c,c->a); break;
      case 0x2A: c->a=op_rol(c,c->a); break;
      case 0x2C: v=rd(c,addr_abs(c)); c->p=(uint8_t)((c->p&~(F_N|F_V|F_Z))|(v&(F_N|F_V))|((c->a&v)?0U:F_Z)); break;
      case 0x2D: c->a &= rd(c,addr_abs(c)); set_nz(c,c->a); break;
      case 0x2E: a=addr_abs(c); wr(c,a,op_rol(c,rd(c,a))); break;
      case 0x30: extra=branch(c,(c->p&F_N)!=0U); break;
      case 0x31: c->a &= rd(c,addr_indy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x35: c->a &= rd(c,addr_zpx(c)); set_nz(c,c->a); break;
      case 0x36: a=addr_zpx(c); wr(c,a,op_rol(c,rd(c,a))); break;
      case 0x38: c->p|=F_C; break;
      case 0x39: c->a &= rd(c,addr_absy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x3D: c->a &= rd(c,addr_absx(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x3E: a=addr_absx(c,&page); wr(c,a,op_rol(c,rd(c,a))); break;

      case 0x40: c->p=(uint8_t)((pull(c)&~F_B)|F_U); t=pull(c); c->pc=(uint16_t)(t|((uint16_t)pull(c)<<8)); break;
      case 0x41: c->a ^= rd(c,addr_indx(c)); set_nz(c,c->a); break;
      case 0x45: c->a ^= rd(c,addr_zp(c)); set_nz(c,c->a); break;
      case 0x46: a=addr_zp(c); wr(c,a,op_lsr(c,rd(c,a))); break;
      case 0x48: push(c,c->a); break;
      case 0x49: c->a ^= fetch8(c); set_nz(c,c->a); break;
      case 0x4A: c->a=op_lsr(c,c->a); break;
      case 0x4C: c->pc=fetch16(c); break;
      case 0x4D: c->a ^= rd(c,addr_abs(c)); set_nz(c,c->a); break;
      case 0x4E: a=addr_abs(c); wr(c,a,op_lsr(c,rd(c,a))); break;
      case 0x50: extra=branch(c,(c->p&F_V)==0U); break;
      case 0x51: c->a ^= rd(c,addr_indy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x55: c->a ^= rd(c,addr_zpx(c)); set_nz(c,c->a); break;
      case 0x56: a=addr_zpx(c); wr(c,a,op_lsr(c,rd(c,a))); break;
      case 0x58: c->p&=(uint8_t)~F_I; break;
      case 0x59: c->a ^= rd(c,addr_absy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x5D: c->a ^= rd(c,addr_absx(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0x5E: a=addr_absx(c,&page); wr(c,a,op_lsr(c,rd(c,a))); break;

      case 0x60: t=pull(c); c->pc=(uint16_t)((t|((uint16_t)pull(c)<<8))+1U); break;
      case 0x61: op_adc(c,rd(c,addr_indx(c))); break;
      case 0x65: op_adc(c,rd(c,addr_zp(c))); break;
      case 0x66: a=addr_zp(c); wr(c,a,op_ror(c,rd(c,a))); break;
      case 0x68: c->a=pull(c); set_nz(c,c->a); break;
      case 0x69: op_adc(c,fetch8(c)); break;
      case 0x6A: c->a=op_ror(c,c->a); break;
      case 0x6C: a=fetch16(c); c->pc=(uint16_t)(rd(c,a)|((uint16_t)rd(c,(uint16_t)((a&0xFF00U)|((a+1U)&0x00FFU)))<<8)); break;
      case 0x6D: op_adc(c,rd(c,addr_abs(c))); break;
      case 0x6E: a=addr_abs(c); wr(c,a,op_ror(c,rd(c,a))); break;
      case 0x70: extra=branch(c,(c->p&F_V)!=0U); break;
      case 0x71: op_adc(c,rd(c,addr_indy(c,&page))); extra=page; break;
      case 0x75: op_adc(c,rd(c,addr_zpx(c))); break;
      case 0x76: a=addr_zpx(c); wr(c,a,op_ror(c,rd(c,a))); break;
      case 0x78: c->p|=F_I; break;
      case 0x79: op_adc(c,rd(c,addr_absy(c,&page))); extra=page; break;
      case 0x7D: op_adc(c,rd(c,addr_absx(c,&page))); extra=page; break;
      case 0x7E: a=addr_absx(c,&page); wr(c,a,op_ror(c,rd(c,a))); break;

      case 0x81: wr(c,addr_indx(c),c->a); break;
      case 0x84: wr(c,addr_zp(c),c->y); break;
      case 0x85: wr(c,addr_zp(c),c->a); break;
      case 0x86: wr(c,addr_zp(c),c->x); break;
      case 0x88: c->y--; set_nz(c,c->y); break;
      case 0x8A: c->a=c->x; set_nz(c,c->a); break;
      case 0x8C: wr(c,addr_abs(c),c->y); break;
      case 0x8D: wr(c,addr_abs(c),c->a); break;
      case 0x8E: wr(c,addr_abs(c),c->x); break;
      case 0x90: extra=branch(c,(c->p&F_C)==0U); break;
      case 0x91: wr(c,addr_indy(c,&page),c->a); break;
      case 0x94: wr(c,addr_zpx(c),c->y); break;
      case 0x95: wr(c,addr_zpx(c),c->a); break;
      case 0x96: wr(c,addr_zpy(c),c->x); break;
      case 0x98: c->a=c->y; set_nz(c,c->a); break;
      case 0x99: wr(c,addr_absy(c,&page),c->a); break;
      case 0x9A: c->sp=c->x; break;
      case 0x9D: wr(c,addr_absx(c,&page),c->a); break;

      case 0xA0: c->y=fetch8(c); set_nz(c,c->y); break;
      case 0xA1: c->a=rd(c,addr_indx(c)); set_nz(c,c->a); break;
      case 0xA2: c->x=fetch8(c); set_nz(c,c->x); break;
      case 0xA4: c->y=rd(c,addr_zp(c)); set_nz(c,c->y); break;
      case 0xA5: c->a=rd(c,addr_zp(c)); set_nz(c,c->a); break;
      case 0xA6: c->x=rd(c,addr_zp(c)); set_nz(c,c->x); break;
      case 0xA8: c->y=c->a; set_nz(c,c->y); break;
      case 0xA9: c->a=fetch8(c); set_nz(c,c->a); break;
      case 0xAA: c->x=c->a; set_nz(c,c->x); break;
      case 0xAC: c->y=rd(c,addr_abs(c)); set_nz(c,c->y); break;
      case 0xAD: c->a=rd(c,addr_abs(c)); set_nz(c,c->a); break;
      case 0xAE: c->x=rd(c,addr_abs(c)); set_nz(c,c->x); break;
      case 0xB0: extra=branch(c,(c->p&F_C)!=0U); break;
      case 0xB1: c->a=rd(c,addr_indy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0xB4: c->y=rd(c,addr_zpx(c)); set_nz(c,c->y); break;
      case 0xB5: c->a=rd(c,addr_zpx(c)); set_nz(c,c->a); break;
      case 0xB6: c->x=rd(c,addr_zpy(c)); set_nz(c,c->x); break;
      case 0xB8: c->p&=(uint8_t)~F_V; break;
      case 0xB9: c->a=rd(c,addr_absy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0xBA: c->x=c->sp; set_nz(c,c->x); break;
      case 0xBC: c->y=rd(c,addr_absx(c,&page)); set_nz(c,c->y); extra=page; break;
      case 0xBD: c->a=rd(c,addr_absx(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0xBE: c->x=rd(c,addr_absy(c,&page)); set_nz(c,c->x); extra=page; break;

      case 0xC0: op_cmp(c,c->y,fetch8(c)); break;
      case 0xC1: op_cmp(c,c->a,rd(c,addr_indx(c))); break;
      case 0xC4: op_cmp(c,c->y,rd(c,addr_zp(c))); break;
      case 0xC5: op_cmp(c,c->a,rd(c,addr_zp(c))); break;
      case 0xC6: a=addr_zp(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); set_nz(c,v); break;
      case 0xC8: c->y++; set_nz(c,c->y); break;
      case 0xC9: op_cmp(c,c->a,fetch8(c)); break;
      case 0xCA: c->x--; set_nz(c,c->x); break;
      case 0xCC: op_cmp(c,c->y,rd(c,addr_abs(c))); break;
      case 0xCD: op_cmp(c,c->a,rd(c,addr_abs(c))); break;
      case 0xCE: a=addr_abs(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); set_nz(c,v); break;
      case 0xD0: extra=branch(c,(c->p&F_Z)==0U); break;
      case 0xD1: op_cmp(c,c->a,rd(c,addr_indy(c,&page))); extra=page; break;
      case 0xD5: op_cmp(c,c->a,rd(c,addr_zpx(c))); break;
      case 0xD6: a=addr_zpx(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); set_nz(c,v); break;
      case 0xD8: c->p&=(uint8_t)~F_D; break;
      case 0xD9: op_cmp(c,c->a,rd(c,addr_absy(c,&page))); extra=page; break;
      case 0xDD: op_cmp(c,c->a,rd(c,addr_absx(c,&page))); extra=page; break;
      case 0xDE: a=addr_absx(c,&page); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); set_nz(c,v); break;

      case 0xE0: op_cmp(c,c->x,fetch8(c)); break;
      case 0xE1: op_sbc(c,rd(c,addr_indx(c))); break;
      case 0xE4: op_cmp(c,c->x,rd(c,addr_zp(c))); break;
      case 0xE5: op_sbc(c,rd(c,addr_zp(c))); break;
      case 0xE6: a=addr_zp(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); set_nz(c,v); break;
      case 0xE8: c->x++; set_nz(c,c->x); break;
      case 0xE9: case 0xEB: op_sbc(c,fetch8(c)); break;
      case 0xEA: break;
      case 0xEC: op_cmp(c,c->x,rd(c,addr_abs(c))); break;
      case 0xED: op_sbc(c,rd(c,addr_abs(c))); break;
      case 0xEE: a=addr_abs(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); set_nz(c,v); break;
      case 0xF0: extra=branch(c,(c->p&F_Z)!=0U); break;
      case 0xF1: op_sbc(c,rd(c,addr_indy(c,&page))); extra=page; break;
      case 0xF5: op_sbc(c,rd(c,addr_zpx(c))); break;
      case 0xF6: a=addr_zpx(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); set_nz(c,v); break;
      case 0xF8: c->p|=F_D; break;
      case 0xF9: op_sbc(c,rd(c,addr_absy(c,&page))); extra=page; break;
      case 0xFD: op_sbc(c,rd(c,addr_absx(c,&page))); extra=page; break;
      case 0xFE: a=addr_absx(c,&page); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); set_nz(c,v); break;

      /* Stable unofficial instructions used by a number of commercial ROMs. */
      case 0x03: a=addr_indx(c); v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;
      case 0x07: a=addr_zp(c);   v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;
      case 0x0F: a=addr_abs(c);  v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;
      case 0x13: a=addr_indy(c,&page); v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;
      case 0x17: a=addr_zpx(c);  v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;
      case 0x1B: a=addr_absy(c,&page); v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;
      case 0x1F: a=addr_absx(c,&page); v=op_asl(c,rd(c,a)); wr(c,a,v); c->a|=v; set_nz(c,c->a); break;

      case 0x23: a=addr_indx(c); v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;
      case 0x27: a=addr_zp(c);   v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;
      case 0x2F: a=addr_abs(c);  v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;
      case 0x33: a=addr_indy(c,&page); v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;
      case 0x37: a=addr_zpx(c);  v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;
      case 0x3B: a=addr_absy(c,&page); v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;
      case 0x3F: a=addr_absx(c,&page); v=op_rol(c,rd(c,a)); wr(c,a,v); c->a&=v; set_nz(c,c->a); break;

      case 0x43: a=addr_indx(c); v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;
      case 0x47: a=addr_zp(c);   v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;
      case 0x4F: a=addr_abs(c);  v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;
      case 0x53: a=addr_indy(c,&page); v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;
      case 0x57: a=addr_zpx(c);  v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;
      case 0x5B: a=addr_absy(c,&page); v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;
      case 0x5F: a=addr_absx(c,&page); v=op_lsr(c,rd(c,a)); wr(c,a,v); c->a^=v; set_nz(c,c->a); break;

      case 0x63: a=addr_indx(c); v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;
      case 0x67: a=addr_zp(c);   v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;
      case 0x6F: a=addr_abs(c);  v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;
      case 0x73: a=addr_indy(c,&page); v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;
      case 0x77: a=addr_zpx(c);  v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;
      case 0x7B: a=addr_absy(c,&page); v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;
      case 0x7F: a=addr_absx(c,&page); v=op_ror(c,rd(c,a)); wr(c,a,v); op_adc(c,v); break;

      case 0x83: wr(c,addr_indx(c),(uint8_t)(c->a&c->x)); break;
      case 0x87: wr(c,addr_zp(c),(uint8_t)(c->a&c->x)); break;
      case 0x8F: wr(c,addr_abs(c),(uint8_t)(c->a&c->x)); break;
      case 0x97: wr(c,addr_zpy(c),(uint8_t)(c->a&c->x)); break;
      case 0xA3: c->a=c->x=rd(c,addr_indx(c)); set_nz(c,c->a); break;
      case 0xA7: c->a=c->x=rd(c,addr_zp(c)); set_nz(c,c->a); break;
      case 0xAF: c->a=c->x=rd(c,addr_abs(c)); set_nz(c,c->a); break;
      case 0xB3: c->a=c->x=rd(c,addr_indy(c,&page)); set_nz(c,c->a); extra=page; break;
      case 0xB7: c->a=c->x=rd(c,addr_zpy(c)); set_nz(c,c->a); break;
      case 0xBF: c->a=c->x=rd(c,addr_absy(c,&page)); set_nz(c,c->a); extra=page; break;

      case 0xC3: a=addr_indx(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;
      case 0xC7: a=addr_zp(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;
      case 0xCF: a=addr_abs(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;
      case 0xD3: a=addr_indy(c,&page); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;
      case 0xD7: a=addr_zpx(c); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;
      case 0xDB: a=addr_absy(c,&page); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;
      case 0xDF: a=addr_absx(c,&page); v=(uint8_t)(rd(c,a)-1U); wr(c,a,v); op_cmp(c,c->a,v); break;

      case 0xE3: a=addr_indx(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;
      case 0xE7: a=addr_zp(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;
      case 0xEF: a=addr_abs(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;
      case 0xF3: a=addr_indy(c,&page); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;
      case 0xF7: a=addr_zpx(c); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;
      case 0xFB: a=addr_absy(c,&page); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;
      case 0xFF: a=addr_absx(c,&page); v=(uint8_t)(rd(c,a)+1U); wr(c,a,v); op_sbc(c,v); break;

      case 0x0B: case 0x2B: c->a&=fetch8(c); set_nz(c,c->a); c->p=(uint8_t)((c->p&~F_C)|((c->a&0x80U)?F_C:0U)); break;
      case 0x4B: c->a&=fetch8(c); c->a=op_lsr(c,c->a); break;
      case 0x6B:
        c->a&=fetch8(c); c->a=op_ror(c,c->a);
        c->p=(uint8_t)((c->p&~(F_C|F_V))|((c->a&0x40U)?F_C:0U)|((((c->a>>6)^(c->a>>5))&1U)?F_V:0U)); break;
      case 0x8B: c->a=(uint8_t)(c->x&fetch8(c)); set_nz(c,c->a); break;
      case 0xAB: c->a=c->x=(uint8_t)(c->a&fetch8(c)); set_nz(c,c->a); break;
      case 0xCB: v=fetch8(c); t=(uint16_t)(c->a&c->x)-v; c->p=(uint8_t)((c->p&~F_C)|((t<0x100U)?F_C:0U)); c->x=(uint8_t)t; set_nz(c,c->x); break;

      /* NOP variants still consume their operands, preserving the stream. */
      case 0x04: case 0x44: case 0x64: (void)addr_zp(c); break;
      case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4: (void)addr_zpx(c); break;
      case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2: (void)fetch8(c); break;
      case 0x0C: (void)addr_abs(c); break;
      case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC: (void)addr_absx(c,&page); extra=page; break;
      case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: break;

      /* Rare unstable store/load opcodes: implement their deterministic core. */
      case 0x93: a=addr_indy(c,&page); wr(c,a,(uint8_t)(c->a&c->x&((a>>8)+1U))); break;
      case 0x9B: a=addr_absy(c,&page); c->sp=(uint8_t)(c->a&c->x); wr(c,a,(uint8_t)(c->sp&((a>>8)+1U))); break;
      case 0x9C: a=addr_absx(c,&page); wr(c,a,(uint8_t)(c->y&((a>>8)+1U))); break;
      case 0x9E: a=addr_absy(c,&page); wr(c,a,(uint8_t)(c->x&((a>>8)+1U))); break;
      case 0x9F: a=addr_absy(c,&page); wr(c,a,(uint8_t)(c->a&c->x&((a>>8)+1U))); break;
      case 0xBB: c->a=c->x=c->sp=(uint8_t)(rd(c,addr_absy(c,&page))&c->sp); set_nz(c,c->a); extra=page; break;

      case 0x02: case 0x12: case 0x22: case 0x32: case 0x42: case 0x52:
      case 0x62: case 0x72: case 0x92: case 0xB2: case 0xD2: case 0xF2:
        c->jammed=1U; break;
      default:
        c->jammed=1U; break;
    }

    c->p |= F_U;
    used += (int32_t)s_cycles[op] + extra;
    c->total_cycles += (uint32_t)s_cycles[op] + extra;
  }
  return used;
}
