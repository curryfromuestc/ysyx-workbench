#ifndef NPC_H__
#define NPC_H__

#include <klib-macros.h>

// MMIO addresses must stay in sync with npc/csrc/main.cpp.
#define SERIAL_PORT  0x10000000u
#define RTC_ADDR     0xa0000048u   // [RTC_ADDR]=lo32, [RTC_ADDR+4]=hi32

#define nemu_trap(code) asm volatile("mv a0, %0; ebreak" : :"r"(code))

extern char _pmem_start;
#define PMEM_SIZE (128 * 1024 * 1024)
#define PMEM_END  ((uintptr_t)&_pmem_start + PMEM_SIZE)

#endif
