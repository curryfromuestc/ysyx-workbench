#include <common.h>
#include "syscall.h"

static const char *syscall_name(uintptr_t id) {
  switch (id) {
    case SYS_exit:  return "exit";
    case SYS_yield: return "yield";
    case SYS_open:  return "open";
    case SYS_read:  return "read";
    case SYS_write: return "write";
    case SYS_close: return "close";
    case SYS_lseek: return "lseek";
    case SYS_brk:   return "brk";
    default:        return "?";
  }
}

static int sys_yield(void) {
  yield();
  return 0;
}

static int sys_exit(int status) {
  halt(status);
  return 0;  // unreachable
}

static int sys_write(int fd, const char *buf, size_t len) {
  // Only stdout (1) and stderr (2) are wired up so far; everything else is
  // the file-system layer's job (PA3.4).
  if (fd != 1 && fd != 2) return -1;
  for (size_t i = 0; i < len; i++) putch(buf[i]);
  return (int)len;
}

// PA3 is single-task: there's no other program to fight us for memory, so
// every brk() request is granted unconditionally. PA4 will revisit this when
// we introduce real virtual memory.
static int sys_brk(uintptr_t new_brk) {
  (void)new_brk;
  return 0;
}

void do_syscall(Context *c) {
  uintptr_t a[4];
  a[0] = c->GPR1;  // a7 — syscall number
  a[1] = c->GPR2;  // a0 — arg 0
  a[2] = c->GPR3;  // a1 — arg 1
  a[3] = c->GPR4;  // a2 — arg 2

  intptr_t ret = 0;
  switch (a[0]) {
    case SYS_yield: ret = sys_yield(); break;
    case SYS_exit:  ret = sys_exit((int)a[1]); break;
    case SYS_write: ret = sys_write((int)a[1], (const char *)a[2], (size_t)a[3]); break;
    case SYS_brk:   ret = sys_brk(a[1]); break;
    default: panic("Unhandled syscall ID = %d", a[0]);
  }

  c->GPRx = ret;  // syscall return goes back to a0

#ifdef STRACE
  Log("strace: %s(0x%x, 0x%x, 0x%x) = %d",
      syscall_name(a[0]), a[1], a[2], a[3], ret);
#endif
}
