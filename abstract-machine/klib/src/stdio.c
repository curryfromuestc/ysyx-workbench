#include <am.h>
#include <klib.h>
#include <klib-macros.h>
#include <stdarg.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)

typedef struct {
  char *buf;
  size_t pos;
  size_t cap;
  int do_putch;
  int unbounded;
} out_t;

static void emit(out_t *o, char c) {
  if (o->do_putch) {
    putch(c);
    o->pos++;
    return;
  }
  if (o->unbounded) {
    o->buf[o->pos++] = c;
  } else if (o->pos + 1 < o->cap) {
    o->buf[o->pos++] = c;
  } else {
    o->pos++;
  }
}

static void emit_str(out_t *o, const char *s) {
  if (s == NULL) s = "(null)";
  while (*s) emit(o, *s++);
}

static void emit_uint(out_t *o, unsigned long v, int base, int upper, int width, int zero_pad) {
  char tmp[32];
  int len = 0;
  if (v == 0) {
    tmp[len++] = '0';
  } else {
    while (v > 0) {
      unsigned int r = (unsigned int)(v % (unsigned long)base);
      if (r < 10) tmp[len++] = '0' + r;
      else tmp[len++] = (upper ? 'A' : 'a') + (r - 10);
      v /= (unsigned long)base;
    }
  }
  char pad = zero_pad ? '0' : ' ';
  while (len < width) emit(o, pad), width--;
  while (len > 0) emit(o, tmp[--len]);
}

static void emit_int(out_t *o, long v, int width, int zero_pad) {
  unsigned long uv;
  int neg = 0;
  if (v < 0) {
    neg = 1;
    uv = (unsigned long)(-(v + 1)) + 1UL;
  } else {
    uv = (unsigned long)v;
  }
  char tmp[32];
  int len = 0;
  if (uv == 0) tmp[len++] = '0';
  else {
    while (uv > 0) { tmp[len++] = '0' + (int)(uv % 10); uv /= 10; }
  }
  if (neg) tmp[len++] = '-';
  char pad = zero_pad ? '0' : ' ';
  if (zero_pad && neg) {
    emit(o, '-');
    len--;
    while (len < width - 1) { emit(o, pad); width--; }
  } else {
    while (len < width) { emit(o, pad); width--; }
  }
  while (len > 0) emit(o, tmp[--len]);
}

static int do_format(out_t *o, const char *fmt, va_list ap) {
  while (*fmt) {
    if (*fmt != '%') {
      emit(o, *fmt++);
      continue;
    }
    fmt++;
    int width = 0;
    int zero_pad = 0;
    if (*fmt == '0') { zero_pad = 1; fmt++; }
    while (*fmt >= '0' && *fmt <= '9') {
      width = width * 10 + (*fmt - '0');
      fmt++;
    }
    int lflag = 0;
    if (*fmt == 'l') { lflag = 1; fmt++; if (*fmt == 'l') { lflag = 2; fmt++; } }
    switch (*fmt) {
      case 'd':
      case 'i': {
        long v = lflag ? va_arg(ap, long) : (long)va_arg(ap, int);
        emit_int(o, v, width, zero_pad);
        break;
      }
      case 'u': {
        unsigned long v = lflag ? va_arg(ap, unsigned long) : (unsigned long)va_arg(ap, unsigned int);
        emit_uint(o, v, 10, 0, width, zero_pad);
        break;
      }
      case 'x': {
        unsigned long v = lflag ? va_arg(ap, unsigned long) : (unsigned long)va_arg(ap, unsigned int);
        emit_uint(o, v, 16, 0, width, zero_pad);
        break;
      }
      case 'X': {
        unsigned long v = lflag ? va_arg(ap, unsigned long) : (unsigned long)va_arg(ap, unsigned int);
        emit_uint(o, v, 16, 1, width, zero_pad);
        break;
      }
      case 'p': {
        emit(o, '0'); emit(o, 'x');
        unsigned long v = (unsigned long)(uintptr_t)va_arg(ap, void *);
        emit_uint(o, v, 16, 0, width, zero_pad);
        break;
      }
      case 'o': {
        unsigned long v = lflag ? va_arg(ap, unsigned long) : (unsigned long)va_arg(ap, unsigned int);
        emit_uint(o, v, 8, 0, width, zero_pad);
        break;
      }
      case 's': {
        const char *s = va_arg(ap, const char *);
        if (s == NULL) s = "(null)";
        int slen = (int)strlen(s);
        while (slen < width) { emit(o, ' '); width--; }
        emit_str(o, s);
        break;
      }
      case 'c': {
        char c = (char)va_arg(ap, int);
        emit(o, c);
        break;
      }
      case '%': {
        emit(o, '%');
        break;
      }
      default:
        emit(o, '%');
        emit(o, *fmt);
        break;
    }
    fmt++;
  }
  if (!o->do_putch && (o->unbounded || o->pos < o->cap)) {
    o->buf[o->pos] = '\0';
  } else if (!o->do_putch && o->cap > 0) {
    o->buf[o->cap - 1] = '\0';
  }
  return (int)o->pos;
}

int printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  out_t o = { .buf = NULL, .pos = 0, .cap = 0, .do_putch = 1, .unbounded = 0 };
  int r = do_format(&o, fmt, ap);
  va_end(ap);
  return r;
}

int vsprintf(char *out, const char *fmt, va_list ap) {
  out_t o = { .buf = out, .pos = 0, .cap = 0, .do_putch = 0, .unbounded = 1 };
  return do_format(&o, fmt, ap);
}

int sprintf(char *out, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int r = vsprintf(out, fmt, ap);
  va_end(ap);
  return r;
}

int snprintf(char *out, size_t n, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int r = vsnprintf(out, n, fmt, ap);
  va_end(ap);
  return r;
}

int vsnprintf(char *out, size_t n, const char *fmt, va_list ap) {
  out_t o = { .buf = out, .pos = 0, .cap = n, .do_putch = 0, .unbounded = 0 };
  return do_format(&o, fmt, ap);
}

#endif
