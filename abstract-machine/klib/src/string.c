#include <klib.h>
#include <klib-macros.h>
#include <stdint.h>

#if !defined(__ISA_NATIVE__) || defined(__NATIVE_USE_KLIB__)

size_t strlen(const char *s) {
  const char *p = s;
  while (*p != '\0') p++;
  return (size_t)(p - s);
}

char *strcpy(char *dst, const char *src) {
  char *r = dst;
  while ((*dst++ = *src++) != '\0');
  return r;
}

char *strncpy(char *dst, const char *src, size_t n) {
  char *r = dst;
  size_t i = 0;
  while (i < n && src[i] != '\0') {
    dst[i] = src[i];
    i++;
  }
  while (i < n) {
    dst[i] = '\0';
    i++;
  }
  return r;
}

char *strcat(char *dst, const char *src) {
  char *r = dst;
  while (*dst != '\0') dst++;
  while ((*dst++ = *src++) != '\0');
  return r;
}

int strcmp(const char *s1, const char *s2) {
  while (*s1 != '\0' && *s1 == *s2) { s1++; s2++; }
  return (int)(unsigned char)*s1 - (int)(unsigned char)*s2;
}

int strncmp(const char *s1, const char *s2, size_t n) {
  size_t i = 0;
  while (i < n && s1[i] != '\0' && s1[i] == s2[i]) i++;
  if (i == n) return 0;
  return (int)(unsigned char)s1[i] - (int)(unsigned char)s2[i];
}

void *memset(void *s, int c, size_t n) {
  unsigned char *p = (unsigned char *)s;
  unsigned char b = (unsigned char)c;
  for (size_t i = 0; i < n; i++) p[i] = b;
  return s;
}

void *memmove(void *dst, const void *src, size_t n) {
  unsigned char *d = (unsigned char *)dst;
  const unsigned char *s = (const unsigned char *)src;
  if (d == s || n == 0) return dst;
  if (d < s) {
    for (size_t i = 0; i < n; i++) d[i] = s[i];
  } else {
    for (size_t i = n; i > 0; i--) d[i - 1] = s[i - 1];
  }
  return dst;
}

void *memcpy(void *out, const void *in, size_t n) {
  unsigned char *d = (unsigned char *)out;
  const unsigned char *s = (const unsigned char *)in;
  for (size_t i = 0; i < n; i++) d[i] = s[i];
  return out;
}

int memcmp(const void *s1, const void *s2, size_t n) {
  const unsigned char *p1 = (const unsigned char *)s1;
  const unsigned char *p2 = (const unsigned char *)s2;
  for (size_t i = 0; i < n; i++) {
    if (p1[i] != p2[i]) return (int)p1[i] - (int)p2[i];
  }
  return 0;
}

#endif
