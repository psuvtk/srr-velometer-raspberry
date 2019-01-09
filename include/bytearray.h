#ifndef BYTEARRAY_H
#define BYTEARRAY_H

#include <stdlib.h>
#include <stdbool.h>
#include <malloc.h>
#include <string.h>

#define DEFAULT_BUF_LEN 2048

typedef struct {
    char *data;
    int size;
    int capacity;
} bytearray_t;

extern void bytearray_init(bytearray_t *buf);
extern void bytearray_destroy(bytearray_t *buf);

extern void bytearray_append(bytearray_t *buf, char *pdata, int len);
extern void bytearray_lremove(bytearray_t *buf, int len);
extern void bytearray_clear(bytearray_t *buf);

extern bool bytearray_startswith(const bytearray_t *buf, const char *pdata, int len);
extern int bytearray_find(const bytearray_t *buf, const char *data, int len, int skip);

#endif // BYTEARRAY_H
